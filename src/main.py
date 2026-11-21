"""SoccerSim strategy entry point and primary match logic.

Structure, from high level to low level:
- main.py (this file): match strategy. ``play()`` dispatches through the Phase
  state machine to ``_act_*`` functions. Each function selects an attacker,
  usually the player nearest the ball, and invokes Player actions directly.
- player.py: Player control handle and high-level actions such as ``attack``,
  ``take_kickoff``, ``move_to_position``, and ``walk_to``. Add new skills there.
- utils/: movement, geometry, and obstacle-avoidance helpers such as
  ``opponent_goal``, ``dist``, and ``angle_to``.
- framework/: platform plumbing that users normally do not modify.

To change the playing style, primarily edit the Phase state machine, ``_act_*``
behaviors, and positioning formulas in this file.
"""

from __future__ import annotations

import logging
import math
from enum import Enum

from booster_agent_framework import AgentBase

from .framework.agent import SoccerAgentMixin
from .framework.types import KICKING_TEAM_NONE, Context, GameState, SetPlay
from .param import *
from .player import Player
from .utils import angle_to, clamp_inside_field, dist, opponent_goal, own_goal


_log = logging.getLogger(__name__)


# ======================================================================
# Phase state machine: match phase classification
# ======================================================================


class Phase(Enum):
    """Top-level match phase controlling normal play, restarts, and stops."""
    NORMAL = "normal"              # Normal contest while PLAYING
    OUR_KICKOFF = "our_kickoff"    # Our kickoff during early SET/PLAYING
    OPP_KICKOFF = "opp_kickoff"    # Opponent kickoff; keep clear
    OUR_SET_PLAY = "our_set_play"  # Our free kick, corner, or goal kick
    OPP_SET_PLAY = "opp_set_play"  # Opponent set play; keep clear
    READY = "ready"                # Move into READY positions
    STOPPED = "stopped"            # Non-kickoff SET, INITIAL, FINISHED, or stopped


def get_phase(context: Context) -> Phase:
    """Determine the current phase from the GameController state."""
    g = context.game
    if g is None:
        return Phase.STOPPED

    state = g.state

    # READY: move to ready positions.
    if state == GameState.READY:
        return Phase.READY

    # PLAYING: normal play or an active kickoff/set play.
    if state == GameState.PLAYING and not g.stopped:
        # For a set play, kicking_team identifies the team taking it.
        if g.set_play != SetPlay.NONE and g.kicking_team != KICKING_TEAM_NONE:
            our_team = context.team_id
            if g.kicking_team == our_team:
                return Phase.OUR_SET_PLAY
            else:
                return Phase.OPP_SET_PLAY

        # During the kickoff countdown, kicking_team identifies the taker.
        if g.secondary_time > 0 and g.kicking_team != KICKING_TEAM_NONE:
            our_team = context.team_id
            if g.kicking_team == our_team:
                return Phase.OUR_KICKOFF
            else:
                return Phase.OPP_KICKOFF

        # Normal play.
        return Phase.NORMAL

    # Remain stationary during SET, INITIAL, FINISHED, or a stoppage.
    return Phase.STOPPED

def get_set_play_type(context: Context) -> SetPlay:
    """Return the active set-play type, or ``SetPlay.NONE`` when unavailable.

    This reads the GameController's ``set_play`` field without identifying the
    taking team. :func:`get_phase` distinguishes OUR_SET_PLAY from OPP_SET_PLAY;
    this function only identifies the type.

    The seven possible values from framework.types.SetPlay are:
    - ``NONE``: no set play, such as normal play or a kickoff
    - ``DIRECT_FREE_KICK``: a direct free kick that may score immediately
    - ``INDIRECT_FREE_KICK``: another player must touch the ball before a goal
    - ``PENALTY_KICK``: a penalty kick
    - ``THROW_IN``: a kick-in from touch
    - ``GOAL_KICK``: a goal kick
    - ``CORNER_KICK``: a corner kick
    """
    g = context.game
    if g is None:
        return SetPlay.NONE
    return g.set_play


# ======================================================================
# Agent entry point
# ======================================================================


class SoccerSimAgent(SoccerAgentMixin, AgentBase):
    """3v3 SoccerSim agent."""

    player_class = Player

    def init_store(self, store) -> None:
        _log.info("init_store called")
        store.prev_phase = None       # Previous phase, used to detect transitions
        store.cur_phase = None
        store.kickoff_taker = None    # Locked taker ID, reselected for each kickoff
        store.kickoff_zone_active = False  # Two-touch rule until ball leaves kickoff zone
        store.normal_attacker = None

    @staticmethod
    def play(context: Context, players: list[Player], store) -> None:
        phase = get_phase(context)
        store.prev_phase = store.cur_phase
        store.cur_phase = phase

        # Draw per-frame visualizations.
        _analyze_and_draw(context, players, store)

        # Draw the current phase as a label just outside the field.
        from .framework import debugdraw
        g = context.game
        game_state = g.state.value if g is not None else "none"
        set_play = g.set_play.value if g is not None else "none"
        secondary_time = g.secondary_time if g is not None else 0.0
        debugdraw.text(
            0.0, context.field.width / 2.0 + 0.2,
            f"phase={phase.value} state={game_state} set={set_play} secondary={secondary_time:.1f}",
            rgb=(1.0, 1.0, 0.0), ns="phase",
        )

        # Handle readiness and retain only players that can act this frame.
        # ensure_ready asynchronously gets up or switches to walk mode without
        # moving. Penalized players also prepare so they can rejoin immediately.
        # Penalized or unready players are excluded from role assignment to avoid
        # selecting an attacker who cannot move.
        active: list[Player] = []
        for p in players:
            ready = p.ensure_ready()
            if p.is_penalized:
                p.action = "penalized"     # May prepare, but cannot move
                p.stop()
            elif not ready:
                p.action = "fallen" if p.is_fallen else "switching_mode"
            elif p.pose is None:
                p.action = "no_pose"       # Exclude players with unknown positions
                p.stop()
            else:
                active.append(p)

        # Dispatch the team once per phase; _act_* owns team-wide role assignment.
        if phase == Phase.NORMAL:
            _act_normal(context, active, store)
        elif phase == Phase.OUR_KICKOFF:
            _clear_normal_sticky(store)
            _act_our_kickoff(context, active, store)
        elif phase == Phase.OPP_KICKOFF:
            _clear_normal_sticky(store)
            _act_opp_kickoff(context, active)
        elif phase == Phase.OUR_SET_PLAY:
            _clear_normal_sticky(store)
            _act_our_set_play(context, active, store)
        elif phase == Phase.OPP_SET_PLAY:
            _clear_normal_sticky(store)
            _act_opp_set_play(context, active, store)
        elif phase == Phase.READY:
            _clear_normal_sticky(store)
            _act_ready(context, active)
        elif phase == Phase.STOPPED:
            _clear_normal_sticky(store)
            for p in active:
                p.action = "stopped"
                p.stop()

        # Draw every teammate last, including penalized, unready, and stopped
        # players, so markers remain visible during states such as SET.
        for p in players:
            _draw_teammate_marker(p)


def _clear_normal_sticky(store) -> None:
    store.normal_attacker = None


def _player_dist_to_position(p: Player, target: tuple[float, float]) -> float:
    """Return infinity when the player's position is unknown."""
    pose = p.pose
    if pose is None:
        return math.inf
    return dist(pose.x, pose.y, *target)


def _player_dist_to_ball(context: Context, p: Player) -> float:
    """Return a player's distance to the ball's current position."""
    ball = context.ball
    return (
        _player_dist_to_position(p, (ball.x, ball.y)) + _fallen_time_cost(p)
        if ball is not None else math.inf
    )


def _fallen_time_cost(p: Player) -> float:
    return FALLEN_COST if p.is_fallen else 0.0


def _assign_to_slots(
    context: Context,
    players: list[Player],
    slots: list[tuple[float, float]],
    action_label: str,
) -> None:
    """Greedily assign each player to the nearest remaining slot by distance.

    This avoids cross-field travel by matching players to the closest
    unclaimed target, regardless of list order.
    """
    available = list(enumerate(slots))
    for p in players:
        if not available:
            break
        # Find the nearest remaining slot.
        best_idx, best_target = min(
            available,
            key=lambda item: _player_dist_to_position(p, item[1]),
        )
        p.action = action_label
        p.walk_to(best_target, face=0.0, avoid_ball=True, avoid_robots=True)
        available.remove((best_idx, best_target))


def _select_closest_attacker(
    context: Context,
    players: list[Player],
    preferred_id: int | None = None,
) -> Player:
    """Select the player with the lowest effective distance to the ball.

    ``players`` must be nonempty, ready, and have known poses. Normal play and
    kickoffs share this selection logic.
    """
    ranked = [(p, _player_dist_to_ball(context, p)) for p in players]
    best, best_dist = min(ranked, key=lambda item: item[1])
    preferred = next((item for item in ranked if item[0].id == preferred_id), None)
    if (
        preferred is not None
        and preferred[1] <= best_dist + ATTACKER_KEEP_DIST_MARGIN_M
    ):
        return preferred[0]
    return best


def _check_inactivity(context: Context, players: list[Player], store) -> None:
    """M-05 protection: reposition robots that stay too still near the ball.

    The referee may penalize a robot within 3 m of the ball that shows
    almost no displacement for 10 s (local game stuck).  We track each
    player's position and, when the inactivity alert threshold is exceeded,
    command a short reposition to reset the displacement counter.
    """
    ball = context.ball
    if ball is None:
        return
    now = context.now
    for p in players:
        pose = p.pose
        if pose is None:
            continue
        # Only players near the ball are subject to M-05.
        if dist(pose.x, pose.y, ball.x, ball.y) > BALL_NEAR_RADIUS_M:
            p._inactivity_pos = None
            p._inactivity_since = None
            continue
        # Initialise or update tracking.
        if p._inactivity_pos is None or p._inactivity_since is None:
            p._inactivity_pos = (pose.x, pose.y)
            p._inactivity_since = now
        else:
            last_x, last_y = p._inactivity_pos
            moved = dist(pose.x, pose.y, last_x, last_y)
            if moved >= INACTIVITY_MIN_DISPLACEMENT_M:
                # Meaningful movement — reset the timer.
                p._inactivity_pos = (pose.x, pose.y)
                p._inactivity_since = now
            elif now - p._inactivity_since > INACTIVITY_ALERT_S:
                # Too still — nudge slightly away from the current position
                # to break the inactivity window.
                offset_x = pose.x + (0.6 if p.id % 2 == 0 else -0.6)
                offset_y = pose.y + (0.4 if p.id % 3 == 0 else -0.4)
                target = clamp_inside_field(context, offset_x, offset_y, margin=0.5)
                p.action = "inactive:reposition"
                p.walk_to(target, face=0.0, avoid_ball=True, avoid_robots=True)
                p._inactivity_pos = (pose.x, pose.y)
                p._inactivity_since = now


def _act_normal(context: Context, players: list[Player], store) -> None:
    """Assign the nearest player to attack, the next to guard, and others to support.

    ``players`` contains ready players with known poses. Assign and execute
    their roles directly here.
    """
    if not players:
        return

    # Two-touch kickoff aftermath: while the ball is still inside the kickoff
    # zone after our kickoff, keep clearing it out before anyone shoots.
    # Only a seen ball outside the zone clears the flag; a lost ball keeps
    # the gate closed so no shot happens on a stale or missing position.
    if getattr(store, "kickoff_zone_active", False):
        ball = context.ball
        if ball is not None and dist(ball.x, ball.y, 0.0, 0.0) > KICKOFF_CLEAR_RADIUS_M:
            store.kickoff_zone_active = False

    attacker = _select_closest_attacker(
        context, players, getattr(store, "normal_attacker", None),
    )
    store.normal_attacker = attacker.id
    if getattr(store, "kickoff_zone_active", False):
        attacker.action = "kickoff:push"
        attacker.kickoff_push()
    else:
        attacker.action = "attack"
        attacker.attack()

    # The remaining player nearest our goal becomes the guard.
    rest = [p for p in players if p is not attacker]
    if rest:
        gx, gy = own_goal(context)
        guard = min(rest, key=lambda p: _player_dist_to_position(p, (gx, gy)))
        guard.guard()
        rest = [p for p in rest if p is not guard]

    # All other players support.
    for p in rest:
        p.action = "support"
        p.support()

    # M-05 inactivity protection: robots near the ball must keep moving.
    # If any player has been nearly stationary inside the 3 m ball radius
    # for too long, reposition to avoid a local-game-stuck penalty.
    _check_inactivity(context, players, store)


def _act_our_kickoff(context: Context, players: list[Player], store) -> None:
    """Lock the nearest kickoff taker, assign a guard, and leave others in support.

    Two-touch kickoff rule: the taker first pushes the ball out of the
    kickoff zone around the center spot along a non-scoring line, then
    switches to a normal attack once the ball has left the zone.
    """
    if not players:
        return

    active_ids = {p.id for p in players}
    if store.prev_phase != Phase.OUR_KICKOFF or store.kickoff_taker not in active_ids:
        # Select a new taker when entering the kickoff phase.
        store.kickoff_taker = _select_closest_attacker(context, players).id

    attacker_id = store.kickoff_taker
    attacker = next((p for p in players if p.id == attacker_id), None)
    if attacker is None:
        return

    ball = context.ball
    if ball is not None and dist(ball.x, ball.y, 0.0, 0.0) > KICKOFF_CLEAR_RADIUS_M:
        # The ball has left the kickoff zone; goals now count normally.
        store.kickoff_zone_active = False
        attacker.action = "kickoff:go"
        attacker.attack()
    else:
        # First touch: push the ball out of the zone along a non-scoring line.
        store.kickoff_zone_active = True
        attacker.action = "kickoff:push"
        attacker.kickoff_push()

    rest = [p for p in players if p is not attacker]
    if rest:
        gx, gy = own_goal(context)
        guard = min(rest, key=lambda p: _player_dist_to_position(p, (gx, gy)))
        guard.guard()
        rest = [p for p in rest if p is not guard]

    # Non-taker teammates must stay outside the center circle (P-02).
    # Move them to positions behind the circle instead of just stopping in
    # place, in case they were still inside from the READY phase.
    circle_r = context.field.circle_radius
    kickoff_wait_slots = [
        (-circle_r - 0.5, 0.0),
        (-circle_r - 1.5, 0.8),
    ]
    for p, target in zip(rest, kickoff_wait_slots):
        p.action = "kickoff:wait"
        p.walk_to(target, avoid_ball=True, avoid_robots=True)
    # Any remaining players (shouldn't happen in 3v3) also move out.
    for p in rest[len(kickoff_wait_slots):]:
        p.action = "kickoff:wait"
        p.walk_to((-circle_r - 0.5, 0.0), avoid_ball=True, avoid_robots=True)


def _opp_kickoff_press_target(
    context: Context,
    ball,
    index: int,
    fallback: list[tuple[float, float]],
) -> tuple[float, float]:
    """Press-ready position just outside the circle on the ball-to-goal line.

    The non-kicking team may move inside its own half during the kickoff
    countdown, so instead of freezing at ready slots the players hold a
    press shape facing the ball. Targets stay outside the center circle and
    inside our half to respect kickoff positioning rules.
    """
    if ball is None:
        return fallback[min(index, len(fallback) - 1)]
    gx, gy = own_goal(context)
    dx, dy = gx - ball.x, gy - ball.y
    length = math.hypot(dx, dy) or 1.0
    ux, uy = dx / length, dy / length
    # First ring on the line, second ring deeper on the opposite flank.
    reach = OPP_KICKOFF_PRESS_DIST_M + (0.0 if index == 0 else 0.6)
    lateral = -0.9 if index == 0 else 0.9
    tx = ball.x + ux * reach - uy * lateral
    ty = ball.y + uy * reach + ux * lateral
    tx, ty = clamp_inside_field(context, tx, ty, margin=0.3)
    # Legality: outside the center circle and strictly inside our half.
    tx = min(tx, -CIRCLE_MARGIN_M)
    tx = max(tx, -context.field.length / 2.0 + 0.3)
    return (tx, ty)


def _act_opp_kickoff(context: Context, players: list[Player]) -> None:
    """Guard with one player while the others hold press-ready positions.

    Instead of standing still at fixed ready slots during the opponent's
    approach, the two field players keep a press shape just outside the
    center circle on the ball-to-goal line, facing the ball, ready to
    contest the moment ball ownership opens.
    """
    if not players:
        return
    gx, gy = own_goal(context)
    guard = min(players, key=lambda p: _player_dist_to_position(p, (gx, gy)))
    guard.guard()

    ball = context.ball
    rest = [p for p in players if p is not guard]
    r = context.field.circle_radius
    fallback = [(-r - 0.5, 0.0), (-r - 2.0, 0.5)]
    for index, p in enumerate(rest):
        p.action = "opp_kickoff:press"
        target = _opp_kickoff_press_target(context, ball, index, fallback)
        face = (
            angle_to(p.pose.x, p.pose.y, ball.x, ball.y)
            if ball is not None and p.pose is not None
            else None
        )
        p.walk_to(target, face=face, avoid_ball=True, avoid_robots=True)


def _act_our_set_play(context: Context, players: list[Player], store) -> None:
    """Dispatch our set play with own-goal avoidance for indirect restarts.

    THROW_IN, GOAL_KICK, and CORNER_KICK cannot score directly into our own
    goal (G-04): the first touch must go away from our goal.  For these we
    use the normal attack flow, which aims at the opponent goal and
    inherently avoids our own.  Other set plays (free kicks, penalties)
    also default to normal play for now.
    """
    _act_normal(context, players, store)


def _act_opp_set_play(context: Context, players: list[Player], store) -> None:
    """Retreat directly from the ball; wait until the official phase changes."""
    ball = context.ball
    # Account for walk_to's arrival tolerance when targeting the 1.5 m boundary.
    radius = OPP_SET_PLAY_CLEARANCE_M + ARRIVE_DIST
    half_l = context.field.length / 2.0 - OPP_SET_PLAY_FIELD_MARGIN_M
    half_w = context.field.width / 2.0 - OPP_SET_PLAY_FIELD_MARGIN_M
    for p in players:
        pose = p.pose
        p.action = "opp_set:wait"
        if ball is None or pose is None:
            p.stop()
            continue
        if dist(pose.x, pose.y, ball.x, ball.y) >= OPP_SET_PLAY_CLEARANCE_M:
            p.stop()
            continue

        # The radial exit is shortest. At the field edge, also consider the
        # retreat circle's intersections with the in-field boundary.
        outward = angle_to(ball.x, ball.y, pose.x, pose.y)
        candidates = [(
            ball.x + math.cos(outward) * radius,
            ball.y + math.sin(outward) * radius,
        )]
        for x in (-half_l, half_l):
            remaining = radius ** 2 - (x - ball.x) ** 2
            if remaining >= 0.0:
                offset = math.sqrt(remaining)
                candidates.extend((x, ball.y + sign * offset) for sign in (-1.0, 1.0))
        for y in (-half_w, half_w):
            remaining = radius ** 2 - (y - ball.y) ** 2
            if remaining >= 0.0:
                offset = math.sqrt(remaining)
                candidates.extend((ball.x + sign * offset, y) for sign in (-1.0, 1.0))
        candidates = [
            t for t in candidates
            if t == clamp_inside_field(context, *t, margin=OPP_SET_PLAY_FIELD_MARGIN_M)
        ]
        if not candidates:
            p.stop()
            continue

        px, py = pose.x, pose.y
        target = min(candidates, key=lambda t: dist(px, py, *t))
        # Short omnidirectional steps follow the retreat line directly.
        # Do not invoke path avoidance: the movement is away from the ball.
        length = dist(pose.x, pose.y, *target)
        ratio = min(1.0, OMNI_DIST / (2.0 * length))
        step = (
            pose.x + (target[0] - pose.x) * ratio,
            pose.y + (target[1] - pose.y) * ratio,
        )
        p.action = "opp_set:retreat"
        p.walk_to(step, face=angle_to(pose.x, pose.y, ball.x, ball.y))


def _act_ready(context: Context, players: list[Player]) -> None:
    """Move each player to a ready position, assigning by distance not index."""
    game = context.game
    our_kickoff = game is not None and game.kicking_team == context.team_id
    field = context.field
    # Our kickoff priority: center-circle edge, goal-area center, then left side.
    if our_kickoff:
        slots = [
            (-field.circle_radius, 0.0),
            (-field.length / 2.0 + field.goal_area_length, 0.0),
            (-0.5, field.circle_radius + 2),
        ]
    # Opponent kickoff priority: outside center circle, goal area, then penalty area.
    else:
        slots = [
            (-field.circle_radius - 0.5, 0.0),
            (-field.length / 2.0 + field.goal_area_length, 0.0),
            (-field.length / 2.0 + field.penalty_area_length, 0.0),
        ]
    _assign_to_slots(context, players, slots, "ready")



# ======================================================================
# Match visualization: ball position and each player's distance to the ball
# ======================================================================

def _draw_teammate_marker(p: Player) -> None:
    """Draw teammates in red, using cubes while kicking and spheres otherwise.

    This runs for every player each frame regardless of phase, penalty, or
    readiness. The label contains the player ID and current high-level action
    (``p.action``), with ``[KICK]`` appended while kicking. Shape also indicates
    whether the player is in the kicking state.
    """
    from .framework import debugdraw

    pose = p.pose
    if pose is None:
        return
    red = (1.0, 0.2, 0.2)
    if p.is_kicking:
        debugdraw.cube(pose.x, pose.y, rgb=red, scale=0.38, ns="teammate")
    else:
        debugdraw.point(pose.x, pose.y, rgb=red, scale=0.3, ns="teammate")
    kick_tag = " [KICK]" if p.is_kicking else ""
    label = f"{p.id}:{p.action}{kick_tag}"
    debugdraw.text(pose.x, pose.y, label, rgb=(1.0, 0.9, 0.6), ns="teammate_id")


def _analyze_and_draw(context: Context, players: list[Player], store) -> None:
    """Calculate player-to-ball distances and draw them each frame.

    This no longer depends on the analysis module; distances use the ball's
    current position.
    """
    from .framework import debugdraw

    ball = context.ball

    # Nothing can be drawn when the ball is not visible.
    if ball is None:
        return

    # 1. Draw the ball's current position as a green point.
    debugdraw.point(ball.x, ball.y, rgb=(0.0, 1.0, 0.0), scale=0.2, ns="ball_current")

    # 2. Draw distance labels: red for teammates and blue for opponents.
    for p in players:
        pose = p.pose
        if pose is None:
            continue
        d = dist(pose.x, pose.y, ball.x, ball.y)
        debugdraw.text(
            pose.x + 0.3, pose.y - 0.3, f"{d:.1f}m",
            rgb=(1.0, 0.6, 0.6), ns="dist_ours",
        )
    for r in context.opponents.values():
        pose = r.pose
        if pose is None:
            continue
        d = dist(pose.x, pose.y, ball.x, ball.y)
        debugdraw.text(
            pose.x + 0.3, pose.y - 0.3, f"{d:.1f}m",
            rgb=(0.6, 0.6, 1.0), ns="dist_opp",
        )