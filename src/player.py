"""Player control handle for one robot, designed for direct user editing.

Platform primitives such as ``set_velocity``, ``kick``, ``release_kick``,
``request_mode``, and ``get_up`` delegate to the injected framework
``_backend``. State properties such as ``pose``, ``mode``, ``is_fallen``, and
``penalty`` read from ``self.context`` or the backend.

Movement behaviors such as ``walk_to``, ``face_to``, and ``ensure_ready`` are
also Player methods because they command this player and may need cross-frame
state for hysteresis or avoidance. Pure coordinate calculations such as
``dist``, ``angle_to``, and goal coordinates belong in utils/geom.

Instances live for the entire match, while the framework replaces
``self.context`` every frame. Add custom skills directly to this class.
"""

from __future__ import annotations

import logging
import math
from typing import TYPE_CHECKING

from .framework.types import Context, Penalty, Pose2D
from .framework import debugdraw

from .param import *
from .utils.geom import (
    angle_to,
    clamp,
    clamp_inside_field,
    dist,
    normalize_angle,
    opponent_goal,
    own_goal,
    own_goal_area_center,
    segment_point_distance,
)
from .utils.obstacles import collect_obstacles
from .utils.path_planner import plan_global_path

if TYPE_CHECKING:
    from .framework.config import SoccerConfig
    from .framework.robot_backend import RobotBackend


__all__ = ["Player"]


_log = logging.getLogger(__name__)

def _heading_clearance(
    px: float, py: float, heading: float, obstacles: list,
) -> float:
    """Return clearance along a lookahead ray for local obstacle avoidance.

    Only obstacles ahead of (px, py), with projection t > 0, block this heading.
    Ignoring obstacles behind or to the rear prevents a nearby rear obstacle
    from reducing clearance in every direction. Return infinity when clear;
    larger values mean more space and negative values indicate a collision.
    """
    ux, uy = math.cos(heading), math.sin(heading)
    min_clear = math.inf
    for obs in obstacles:
        t = (obs.x - px) * ux + (obs.y - py) * uy
        if t <= 0.0:
            continue                      # Rear obstacles do not block this heading.
        if t > PLAN_LOOKAHEAD:
            t = PLAN_LOOKAHEAD
        nx, ny = px + ux * t, py + uy * t
        clear = math.hypot(obs.x - nx, obs.y - ny) - obs.radius
        if clear < min_clear:
            min_clear = clear
    return min_clear


def _behind_ball(
    ball_x: float, ball_y: float, aim: tuple[float, float], offset: float,
) -> tuple[float, float]:
    """Return a position ``offset`` behind the ball relative to ``aim``.

    This is used as the chase target so the ball lies between the robot and
    ``aim``, naturally aligning the robot on arrival. If ``aim`` coincides with
    the ball and the direction is undefined, return the ball position.
    """
    dx, dy = aim[0] - ball_x, aim[1] - ball_y
    d = math.hypot(dx, dy)
    if d < 1e-6:
        return (ball_x, ball_y)
    ux, uy = dx / d, dy / d              # Unit vector from the ball to aim.
    return (ball_x - ux * offset, ball_y - uy * offset)   # Move away from aim.


class Player:
    """Handle for one player. Add new skills here.
    """

    def __init__(
        self,
        player_id: int,
        config: "SoccerConfig",
        _backend: "RobotBackend | None",
    ) -> None:
        self.id: int = player_id
        self.config: "SoccerConfig" = config
        self._backend = _backend           # Robot control wrapper; rarely changed
        self.context: Context | None = None

        # Current high-level action for visualization and debugging. Strategy
        # dispatch updates it each frame; actions such as guard refine substates
        # internally. The visualization pass in main.py uses it as a label.
        self.action: str = "init"

        # SDK cache fields updated automatically by the framework backend.
        self._mode: str | None = None
        self._fall_down_state: str | None = None

        # Cross-frame detour-side memory; None means no active detour.
        self._avoid_side: float | None = None

        # Cross-frame kick hysteresis state.
        self._kicking: bool = False

        # Cross-frame guard threat hysteresis.
        self._guard_threatened: bool = False
        # Cross-frame state for temporarily attacking when support gets stuck.
        self._support_last_pos: tuple[float, float] | None = None
        self._support_stationary_since: float | None = None
        self._support_last_update_at: float | None = None

        # M-05 inactivity tracking: last position and time of meaningful displacement.
        self._inactivity_pos: tuple[float, float] | None = None
        self._inactivity_since: float | None = None

    # ------------------------------------------------------------------
    # State access
    # ------------------------------------------------------------------

    @property
    def is_kicking(self) -> bool:
        """Return whether the player is currently kicking."""
        return self._kicking

    @property
    def pose(self) -> Pose2D | None:
        ctx = self.context
        if ctx is None:
            return None
        robot = ctx.teammates.get(self.id)
        return None if robot is None else robot.pose

    @property
    def mode(self) -> str | None:
        if self._backend is not None:
            return self._backend.mode
        return self._mode

    @property
    def is_fallen(self) -> bool:
        return self.fall_down_state not in (None, "normal")

    @property
    def fall_down_state(self) -> str | None:
        if self._backend is not None:
            return getattr(self._backend, "fall_down_state", None)
        return self._fall_down_state

    @property
    def penalty(self) -> Penalty:
        ctx = self.context
        game = ctx.game if ctx is not None else None
        if game is None:
            return Penalty.NONE
        state = game.get_player_state(self.config.team_id, self.id)
        return Penalty.NONE if state is None else state.penalty

    @property
    def is_penalized(self) -> bool:
        return self.penalty != Penalty.NONE

    # ------------------------------------------------------------------
    # Chassis control
    # ------------------------------------------------------------------

    def set_velocity(self, vx: float, vy: float, vyaw: float) -> None:
        if self._backend is None:
            _log.debug(
                "player %d set_velocity vx=%.3f vy=%.3f vyaw=%.3f (no backend)",
                self.id, vx, vy, vyaw,
            )
            return
        self._backend.set_velocity(vx, vy, vyaw)

    def stop(self) -> None:
        self.release_kick()
        self.set_velocity(0.0, 0.0, 0.0)

    # ------------------------------------------------------------------
    # Kicking
    # ------------------------------------------------------------------

    def kick(
        self,
        kick_direction: float | None = None,
        power: float = KICK_POWER_DEFAULT,
    ) -> None:
        pose = self.pose
        if pose is None:
            _log.warning("player %d kick skipped: pose unknown", self.id)
            return

        ctx = self.context
        ball = ctx.ball if ctx is not None else None
        if ball is None:
            _log.warning("player %d kick skipped: ball unknown", self.id)
            return

        if kick_direction is None:
            kick_plan = self.plan_kick()
            if kick_plan is None:
                _log.warning("player %d kick skipped: kick plan unavailable", self.id)
                return
            kick_direction, power = kick_plan

        if self._backend is None:
            _log.debug(
                "player %d kick ball=(%.3f, %.3f) dir=%.3f (no backend)",
                self.id, ball.x, ball.y, kick_direction,
            )
            return
        # Transform field coordinates to body coordinates using the current pose.
        dx = ball.x - pose.x
        dy = ball.y - pose.y
        cos_t = math.cos(pose.theta)
        sin_t = math.sin(pose.theta)
        ball_x_body = dx * cos_t + dy * sin_t
        ball_y_body = -dx * sin_t + dy * cos_t
        kick_direction = normalize_angle(kick_direction)
        direction_body = normalize_angle(kick_direction - pose.theta)
        power_clamped = max(KICK_POWER_MIN, min(KICK_POWER_MAX, power))
        self._kicking = True
        self._backend.kick(direction_body, power_clamped, ball_x_body, ball_y_body)

    def plan_kick(self) -> tuple[float, float] | None:
        """Decide this frame's kick direction and power.

        Decision ladder, safest option first:
        1. Shot: only when the straight ball line can enter the goal mouth
           and the goal is within ``SHOT_MAX_RANGE_M``. Near the sideline a
           shot across field is never taken.
        2. Pass: best forward teammate with a clear receive lane.
        3. Carry: low-power nudge forward-infield, keeping possession.

        Returns ``(kick_direction, kick_power)``, or None when the ball or
        context is unavailable.
        """
        ctx = self.context
        ball = ctx.ball if ctx is not None else None
        if ctx is None or ball is None:
            return None

        near_sideline = (
            abs(ball.y) > ctx.field.width / 2.0 - SIDELINE_RECOVER_Y_M
        )
        goal_x = ctx.field.length / 2.0
        shot_range = dist(ball.x, ball.y, goal_x, 0.0)

        shot = None
        if not near_sideline and shot_range <= SHOT_MAX_RANGE_M:
            shot = self._best_shot()
        if shot is not None:
            self._draw_kick_target(shot)
            return angle_to(ball.x, ball.y, shot[0], shot[1]), SHOT_POWER

        mate = self._best_pass_target()
        if mate is not None:
            pass_distance = dist(ball.x, ball.y, mate[0], mate[1])
            pass_power = clamp(
                PASS_POWER_MIN + PASS_POWER_PER_M * pass_distance,
                PASS_POWER_MIN, PASS_POWER_MAX,
            )
            self._draw_kick_target(mate)
            return angle_to(ball.x, ball.y, mate[0], mate[1]), pass_power

        carry = self._carry_target()
        self._draw_kick_target(carry)
        return angle_to(ball.x, ball.y, carry[0], carry[1]), CARRY_POWER

    def _best_shot(self) -> tuple[float, float] | None:
        """Best aim point across the opponent goal mouth, or None.

        Scans aim points inside the posts (each validated with
        ``kick_can_score``) and ranks them by lane clearance from opponents
        blended with openness (distance from the nearest opponent to the aim
        point), so shots go to the corner the goalkeeper covers least.
        Candidates below ``SHOT_MIN_LANE_SCORE`` are rejected; None means no
        candidate qualifies and the caller should pass or carry instead.
        """
        ctx = self.context
        ball = ctx.ball if ctx is not None else None
        if ctx is None or ball is None:
            return None

        goal_x = ctx.field.length / 2.0
        reach = max(0.0, ctx.field.goal_width / 2.0 - SHOT_POST_MARGIN_M)
        steps = max(1, int(reach / SHOT_SCAN_STEP_M))
        opponents = [
            (robot.pose.x, robot.pose.y)
            for robot in ctx.opponents.values()
            if robot.pose is not None
        ]

        best: tuple[float, tuple[float, float]] | None = None
        for index in range(-steps, steps + 1):
            aim_y = reach * index / steps
            if not self.kick_can_score(
                angle_to(ball.x, ball.y, goal_x, aim_y)
            ):
                continue
            lane = 1.0
            openness = 1.0
            if opponents:
                nearest = math.inf
                lane_min = math.inf
                for ox, oy in opponents:
                    distance, t = segment_point_distance(
                        ball.x, ball.y, goal_x, aim_y, ox, oy,
                    )
                    nearest = min(nearest, distance)
                    if t >= 0.1:
                        lane_min = min(lane_min, distance)
                if lane_min < SHOT_LANE_CLEAR_REF_M * SHOT_MIN_LANE_SCORE:
                    continue  # This corner is too well blocked.
                lane = clamp(lane_min / SHOT_LANE_CLEAR_REF_M, 0.0, 1.0)
                openness = clamp(nearest / SHOT_OPEN_REF_M, 0.0, 1.0)
            score = (1.0 - SHOT_OPEN_WEIGHT) * lane + SHOT_OPEN_WEIGHT * openness
            if best is None or score > best[0]:
                best = (score, (goal_x, aim_y))
        return None if best is None else best[1]

    def _best_pass_target(self) -> tuple[float, float] | None:
        """Pick the best forward teammate to pass to, or None.

        A valid receiver has a known pose, is not penalized, is at least
        ``PASS_MIN_FORWARD_M`` ahead of the ball, and has a receive lane at
        least ``PASS_LANE_CLEARANCE_M`` from every opponent. Among valid
        receivers, prefer the furthest forward with a central preference.
        """
        ctx = self.context
        ball = ctx.ball if ctx is not None else None
        if ctx is None or ball is None:
            return None

        best: tuple[float, tuple[float, float]] | None = None
        game = ctx.game
        for mate_id, mate in ctx.teammates.items():
            if mate_id == self.id or mate.pose is None:
                continue
            if game is not None:
                state = game.get_player_state(ctx.team_id, mate_id)
                if state is not None and state.penalty != Penalty.NONE:
                    continue
            forward = mate.pose.x - ball.x
            if forward < PASS_MIN_FORWARD_M:
                continue
            if not self._lane_is_clear(ball.x, ball.y, mate.pose.x, mate.pose.y):
                continue
            center_pref = 1.0 - abs(mate.pose.y) / (ctx.field.width / 2.0)
            score = forward + PASS_CENTER_WEIGHT_M * center_pref
            if best is None or score > best[0]:
                best = (score, (mate.pose.x, mate.pose.y))
        return None if best is None else best[1]

    def _lane_is_clear(
        self, ax: float, ay: float, bx: float, by: float,
    ) -> bool:
        """Whether the ball-to-receiver segment keeps clearance from opponents.

        Opponents projected very close to the segment start (t < 0.1) are
        ignored: they are contesting the ball itself, and requiring clearance
        there would cancel every pass.
        """
        ctx = self.context
        if ctx is None:
            return False
        for opp in ctx.opponents.values():
            if opp.pose is None:
                continue
            distance, t = segment_point_distance(
                ax, ay, bx, by, opp.pose.x, opp.pose.y,
            )
            if t >= 0.1 and distance < PASS_LANE_CLEARANCE_M:
                return False
        return True

    def _carry_target(self) -> tuple[float, float]:
        """Forward-infield carry target: advance and pull toward the centerline.

        This doubles as the sideline recovery target: near the sideline the
        center pull dominates, so the ball is nudged back into play instead
        of being crossed out of bounds.
        """
        ctx = self.context
        ball = ctx.ball if ctx is not None else None
        if ctx is None or ball is None:
            return (0.0, 0.0)
        return clamp_inside_field(
            ctx,
            ball.x + CARRY_ADVANCE_M,
            ball.y * CARRY_CENTER_PULL,
            margin=0.4,
        )

    def _goal_target_for_direction(
        self, kick_direction: float,
    ) -> tuple[float, float]:
        """Project the shot direction onto the opponent's goal line for display."""
        ctx = self.context
        ball = ctx.ball if ctx is not None else None
        if ctx is None or ball is None:
            return (0.0, 0.0)

        dx = math.cos(kick_direction)
        if dx <= 1e-6:
            return opponent_goal(ctx)
        goal_x = ctx.field.length / 2.0
        t = max(0.0, (goal_x - ball.x) / dx)
        return (goal_x, ball.y + math.sin(kick_direction) * t)

    def _draw_kick_target(self, target: tuple[float, float]) -> None:
        """Mark the kick target selected by plan_kick with an X."""
        from .framework import debugdraw

        x, y = target
        s = KICK_TARGET_MARK_SIZE_M
        debugdraw.line(
            [(x - s, y - s), (x + s, y + s)],
            rgb=(1.0, 0.0, 1.0), ns="kick_target",
        )
        debugdraw.line(
            [(x - s, y + s), (x + s, y - s)],
            rgb=(1.0, 0.0, 1.0), ns="kick_target",
        )

    def release_kick(self) -> None:
        self._kicking = False  # Clear kick hysteresis and cube visualization.
        if self._backend is None:
            _log.debug("player %d release_kick (no backend)", self.id)
            return
        self._backend.release_kick()

    def kick_can_score(self, kick_direction: float) -> bool:
        """Return whether a straight kick in ``kick_direction`` can score.
        """
        ctx = self.context
        ball = ctx.ball if ctx is not None else None
        if ctx is None or ball is None:
            return False

        goal_x = ctx.field.length / 2.0
        dx = math.cos(kick_direction)
        dy = math.sin(kick_direction)
        if dx <= 1e-6:
            return False

        half_goal = ctx.field.goal_width / 2.0
        if half_goal <= 0.0:
            return False
        if ball.x >= goal_x:
            y_at_goal = ball.y
        else:
            t = (goal_x - ball.x) / dx
            if t < 0.0:
                return False
            y_at_goal = ball.y + dy * t
        return -half_goal <= y_at_goal <= half_goal

    # ------------------------------------------------------------------
    # Slow operations, invoked asynchronously to avoid blocking
    # ------------------------------------------------------------------

    @property
    def is_getting_up(self) -> bool:
        """Return whether recovery is queued or still executing in the SDK."""
        return self._backend is not None and self._backend.is_getting_up

    def request_mode(self, mode: str) -> None:
        if self._backend is None:
            _log.debug("player %d request_mode -> %s (no backend)", self.id, mode)
            return
        self._backend.request_mode(mode)

    def get_up(self) -> None:
        if self._backend is None:
            _log.debug("player %d get_up (no backend)", self.id)
            return
        self._backend.get_up()

    # ------------------------------------------------------------------
    # Movement
    # ------------------------------------------------------------------

    def ensure_ready(self) -> bool:
        """Recover from falls and switch to walk mode asynchronously.

        Return whether the player is ready to act during this frame. Mode
        changes must wait for the get-up task, not just a normal fall snapshot.
        """
        if self.is_getting_up:
            return False
        if self.is_fallen:
            self.get_up()
            return False
        if self.mode != "walk":
            self.request_mode("walk")
            return False
        return True

    def face_to(self, target_theta: float) -> None:
        """Turn in place to the target heading."""
        pose = self.pose
        if pose is None:
            self.stop()
            return
        err = normalize_angle(target_theta - pose.theta)
        self.set_velocity(0.0, 0.0, self._angular(err))

    def walk_to(
        self,
        target: tuple[float, float],
        *,
        face: float | None = None,
        avoid_ball: bool = False,
        avoid_robots: bool = False,
        arrive_dist: float = ARRIVE_DIST,
    ) -> bool:
        """Walk toward a target point and return whether it has been reached.

        With ``avoid_ball`` or ``avoid_robots`` enabled, the simplified VFH
        local planner models the ball, robots, and goals as circular obstacles.
        It scans candidate headings and selects the one closest to the target
        that remains collision-free over ``PLAN_LOOKAHEAD``. This handles
        multiple obstacles, concave structures, and symmetric conflicts more
        reliably than single-obstacle detours or potential-field repulsion.

        Nearby targets use omnidirectional walking; distant targets use a
        turn-walk-turn sequence. ``face`` sets the heading near or at the target.
        """
        self.release_kick() # Walking is overridden while kicking, so release first.
        pose = self.pose
        if pose is None:
            self.stop()
            return False

        tx, ty = target
        dx = tx - pose.x
        dy = ty - pose.y
        distance = math.hypot(dx, dy)

        if distance < arrive_dist:
            # At the target, turn to the requested heading if needed.
            if face is not None:
                err = normalize_angle(face - pose.theta)
                if abs(err) > 0.1:
                    self.set_velocity(0.0, 0.0, self._angular(err))
                else:
                    self.stop()
            else:
                self.stop()
            return True

        # Prefer global A* planning and fall back to the legacy local planner.
        goal_dir = math.atan2(dy, dx)
        planned_path: list[tuple[float, float]] | None = None
        waypoint: tuple[float, float] | None = None
        ctx = self.context
        if (avoid_ball or avoid_robots) and ctx is not None:
            obstacles = collect_obstacles(
                ctx, self.id,
                ball=avoid_ball, robots=avoid_robots,
                goals=(avoid_ball or avoid_robots),
            )
            if USE_GLOBAL_PATH_PLANNER:
                planned_path = plan_global_path(
                    ctx,
                    (pose.x, pose.y),
                    (tx, ty),
                    obstacles,
                )
                if planned_path is not None:
                    waypoint = self._path_waypoint(pose, planned_path)
                    heading = angle_to(pose.x, pose.y, waypoint[0], waypoint[1])
                else:
                    heading = self._plan_heading(pose, goal_dir, obstacles)
            else:
                heading = self._plan_heading(pose, goal_dir, obstacles)
        else:
            heading = goal_dir

        # Visualize the target (green), direct line (gray), planned heading
        # (yellow), and forward lookahead probe (cyan).
        from .framework import debugdraw
        debugdraw.point(tx, ty, rgb=(0.0, 1.0, 0.0), scale=0.15, ns="target")
        debugdraw.line([(pose.x, pose.y), (tx, ty)], rgb=(0.4, 0.4, 0.4), ns="to_target")
        if planned_path is not None and len(planned_path) >= 2:
            debugdraw.line(planned_path, rgb=(0.2, 0.8, 1.0), ns="global_path")
        if waypoint is not None:
            debugdraw.point(
                waypoint[0], waypoint[1],
                rgb=(0.2, 0.8, 1.0), scale=0.12, ns="global_waypoint",
            )
        debugdraw.arrow(
            pose.x, pose.y,
            pose.x + math.cos(heading) * 0.6, pose.y + math.sin(heading) * 0.6,
            rgb=(1.0, 1.0, 0.0), ns="heading",
        )
        debugdraw.line(
            [(pose.x, pose.y),
             (pose.x + math.cos(heading) * PLAN_LOOKAHEAD,
              pose.y + math.sin(heading) * PLAN_LOOKAHEAD)],
            rgb=(0.0, 0.8, 0.8), ns="lookahead",
        )

        if distance <= OMNI_DIST:
            # Nearby: translate along heading while turning toward face.
            wdx, wdy = math.cos(heading) * distance, math.sin(heading) * distance
            cos_t, sin_t = math.cos(pose.theta), math.sin(pose.theta)
            vx = LINEAR_GAIN * (wdx * cos_t + wdy * sin_t)
            vy = LINEAR_GAIN * (-wdx * sin_t + wdy * cos_t)
            speed = math.hypot(vx, vy)
            if speed > MAX_LINEAR:
                vx *= MAX_LINEAR / speed
                vy *= MAX_LINEAR / speed
            vyaw = (
                self._angular(normalize_angle(face - pose.theta))
                if face is not None else 0.0
            )
            self.set_velocity(vx, vy, vyaw)
        else:
            # Distant: turn, walk, and turn along heading.
            angle_err = normalize_angle(heading - pose.theta)
            if abs(angle_err) > TURN_THRESHOLD:
                self.set_velocity(0.0, 0.0, self._angular(angle_err))
            else:
                vx = clamp(
                    LINEAR_GAIN * distance * math.cos(angle_err), 0.0, MAX_LINEAR,
                )
                self.set_velocity(vx, 0.0, self._angular(angle_err))
        return False

    def _path_waypoint(
        self, pose: Pose2D, path: list[tuple[float, float]],
    ) -> tuple[float, float]:
        """Pick a short lookahead waypoint from a planned global path."""
        if not path:
            return (pose.x, pose.y)
        prev = (pose.x, pose.y)
        points = path[1:] if len(path) > 1 else path
        for point in points:
            seg_len = dist(prev[0], prev[1], point[0], point[1])
            if seg_len >= GLOBAL_PATH_LOOKAHEAD_M:
                ratio = GLOBAL_PATH_LOOKAHEAD_M / max(seg_len, 1e-6)
                return (
                    prev[0] + (point[0] - prev[0]) * ratio,
                    prev[1] + (point[1] - prev[1]) * ratio,
                )
            prev = point
        return path[-1]

    def _plan_heading(
        self, pose: Pose2D, goal_dir: float, obstacles: list,
    ) -> float:
        """Choose the clearest candidate heading nearest the target direction.

        Candidates are tested by increasing absolute offset from the target.
        Player ID parity chooses which side is tried first, breaking symmetry
        when two players avoid each other.
        """
        sign_first = 1.0 if self.id % 2 == 0 else -1.0
        best_h = goal_dir
        best_clear = -math.inf

        offsets = [0.0]
        k = 1
        while k * PLAN_STEP <= PLAN_MAX_OFFSET + 1e-9:
            offsets.append(sign_first * k * PLAN_STEP)
            offsets.append(-sign_first * k * PLAN_STEP)
            k += 1

        for off in offsets:
            h = goal_dir + off
            clear = _heading_clearance(pose.x, pose.y, h, obstacles)
            if clear >= PLAN_CLEARANCE:
                return h
            if clear > best_clear:
                best_clear, best_h = clear, h
        return best_h

    # ------------------------------------------------------------------
    # High-level actions called directly by the strategy in main.py
    # ------------------------------------------------------------------


    def block_path_projection(
        self, opponent_id: int,
    ) -> tuple[float, float, float, float] | None:
        """Project this player onto an opponent-to-ball segment.

        Return ``(x, y, perpendicular_distance, segment_parameter_t)``.
        """
        ctx = self.context
        pose = self.pose
        ball = ctx.ball if ctx is not None else None
        opponent = ctx.opponents.get(opponent_id) if ctx is not None else None
        if ctx is None or pose is None or ball is None or opponent is None:
            return None
        opponent_pose = opponent.pose
        if opponent_pose is None:
            return None

        ax, ay = opponent_pose.x, opponent_pose.y
        bx, by = ball.x, ball.y
        vx, vy = bx - ax, by - ay
        length2 = vx * vx + vy * vy
        if length2 < 1e-6:
            return None

        raw_t = ((pose.x - ax) * vx + (pose.y - ay) * vy) / length2
        t = clamp(raw_t, 0.0, 1.0)
        tx = ax + vx * t
        ty = ay + vy * t
        return tx, ty, dist(pose.x, pose.y, tx, ty), raw_t

    def attack(self, kick_target: tuple[float, float] | None = None) -> None:
        """Chase the ball and shoot toward ``kick_target`` or the opponent goal.

        Kick hysteresis enters below ENTER and exits above EXIT, where EXIT is
        greater than ENTER, to prevent threshold oscillation. Normal challenges
        approach the ball directly without avoidance. The chase target lies
        ``CHASE_BEHIND_M`` behind the ball on the ball-to-goal line, naturally
        aligning the player for a shot on arrival.
        """
        ctx = self.context
        pose = self.pose
        ball = ctx.ball if ctx is not None else None
        if ctx is None or ball is None or pose is None:
            self.stop()
            return
        if kick_target is None:
            kick_target = opponent_goal(ctx)

        d = dist(pose.x, pose.y, ball.x, ball.y)
        self._kicking = d <= (KICK_EXIT_M if self._kicking else KICK_ENTER_M)
        if self._kicking:
            kick_plan = self.plan_kick()
            if kick_plan is None:
                self.stop()
                return
            kick_direction, kick_power = kick_plan
            self.kick(kick_direction, kick_power)
        else:
            self.release_kick()
            self.walk_to(
                _behind_ball(ball.x, ball.y, kick_target, CHASE_BEHIND_M)
            )

    def guard(self) -> None:
        """Guard with active interception when the ball enters our half.

        When the ball is far away, stay at the goal-area center.  When it
        enters our half (x < GUARD_THREAT_ENTER_X), move forward along the
        ball-to-goal line to actively intercept shots.

        Hysteresis: enter active mode at ENTER_X, exit at EXIT_X to prevent
        rapid switching near the boundary.
        """
        ctx = self.context
        pose = self.pose
        if ctx is None or pose is None:
            self.action = "guard:stop"
            self.stop()
            return
        home = own_goal_area_center(ctx)

        ball = ctx.ball

        # Active interception hysteresis.
        if ball is not None:
            if ball.x >= GUARD_THREAT_EXIT_X:
                self._guard_threatened = False
            elif ball.x < GUARD_THREAT_ENTER_X:
                self._guard_threatened = True

        face = 0.0
        if GUARD_FACE_BALL and ball is not None:
            face = angle_to(pose.x, pose.y, ball.x, ball.y)

        if self._guard_threatened and ball is not None:
            # Move forward along the ball-to-goal line to intercept.
            gx, gy = own_goal(ctx)
            bx, by = ball.x, ball.y
            intercept_x = gx + (bx - gx) * GUARD_INTERCEPT_FRACTION
            intercept_y = gy + (by - gy) * GUARD_INTERCEPT_FRACTION
            # Ensure a minimum distance from our goal line.
            min_x = -ctx.field.length / 2.0 + GUARD_INTERCEPT_MIN_DIST_M
            if intercept_x < min_x:
                intercept_x = min_x
            target = clamp_inside_field(ctx, intercept_x, intercept_y, margin=0.3)
            self.action = "guard:intercept"
            debugdraw.point(
                target[0], target[1], rgb=(1.0, 0.4, 0.0), scale=0.2, ns="guard_intercept",
            )
            self.walk_to(target, face=face, avoid_ball=True, avoid_robots=True)
        else:
            self.action = "guard:home"
            debugdraw.point(
                home[0], home[1], rgb=(0.0, 0.6, 1.0), scale=0.2, ns="guard_home",
            )
            self.walk_to(home, face=face, avoid_ball=True, avoid_robots=True)

    def support(self) -> None:
        """Support: offer a forward pass option when attacking, cover the
        ball-to-own-goal line when defending.

        Attacking (ball deep in the opponent half): stand ahead-lateral of
        the ball so the attacker always has a forward receiver with a clear
        lane. Defending (ball in our half): keep the blocking line between
        the ball and our goal.
        """
        ctx = self.context
        ball = ctx.ball if ctx is not None else None
        if ctx is None or ball is None:
            self.stop()
            return

        if ball.x >= SUPPORT_ATTACK_BALL_X:
            side = -1.0 if ball.y > 0.0 else 1.0  # Offer on the center side.
            target = clamp_inside_field(
                ctx,
                ball.x + SUPPORT_AHEAD_M,
                ball.y + side * SUPPORT_LATERAL_M,
                margin=0.35,
            )
            self.action = "support:attack"
            self.move_to_position(target)
            return

        gx, gy = own_goal(ctx)
        bx, by = (ball.x, ball.y)
        dx, dy = gx - bx, gy - by
        d = math.hypot(dx, dy)
        if d < 1e-6:
            ux, uy = -1.0, 0.0
        else:
            ux, uy = dx / d, dy / d
        along = min(SUPPORT_DIST_M, d)
        tx = bx + ux * along
        ty = by + uy * along

        # Stay 0.3 m inside our goal line and clamp laterally inside the field.
        half_l = ctx.field.length / 2.0
        half_w = ctx.field.width / 2.0 - 0.3
        tx = clamp(tx, -half_l + 0.3, half_l)
        ty = clamp(ty, -half_w, half_w)
        self.move_to_position((tx, ty))

    def kickoff_push(self) -> None:
        """First kickoff touch: push the ball out of the kickoff zone.

        Two-touch kickoff rule: a goal scored directly from the kickoff spot
        does not count. This skill nudges the ball forward along the smallest
        offset from the opponent-goal direction whose straight line cannot
        enter the goal, so the follow-up shot happens outside the kickoff
        zone and counts normally.
        """
        ctx = self.context
        ball = ctx.ball if ctx is not None else None
        if ctx is None or ball is None or self.pose is None:
            self.stop()
            return
        # Fallback: choose the widest non-scoring angle, i.e. the one whose
        # projection at the goal is farthest from the goal center.  When every
        # scanned angle can score (unlikely from the center spot with small
        # offsets), default to the side that keeps the ball farther from the
        # goal center.
        best_direction = None
        best_offset = 0.0
        offsets = (
            sign * step * KICKOFF_PUSH_SCAN_STEP_RAD
            for step in range(1, KICKOFF_PUSH_SCAN_STEPS + 1)
            for sign in (1.0, -1.0)
        )
        for candidate in offsets:
            if not self.kick_can_score(candidate):
                if best_direction is None:
                    best_direction = candidate
                break  # Take the first non-scoring candidate by increasing offset.
            # Track the candidate whose straight line ends farthest from goal center.
            goal_y = self._goal_target_for_direction(candidate)[1]
            if abs(goal_y) > abs(best_offset):
                best_offset = goal_y
                best_direction = candidate
        if best_direction is None:
            best_direction = math.radians(90.0)
        self.kick(best_direction, KICK_POWER_KICKOFF_PUSH)

    def take_kickoff(self, kick_target: tuple[float, float] | None = None) -> None:
        """Stage behind the ball for our restart, then approach and kick."""
        ctx = self.context
        pose = self.pose
        ball = ctx.ball if ctx is not None else None
        if ctx is None or ball is None or pose is None:
            self.stop()
            return
        if kick_target is None:
            kick_target = opponent_goal(ctx)
        kick_dir = angle_to(ball.x, ball.y, *kick_target)
        cos_k, sin_k = math.cos(kick_dir), math.sin(kick_dir)
        rel_x, rel_y = pose.x - ball.x, pose.y - ball.y
        behind = rel_x * cos_k + rel_y * sin_k          # Negative is our side.
        lateral = abs(-rel_x * sin_k + rel_y * cos_k)
        if behind > KICKOFF_FRONT_MARGIN or lateral > KICKOFF_LATERAL_TOL:
            stage = (
                ball.x - cos_k * KICKOFF_STAGE_M,
                ball.y - sin_k * KICKOFF_STAGE_M,
            )
            self.release_kick()
            self.walk_to(stage, face=kick_dir, avoid_ball=True)
        else:
            self.attack(kick_target)

    def move_to_position(self, target: tuple[float, float] | None) -> None:
        """Move to a support/defense position while facing and avoiding the ball."""
        if target is None:
            self.stop()
            return
        face = None
        ctx = self.context
        pose = self.pose
        ball = ctx.ball if ctx is not None else None
        if ball is not None and pose is not None:
            face = angle_to(pose.x, pose.y, ball.x, ball.y)
        self.release_kick()
        self.walk_to(target, face=face, avoid_ball=True, avoid_robots=True)

    @staticmethod
    def _angular(err: float) -> float:
        return clamp(ANGULAR_GAIN * err, -MAX_ANGULAR, MAX_ANGULAR)
