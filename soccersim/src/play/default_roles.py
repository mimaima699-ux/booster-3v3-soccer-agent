"""Default dynamic role implementations for SoccerSim.

Each role extends :class:`RoleStrategy` and assembles its subtree through
:meth:`build_subtree`. Methods such as ``target``, ``wants_to_kick``, and
``kick_target`` are implementation helpers for role utility nodes, not
base-class contracts.

Tactical contract of the dedicated-keeper strategy:

- The goalkeeper never leaves its zone and never joins the attack. It clears
  only balls inside its challenge radius, laterally when the ball sits in
  front of the goal mouth.
- The chaser is the single primary ball challenger, with a stuck-recovery
  backoff so it never stands still into an incapable-robot penalty.
- The supporter offers a forward passing lane in ATTACK mode and keeps the
  defensive blocking line otherwise.
- The defender (DEFENSE mode) covers the central lane goal-side of the ball,
  offset off the chaser's pressure lane.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import py_trees

from ..soccer_framework import PlayContext, Pose2D, ReadySlot
from .nodes import AttackSubtreeConfig, MoveToTarget, build_attack_subtree
from .role import RoleStrategy

if TYPE_CHECKING:
    from ..runtime import SoccerKit


# ----------------------------------------------------------------------
# Chaser: primary ball challenger with stuck recovery
# ----------------------------------------------------------------------

# Approach alignment distance behind the ball while chasing, in meters.
_CHASER_APPROACH_OFFSET = 0.4

# Chaser stuck detection is active only while this close to the ball (m).
_CHASER_STUCK_RADIUS_M = 2.5


class _ChaserRecovery:
    """Per-player stuck/backoff state for the chaser role."""

    def __init__(self) -> None:
        self.anchor: tuple[float, float] | None = None
        self.anchor_at: float = 0.0
        self.backoff_until: float = 0.0
        self.cooldown_until: float = 0.0


class ChaserRole(RoleStrategy):
    """Default chaser; primary challenger with a stuck-recovery backoff.

    ``kick_target`` is split by ReadySlot into center shot or side clearance;
    ``target`` normally approaches from behind the ball, but while the player
    is flagged stuck it backs off to a lateral waypoint for
    ``recovery_backoff_sec`` (kicking disabled), then re-arms detection after
    ``recovery_cooldown_sec``. Timing uses ``ball.last_seen_at`` as the
    monotonic per-frame clock.
    """

    name = "chaser"

    def __init__(self) -> None:
        super().__init__()
        self._recovery: dict[int, _ChaserRecovery] = {}

    # -- stuck recovery ------------------------------------------------

    def _backoff_target(
        self,
        kit: "SoccerKit",
        player_id: int,
        context: PlayContext,
    ) -> Pose2D:
        ball = context.known_ball
        side = 1.0 if player_id % 2 == 0 else -1.0
        x = ball.x - 1.4
        y = ball.y + side * 1.2
        return kit.field.clamp_inside_field(
            Pose2D(x, y, kit.field.face_ball_theta(x, y, ball))
        )

    def _recovery_target(
        self,
        kit: "SoccerKit",
        player_id: int,
        context: PlayContext,
    ) -> Pose2D | None:
        """Return a backoff waypoint while the chaser is stuck, else None."""
        ball = context.ball
        robot = context.teammates.get(player_id)
        if ball is None or robot is None or robot.pose is None:
            return None
        now = ball.last_seen_at
        if now <= 0.0:
            return None
        strategy = kit.config.strategy
        state = self._recovery.setdefault(player_id, _ChaserRecovery())

        if now < state.backoff_until:
            return self._backoff_target(kit, player_id, context)
        if state.backoff_until > 0.0:
            # The backoff window just ended: arm the cooldown.
            state.backoff_until = 0.0
            state.cooldown_until = now + strategy.recovery_cooldown_sec
            state.anchor = None
        if now < state.cooldown_until:
            return None

        pose = robot.pose
        if math.hypot(pose.x - ball.x, pose.y - ball.y) > _CHASER_STUCK_RADIUS_M:
            state.anchor = None
            return None
        if (
            state.anchor is None
            or math.hypot(pose.x - state.anchor[0], pose.y - state.anchor[1])
            >= strategy.recovery_stuck_displacement_m
        ):
            state.anchor = (pose.x, pose.y)
            state.anchor_at = now
        elif now - state.anchor_at > strategy.recovery_stuck_sec:
            state.backoff_until = now + strategy.recovery_backoff_sec
            self._log(kit, player_id, "chaser stuck near ball; backing off")
            return self._backoff_target(kit, player_id, context)
        return None

    def _is_backing_off(self, context: PlayContext, player_id: int) -> bool:
        ball = context.ball
        state = self._recovery.get(player_id)
        return (
            ball is not None
            and state is not None
            and ball.last_seen_at < state.backoff_until
        )

    def _log(self, kit: "SoccerKit", player_id: int, message: str) -> None:
        logger = kit.logger()
        if logger is not None:
            logger.info(
                f"p{player_id} {message}",
                event="recovery",
                console=False,
                player=player_id,
            )

    # -- targeting ------------------------------------------------------

    def target(
        self,
        kit: "SoccerKit",
        player_id: int,
        context: PlayContext,
    ) -> Pose2D:
        recovery = self._recovery_target(kit, player_id, context)
        if recovery is not None:
            return recovery
        ball = context.known_ball
        kt = self.kick_target(kit, player_id, context)
        kick_theta = math.atan2(kt.y - ball.y, kt.x - ball.x)
        return kit.motion.approach_target(
            ball,
            kick_theta,
            _CHASER_APPROACH_OFFSET,
        )

    def wants_to_kick(
        self,
        kit: "SoccerKit",
        player_id: int,
        context: PlayContext,
    ) -> bool:
        return not self._is_backing_off(context, player_id)

    def kick_target(
        self,
        kit: "SoccerKit",
        player_id: int,
        context: PlayContext,
    ) -> Pose2D:
        slot = kit.config.ready_slot_for_player(player_id)
        if slot == ReadySlot.SIDE:
            return kit.targeting.select_clear_or_pass_target(
                player_id,
                context,
                kit.is_player_allowed,
            )
        return kit.targeting.select_kick_target(
            player_id,
            context,
            kit.is_player_allowed,
        )

    def _approach_reason(self, kit: "SoccerKit", player_id: int) -> str:
        slot = kit.config.ready_slot_for_player(player_id)
        return f"{slot.value} approach ball"

    def _kick_reason(
        self,
        kit: "SoccerKit",
        player_id: int,
        target: Pose2D,
    ) -> str:
        slot = kit.config.ready_slot_for_player(player_id)
        default = "side clear" if slot == ReadySlot.SIDE else "center kick"
        return kit.targeting.kick_reason(target, default=default)

    def build_subtree(
        self,
        kit: "SoccerKit",
        player_id: int,
    ) -> py_trees.behaviour.Behaviour:
        return build_attack_subtree(
            kit,
            player_id,
            AttackSubtreeConfig(
                target_fn=lambda context: self.target(kit, player_id, context),
                kick_target_fn=lambda context: self.kick_target(
                    kit,
                    player_id,
                    context,
                ),
                wants_kick_fn=lambda context: self.wants_to_kick(
                    kit,
                    player_id,
                    context,
                ),
                reason_fn=lambda: self._approach_reason(kit, player_id),
                kick_reason_fn=lambda target: self._kick_reason(
                    kit,
                    player_id,
                    target,
                ),
            ),
        )


# ----------------------------------------------------------------------
# Supporter: forward pass offer in ATTACK mode, blocking line otherwise
# ----------------------------------------------------------------------


class SupporterRole(RoleStrategy):
    """Support role; forward passing offer in ATTACK mode, defensive line otherwise."""

    name = "supporter"

    def target(
        self,
        kit: "SoccerKit",
        player_id: int,
        context: PlayContext,
    ) -> Pose2D:
        ball = context.ball
        if (
            ball is not None
            and kit.team_mode == "ATTACK"
            and ball.x > 0.2
        ):
            return kit.targeting.attack_support_target(
                player_id,
                context,
                kit.is_player_allowed,
            )
        return kit.targeting.support_target(
            player_id,
            context,
            kit.is_player_allowed,
        )

    def build_subtree(
        self,
        kit: "SoccerKit",
        player_id: int,
    ) -> py_trees.behaviour.Behaviour:
        return MoveToTarget(
            kit,
            player_id,
            lambda context: self.target(kit, player_id, context),
            reason_fn=lambda: "supporter hold",
            hold_vyaw=0.12,
        )


# ----------------------------------------------------------------------
# Defender: central-lane cover for defensive mode
# ----------------------------------------------------------------------


class DefenderRole(RoleStrategy):
    """Defensive-mode cover: goal-side of the ball, off the pressure lane."""

    name = "defender"

    def target(
        self,
        kit: "SoccerKit",
        player_id: int,
        context: PlayContext,
    ) -> Pose2D:
        return kit.targeting.defender_cover_target(
            player_id,
            context,
            kit.is_player_allowed,
        )

    def build_subtree(
        self,
        kit: "SoccerKit",
        player_id: int,
    ) -> py_trees.behaviour.Behaviour:
        return MoveToTarget(
            kit,
            player_id,
            lambda context: self.target(kit, player_id, context),
            reason_fn=lambda: "defender cover",
            hold_vyaw=0.12,
        )


# ----------------------------------------------------------------------
# Goalkeeper: dedicated, zone-locked defensive controller
# ----------------------------------------------------------------------


class GoalkeeperRole(RoleStrategy):
    """Dedicated goalkeeper: zone-locked guard with challenge-radius clears.

    The guard target covers the ball-to-goal-center shooting line inside the
    keeper zone, with a deadband so noisy ball positions never cause
    oscillation. Only balls within ``keeper_challenge_radius_m`` of the guard
    target (and inside the zone's deepest line) may be cleared; when the ball
    sits in front of the goal mouth the clear goes lateral-forward out of the
    danger band instead of across the mouth.
    """

    name = "goalkeeper"

    # Approach alignment distance for goalkeeper challenges (m).
    _APPROACH_OFFSET = 0.22

    def __init__(self) -> None:
        super().__init__()
        self._last_target: Pose2D | None = None

    def guard_target(
        self,
        kit: "SoccerKit",
        context: PlayContext,
    ) -> Pose2D:
        """Deadband-filtered guard target; small corrections hold the last pose."""
        raw = kit.ready_stance.goalkeeper_guard_target(context.ball)
        last = self._last_target
        if (
            last is not None
            and math.hypot(raw.x - last.x, raw.y - last.y)
            <= kit.config.strategy.keeper_track_deadband_m
        ):
            return last
        self._last_target = raw
        return raw

    def target(
        self,
        kit: "SoccerKit",
        context: PlayContext,
    ) -> Pose2D:
        # When a clearable ball is close, target the approach point behind it.
        # Otherwise return to the deadband-filtered guard target.
        if self.wants_to_kick(kit, context):
            ball = context.known_ball
            kt = self.kick_target(kit, context)
            kick_theta = math.atan2(kt.y - ball.y, kt.x - ball.x)
            return kit.motion.approach_target(
                ball,
                kick_theta,
                self._APPROACH_OFFSET,
            )
        return self.guard_target(kit, context)

    def wants_to_kick(
        self,
        kit: "SoccerKit",
        context: PlayContext,
    ) -> bool:
        ball = context.ball
        if ball is None:
            return False
        strategy = kit.config.strategy
        guard = kit.ready_stance.goalkeeper_guard_target(ball)
        if math.hypot(ball.x - guard.x, ball.y - guard.y) > strategy.keeper_challenge_radius_m:
            return False
        # Never advance beyond the zone's deepest line.
        return (
            ball.x
            <= kit.field.own_goal_x() + strategy.keeper_zone_max_x_offset_m + 0.3
        )

    def kick_target(
        self,
        kit: "SoccerKit",
        context: PlayContext,
    ) -> Pose2D:
        ball = context.known_ball
        strategy = kit.config.strategy
        near_mouth = (
            abs(ball.y) < kit.config.goal_width / 2.0 + 0.4
            and ball.x
            <= kit.field.own_goal_x() + strategy.keeper_zone_max_x_offset_m + 0.8
        )
        if near_mouth:
            # Lateral-forward clear: leave the mouth band, never across it.
            side = 1.0 if ball.y >= 0.0 else -1.0
            return Pose2D(
                0.0,
                side * (kit.config.field_width / 2.0 - 0.8),
                0.0,
            )
        if kit.targeting.ball_near_sideline(ball):
            return kit.targeting.sideline_recovery_target(ball)
        return Pose2D(
            kit.field.opponent_goal_x(),
            0.0,
            kit.field.attack_theta(),
        )

    def _guard_reason(self) -> str:
        return "goalkeeper guard"

    def _kick_reason(
        self,
        kit: "SoccerKit",
        target: Pose2D,
    ) -> str:
        return kit.targeting.kick_reason(target, default="goalkeeper clear")

    def build_subtree(
        self,
        kit: "SoccerKit",
        player_id: int,
    ) -> py_trees.behaviour.Behaviour:
        return build_attack_subtree(
            kit,
            player_id,
            AttackSubtreeConfig(
                target_fn=lambda context: self.target(kit, context),
                kick_target_fn=lambda context: self.kick_target(kit, context),
                wants_kick_fn=lambda context: self.wants_to_kick(kit, context),
                reason_fn=self._guard_reason,
                kick_reason_fn=lambda target: self._kick_reason(kit, target),
                hold_vyaw=0.12,
            ),
        )
