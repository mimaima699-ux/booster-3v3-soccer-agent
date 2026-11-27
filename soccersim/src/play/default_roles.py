"""Default dynamic role implementations for SoccerSim.

Each role extends :class:`RoleStrategy` and assembles its subtree through
:meth:`build_subtree`. Methods such as ``target``, ``wants_to_kick``, and
``kick_target`` are implementation helpers for role utility nodes, not
base-class contracts.

Tactical contract of the dedicated-keeper strategy:

- The goalkeeper tracks shots and challenges loose balls when it can get there first.
- The chaser is the single primary ball challenger, with a stuck-recovery
  backoff so it never stands still into an incapable-robot penalty.
- The supporter offers a forward passing lane in ATTACK mode and keeps the
  defensive blocking line otherwise.
- The defender (DEFENSE mode) covers the central lane between the chaser and goal.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import py_trees

from ..soccer_framework import BallState, PlayContext, Pose2D
from ..tactics.goalkeeper import (
    KEEPER_MODE_EMERGENCY,
    KEEPER_MODE_POSITION,
    KeeperController,
    KeeperDecision,
)
from .nodes import (
    AttackSubtreeConfig,
    IsKickWanted,
    KickAction,
    MoveToTarget,
    StrafeToTarget,
    build_attack_subtree,
)
from .playbook import ROLE_GOAL_COVER
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
        return kit.targeting.select_kick_target(
            player_id,
            context,
            kit.is_player_allowed,
        )

    def _approach_reason(self, kit: "SoccerKit", player_id: int) -> str:
        slot = kit.config.ready_slot_for_player(player_id)
        return f"{slot.value} approach ball"

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
                kick_reason_fn=lambda target: kit.targeting.kick_reason(
                    target, default="chaser kick",
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
    """Protect the central lane in defensive mode."""

    name = "defender"

    def target(
        self,
        kit: "SoccerKit",
        player_id: int,
        context: PlayContext,
    ) -> Pose2D:
        return kit.targeting.defender_cover_target(context)

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
        )


# ----------------------------------------------------------------------
# Goalkeeper and cover during a clearance
# ----------------------------------------------------------------------


class GoalCoverRole(RoleStrategy):
    """Fill the goalmouth while the goalkeeper plays a loose ball."""

    name = ROLE_GOAL_COVER

    def build_subtree(
        self, kit: "SoccerKit", player_id: int,
    ) -> py_trees.behaviour.Behaviour:
        return MoveToTarget(
            kit,
            player_id,
            lambda context: kit.ready_stance.goalkeeper_guard_target(context.known_ball),
            reason_fn=lambda: "cover goal for goalkeeper",
        )


class GoalkeeperRole(RoleStrategy):
    """Adjust guard depth, block shots, and sweep loose balls."""

    name = "goalkeeper"

    def __init__(self) -> None:
        super().__init__()
        self._controller: KeeperController | None = None
        self._last_mode: str = KEEPER_MODE_POSITION
        self._last_decision: KeeperDecision | None = None
        self._last_ball: BallState | None = None
        self._last_player_id: int | None = None

    def prepare(
        self,
        kit: "SoccerKit",
        player_id: int,
        context: PlayContext,
    ) -> KeeperDecision:
        ball = context.known_ball
        now = ball.last_seen_at
        if self._controller is None:
            self._controller = KeeperController(kit.config)
        robot = context.teammates.get(player_id)
        keeper_pose = robot.pose if robot is not None else None
        game = context.known_game
        if (
            keeper_pose is None or now <= 0.0
            or not kit.is_player_allowed(game, player_id)
        ):
            decision = KeeperDecision(
                KEEPER_MODE_POSITION, self._controller.keeper_line_x(), 0.0, False,
            )
        else:
            distances = [
                math.hypot(robot.pose.x - ball.x, robot.pose.y - ball.y)
                for mate_id, robot in context.teammates.items()
                if mate_id != player_id and robot.pose is not None
                and kit.is_player_allowed(game, mate_id)
            ]
            opponent_distance = min(
                (
                    math.hypot(robot.pose.x - ball.x, robot.pose.y - ball.y)
                    for robot in context.opponents.values() if robot.pose is not None
                ),
                default=math.inf,
            )
            nearest_mate = min(distances, default=math.inf)
            decision = self._controller.update(
                ball,
                keeper_pose,
                now,
                challenger_distance=min(nearest_mate, opponent_distance),
                has_cover=bool(distances),
                can_clear=not game.has_kicking_team(),
            )
        self._log_mode_change(kit, decision.mode)
        self._last_decision = decision
        self._last_ball = ball
        self._last_player_id = player_id
        return decision

    def _decision(
        self, kit: "SoccerKit", player_id: int, context: PlayContext,
    ) -> KeeperDecision:
        if (
            self._last_decision is not None
            and self._last_ball is context.known_ball
            and self._last_player_id == player_id
        ):
            return self._last_decision
        return self.prepare(kit, player_id, context)

    def _log_mode_change(self, kit: "SoccerKit", mode: str) -> None:
        if mode == self._last_mode:
            return
        self._last_mode = mode
        logger = kit.logger()
        if logger is not None:
            logger.info(
                f"goalkeeper mode -> {mode}",
                event="keeper_mode",
                console=False,
                mode=mode,
            )

    def target(
        self,
        kit: "SoccerKit",
        player_id: int,
        context: PlayContext,
    ) -> Pose2D:
        decision = self._decision(kit, player_id, context)
        if decision.claims_ball:
            ball = context.known_ball
            clear_target = self.kick_target(kit, player_id, context)
            direction = math.atan2(clear_target.y - ball.y, clear_target.x - ball.x)
            target = kit.motion.approach_target(
                ball, direction, kit.config.strategy.keeper_clear_approach_m,
            )
            # At the goal line the legal approach can be field-side of the
            # ball. Face the ball there so the clearance can take over.
            if target.x >= ball.x:
                return Pose2D(
                    target.x, target.y, kit.field.face_ball_theta(target.x, target.y, ball),
                )
            return target
        return Pose2D(
            decision.guard_x,
            decision.lateral_y,
            kit.field.face_ball_theta(
                decision.guard_x, decision.lateral_y, context.known_ball,
            ),
        )

    def wants_to_kick(
        self,
        kit: "SoccerKit",
        player_id: int,
        context: PlayContext,
    ) -> bool:
        return self._decision(kit, player_id, context).wants_clear

    def kick_target(
        self,
        kit: "SoccerKit",
        player_id: int,
        context: PlayContext,
    ) -> Pose2D:
        # Clear forward-diagonal out of the mouth band. The own-goal guard
        # in KickAction rejects anything unsafe on top of this.
        ball = context.known_ball
        side = 1.0 if ball.y >= 0.0 else -1.0
        forward_x = max(0.0, ball.x + kit.config.strategy.keeper_clear_forward_m)
        return Pose2D(
            min(forward_x, kit.field.opponent_goal_x()),
            side * (kit.config.field_width / 2.0 - 0.8),
            0.0,
        )

    def _reason(self) -> str:
        if self._last_decision is not None:
            return f"goalkeeper {self._last_decision.mode.lower()}"
        return "goalkeeper guard"

    def _blocking_shot(self) -> bool:
        return self._last_decision is not None and self._last_decision.mode == KEEPER_MODE_EMERGENCY

    def build_subtree(
        self,
        kit: "SoccerKit",
        player_id: int,
    ) -> py_trees.behaviour.Behaviour:
        return py_trees.composites.Selector(
            name=f"Keeper({player_id})",
            memory=False,
            children=[
                py_trees.composites.Sequence(
                    name=f"KeeperClear({player_id})",
                    memory=False,
                    children=[
                        IsKickWanted(
                            kit, player_id,
                            lambda context: self.wants_to_kick(kit, player_id, context),
                        ),
                        KickAction(
                            kit, player_id,
                            lambda context: self.kick_target(kit, player_id, context),
                            reason_fn=lambda target: "goalkeeper clear",
                        ),
                    ],
                ),
                StrafeToTarget(
                    kit, player_id,
                    lambda context: self.target(kit, player_id, context),
                    reason_fn=self._reason,
                    release_kick_immediately=True,
                    urgent_fn=self._blocking_shot,
                ),
            ],
        )

