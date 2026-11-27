"""Goalkeeper positioning and ball trajectory prediction."""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..soccer_framework import BallState, Pose2D, SoccerConfig
from .geometry import TeamFieldFrame, clamp, field_to_relative, normalize_angle


__all__ = [
    "KEEPER_MODE_EMERGENCY",
    "KEEPER_MODE_INTERCEPT",
    "KEEPER_MODE_POSITION",
    "KEEPER_MODE_SWEEP",
    "BallVelocityTracker",
    "KeeperController",
    "KeeperDecision",
]


KEEPER_MODE_POSITION = "POSITION"
KEEPER_MODE_INTERCEPT = "INTERCEPT"
KEEPER_MODE_EMERGENCY = "EMERGENCY"
KEEPER_MODE_SWEEP = "SWEEP"


@dataclass(frozen=True)
class KeeperDecision:
    """Guard position and permission to clear the ball for this frame."""

    mode: str
    guard_x: float
    lateral_y: float
    wants_clear: bool

    @property
    def claims_ball(self) -> bool:
        return self.mode == KEEPER_MODE_SWEEP


class BallVelocityTracker:
    """Estimate velocity from two observations, ignoring stale samples and jumps."""

    def __init__(
        self,
        max_speed_mps: float | None = None,
        max_age_sec: float = math.inf,
        max_step_m: float = math.inf,
    ) -> None:
        self._max_speed = max_speed_mps
        self._max_age = max_age_sec
        self._max_step = max_step_m
        self._prev: tuple[float, float, float] | None = None  # (x, y, t)
        self._velocity: tuple[float, float] | None = None

    def update(self, ball: BallState, now: float) -> tuple[float, float] | None:
        velocity: tuple[float, float] | None = None
        if self._prev is not None:
            prev_x, prev_y, prev_time = self._prev
            elapsed = now - prev_time
            if elapsed == 0.0:
                return self._velocity
            dx, dy = ball.x - prev_x, ball.y - prev_y
            if 0.0 < elapsed <= self._max_age and math.hypot(dx, dy) <= self._max_step:
                vx = dx / elapsed
                vy = dy / elapsed
                if self._max_speed is None or math.hypot(vx, vy) <= self._max_speed:
                    velocity = (vx, vy)
        self._prev = (ball.x, ball.y, now)
        self._velocity = velocity
        return velocity


class KeeperController:
    """Track shots and claim reachable loose balls in front of goal."""

    def __init__(self, config: SoccerConfig) -> None:
        self._config = config
        self._field = TeamFieldFrame(config)
        self._velocity = BallVelocityTracker(
            max_speed_mps=config.strategy.ball_velocity_max_speed_mps,
            max_age_sec=config.strategy.ball_velocity_max_age_sec,
            max_step_m=config.strategy.ball_velocity_max_step_m,
        )
        self._mode = KEEPER_MODE_POSITION
        self._last_guard: tuple[float, float] | None = None

    @property
    def mode(self) -> str:
        return self._mode

    def keeper_line_x(self) -> float:
        return (
            -self._config.field_length / 2.0
            + self._config.goal_area_length / 2.0
        )

    def update(
        self,
        ball: BallState,
        keeper_pose: Pose2D,
        now: float,
        challenger_distance: float = math.inf,
        has_cover: bool = False,
        can_clear: bool = True,
    ) -> KeeperDecision:
        strategy = self._config.strategy
        goal_x = -self._config.field_length / 2.0
        home_x = self.keeper_line_x()
        velocity = self._velocity.update(ball, now)
        ball_speed = math.hypot(*velocity) if velocity is not None else None
        shot_block = self._shot_block(ball, velocity, keeper_pose, goal_x)
        dx, dy = ball.x - keeper_pose.x, ball.y - keeper_pose.y
        keeper_distance = math.hypot(dx, dy)
        in_clear_range = keeper_distance <= strategy.keeper_clear_range_m

        if shot_block is not None:
            self._mode = KEEPER_MODE_EMERGENCY
            self._last_guard = None
            return KeeperDecision(self._mode, shot_block.x, shot_block.y, False)

        lead = (
            strategy.keeper_sweep_hold_lead_m
            if self._mode == KEEPER_MODE_SWEEP
            else strategy.keeper_sweep_lead_m
        )
        clearable_ball = (
            can_clear
            and ball.x >= goal_x
            and (ball_speed is None or ball_speed <= strategy.keeper_sweep_max_ball_speed_mps)
        )
        heading = math.atan2(dy, dx)
        turn = abs(normalize_angle(heading - keeper_pose.theta))
        approach_cost = keeper_distance + strategy.max_linear_speed * turn / max(
            strategy.max_angular_speed, 1e-6,
        )
        recover_at_goal = math.hypot(ball.x - goal_x, ball.y) <= strategy.keeper_uncovered_recovery_m
        can_reach_from_pose = in_clear_range and (
            dx >= 0.0 or (recover_at_goal and turn <= math.pi / 2.0)
        )
        wins_loose_ball = clearable_ball and (
            (can_reach_from_pose and keeper_distance + lead <= challenger_distance)
            or (ball_speed is not None and approach_cost + lead <= challenger_distance)
            or (recover_at_goal and in_clear_range and approach_cost + lead <= challenger_distance)
        )
        if wins_loose_ball and (has_cover or recover_at_goal):
            self._mode = KEEPER_MODE_SWEEP
            self._last_guard = None
            return KeeperDecision(self._mode, ball.x, ball.y, can_reach_from_pose)

        # Simple-framework guard geometry. Hysteresis is based on the ball's
        # field x position, so the keeper starts moving before the ball reaches
        # the penalty area and keeps moving until it is safely back out.
        if self._mode == KEEPER_MODE_POSITION:
            threatened = ball.x < strategy.guard_threat_enter_x
        else:
            threatened = ball.x < strategy.guard_threat_exit_x

        if threatened:
            self._mode = KEEPER_MODE_INTERCEPT
            fraction = strategy.keeper_intercept_fraction
            keeper_x = goal_x + (ball.x - goal_x) * fraction
            keeper_x = max(
                goal_x + strategy.keeper_zone_min_x_offset_m,
                keeper_x,
            )
            y = ball.y * fraction
        else:
            self._mode = KEEPER_MODE_POSITION
            keeper_x = home_x
            y = ball.y * 0.5 if ball.x <= home_x else 0.0

        half_width = strategy.keeper_zone_half_width_m
        y = clamp(y, -half_width, half_width)
        if self._last_guard is not None:
            last_x, last_y = self._last_guard
            if math.hypot(keeper_x - last_x, y - last_y) <= strategy.keeper_track_deadband_m:
                keeper_x, y = last_x, last_y
        self._last_guard = (keeper_x, y)
        return KeeperDecision(self._mode, keeper_x, y, wins_loose_ball and can_reach_from_pose)

    def _shot_block(
        self,
        ball: BallState,
        velocity: tuple[float, float] | None,
        keeper_pose: Pose2D,
        goal_x: float,
    ) -> Pose2D | None:
        strategy = self._config.strategy
        if velocity is None or velocity[0] > -strategy.ball_velocity_min_toward_goal_mps:
            return None
        vx, vy = velocity
        goal_time = (goal_x - ball.x) / vx
        if (
            not 0.0 < goal_time <= strategy.keeper_prediction_horizon_sec
            or abs(ball.y + vy * goal_time) > self._config.goal_width / 2.0
        ):
            return None
        # Include the current depth as a candidate and sample the whole path
        # up to the goal, including shots that already passed the keeper.
        steps = max(1, math.ceil(goal_time / strategy.keeper_intercept_step_sec))
        current_depth_time = (keeper_pose.x - ball.x) / vx
        times = [goal_time * index / steps for index in range(steps + 1)]
        if 0.0 <= current_depth_time <= goal_time:
            times.append(current_depth_time)

        best: tuple[float, float, Pose2D] | None = None
        for time in times:
            target = self._field.clamp_inside_field(
                Pose2D(ball.x + vx * time, ball.y + vy * time),
            )
            relative = field_to_relative(target.x, target.y, keeper_pose)
            # Linear walking and lateral steps occupy separate gait commands.
            travel = (
                abs(relative.x) / max(strategy.max_linear_speed, 1e-6)
                + abs(relative.y) / max(strategy.keeper_max_lateral_speed_mps, 1e-6)
            )
            score = (max(0.0, travel - time), travel)
            if best is None or score < best[:2]:
                best = (*score, target)
        return best[2] if best is not None else None
