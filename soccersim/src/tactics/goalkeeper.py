"""Goalkeeper tactical controller: mode machine, ball velocity, interception.

Pure model layer: no BT, ROS, or runtime imports, so every decision here is
unit-testable on the host. :class:`GoalkeeperRole` in ``src.play`` is a thin
adapter that feeds it the ball and applies its decision.

Design notes:

- The keeper has three modes with separate enter/exit radii (hysteresis):
  POSITION (far ball), INTERCEPT (ball in the defensive vicinity), and
  EMERGENCY (immediate threat; bypasses the tracking deadband).
- The lateral target is where the ball-to-goal-center line crosses the keeper
  line; when a reliable velocity estimate says the ball is closing on our
  goal, the predicted crossing point is used instead.
- Ball velocity is estimated from the two most recent observations and is
  only trusted within ``ball_velocity_min_span_sec`` /
  ``ball_velocity_max_age_sec``; unreliable estimates never block behavior.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..soccer_framework import BallState, Pose2D, SoccerConfig


__all__ = [
    "KEEPER_MODE_CHALLENGE",
    "KEEPER_MODE_EMERGENCY",
    "KEEPER_MODE_INTERCEPT",
    "KEEPER_MODE_POSITION",
    "BallVelocityTracker",
    "KeeperController",
    "KeeperDecision",
]


KEEPER_MODE_POSITION = "POSITION"
KEEPER_MODE_INTERCEPT = "INTERCEPT"
KEEPER_MODE_CHALLENGE = "CHALLENGE"
KEEPER_MODE_EMERGENCY = "EMERGENCY"


@dataclass(frozen=True)
class KeeperDecision:
    """One frame's keeper decision.

    ``lateral_y`` is the target y on the keeper line; ``wants_clear`` allows
    the kick branch; ``challenge`` sends the keeper to attack the ball
    itself (approach target instead of the guard line); ``reason`` explains
    the mode for logging.
    """

    mode: str
    lateral_y: float
    wants_clear: bool
    challenge: bool = False
    reason: str = ""


class BallVelocityTracker:
    """Estimate ball velocity from the two most recent observations.

    Estimates faster than ``max_speed_mps`` are rejected: they come from
    teleported or re-spotted ball positions, not real ball movement, and
    extrapolating them would send the keeper to garbage interception points.
    """

    def __init__(self, max_speed_mps: float | None = None) -> None:
        self._max_speed = max_speed_mps
        self._prev: tuple[float, float, float] | None = None  # (x, y, t)

    def update(self, ball: BallState, now: float) -> tuple[float, float] | None:
        """Return ``(vx, vy)`` from the freshest usable sample pair, else None."""
        velocity: tuple[float, float] | None = None
        if self._prev is not None:
            px, py, pt = self._prev
            dt = now - pt
            if 0.0 < dt:
                candidate = ((ball.x - px) / dt, (ball.y - py) / dt)
                speed = math.hypot(candidate[0], candidate[1])
                if self._max_speed is None or speed <= self._max_speed:
                    velocity = candidate
        self._prev = (ball.x, ball.y, now)
        return velocity


class KeeperController:
    """Stateful per-keeper tactical controller (one instance per goalkeeper)."""

    def __init__(self, config: SoccerConfig):
        self._config = config
        self._velocity = BallVelocityTracker(
            max_speed_mps=config.strategy.ball_velocity_max_speed_mps,
        )
        self._mode: str = KEEPER_MODE_POSITION
        self._last_y: float | None = None

    @property
    def mode(self) -> str:
        return self._mode

    def keeper_line_x(self) -> float:
        strategy = self._config.strategy
        return -self._config.field_length / 2.0 + strategy.keeper_zone_min_x_offset_m

    def update(
        self,
        ball: BallState,
        keeper_pose: Pose2D,
        now: float,
    ) -> KeeperDecision:
        strategy = self._config.strategy
        goal_x = -self._config.field_length / 2.0
        keeper_x = self.keeper_line_x()
        ball_to_goal = math.hypot(ball.x - goal_x, ball.y)
        velocity = self._velocity.update(ball, now)
        crossing_y = self._predicted_crossing_y(ball, velocity, keeper_x)
        fast_threat = self._fast_threat(velocity, crossing_y)

        self._update_mode(ball_to_goal, fast_threat)

        if self._mode == KEEPER_MODE_CHALLENGE:
            # Sweeper-keeper: go win the ball and clear it. The kick branch
            # is always wanted; the attack subtree gates it by kick range.
            y = crossing_y if self._last_y is None else self._last_y
            return KeeperDecision(
                self._mode,
                y,
                wants_clear=True,
                challenge=True,
                reason="slow ball in challenge zone; attack and clear",
            )

        if self._mode == KEEPER_MODE_EMERGENCY:
            # Fast shot: block the predicted crossing point; bypass the
            # deadband so the reaction is immediate. Clear only when the
            # ball is already at the keeper's feet.
            y = crossing_y
            wants_clear = (
                math.hypot(ball.x - keeper_pose.x, ball.y - keeper_pose.y)
                <= strategy.keeper_clear_range_m
            )
            reason = "fast shot; block crossing point"
        else:
            y = crossing_y
            wants_clear = False
            reason = (
                "ball in defensive vicinity"
                if self._mode == KEEPER_MODE_INTERCEPT
                else "ball far; hold guard line"
            )

        half_width = strategy.keeper_zone_half_width_m
        y = max(-half_width, min(half_width, y))

        if self._mode != KEEPER_MODE_EMERGENCY and self._last_y is not None:
            if abs(y - self._last_y) <= strategy.keeper_track_deadband_m:
                y = self._last_y
        self._last_y = y

        return KeeperDecision(
            mode=self._mode,
            lateral_y=y,
            wants_clear=wants_clear,
            reason=reason,
        )

    # -- internals ------------------------------------------------------

    def _update_mode(self, ball_to_goal: float, fast_threat: bool) -> None:
        strategy = self._config.strategy

        if self._mode == KEEPER_MODE_CHALLENGE:
            if fast_threat and ball_to_goal < strategy.keeper_intercept_enter_radius_m:
                self._mode = KEEPER_MODE_EMERGENCY
            elif ball_to_goal > strategy.keeper_challenge_exit_radius_m:
                self._mode = (
                    KEEPER_MODE_INTERCEPT
                    if ball_to_goal < strategy.keeper_intercept_exit_radius_m
                    else KEEPER_MODE_POSITION
                )
        elif self._mode == KEEPER_MODE_EMERGENCY:
            if not fast_threat and ball_to_goal > strategy.keeper_emergency_exit_radius_m:
                if ball_to_goal < strategy.keeper_challenge_exit_radius_m:
                    self._mode = KEEPER_MODE_CHALLENGE
                elif ball_to_goal < strategy.keeper_intercept_exit_radius_m:
                    self._mode = KEEPER_MODE_INTERCEPT
                else:
                    self._mode = KEEPER_MODE_POSITION
            elif ball_to_goal > strategy.keeper_intercept_exit_radius_m:
                self._mode = KEEPER_MODE_POSITION
        elif self._mode == KEEPER_MODE_INTERCEPT:
            if fast_threat and ball_to_goal < strategy.keeper_intercept_enter_radius_m:
                self._mode = KEEPER_MODE_EMERGENCY
            elif not fast_threat and ball_to_goal < strategy.keeper_challenge_enter_radius_m:
                self._mode = KEEPER_MODE_CHALLENGE
            elif ball_to_goal >= strategy.keeper_intercept_exit_radius_m:
                self._mode = KEEPER_MODE_POSITION
        else:  # POSITION
            if fast_threat and ball_to_goal < strategy.keeper_intercept_enter_radius_m:
                self._mode = KEEPER_MODE_EMERGENCY
            elif not fast_threat and ball_to_goal < strategy.keeper_challenge_enter_radius_m:
                self._mode = KEEPER_MODE_CHALLENGE
            elif ball_to_goal < strategy.keeper_intercept_enter_radius_m:
                self._mode = KEEPER_MODE_INTERCEPT

    def _fast_threat(
        self,
        velocity: tuple[float, float] | None,
        crossing_y: float,
    ) -> bool:
        """A fast ball closing on our goal with a crossing inside the mouth band.

        Slow dribbles never count: those are challenge opportunities, and the
        keeper should win the ball instead of retreating to its line.
        """
        if velocity is None:
            return False
        if velocity[0] > -self._config.strategy.ball_fast_closing_mps:
            return False
        return abs(crossing_y) <= self._config.goal_width / 2.0 + 0.3

    def _velocity_closes_on_goal(self, velocity: tuple[float, float] | None) -> bool:
        if velocity is None:
            return False
        strategy = self._config.strategy
        return velocity[0] <= -strategy.ball_velocity_min_toward_goal_mps

    def _predicted_crossing_y(
        self,
        ball: BallState,
        velocity: tuple[float, float] | None,
        keeper_x: float,
    ) -> float:
        """Best y on the keeper line: velocity crossing when reliable, else the
        ball-to-goal-center line crossing."""
        goal_x = -self._config.field_length / 2.0
        if (
            velocity is not None
            and self._velocity_closes_on_goal(velocity)
            and velocity[0] < -1e-6
            and ball.x > keeper_x
        ):
            strategy = self._config.strategy
            t = (keeper_x - ball.x) / velocity[0]
            if 0.0 < t <= strategy.keeper_prediction_horizon_sec:
                return ball.y + velocity[1] * t
        # Fallback: ball -> goal-center line crossing at the keeper line.
        if ball.x > keeper_x + 0.2:
            t = (keeper_x - goal_x) / (ball.x - goal_x)
            return ball.y * t
        return ball.y * 0.5
