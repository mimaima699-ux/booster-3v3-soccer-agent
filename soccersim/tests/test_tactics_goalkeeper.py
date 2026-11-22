"""Pure-function tests for the goalkeeper controller and strafe movement.

Covers ``src/tactics/goalkeeper.py`` (velocity tracker, mode hysteresis,
interception point, emergency deadband bypass) and
``MotionController.strafe_command`` (lateral vy movement, face-up turn,
walk fallback, lateral arrival).
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from src.soccer_framework import (  # noqa: E402
    BallState,
    MoveIntent,
    Pose2D,
    RobotState,
    SoccerConfig,
    PlayContext,
)
from src.soccer_framework.types import StopIntent  # noqa: E402
from src.tactics.geometry import TeamFieldFrame  # noqa: E402
from src.tactics.goalkeeper import (  # noqa: E402
    KEEPER_MODE_CHALLENGE,
    KEEPER_MODE_EMERGENCY,
    KEEPER_MODE_INTERCEPT,
    KEEPER_MODE_POSITION,
    BallVelocityTracker,
    KeeperController,
)
from src.tactics.kick_hysteresis import KickHysteresis  # noqa: E402
from src.tactics.motion import MotionController  # noqa: E402
from src.tactics.navigation import ObstacleCollector  # noqa: E402


def _ball(x: float, y: float, t: float) -> BallState:
    return BallState(x=x, y=y, last_seen_at=t)


def _keeper(x: float, y: float, theta: float) -> Pose2D:
    return Pose2D(x, y, theta)


class BallVelocityTrackerTests(unittest.TestCase):
    def test_estimates_velocity_from_pair(self) -> None:
        tracker = BallVelocityTracker()
        self.assertIsNone(tracker.update(_ball(0.0, 0.0, 1.0), 1.0))
        velocity = tracker.update(_ball(1.0, 0.5, 1.5), 1.5)
        self.assertIsNotNone(velocity)
        assert velocity is not None  # noqa: S101 - narrowed for the type checker
        self.assertAlmostEqual(velocity[0], 2.0)
        self.assertAlmostEqual(velocity[1], 1.0)

    def test_same_timestamp_yields_none(self) -> None:
        tracker = BallVelocityTracker()
        tracker.update(_ball(0.0, 0.0, 1.0), 1.0)
        self.assertIsNone(tracker.update(_ball(1.0, 1.0, 1.0), 1.0))


class KeeperModeTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = SoccerConfig()
        self.controller = KeeperController(self.config)

    def _step(self, x: float, y: float = 0.0, t: float = 1.0):
        return self.controller.update(
            _ball(x, y, t),
            _keeper(-5.9, 0.0, 0.0),
            t,
        )

    def test_far_ball_is_position(self) -> None:
        decision = self._step(3.0)
        self.assertEqual(decision.mode, KEEPER_MODE_POSITION)
        self.assertFalse(decision.wants_clear)

    def test_guard_point_dynamic_depth(self) -> None:
        # Simple-framework geometry: the guard point sits at
        # keeper_intercept_fraction along the ball-to-goal-center line,
        # clamped to the [min, max] depth band.
        strategy = self.config.strategy
        goal_x = -self.config.field_length / 2.0
        # Ball at (-1, 2): raw point = goal + 0.4 * (ball - goal),
        # inside the band -> unclamped fraction value.
        decision = self._step(-1.0, y=2.0)
        self.assertAlmostEqual(
            decision.guard_x, goal_x + 6.0 * strategy.keeper_intercept_fraction, places=6,
        )
        self.assertAlmostEqual(
            decision.lateral_y, 2.0 * strategy.keeper_intercept_fraction, places=6,
        )
        # Ball near the goal but outside the challenge radius: raw depth
        # below min -> clamped to the shallowest allowed depth.
        decision = self._step(-4.5, y=2.0, t=1.1)  # ~3.2 m from goal center
        min_x = goal_x + strategy.keeper_zone_min_x_offset_m
        self.assertEqual(decision.mode, KEEPER_MODE_INTERCEPT)
        self.assertAlmostEqual(decision.guard_x, min_x, places=6)

    def test_medium_ball_is_intercept(self) -> None:
        decision = self._step(0.0)  # 7 m: POSITION first
        decision = self._step(-3.0)  # ~4.0 m from goal: INTERCEPT
        self.assertEqual(decision.mode, KEEPER_MODE_INTERCEPT)

    def test_close_slow_ball_challenges(self) -> None:
        self._step(0.0, t=1.0)  # POSITION
        # Ball 2.0 m from goal, appearing without a usable velocity estimate
        # (the teleport-speed pair is rejected by the sanity clamp).
        decision = self._step(-5.0, t=1.1)
        self.assertEqual(decision.mode, KEEPER_MODE_CHALLENGE)
        self.assertTrue(decision.challenge)
        self.assertTrue(decision.wants_clear)

    def test_challenge_hysteresis(self) -> None:
        self._step(0.0, t=1.0)
        self._step(-5.0, t=1.1)  # 2.0 m: CHALLENGE
        decision = self._step(-4.4, t=1.2)  # 2.6 m: inside 3.6 exit
        self.assertEqual(decision.mode, KEEPER_MODE_CHALLENGE)
        decision = self._step(-3.8, t=1.3)  # 3.2 m: still inside
        self.assertEqual(decision.mode, KEEPER_MODE_CHALLENGE)
        decision = self._step(-3.2, t=1.4)  # 3.8 m: exits to INTERCEPT
        self.assertEqual(decision.mode, KEEPER_MODE_INTERCEPT)

    def test_fast_shot_is_emergency_not_challenge(self) -> None:
        # Prime in INTERCEPT, then a 2 m/s closing shot crossing the mouth.
        self.controller.update(
            _ball(-3.0, 0.5, 1.0), _keeper(-5.9, 0.0, 0.0), 1.0,
        )
        decision = self.controller.update(
            _ball(-3.4, 0.5, 1.2),  # vx = -2 m/s toward our goal
            _keeper(-5.9, 0.0, 0.0),
            1.2,
        )
        self.assertEqual(decision.mode, KEEPER_MODE_EMERGENCY)
        self.assertFalse(decision.challenge)
        # Ball 2.5 m from the keeper: block, do not clear yet.
        self.assertFalse(decision.wants_clear)

    def test_emergency_bypasses_deadband(self) -> None:
        # Establish a small guard y, then a fast shot whose crossing is far
        # from the held value: the deadband must be bypassed.
        self.controller.update(
            _ball(-4.0, 0.5, 1.0), _keeper(-5.9, 0.0, 0.0), 1.0,
        )
        decision = self.controller.update(
            _ball(-4.6, 0.5, 1.2),  # vx = -3 m/s, crossing y = 0.5
            _keeper(-5.9, 0.0, 0.0),
            1.2,
        )
        self.assertEqual(decision.mode, KEEPER_MODE_EMERGENCY)
        self.assertAlmostEqual(decision.lateral_y, 0.5, places=5)

    def test_velocity_crossing_used_when_closing(self) -> None:
        controller = KeeperController(self.config)
        # Prime the tracker, then a ball closing at 2 m/s along y = 1.0.
        controller.update(_ball(-4.0, 1.0, 1.0), _keeper(-5.9, 0.0, 0.0), 1.0)
        decision = controller.update(
            _ball(-4.4, 1.0, 1.2),  # vx = -2 m/s, vy = 0
            _keeper(-5.9, 0.0, 0.0),
            1.2,
        )
        self.assertEqual(decision.mode, KEEPER_MODE_EMERGENCY)
        # Velocity crossing at the keeper line: t = 1.5/2 = 0.75 s (within the
        # 1.2 s horizon), so y follows the ball line (1.0) instead of the
        # geometric ball-to-goal-center crossing (1.0 * 1.1/3.0 = 0.37).
        self.assertAlmostEqual(decision.lateral_y, 1.0, places=5)


class StrafeCommandTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = SoccerConfig()
        self.field = TeamFieldFrame(self.config)
        kicker = KickHysteresis(enter=2.5, exit=3.0, exit_delay=1.5)
        self.motion = MotionController(
            self.config, self.field, kicker, ObstacleCollector(self.config, self.field),
        )

    def _context(self, pose: Pose2D) -> PlayContext:
        return PlayContext(
            teammates={3: RobotState(player_id=3, pose=pose, last_seen_at=1.0)},
            ball=_ball(2.0, 0.0, 1.0),
        )

    def test_lateral_command_while_facing_ball(self) -> None:
        pose = _keeper(-5.9, 0.0, 0.0)
        target = Pose2D(-5.9, 0.6, 0.0)
        command = self.motion.strafe_command(3, self._context(pose), target, "keeper line")
        intent = command.intent
        self.assertIsInstance(intent, MoveIntent)
        assert isinstance(intent, MoveIntent)  # noqa: S101
        self.assertAlmostEqual(intent.vx, 0.0)
        self.assertAlmostEqual(intent.vyaw, 0.0)
        self.assertGreater(intent.vy, 0.0)
        self.assertLessEqual(intent.vy, self.config.strategy.keeper_max_lateral_speed_mps)

    def test_big_heading_error_pure_turn(self) -> None:
        pose = _keeper(-5.9, 0.0, 1.2)  # Facing far off the target heading.
        target = Pose2D(-5.9, 0.3, 0.0)
        command = self.motion.strafe_command(3, self._context(pose), target, "keeper line")
        intent = command.intent
        self.assertIsInstance(intent, MoveIntent)
        assert isinstance(intent, MoveIntent)  # noqa: S101
        self.assertAlmostEqual(intent.vy, 0.0)
        self.assertNotAlmostEqual(intent.vyaw, 0.0)

    def test_small_lateral_error_stops(self) -> None:
        pose = _keeper(-5.9, 0.0, 0.0)
        target = Pose2D(-5.9, 0.05, 0.0)
        command = self.motion.strafe_command(3, self._context(pose), target, "keeper line")
        self.assertIsInstance(command.intent, StopIntent)

    def test_forward_offset_falls_back_to_walk(self) -> None:
        pose = _keeper(-5.9, 0.0, 0.0)
        target = Pose2D(-5.0, 0.3, 0.0)  # 0.9 m forward: beyond strafe scope.
        command = self.motion.strafe_command(3, self._context(pose), target, "keeper line")
        intent = command.intent
        self.assertIsInstance(intent, MoveIntent)
        assert isinstance(intent, MoveIntent)  # noqa: S101
        self.assertGreater(intent.vx, 0.0)  # Walking controller took over.


if __name__ == "__main__":
    unittest.main()
