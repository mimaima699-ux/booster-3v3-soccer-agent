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

    def test_medium_ball_is_intercept(self) -> None:
        decision = self._step(-2.0)  # ~5.4 m from goal center... closer below
        decision = self._step(0.0)  # 7 m: POSITION first
        decision = self._step(-3.0)  # ~4.0 m: INTERCEPT
        self.assertEqual(decision.mode, KEEPER_MODE_INTERCEPT)

    def test_close_ball_is_emergency_with_hysteresis(self) -> None:
        self._step(0.0)  # POSITION
        decision = self._step(-5.5)  # 1.5 m from goal: EMERGENCY
        self.assertEqual(decision.mode, KEEPER_MODE_EMERGENCY)
        # 2.2 m: inside the 2.8 m exit radius, must stay EMERGENCY.
        decision = self._step(-4.8, t=1.1)
        self.assertEqual(decision.mode, KEEPER_MODE_EMERGENCY)
        # 3.0 m: beyond the exit radius, leaves into INTERCEPT (inside 5.2 m).
        decision = self._step(-4.0, t=1.2)
        self.assertEqual(decision.mode, KEEPER_MODE_INTERCEPT)

    def test_emergency_bypasses_deadband(self) -> None:
        # Establish POSITION with y near 0.
        self.controller.update(
            _ball(3.0, 0.1, 1.0), _keeper(-5.9, 0.0, 0.0), 1.0,
        )
        # Emergency ball to the side: y must follow the raw geometric
        # crossing (teleport-speed velocity estimate is rejected by the
        # sanity clamp), not the deadband-held previous value.
        decision = self.controller.update(
            _ball(-5.5, 1.4, 1.2), _keeper(-5.9, 0.0, 0.0), 1.2,
        )
        self.assertEqual(decision.mode, KEEPER_MODE_EMERGENCY)
        self.assertAlmostEqual(decision.lateral_y, 1.4 * (1.1 / 1.5), places=5)

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
        # geometric ball-to-goal-center crossing (1.0 * 1.1/2.6 = 0.42).
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
