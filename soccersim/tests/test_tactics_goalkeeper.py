"""Pure-function tests for the goalkeeper controller and strafe movement.

Covers ``src/tactics/goalkeeper.py`` (velocity tracker, mode hysteresis,
interception point, emergency deadband bypass) and
``MotionController.strafe_command`` (lateral vy movement, face-up turn,
walk fallback, lateral arrival).
"""

from __future__ import annotations

import math
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
    KEEPER_MODE_SWEEP,
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

    def test_guard_point_advances_into_own_half_and_retreats_near_goal(self) -> None:
        goal_x = -self.config.field_length / 2.0
        away = self._step(2.0)
        midfield = self._step(-3.0, y=2.0, t=2.0)
        near = self._step(-5.5, y=0.5, t=3.0)
        self.assertAlmostEqual(away.guard_x, self.controller.keeper_line_x())
        self.assertGreater(midfield.guard_x, near.guard_x)
        self.assertLessEqual(
            midfield.guard_x, goal_x + self.config.strategy.keeper_zone_max_x_offset_m,
        )
        self.assertAlmostEqual(midfield.lateral_y, 2.0 * (midfield.guard_x - goal_x) / 4.0)

    def test_keeper_claims_a_slow_ball_outside_clear_range_with_cover(self) -> None:
        pose = _keeper(-5.9, 0.0, 0.0)
        self.controller.update(_ball(-4.3, 0.0, 1.0), pose, 1.0, 2.2, has_cover=True)
        decision = self.controller.update(
            _ball(-4.3, 0.0, 1.2), pose, 1.2, 2.2, has_cover=True,
        )
        self.assertEqual(decision.mode, KEEPER_MODE_SWEEP)
        self.assertFalse(decision.wants_clear)
        self.assertAlmostEqual(decision.guard_x, -4.3)

        pressured = self.controller.update(
            _ball(-4.3, 0.0, 1.3), pose, 1.3, 1.5, has_cover=True,
        )
        self.assertFalse(pressured.claims_ball)

    def test_fast_shot_blocks_at_current_depth_when_keeper_has_advanced(self) -> None:
        pose = _keeper(-5.0, 0.0, 0.0)
        self.controller.update(_ball(-3.4, 0.6, 1.0), pose, 1.0)
        decision = self.controller.update(_ball(-3.8, 0.6, 1.2), pose, 1.2)
        self.assertEqual(decision.mode, KEEPER_MODE_EMERGENCY)
        self.assertAlmostEqual(decision.guard_x, pose.x)
        self.assertAlmostEqual(decision.lateral_y, 0.6)

    def test_slow_ball_must_be_within_reach_and_in_front_to_clear(self) -> None:
        too_far = self._step(-4.2)
        in_reach = self._step(-5.0, t=2.0)
        behind = self._step(-6.2, t=3.0)
        self.assertFalse(too_far.wants_clear)
        self.assertTrue(in_reach.wants_clear)
        self.assertFalse(behind.wants_clear)
        self.assertTrue(behind.claims_ball)

    def test_keeper_claims_wide_loose_ball_when_team_can_cover(self) -> None:
        pose = _keeper(-5.1, 1.5, 0.4)
        self.controller.update(_ball(-3.4, 2.4, 1.0), pose, 1.0)
        decision = self.controller.update(
            _ball(-3.4, 2.4, 1.2), pose, 1.2, challenger_distance=3.1, has_cover=True,
        )
        self.assertTrue(decision.claims_ball)
        self.assertAlmostEqual(decision.lateral_y, 2.4)

    def test_fast_shot_already_behind_keeper_stays_an_emergency(self) -> None:
        pose = _keeper(-5.0, 0.0, 0.0)
        self.controller.update(_ball(-5.0, 0.35, 1.0), pose, 1.0)
        decision = self.controller.update(_ball(-6.0, 0.35, 1.1), pose, 1.1)
        self.assertEqual(decision.mode, KEEPER_MODE_EMERGENCY)
        self.assertLessEqual(decision.guard_x, -6.0)
        self.assertAlmostEqual(decision.lateral_y, 0.35)

    def test_slow_rolling_shot_is_tracked(self) -> None:
        pose = _keeper(-5.8, 0.0, 0.0)
        self.controller.update(_ball(-4.9, -0.65, 1.0), pose, 1.0)
        self.controller.update(_ball(-5.0, -0.65, 1.2), pose, 1.2)
        # On the next frame the ball is within the prediction horizon.
        decision = self.controller.update(_ball(-5.3, -0.65, 1.6), pose, 1.6)
        self.assertEqual(decision.mode, KEEPER_MODE_EMERGENCY)
        self.assertAlmostEqual(decision.lateral_y, -0.65)

    def test_fast_ball_at_our_feet_still_uses_the_blocking_branch(self) -> None:
        self._step(-4.8)
        decision = self._step(-5.3, t=1.2)
        self.assertEqual(decision.mode, KEEPER_MODE_EMERGENCY)
        self.assertFalse(decision.wants_clear)

    def test_shot_that_crosses_keeper_line_but_misses_goal_is_not_a_threat(self) -> None:
        self._step(-4.4, y=0.4)
        decision = self._step(-4.8, y=0.6, t=1.2)
        self.assertEqual(decision.mode, KEEPER_MODE_INTERCEPT)

    def test_fast_shot_is_emergency_not_challenge(self) -> None:
        # Prime in INTERCEPT, then a 2 m/s closing shot crossing the mouth.
        self.controller.update(
            _ball(-3.2, 0.5, 1.0), _keeper(-5.9, 0.0, 0.0), 1.0,
        )
        decision = self.controller.update(
            _ball(-3.6, 0.5, 1.2),  # vx = -2 m/s toward our goal
            _keeper(-5.9, 0.0, 0.0),
            1.2,
        )
        self.assertEqual(decision.mode, KEEPER_MODE_EMERGENCY)
        self.assertAlmostEqual(decision.guard_x, -5.9)
        # Ball 2.3 m from the keeper: block, do not clear yet.
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

    def test_corrects_position_before_turning(self) -> None:
        pose = _keeper(-5.9, 0.0, 1.2)  # Facing far off the target heading.
        target = Pose2D(-5.9, 0.3, 0.0)
        command = self.motion.strafe_command(3, self._context(pose), target, "keeper line")
        intent = command.intent
        self.assertIsInstance(intent, MoveIntent)
        assert isinstance(intent, MoveIntent)  # noqa: S101
        self.assertAlmostEqual(intent.vy, 0.0)
        self.assertEqual(intent.vyaw, 0.0)
        self.assertGreater(intent.vx, 0.0)

    def test_turns_to_face_ball_at_target(self) -> None:
        pose = _keeper(-5.9, 0.3, 1.2)
        target = Pose2D(-5.9, 0.3, 0.0)
        intent = self.motion.strafe_command(3, self._context(pose), target, "keeper line").intent
        self.assertIsInstance(intent, MoveIntent)
        assert isinstance(intent, MoveIntent)  # noqa: S101
        self.assertEqual((intent.vx, intent.vy), (0.0, 0.0))
        self.assertLess(intent.vyaw, 0.0)

    def test_small_lateral_error_stops(self) -> None:
        pose = _keeper(-5.9, 0.0, 0.0)
        target = Pose2D(-5.9, 0.05, 0.0)
        command = self.motion.strafe_command(3, self._context(pose), target, "keeper line")
        self.assertIsInstance(command.intent, StopIntent)

    def test_twenty_centimeter_depth_correction_moves(self) -> None:
        pose = _keeper(-5.9, 0.0, 0.0)
        for offset in (-0.20, 0.20):
            with self.subTest(offset=offset):
                target = Pose2D(pose.x + offset, pose.y, pose.theta)
                command = self.motion.strafe_command(3, self._context(pose), target, "keeper line")
                intent = command.intent
                self.assertIsInstance(intent, MoveIntent)
                assert isinstance(intent, MoveIntent)  # noqa: S101
                self.assertGreater(math.copysign(1.0, offset) * intent.vx, 0.0)
                self.assertEqual((intent.vy, intent.vyaw), (0.0, 0.0))

    def test_distant_offset_falls_back_to_walk(self) -> None:
        pose = _keeper(-5.9, 0.0, 0.0)
        target = Pose2D(-4.7, 0.3, 0.0)
        command = self.motion.strafe_command(3, self._context(pose), target, "keeper line")
        intent = command.intent
        self.assertIsInstance(intent, MoveIntent)
        assert isinstance(intent, MoveIntent)  # noqa: S101
        self.assertGreater(intent.vx, 0.0)
        self.assertGreater(intent.vyaw, 0.0)

    def test_uses_infield_axis_when_other_step_crosses_goal_line(self) -> None:
        x = self.field.own_goal_x() + 0.3
        pose = _keeper(x, 0.0, 0.8)
        target = Pose2D(x, -0.7, 0.0)
        intent = self.motion.strafe_command(3, self._context(pose), target, "keeper line").intent
        self.assertIsInstance(intent, MoveIntent)
        assert isinstance(intent, MoveIntent)  # noqa: S101
        self.assertEqual((intent.vx, intent.vyaw), (0.0, 0.0))
        self.assertLess(intent.vy, 0.0)


if __name__ == "__main__":
    unittest.main()
