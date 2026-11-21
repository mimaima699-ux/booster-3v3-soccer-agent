"""Pure-function tests for distance-scaled kick power and corner shot aiming.

These cover :func:`src.tactics.targeting.attack.kick_power_for_distance` and
:func:`src.tactics.targeting.attack.best_shot_target`, the two new tactic
helpers behind tiered kick power and post-corner aiming.
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from src.soccer_framework import (  # noqa: E402
    BallState,
    Pose2D,
    RobotState,
    SoccerConfig,
    PlayContext,
)
from src.tactics.geometry import TeamFieldFrame  # noqa: E402
from src.tactics.navigation import ObstacleCollector  # noqa: E402
from src.tactics.targeting.attack import (  # noqa: E402
    best_shot_target,
    kick_power_for_distance,
)


def _context_with_opponents(opponents: dict[int, RobotState]) -> PlayContext:
    return PlayContext(
        game_state=None,
        teammates={},
        opponents=opponents,
        ball=BallState(x=4.0, y=0.0, last_seen_at=0.0),
    )


def _opponent(player_id: int, x: float, y: float) -> RobotState:
    return RobotState(player_id=player_id, pose=Pose2D(x, y, 0.0), last_seen_at=0.0)


class KickPowerTests(unittest.TestCase):
    def test_distance_zero_keeps_baseline_power(self) -> None:
        config = SoccerConfig()
        self.assertAlmostEqual(
            kick_power_for_distance(config, 0.0),
            config.strategy.soccer_kick_power,
        )

    def test_power_grows_with_distance_up_to_max(self) -> None:
        config = SoccerConfig()
        near = kick_power_for_distance(config, 1.0)
        far = kick_power_for_distance(config, 5.0)
        very_far = kick_power_for_distance(config, 20.0)
        self.assertGreater(far, near)
        self.assertEqual(very_far, config.strategy.kick_power_max)

    def test_min_clamp_binds_for_small_baselines(self) -> None:
        tuning = SoccerConfig().strategy
        tuning.soccer_kick_power = 0.8
        config = SoccerConfig(strategy=tuning)
        self.assertEqual(
            kick_power_for_distance(config, 0.0),
            config.strategy.kick_power_min,
        )


class BestShotTargetTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = SoccerConfig()
        self.field = TeamFieldFrame(self.config)
        self.obstacles = ObstacleCollector(self.config, self.field)

    def test_empty_field_returns_goal_mouth_target(self) -> None:
        target = best_shot_target(
            self.config, self.field, self.obstacles,
            _context_with_opponents({}),
        )
        self.assertIsNotNone(target)
        assert target is not None  # noqa: S101 - narrowed for the type checker
        reach = self.config.goal_width / 2.0 - self.config.strategy.shot_post_margin_m
        self.assertAlmostEqual(target.x, self.field.opponent_goal_x())
        self.assertLessEqual(abs(target.y), reach + 1e-9)

    def test_center_keeper_forces_corner_aim(self) -> None:
        goal_x = self.field.opponent_goal_x()
        context = _context_with_opponents(
            {1: _opponent(1, goal_x - 0.3, 0.0)},
        )
        target = best_shot_target(self.config, self.field, self.obstacles, context)
        self.assertIsNotNone(target)
        assert target is not None  # noqa: S101 - narrowed for the type checker
        self.assertGreater(abs(target.y), 0.5)

    def test_full_wall_blocks_every_candidate(self) -> None:
        goal_x = self.field.opponent_goal_x()
        context = _context_with_opponents(
            {
                1: _opponent(1, goal_x - 0.2, -0.6),
                2: _opponent(2, goal_x - 0.2, 0.0),
                3: _opponent(3, goal_x - 0.2, 0.6),
            },
        )
        target = best_shot_target(self.config, self.field, self.obstacles, context)
        self.assertIsNone(target)


if __name__ == "__main__":
    unittest.main()
