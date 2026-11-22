"""Pure-function tests for the dedicated-keeper defensive tactic helpers.

Covers the goalkeeper guard geometry in ``ready_stance``, the attack support
offer and defender cover in ``targeting.support``, and the own-goal kick
direction guard in ``targeting.attack``.
"""

from __future__ import annotations

import math
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
from src.soccer_framework.types import GameControlState  # noqa: E402
from src.tactics.geometry import TeamFieldFrame  # noqa: E402
from src.tactics.ready_stance import ReadyStance  # noqa: E402
from src.tactics.targeting import attack, support  # noqa: E402


def _allow_all(game: GameControlState, player_id: int) -> bool:
    return game is not None and player_id > 0


def _context(
    robots: dict[int, RobotState],
    ball: BallState,
) -> PlayContext:
    return PlayContext(
        game_state=GameControlState(),
        teammates=robots,
        ball=ball,
    )


def _robot(player_id: int, x: float, y: float) -> RobotState:
    return RobotState(player_id=player_id, pose=Pose2D(x, y, 0.0), last_seen_at=1.0)


class GoalkeeperGuardTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = SoccerConfig()
        self.field = TeamFieldFrame(self.config)
        self.stance = ReadyStance(self.config, self.field)

    def test_guard_covers_ball_goal_line(self) -> None:
        ball = BallState(x=0.0, y=3.0, last_seen_at=1.0)
        target = self.stance.goalkeeper_guard_target(ball)
        keeper_x = self.field.own_goal_x() + self.config.strategy.keeper_zone_min_x_offset_m
        self.assertAlmostEqual(target.x, keeper_x)
        # Crossing of ball(0, 3) -> goal(-7, 0) at keeper_x = -7 + 1.1:
        t = 1.1 / 7.0
        self.assertAlmostEqual(target.y, 3.0 * t, places=6)

    def test_guard_lateral_clamp(self) -> None:
        # Wide, close ball: the line crossing at the keeper line exceeds the
        # zone half-width, so the target clamps to the zone edge.
        ball = BallState(x=-2.0, y=8.9, last_seen_at=1.0)
        target = self.stance.goalkeeper_guard_target(ball)
        half_width = self.config.strategy.keeper_zone_half_width_m
        self.assertAlmostEqual(target.y, half_width)

    def test_guard_without_ball_returns_center(self) -> None:
        target = self.stance.goalkeeper_guard_target(None)
        self.assertAlmostEqual(target.x, -self.config.field_length / 2.0 + self.config.strategy.keeper_zone_min_x_offset_m)
        self.assertAlmostEqual(target.y, 0.0)


class SupportTargetsTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = SoccerConfig()
        self.field = TeamFieldFrame(self.config)

    def test_attack_support_stands_ahead_lateral(self) -> None:
        ball = BallState(x=2.0, y=1.0, last_seen_at=1.0)
        context = _context({}, ball)
        target = support.attack_support_target(
            self.config, self.field, 1, context, _allow_all,
        )
        strategy = self.config.strategy
        # Ball y > 0: offer goes to the center side (negative lateral).
        self.assertAlmostEqual(target.x, ball.x + strategy.attack_support_ahead_m)
        self.assertAlmostEqual(target.y, ball.y - strategy.attack_support_lateral_m)

    def test_defender_cover_is_goal_side_and_central(self) -> None:
        ball = BallState(x=-3.0, y=2.0, last_seen_at=1.0)
        context = _context(
            {1: _robot(1, -2.0, 2.0)},  # chaser pressing near the ball
            ball,
        )
        target = support.defender_cover_target(
            self.config, self.field, 2, context, _allow_all,
        )
        strategy = self.config.strategy
        # Goal-side: strictly between the ball and our goal, own half only.
        self.assertLess(target.x, ball.x)
        self.assertLessEqual(target.x, -0.3)
        # Central band respected.
        self.assertLessEqual(abs(target.y), strategy.defender_center_half_width_m)
        # Offset away from the pressure lane (chaser at y=2 pushes cover down).
        self.assertLess(target.y, 2.0)


class OwnGoalGuardTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = SoccerConfig()
        self.field = TeamFieldFrame(self.config)

    def test_forward_kick_is_safe(self) -> None:
        ball = BallState(x=-5.0, y=0.0, last_seen_at=1.0)
        self.assertTrue(
            attack.kick_is_own_goal_safe(self.config, self.field, ball, 0.1),
        )

    def test_kick_into_own_mouth_is_unsafe(self) -> None:
        ball = BallState(x=-5.0, y=1.0, last_seen_at=1.0)
        direction = math.atan2(-1.0, -2.0)  # Aimed back toward our goal center.
        self.assertFalse(
            attack.kick_is_own_goal_safe(self.config, self.field, ball, direction),
        )

    def test_kick_wide_of_own_goal_is_safe(self) -> None:
        ball = BallState(x=-5.0, y=-2.5, last_seen_at=1.0)
        # Straight backward but crossing the goal line far from the mouth.
        direction = math.pi
        self.assertTrue(
            attack.kick_is_own_goal_safe(self.config, self.field, ball, direction),
        )


if __name__ == "__main__":
    unittest.main()
