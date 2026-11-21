"""Pure-function tests for support-target spacing pushout.

These tests call :func:`src.tactics.targeting.support._spaced_support_target`
directly. It is the last step run by ``SupporterRole`` when the ball is known,
and decides whether a supporter target should be pushed away from teammates.
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
from src.soccer_framework.types import (  # noqa: E402
    GameControlState,
)
from src.tactics.geometry import TeamFieldFrame  # noqa: E402
from src.tactics.targeting.support import _spaced_support_target  # noqa: E402


def _allow_all(game: GameControlState, player_id: int) -> bool:
    return game is not None and player_id > 0


def _make_context(
    robots: dict[int, RobotState],
    ball: BallState | None = None,
    game: GameControlState | None = None,
) -> PlayContext:
    if ball is None:
        ball = BallState(x=0.0, y=0.0, last_seen_at=0.0)
    if game is None:
        game = GameControlState()
    return PlayContext(game_state=game, teammates=robots, ball=ball)


def _robot(
    player_id: int,
    x: float,
    y: float,
) -> RobotState:
    return RobotState(
        player_id=player_id,
        pose=Pose2D(x, y, 0.0),
        last_seen_at=0.0,
    )


def _game() -> GameControlState:
    # _spaced_support_target does not read game fields itself; it only passes
    # the object through to the is_player_allowed callback.
    return GameControlState()


class SpacedSupportTargetTests(unittest.TestCase):
    """Behavior-level assertions for teammate-spacing pushout.

    The tests intentionally assert qualitative properties such as direction
    and lower distance bounds instead of pinning an exact Pose2D. That lets the
    newer closest-teammate push and the older 5x3 candidate scoring algorithm
    share the same test set.
    """

    def setUp(self) -> None:
        self.config = SoccerConfig()
        self.field = TeamFieldFrame(self.config)
        # Ball is near midfield on the defensive side, away from touchlines, so
        # there is enough room for pushout.
        self.ball = BallState(x=0.0, y=0.0, last_seen_at=0.0)

    def test_no_teammate_returns_original(self) -> None:
        """Target is unchanged when there are no visible teammates."""

        target = Pose2D(-1.0, 0.5, 0.0)
        context = _make_context(robots={1: _robot(1, -2.0, 0.0)})
        out = _spaced_support_target(
            self.config, self.field,
            player_id=1, context=context, target=target,
            is_player_allowed=_allow_all,
        )
        self.assertEqual((out.x, out.y), (target.x, target.y))

    def test_disabled_min_spacing_returns_original(self) -> None:
        """min_spacing <= 0 short-circuits even when a teammate overlaps."""

        target = Pose2D(-1.0, 0.5, 0.0)
        config = SoccerConfig()
        config.strategy.support_min_spacing_m = 0.0
        context = _make_context(
            robots={
                1: _robot(1, -2.0, 0.0),
                2: _robot(2, -1.0, 0.5),  # Overlaps the target.
            },
        )
        out = _spaced_support_target(
            config, self.field,
            player_id=1, context=context, target=target, is_player_allowed=_allow_all,
        )
        self.assertEqual((out.x, out.y), (target.x, target.y))

    def test_far_from_teammate_returns_original_xy(self) -> None:
        """Target xy is unchanged when the nearest teammate is far enough.

        theta may still be adjusted by face_ball. The old algorithm returned
        the original target directly in this branch; the new algorithm takes an
        early-return path that also leaves xy unchanged.
        """

        target = Pose2D(-1.0, 0.5, 0.0)
        # Teammate 2 is 5 m from target, far beyond min_spacing (default 1.15 m).
        context = _make_context(
            robots={
                1: _robot(1, -2.0, 0.0),
                2: _robot(2, 4.0, 0.5),
            },
        )
        out = _spaced_support_target(
            self.config, self.field,
            player_id=1, context=context, target=target, is_player_allowed=_allow_all,
        )
        self.assertAlmostEqual(out.x, target.x, places=6)
        self.assertAlmostEqual(out.y, target.y, places=6)

    def test_close_teammate_pushes_target_to_min_spacing(self) -> None:
        """Close teammates push the target to at least min_spacing - 0.1 m.

        The 0.1 m tolerance keeps closest-push valid when clamp_inside_field is
        limited by field boundaries. This scenario keeps target=(-1, 0.5) away
        from touchlines, so clamp should not actually trigger.
        """

        target = Pose2D(-1.0, 0.5, 0.0)
        min_spacing = self.config.strategy.support_min_spacing_m
        # Teammate 2 is only 0.3 m from target, well below min_spacing.
        context = _make_context(
            robots={
                1: _robot(1, -2.0, 0.0),
                2: _robot(2, -1.0, 0.2),
            },
        )
        out = _spaced_support_target(
            self.config, self.field,
            player_id=1, context=context, target=target, is_player_allowed=_allow_all,
        )
        teammate_pose = context.teammates[2].pose
        out_distance = math.hypot(out.x - teammate_pose.x, out.y - teammate_pose.y)
        self.assertGreaterEqual(out_distance, min_spacing - 0.1)

    def test_close_teammate_pushed_target_faces_ball(self) -> None:
        """Pushed targets always face the ball, which is better for support."""

        target = Pose2D(-1.0, 0.5, 1.234)  # Deliberately odd initial theta.
        context = _make_context(
            robots={
                1: _robot(1, -2.0, 0.0),
                2: _robot(2, -1.0, 0.2),
            },
        )
        out = _spaced_support_target(
            self.config, self.field,
            player_id=1, context=context, target=target, is_player_allowed=_allow_all,
        )
        expected_theta = self.field.face_ball_theta(out.x, out.y, self.ball)
        self.assertAlmostEqual(out.theta, expected_theta, places=5)

    def test_target_overlaps_teammate_picks_a_direction(self) -> None:
        """Overlapping target and teammate still choose a push direction.

        When target and teammate nearly overlap, the push direction should not
        depend on the initial distance. It falls back to lane_sign and still
        pushes out to at least min_spacing.
        """

        teammate_pose = Pose2D(-1.0, 0.5, 0.0)
        target = Pose2D(-1.0, 0.5, 0.0)  # Exact overlap.
        min_spacing = self.config.strategy.support_min_spacing_m
        context = _make_context(
            robots={
                1: _robot(1, -2.0, 0.0),
                2: RobotState(
                    player_id=2,
                    pose=teammate_pose,
                    last_seen_at=0.0,
                ),
            },
        )
        out = _spaced_support_target(
            self.config, self.field,
            player_id=1, context=context, target=target, is_player_allowed=_allow_all,
        )
        out_distance = math.hypot(out.x - teammate_pose.x, out.y - teammate_pose.y)
        self.assertGreaterEqual(out_distance, min_spacing - 0.1)

    def test_disallowed_teammate_does_not_block(self) -> None:
        """Disallowed teammates do not participate in pushout."""

        target = Pose2D(-1.0, 0.5, 0.0)
        # Teammate 2 overlaps target but is excluded by is_player_allowed, so
        # target should remain unchanged.
        context = _make_context(
            robots={
                1: _robot(1, -2.0, 0.0),
                2: _robot(2, -1.0, 0.5),
            },
        )

        def deny_p2(game: GameControlState, player_id: int) -> bool:
            return _allow_all(game, player_id) and player_id != 2

        out = _spaced_support_target(
            self.config, self.field,
            player_id=1, context=context, target=target,
            is_player_allowed=deny_p2,
        )
        self.assertAlmostEqual(out.x, target.x, places=6)
        self.assertAlmostEqual(out.y, target.y, places=6)


if __name__ == "__main__":
    unittest.main()
