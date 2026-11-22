"""Support-position targets plus teammate-spacing pushout.

SafetyGuards ensure PLAY support targets only run with fresh ball and
GameController data; support then stays behind the ball by ``support_depth_m``
and ``support_lateral_m`` and pushes away from teammates to avoid stacking.
"""

from __future__ import annotations

import math

from ...soccer_framework import (
    Pose2D,
    SoccerConfig,
    PlayContext,
)
from ..geometry import clamp
from ..geometry import TeamFieldFrame
from .attack import PlayerAllowed


__all__ = [
    "attack_support_target",
    "defender_cover_target",
    "support_target",
]


def support_target(
    config: SoccerConfig,
    field: TeamFieldFrame,
    player_id: int,
    context: PlayContext,
    is_player_allowed: PlayerAllowed,
) -> Pose2D:
    """Compute this tick's supporter target Pose2D.

    Stay behind the ball by ``support_depth_m`` and laterally split by player_id parity, clamped to our half.
    Pushout: use :func:`_spaced_support_target` to avoid overlapping other supporters.
    """

    side = 1.0 if player_id % 2 == 0 else -1.0
    lateral = config.strategy.support_lateral_m * side
    ball = context.known_ball
    x = ball.x - config.strategy.support_depth_m
    x = field.own_half_x(x, margin=0.35)
    y = clamp(
        ball.y + lateral,
        -config.field_width / 2.0 + 0.45,
        config.field_width / 2.0 - 0.45,
    )
    target = field.clamp_inside_field(
        Pose2D(x, y, field.face_ball_theta(x, y, ball))
    )
    return _spaced_support_target(
        config,
        field,
        player_id,
        context,
        target,
        is_player_allowed,
    )


# Teammate spacing pushout


def _spaced_support_target(
    config: SoccerConfig,
    field: TeamFieldFrame,
    player_id: int,
    context: PlayContext,
    target: Pose2D,
    is_player_allowed: PlayerAllowed,
) -> Pose2D:
    """If target is closer than min_spacing to the nearest teammate, push it along "teammate -> target" out to ``min_spacing``.

    Steps:
    1. Find the nearest legal teammate.
    2. If distance is large enough, do nothing.
    3. Otherwise scale the "teammate -> target" unit vector to min_spacing.
    4. Clamp inside the field and finally face the ball.

    Degenerate case: when target almost overlaps the teammate, no direction can
    be scaled, so fall back to ``lane_sign`` based on which side of the ball target
    is on; if target is exactly on the ball, split by player_id parity.

    In extreme corners with teammate pressure, clamping can make the final target
    slightly closer than min_spacing. With at most three teammates this is rare; if
    strict final distance is needed, iterate once more after clamping.
    """

    min_spacing = config.strategy.support_min_spacing_m
    if min_spacing <= 0.0:
        return target

    ball = context.known_ball
    game = context.known_game
    teammate_poses = tuple(
        robot.pose
        for teammate_id, robot in context.teammates.items()
        if teammate_id != player_id
        and robot.pose is not None
        and is_player_allowed(game, teammate_id)
    )
    if not teammate_poses:
        return target

    closest = min(
        teammate_poses,
        key=lambda pose: math.hypot(pose.x - target.x, pose.y - target.y),
    )
    dx = target.x - closest.x
    dy = target.y - closest.y
    distance = math.hypot(dx, dy)
    if distance >= min_spacing:
        return target

    if distance <= 1e-6:
        # Target overlaps the nearest teammate; use the original lane_sign fallback direction.
        lane_sign = 1.0 if target.y >= ball.y else -1.0
        if abs(target.y - ball.y) < 1e-6:
            lane_sign = 1.0 if player_id % 2 == 0 else -1.0
        dx, dy = 0.0, lane_sign
        distance = 1.0

    scale = min_spacing / distance
    pushed = field.clamp_inside_field(
        Pose2D(
            closest.x + dx * scale,
            closest.y + dy * scale,
            target.theta,
        )
    )
    return Pose2D(
        pushed.x,
        pushed.y,
        field.face_ball_theta(pushed.x, pushed.y, ball),
    )


def attack_support_target(
    config: SoccerConfig,
    field: TeamFieldFrame,
    player_id: int,
    context: PlayContext,
    is_player_allowed: PlayerAllowed,
) -> Pose2D:
    """Attacking support offer: stand ahead-lateral of the ball.

    Unlike the defensive :func:`support_target`, this position is ahead of
    the ball on the center side, so the chaser always has a forward receiver
    that satisfies pass eligibility. Teammate spacing pushout still applies.
    """
    strategy = config.strategy
    ball = context.known_ball
    side = -1.0 if ball.y > 0.0 else 1.0
    target = field.clamp_inside_field(
        Pose2D(
            ball.x + strategy.attack_support_ahead_m,
            ball.y + side * strategy.attack_support_lateral_m,
            field.face_ball_theta(
                ball.x + strategy.attack_support_ahead_m,
                ball.y + side * strategy.attack_support_lateral_m,
                ball,
            ),
        )
    )
    return _spaced_support_target(
        config, field, player_id, context, target, is_player_allowed,
    )


def defender_cover_target(
    config: SoccerConfig,
    field: TeamFieldFrame,
    player_id: int,
    context: PlayContext,
    is_player_allowed: PlayerAllowed,
) -> Pose2D:
    """Secondary-defender cover point during defensive mode.

    The cover sits goal-side of the ball on the ball-to-goal-center line at
    ``defender_cover_depth_m``, then shifts laterally off the pressure lane
    (away from the nearest teammate, which is normally the primary defender)
    so the two field robots never stack on the same spot. The point is held
    inside a central band so the defensive middle is never abandoned.
    """
    strategy = config.strategy
    ball = context.known_ball
    goal_x = field.own_goal_x()
    dx = goal_x - ball.x
    dy = -ball.y
    length = math.hypot(dx, dy)
    if length < 1e-6:
        ux, uy = -1.0, 0.0
    else:
        ux, uy = dx / length, dy / length
    x = ball.x + ux * strategy.defender_cover_depth_m
    y = ball.y + uy * strategy.defender_cover_depth_m

    # Shift off the pressure lane, away from the nearest legal teammate.
    game = context.known_game
    nearest: tuple[float, float] | None = None
    nearest_d = math.inf
    for mate_id, robot in context.teammates.items():
        if mate_id == player_id or robot.pose is None:
            continue
        if not is_player_allowed(game, mate_id):
            continue
        d = math.hypot(robot.pose.x - x, robot.pose.y - y)
        if d < nearest_d:
            nearest_d = d
            nearest = (robot.pose.x, robot.pose.y)
    if nearest is not None:
        shift = (
            strategy.defender_cover_lateral_m
            if nearest[1] <= y
            else -strategy.defender_cover_lateral_m
        )
        y += shift

    half_width = strategy.defender_center_half_width_m
    y = clamp(y, -half_width, half_width)
    target = field.clamp_inside_field(
        Pose2D(x, y, field.face_ball_theta(x, y, ball))
    )
    # The cover never crosses halfway.
    return Pose2D(
        min(target.x, -0.3),
        target.y,
        field.face_ball_theta(min(target.x, -0.3), target.y, ball),
    )
