"""Avoidance targets during opponent restarts.

When GameController marks an opponent ``set_play``, all our players must stay
outside the configured ball-avoidance radius. This module passes position targets
through constraints for radius safety, penalty-area legality, and field bounds.

Goal kicks, corners, and throw-ins have dedicated formations. Other restarts use the
base READY position or a retreat waypoint when the player is close to the ball.
"""

from __future__ import annotations

import math

from ...soccer_framework import (
    BallState,
    GameControlState,
    GameState,
    Pose2D,
    ReadySlot,
    SetPlay,
    SoccerConfig,
    PlayContext,
)
from ..geometry import TeamFieldFrame
from .predicates import sideline_sign
from .recovery import BaseReadyTarget


__all__ = [
    "opponent_restart_hold_vyaw",
    "opponent_restart_target",
]

DEFENDED_RESTARTS = frozenset({SetPlay.GOAL_KICK, SetPlay.CORNER_KICK, SetPlay.THROW_IN})


def opponent_restart_target(
    config: SoccerConfig,
    field: TeamFieldFrame,
    player_id: int,
    slot: ReadySlot,
    context: PlayContext,
    base_ready_target: BaseReadyTarget,
) -> Pose2D:
    """Choose a restart formation or move the player away from the ball."""

    base_target = base_ready_target(slot, False)
    ball = context.known_ball
    game = context.known_game

    min_distance = config.strategy.opponent_restart_avoid_distance_m
    if game.set_play in DEFENDED_RESTARTS:
        return _restart_formation_target(config, field, player_id, slot, context, base_target)

    robot = context.teammates.get(player_id)
    if robot is not None and robot.pose is not None:
        distance_to_ball = math.hypot(
            robot.pose.x - ball.x,
            robot.pose.y - ball.y,
        )
        if distance_to_ball < min_distance + 0.25:
            escape_target = _escape_initial_target(
                field, robot.pose, ball, game,
                min_distance=min_distance + 0.35,
            )
            return _radius_safe(field, escape_target, ball, min_distance + 0.35)

    return _radius_safe(field, base_target, ball, min_distance)


def _restart_formation_target(
    config: SoccerConfig,
    field: TeamFieldFrame,
    player_id: int,
    slot: ReadySlot,
    context: PlayContext,
    base_target: Pose2D,
) -> Pose2D:
    """Position for the next pass or shot while respecting restart exclusions."""
    strategy = config.strategy
    ball = context.known_ball
    game = context.known_game
    side = sideline_sign(ball.y)
    clearance = strategy.opponent_restart_avoid_distance_m + strategy.restart_formation_clearance_m
    own_goal_x = field.own_goal_x()
    front_x = (
        field.opponent_goal_x() - config.penalty_area_length - strategy.goal_kick_area_margin_m
    )
    box_x = own_goal_x + config.penalty_area_length
    kicker = min(
        (robot.pose for robot in context.opponents.values() if robot.pose is not None),
        key=lambda pose: math.hypot(pose.x - ball.x, pose.y - ball.y),
        default=None,
    )
    range_m = strategy.opponent_kickoff_kicker_range_m
    approach = 0.0
    shade = 0.0
    if kicker is not None:
        distance = math.hypot(kicker.x - ball.x, kicker.y - ball.y)
        approach = max(0.0, 1.0 - distance / range_m)
        shade = (kicker.y - ball.y) / range_m * approach * strategy.restart_kicker_shade_m

    if slot == ReadySlot.KEEPER:
        depth = strategy.keeper_zone_min_x_offset_m + approach * strategy.keeper_kickoff_advance_m
        x = own_goal_x + depth
        if game.set_play == SetPlay.CORNER_KICK:
            y = side * config.goal_width / 2.0 * strategy.corner_keeper_post_fraction
        else:
            fraction = depth / max(depth, ball.x - own_goal_x)
            limit = strategy.keeper_zone_half_width_m
            y = max(-limit, min(limit, ball.y * fraction))
    elif game.set_play == SetPlay.GOAL_KICK:
        if slot == ReadySlot.CENTER:
            x = front_x - (1.0 - approach) * strategy.outfield_kickoff_advance_m
            y = ball.y * strategy.defender_cover_center_pull + shade
        else:
            x, y = front_x - strategy.goal_kick_cover_gap_m, -side * abs(base_target.y) / 2.0
    elif game.set_play == SetPlay.THROW_IN:
        if slot == ReadySlot.CENTER:
            x = ball.x - clearance * (1.0 - approach / 2.0)
            y = ball.y - side * clearance + shade
        else:
            x = max(box_x, min(-0.3, ball.x - strategy.defender_cover_depth_m))
            y = side * strategy.defender_center_half_width_m / 2.0
    elif slot == ReadySlot.CENTER:
        # Intercept the cross on the line from the corner to the middle of the box.
        dx, dy = box_x - ball.x, -ball.y
        distance = math.hypot(dx, dy)
        clearance += (1.0 - approach) * strategy.restart_formation_clearance_m
        if distance > clearance:
            x = ball.x + clearance * dx / distance
            y = ball.y + clearance * dy / distance
        else:
            x, y = box_x, 0.0
    else:
        x = own_goal_x + config.penalty_area_length
        y = -side * config.goal_width / 4.0

    target = _radius_safe(field, Pose2D(x, y), ball, clearance)
    robot = context.teammates.get(player_id)
    pose = robot.pose if robot is not None else None
    if game.set_play == SetPlay.GOAL_KICK:
        target = Pose2D(min(target.x, front_x), target.y)
        box_front = field.opponent_goal_x() - config.penalty_area_length
        if pose is not None and pose.x > box_front:
            # Exit along x first so lateral runs do not cut through the box.
            target = field.clamp_inside_field(Pose2D(min(front_x, ball.x - clearance), pose.y))
    elif pose is not None and _path_cuts_ball_radius(
        pose, target, ball, strategy.opponent_restart_avoid_distance_m,
    ):
        # Move infield before taking a route that would cut across the ball.
        target = field.clamp_inside_field(Pose2D(pose.x, ball.y - side * clearance))

    theta = (
        field.attack_theta()
        if slot == ReadySlot.KEEPER and game.set_play == SetPlay.CORNER_KICK
        else field.face_ball_theta(target.x, target.y, ball)
    )
    return Pose2D(target.x, target.y, theta)


def _path_cuts_ball_radius(start: Pose2D, target: Pose2D, ball: BallState, radius: float) -> bool:
    dx, dy = target.x - start.x, target.y - start.y
    segment_length_sq = dx * dx + dy * dy
    fraction = ((ball.x - start.x) * dx + (ball.y - start.y) * dy) / max(segment_length_sq, 1e-6)
    fraction = max(0.0, min(1.0, fraction))
    return math.hypot(start.x + dx * fraction - ball.x, start.y + dy * fraction - ball.y) < radius


def opponent_restart_hold_vyaw(
    config: SoccerConfig,
    player_id: int,
    game: GameControlState,
) -> float:
    """During opponent restart in PLAYING, apply a small yaw rate so players scan for the ball.

    Return 0 before PLAYING or while stopped. Direction alternates by player_id
    parity, with magnitude capped by min(0.12, ``max_angular_speed``).
    """

    if game.state != GameState.PLAYING or game.stopped:
        return 0.0
    direction = 1.0 if player_id % 2 else -1.0
    return direction * min(0.12, config.strategy.max_angular_speed)


def _escape_initial_target(
    field: TeamFieldFrame,
    robot_pose: Pose2D,
    ball: BallState,
    game: GameControlState,
    min_distance: float,
) -> Pose2D:
    """Compute an initial escape target away from the ball when a player is already too close.

    Move along "ball -> current pose" out to ``min_distance``; THROW_IN adds a
    backfield bias. The target still must respect the ball clearance.
    """

    dx = robot_pose.x - ball.x
    dy = robot_pose.y - ball.y
    if math.hypot(dx, dy) <= 1e-6:
        dx = -1.0
        dy = -sideline_sign(ball.y)

    if game.set_play == SetPlay.THROW_IN:
        dx -= 0.8
        dy -= sideline_sign(ball.y) * 0.9

    distance = math.hypot(dx, dy)
    if distance <= 1e-6:
        distance = 1.0
    target_x = ball.x + dx / distance * min_distance
    target_y = ball.y + dy / distance * min_distance
    target = Pose2D(
        x=target_x,
        y=target_y,
        theta=field.face_ball_theta(target_x, target_y, ball),
    )
    if game.set_play == SetPlay.THROW_IN:
        target = Pose2D(
            field.own_half_x(target.x, margin=0.45),
            target.y,
            target.theta,
        )
    return target


def _radius_safe(
    field: TeamFieldFrame,
    target: Pose2D,
    ball: BallState,
    min_distance: float,
) -> Pose2D:
    """Ensure target is at least ``min_distance`` from the ball.

    If too close, scale along the original direction. If scaling fails because
    target is exactly on the ball, fall back to "back from ball plus opposite sideline";
    if still invalid, fall back to our half.
    """

    candidate = field.clamp_inside_field(target, margin=0.35)
    dx = candidate.x - ball.x
    dy = candidate.y - ball.y
    distance = math.hypot(dx, dy)
    if distance >= min_distance:
        return candidate

    if distance <= 1e-6:
        dx = -1.0
        dy = -sideline_sign(ball.y)
        distance = math.hypot(dx, dy)

    candidate = field.clamp_inside_field(
        Pose2D(
            x=ball.x + dx / distance * min_distance,
            y=ball.y + dy / distance * min_distance,
            theta=target.theta,
        ),
        margin=0.35,
    )
    if math.hypot(candidate.x - ball.x, candidate.y - ball.y) >= min_distance:
        return candidate

    fallback_y = ball.y - sideline_sign(ball.y) * min_distance
    fallback_x = field.own_half_x(
        ball.x - min_distance,
        margin=0.45,
    )
    return field.clamp_inside_field(
        Pose2D(
            fallback_x,
            fallback_y,
            field.face_ball_theta(fallback_x, fallback_y, ball),
        ),
        margin=0.35,
    )
