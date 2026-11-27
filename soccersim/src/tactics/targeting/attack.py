"""Kick target selection, lane scoring, and distance-scaled power."""

from __future__ import annotations

import math
from collections.abc import Callable

from ...soccer_framework import (
    BallState,
    GameControlState,
    Pose2D,
    SetPlay,
    SoccerConfig,
    PlayContext,
)
from ..geometry import clamp
from ..navigation import Obstacle, ObstacleCollector
from ..geometry import TeamFieldFrame
from . import recovery
from .predicates import ball_near_sideline


__all__ = [
    "PlayerAllowed",
    "best_pass_target",
    "best_shot_target",
    "dribble_target",
    "kick_power_for_distance",
    "kick_is_own_goal_safe",
    "kick_reason",
    "lane_clear_score",
    "select_clear_or_pass_target",
    "select_kick_target",
    "shot_lane_is_clear",
    "should_make_restart_touch",
]


# Shared with upper layers: test whether a teammate can legally join tactics, excluding penalized/substitute players.
PlayerAllowed = Callable[[GameControlState, int], bool]


# Top-level selection


def select_kick_target(
    config: SoccerConfig,
    field: TeamFieldFrame,
    obstacles: ObstacleCollector,
    player_id: int,
    context: PlayContext,
    is_player_allowed: PlayerAllowed,
) -> Pose2D:
    """Decide this tick's aim target for the chaser.

    Decision order: sideline recovery, restart touch, clear shot lane, best pass,
    and finally dribble forward.
    """

    ball = context.known_ball
    if ball_near_sideline(config, ball):
        return recovery.sideline_recovery_target(config, field, ball)

    game = context.known_game
    if should_make_restart_touch(config, game):
        teammate = best_pass_target(
            config, obstacles,
            player_id, context, is_player_allowed,
        )
        if teammate is not None:
            return Pose2D(teammate.x, teammate.y, 0.0)
        return dribble_target(config, field, ball)

    shot = best_shot_target(config, field, obstacles, context)
    if shot is not None:
        return shot

    teammate = best_pass_target(
        config, obstacles,
        player_id, context, is_player_allowed,
    )
    if teammate is not None:
        return Pose2D(teammate.x, teammate.y, 0.0)

    return dribble_target(config, field, ball)


def select_clear_or_pass_target(
    config: SoccerConfig,
    field: TeamFieldFrame,
    obstacles: ObstacleCollector,
    player_id: int,
    context: PlayContext,
    is_player_allowed: PlayerAllowed,
) -> Pose2D:
    """Side-lane view: pass if possible, otherwise clear toward the opponent goal without shooting or dribbling.

    Side chasers prefer clearing toward the middle rather than dribbling along the sideline into pressure.
    """

    ball = context.known_ball
    if ball_near_sideline(config, ball):
        return recovery.sideline_recovery_target(config, field, ball)

    teammate = best_pass_target(
        config, obstacles,
        player_id, context, is_player_allowed,
    )
    if teammate is not None:
        return Pose2D(teammate.x, teammate.y, 0.0)
    return Pose2D(field.opponent_goal_x(), 0.0, 0.0)


# Restart touch


def should_make_restart_touch(
    config: SoccerConfig,
    game: GameControlState,
) -> bool:
    """Whether our team should actively touch the ball on restart: kickoff, throw-in, or indirect free kick."""

    return game.is_kickoff_for_team(config.team_id) or (
        game.is_restart_for_team(config.team_id)
        and game.set_play in {SetPlay.THROW_IN, SetPlay.INDIRECT_FREE_KICK}
    )


# Pass scoring


def best_pass_target(
    config: SoccerConfig,
    obstacles: ObstacleCollector,
    player_id: int,
    context: PlayContext,
    is_player_allowed: PlayerAllowed,
) -> Pose2D | None:
    """Select this tick's best legal passing target; return ``None`` when no candidate qualifies.

    Score weights are lane clearance 0.55, forward gain 0.30, center pull 0.15,
    minus distance penalty. Candidates below ``pass_min_score`` are discarded.
    """

    if not config.strategy.pass_enabled:
        return None
    ball = context.known_ball
    game = context.known_game
    candidates: list[tuple[float, Pose2D]] = []
    for teammate_id, robot in context.teammates.items():
        if teammate_id == player_id or robot.pose is None:
            continue
        if not is_player_allowed(game, teammate_id):
            continue
        forward_gain = robot.pose.x - ball.x
        if forward_gain < config.strategy.pass_min_forward_m:
            continue
        lane_score = lane_clear_score(
            config,
            ball.x,
            ball.y,
            robot.pose.x,
            robot.pose.y,
            obstacles.opponent_obstacles(context),
        )
        distance = math.hypot(robot.pose.x - ball.x, robot.pose.y - ball.y)
        field_score = clamp(
            forward_gain / max(1.0, config.field_length),
            0.0,
            1.0,
        )
        center_score = 1.0 - clamp(
            abs(robot.pose.y) / (config.field_width / 2.0),
            0.0,
            1.0,
        )
        score = 0.55 * lane_score + 0.30 * field_score + 0.15 * center_score
        score -= clamp(distance / 12.0, 0.0, 0.25)
        if score >= config.strategy.pass_min_score:
            candidates.append((score, robot.pose))
    if not candidates:
        return None
    return max(candidates, key=lambda item: item[0])[1]


def shot_lane_is_clear(
    config: SoccerConfig,
    field: TeamFieldFrame,
    obstacles: ObstacleCollector,
    context: PlayContext,
) -> bool:
    """Return whether the shot selector can find a viable lane into goal."""
    return best_shot_target(config, field, obstacles, context) is not None


def lane_clear_score(
    config: SoccerConfig,
    start_x: float,
    start_y: float,
    target_x: float,
    target_y: float,
    obstacles: tuple[Obstacle, ...],
) -> float:
    """Generic lane-obstacle score in [0, 1], where 1 is clear and 0 is blocked.

    Project each obstacle onto the lane; if it lies within the segment and inside
    lateral clearance, reduce score proportionally. Multiple obstacles take the worst score.
    """

    if not obstacles:
        return 1.0
    seg_dx = target_x - start_x
    seg_dy = target_y - start_y
    seg_len = math.hypot(seg_dx, seg_dy)
    if seg_len < 1e-6:
        return 0.0
    dir_x = seg_dx / seg_len
    dir_y = seg_dy / seg_len
    left_x = -dir_y
    left_y = dir_x
    score = 1.0
    for obstacle in obstacles:
        rel_x = obstacle.x - start_x
        rel_y = obstacle.y - start_y
        along = rel_x * dir_x + rel_y * dir_y
        if along <= 0.0 or along >= seg_len:
            continue
        lateral = abs(rel_x * left_x + rel_y * left_y)
        clearance = max(config.strategy.pass_lane_clearance, obstacle.radius)
        if lateral < clearance:
            score = min(score, clamp(lateral / clearance, 0.0, 1.0))
    return score


# Shot aiming and kick power scaling


def best_shot_target(
    config: SoccerConfig,
    field: TeamFieldFrame,
    obstacles: ObstacleCollector,
    context: PlayContext,
) -> Pose2D | None:
    """Pick the best aim point across the opponent goal mouth, or ``None``.

    Candidates and their angular error bounds must cross inside the posts.
    Score the worst sampled lane and favor space away from opponents.
    """

    strategy = config.strategy
    ball = context.known_ball
    goal_x = field.opponent_goal_x()
    depth = goal_x - ball.x
    if depth <= 0.0:
        return None
    reach = max(0.0, config.goal_width / 2.0 - strategy.shot_post_margin_m)
    steps = max(1, math.ceil(reach / 0.18))
    opponents = obstacles.opponent_obstacles(context)
    openness_weight = strategy.shot_openness_weight

    best: tuple[float, Pose2D] | None = None
    for index in range(-steps, steps + 1):
        aim_y = reach * index / steps
        theta = math.atan2(aim_y - ball.y, depth)
        angles = (theta - strategy.shot_aim_error_rad, theta, theta + strategy.shot_aim_error_rad)
        if any(abs(angle) >= math.pi / 2 for angle in angles):
            continue
        crossings = tuple(ball.y + depth * math.tan(angle) for angle in angles)
        if any(abs(y) > reach + 1e-6 for y in crossings):
            continue
        lane = min(
            lane_clear_score(config, ball.x, ball.y, goal_x, y, opponents)
            for y in crossings
        )
        if lane < strategy.shot_lane_min_score:
            continue
        openness = 1.0
        if opponents:
            nearest = min(
                math.hypot(obstacle.x - goal_x, obstacle.y - aim_y)
                for obstacle in opponents
            )
            openness = clamp(nearest / 2.0, 0.0, 1.0)
        score = (1.0 - openness_weight) * lane + openness_weight * openness
        if best is None or (score, -abs(aim_y)) > (best[0], -abs(best[1].y)):
            best = (score, Pose2D(goal_x, aim_y, 0.0))
    return None if best is None else best[1]


def kick_power_for_distance(config: SoccerConfig, distance_m: float) -> float:
    """Distance-scaled kick power: baseline plus slope, clamped to [min, max].

    Short passes keep roughly the historical baseline power while long shots
    and clears get proportionally stronger.
    """

    strategy = config.strategy
    power = strategy.soccer_kick_power + strategy.kick_power_scale_per_m * max(
        0.0, distance_m,
    )
    return min(strategy.kick_power_max, max(strategy.kick_power_min, power))


def kick_is_own_goal_safe(
    config: SoccerConfig,
    field: TeamFieldFrame,
    ball: BallState,
    kick_direction: float,
) -> bool:
    """Whether a straight kick in ``kick_direction`` cannot enter our own goal.

    Unsafe only when the line from the ball travels toward our goal line and
    crosses it inside the guarded mouth band
    (``own_goal_guard_half_width_m``). Forward or lateral-forward kicks are
    always safe. The kick executor rejects unsafe directions so the robot
    repositions instead of pushing the ball toward its own net.
    """

    dx = math.cos(kick_direction)
    if dx >= -1e-6:
        return True  # Not traveling toward our goal line.
    goal_x = field.own_goal_x()
    if ball.x <= goal_x + 0.05:
        return True  # Level with or behind the line; nothing left to guard.
    t = (goal_x - ball.x) / dx
    y_at_goal = ball.y + math.sin(kick_direction) * t
    return abs(y_at_goal) > config.strategy.own_goal_guard_half_width_m


# Dribble and reason text


def dribble_target(
    config: SoccerConfig,
    field: TeamFieldFrame,
    ball: BallState,
) -> Pose2D:
    """Simple dribble target: advance by ``dribble_advance_m`` along ``+x`` and pull ``y`` toward center."""

    target_x = ball.x + config.strategy.dribble_advance_m
    target_y = ball.y * config.strategy.dribble_center_pull
    return field.clamp_inside_field(
        Pose2D(target_x, target_y, field.attack_theta())
    )


def kick_reason(
    config: SoccerConfig,
    target: Pose2D,
    default: str = "chaser kick",
) -> str:
    """Choose the reason suffix from whether target points at the opponent goal center.

    When target is near the goal mouth, keep ``default``; otherwise append
    ``" to target"`` so logs distinguish shooting from pass/clear targets.
    """

    if abs(target.x) >= config.field_length / 2.0 - 0.2 and abs(target.y) < 0.2:
        return default
    return f"{default} to target"
