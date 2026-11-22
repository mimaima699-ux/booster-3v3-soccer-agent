"""Team and strategy configuration loaded from defaults or environment.

Competitor-facing tuning knobs live here: field dimensions, robot names, kick
hysteresis, avoidance options, pass thresholds, :class:`SoccerStrategyTuning`,
and :class:`SoccerDebugConfig`.

:meth:`SoccerConfig.from_env` loads per-match config from env vars such as
``SOCCER_TEAM_ID`` and ``SOCCER_ROBOT_NAMES``. ``ros_debug/.soccer_sim.env`` is a
separate host-deployment config for scripts and is not consumed by the runtime process.

The environment surface is intentionally limited to fields competitors must change
per match. Other fields are changed by dataclass defaults or constructor
arguments during debugging.
"""

from __future__ import annotations

import os
from collections.abc import Mapping
from dataclasses import dataclass, field

from .types import (
    ADULT_FIELD_DIMENSIONS,
    FieldDimensions,
    ReadySlot,
)


__all__ = [
    "SoccerConfig",
    "SoccerDebugConfig",
    "SoccerStrategyTuning",
]


DEFAULT_READY_SLOT_SEQUENCE = (
    ReadySlot.CENTER,
    ReadySlot.SIDE,
    ReadySlot.KEEPER,
)


@dataclass
class SoccerDebugConfig:
    """Debug-only switches for optional diagnostics.

    These controls are intentionally kept out of environment parsing so normal
    match startup has a small, predictable public surface. Change defaults or
    pass a custom instance to :class:`SoccerConfig` when debugging locally.
    """

    bt_trace_ticks: str = "off"
    bt_trace_sample_sec: float = 0.5


@dataclass
class SoccerStrategyTuning:
    """Collection of tuning knobs for tactic behavior.

    These are "how to play" parameters: speed limits, kick hysteresis, avoidance
    margins, pass/dribble/support thresholds, and so on. Fields are grouped by
    function, with defaults tied to hardware or rules.

    These parameters are not exposed as individual env vars; change defaults or
    pass custom tuning to :class:`SoccerConfig` during debugging.
    """

    # Speed limits
    # Hard output clamps in the motion layer, tied to chassis stability and field friction.
    max_linear_speed: float = 0.8  #  Linear speed limit in m/s.
    max_angular_speed: float = 1.0  #  Angular speed limit in rad/s.

    # Kick hysteresis
    # Use enter/exit thresholds plus delay to prevent flapping around distance boundaries.
    soccer_kick_enter_distance: float = 2.5  #  Enter kick mode when distance to ball is below enter.
    soccer_kick_exit_distance: float = 3.0  #  Exit kick mode when distance to ball is above exit; must exceed enter.
    soccer_kick_power: float = 1.5  #  Baseline kick power; also the base of the distance-scaled power.
    soccer_kick_min_active_sec: float = 1.0  #  Minimum active kick duration to avoid instant switching.
    soccer_kick_exit_delay_sec: float = 1.5  #  Delay after exit condition before actually leaving kick mode.

    # Kick power scaling
    # Distance-scaled kick power: short passes stay gentle while long shots and
    # clears get proportionally stronger. ``soccer_kick_power`` stays the
    # baseline and the only power for callers that pass no distance.
    kick_power_min: float = 1.2  #  Lower clamp of the scaled kick power.
    kick_power_max: float = 4.0  #  Upper clamp of the scaled kick power; excessive values may destabilize the robot.
    kick_power_scale_per_m: float = 0.45  #  Power added per meter of ball-to-target distance.

    # Shot aiming
    # Aim-point search across the opponent goal mouth instead of always the
    # center, so shots go to the corner the opponents cover least.
    shot_lane_min_score: float = 0.55  #  Minimum lane clearance for a shot candidate.
    shot_openness_weight: float = 0.35  #  Blend of aim-point openness vs lane clearance when ranking candidates.
    shot_post_margin_m: float = 0.25  #  Keep aim points this far inside each goal post.

    # Set plays and restarts
    restart_touch_distance: float = 0.45  #  Distance threshold for "touched the ball".
    opponent_restart_avoid_distance_m: float = (
        1.6  #  Rule requires 1.45 m; default adds 0.15 m buffer for a 1.60 m threshold.
    )

    # Path detour via points
    # First avoidance layer for blockers: draw a line from current pose to target; if an
    # obstacle, opponent, teammate, or goal, lies inside that corridor, compute a side via point
    # and rewrite target so the robot goes around. Triggering depends on corridor blockage,
    # not pure distance: far blockers trigger, nearby non-blockers do not.
    # Goal dimensions are rule-fixed in navigation.goal_structure_obstacles, not tuned here.
    # Opponent radius exceeds teammate radius because opponents are contested and less predictable.
    opponent_obstacle_radius: float = 0.55  #  Circular radius used to detour around opponents.
    teammate_obstacle_radius: float = (
        0.48  #  Circular radius used to detour around teammates; smaller because teammates are predictable.
    )
    obstacle_safety_margin: float = 0.22  #  Extra safety margin outside obstacle radius, shared by all obstacle types.
    obstacle_start_ignore_distance: float = 0.35  #  Ignore obstacles this close to the start to avoid close-contact jitter.
    obstacle_target_ignore_distance: float = 0.35  #  Ignore obstacles this close to the target to avoid arrival blocking.

    # Yaw avoidance bias
    # Second avoidance layer for close neighbors: keep the target, inspect nearby robots whose
    # current distance is below min_distance or predicted closest distance within horizon is below it,
    # then add +/-bias_max to vyaw so the robot turns slightly while passing.
    # For the biped base, the stable set_velocity combination is vx + vyaw; vy lateral
    # movement comes from gait synthesis, so this layer changes only vyaw and never vy.
    # Teammates always count as neighbors; opponents are included by BT phase through move_to.
    # PLAY excludes opponents so chasers are not pushed away; READY/recovery includes them.
    yaw_avoid_horizon_sec: float = 1.0  #  Prediction horizon for nearby-neighbor trajectories.
    yaw_avoid_min_distance_m: float = 0.78  #  Apply bias only when current or predicted distance is below this value.
    yaw_avoid_bias_max: float = (
        0.6  #  Maximum vyaw bias per neighbor in rad/s, reduced by scale.
    )

    # Ball-claim arbitration
    teammate_challenge_tie_margin_m: float = (
        0.15  #  Tie band for teammate ball-claim distances to prevent oscillating handoff.
    )

    # All-out press (disabled by default: the dedicated-keeper strategy keeps
    # the goalkeeper permanently defensive; enable explicitly to trade an
    # empty goal for an equalizer when losing late).
    press_enabled: bool = False  #  Master switch for the losing-scoreline press.
    press_min_goal_deficit: int = 1  #  Minimum goals behind that may trigger the press.
    press_when_losing_after_sec: int = 180  #  Press only when remaining seconds are at or below this.

    # Team attack/defense mode hysteresis. Separate enter/exit thresholds keep
    # the mode stable while the ball moves back and forth across halfway.
    defense_enter_ball_x: float = -0.5  #  Ball below this x switches to DEFENSE (m).
    defense_exit_ball_x: float = 0.5  #  Ball above this x returns to ATTACK (m).

    # Goalkeeper zone and behavior. The keeper never leaves this box except to
    # clear a ball inside its challenge radius, and never crosses halfway.
    # Guard depth is dynamic: the keeper stands at ``keeper_intercept_fraction``
    # along the ball-to-goal-center line (ball far -> advance and compress the
    # angle, ball close -> retreat toward the line), clamped to the band
    # [min, max] offsets from our goal line. This is the proven simple-framework
    # goalkeeper geometry.
    keeper_intercept_fraction: float = 0.4  #  Position along the ball-to-goal line (0 = goal, 1 = ball).
    keeper_zone_min_x_offset_m: float = 1.1  #  Shallowest guard depth from our goal line (m).
    keeper_zone_max_x_offset_m: float = 2.5  #  Deepest guard depth from our goal line (m).
    keeper_zone_half_width_m: float = 1.6  #  Lateral clamp of the keeper zone (m).
    keeper_track_deadband_m: float = 0.15  #  Ignore smaller guard-target corrections (m).
    keeper_challenge_radius_m: float = 1.7  #  Ball within this of the guard target may be cleared (m).

    # Goalkeeper lateral movement. The biped gait accepts single-axis lateral
    # commands (vy), so the keeper strafes while facing the ball instead of
    # the turn-walk-turn cycle that produced in-place spinning.
    keeper_max_lateral_speed_mps: float = 0.5  #  Lateral strafe speed clamp (m/s).
    keeper_lateral_arrive_m: float = 0.12  #  Lateral arrival threshold (m).
    keeper_strafe_forward_tolerance_m: float = 0.35  #  Forward error above this falls back to walking (m).
    keeper_face_ball_turn_threshold: float = 0.35  #  Heading error that triggers a pure face-ball turn (rad).

    # Goalkeeper mode machine: POSITION / INTERCEPT / CHALLENGE / EMERGENCY
    # with separate enter/exit radii (hysteresis) measured from our goal center.
    keeper_intercept_enter_radius_m: float = 4.5  #  Ball closer than this enters INTERCEPT (m).
    keeper_intercept_exit_radius_m: float = 5.2  #  Ball farther than this returns to POSITION (m).
    keeper_challenge_enter_radius_m: float = 3.0  #  Slow ball closer than this: keeper attacks and clears (m).
    keeper_challenge_exit_radius_m: float = 3.6  #  Ball farther than this ends the challenge (m).
    keeper_emergency_exit_radius_m: float = 2.8  #  Ball farther than this leaves EMERGENCY once slow (m).
    keeper_clear_range_m: float = 1.1  #  Ball within this of the keeper may be cleared instantly (m).

    # Ball velocity estimation and interception prediction.
    ball_velocity_min_span_sec: float = 0.15  #  Minimum sample pair spacing for a velocity estimate (s).
    ball_velocity_max_age_sec: float = 0.6  #  Older samples are discarded (s).
    ball_velocity_max_speed_mps: float = 6.0  #  Faster estimates are teleport artifacts, not real ball speed (m/s).
    ball_fast_closing_mps: float = 1.5  #  Closing speed above which the keeper blocks the line instead of challenging (m/s).
    ball_velocity_min_toward_goal_mps: float = 0.4  #  Minimum closing speed counted as a threat (m/s).
    keeper_prediction_horizon_sec: float = 1.2  #  Maximum ball look-ahead for the crossing point (s).

    # Own-goal safety: reject kicks whose line from the ball would enter this
    # half-width band at our goal line.
    own_goal_guard_half_width_m: float = 1.8  #  Guarded band half-width at our goal line (m).

    # Defensive cover for the secondary defender: goal-side of the ball on the
    # ball-to-goal line, offset off the pressure lane, held in a central band.
    defender_cover_depth_m: float = 1.8  #  Cover point distance goal-side of the ball (m).
    defender_cover_lateral_m: float = 1.0  #  Lateral offset away from the pressure lane (m).
    defender_center_half_width_m: float = 1.5  #  Central band the cover must stay within (m).

    # Attacking support offer: ahead-lateral of the ball so passes can exist.
    attack_support_ahead_m: float = 1.8  #  How far ahead of the ball the offer stands (m).
    attack_support_lateral_m: float = 1.4  #  Lateral offset from the ball (m).

    # Recovery: back off briefly when the chaser makes no progress near the
    # ball, instead of standing still into an incapable-robot penalty.
    recovery_stuck_sec: float = 6.0  #  Near-ball time with no meaningful displacement before backoff (s).
    recovery_stuck_displacement_m: float = 0.5  #  Displacement that counts as progress (m).
    recovery_backoff_sec: float = 2.5  #  Backoff window duration (s).
    recovery_cooldown_sec: float = 4.0  #  Cooldown before stuck detection re-arms (s).

    # Passing
    pass_enabled: bool = True  #  Master pass switch.
    pass_min_score: float = 0.52  #  Minimum pass-candidate score; below this, dribble instead.
    pass_min_forward_m: float = 0.35  #  Minimum forward progress for passes; discourages lateral/back passes.
    pass_lane_clearance: float = 0.75  #  Required clearance around the pass lane to avoid interception.

    # Dribbling
    dribble_advance_m: float = 1.15  #  Forward distance advanced by one dribble target.
    dribble_center_pull: float = 0.65  #  Pull toward centerline while dribbling to avoid hugging the sideline.

    # Support positioning
    support_depth_m: float = 1.05  #  Supporter depth behind the ball carrier.
    support_lateral_m: float = 1.25  #  Lateral spacing for supporters.
    support_min_spacing_m: float = 1.15  #  Minimum teammate spacing to avoid clustering.

    # Goalkeeping and challenges
    goalkeeper_challenge_margin_m: float = 0.70  #  Margin that triggers goalkeeper challenge.

    # Sideline and goal-line recovery
    sideline_recovery_margin_m: float = 0.90  #  Sideline distance threshold for recovery.
    sideline_recovery_infield_m: float = 1.60  #  Infield pull depth during recovery.
    sideline_recovery_advance_m: float = 0.75  #  Forward advance during recovery.
    goal_line_recovery_margin_m: float = 0.08  #  Goal-line recovery margin; small to prevent crossing the line.


@dataclass
class SoccerConfig:
    """Whole-team configuration.

    Fields are roughly split into two layers:

    "Environment identity": ``team_id`` and ``robot_names``. These are the only
    fields loaded by :meth:`from_env`.
    "Constructor/ROS parameter config": ``opponent_robot_names``, ``control_hz``,
    ``game_controller_topic``, field dimensions, initial ``ready_slots``,
    :class:`SoccerStrategyTuning`, and :class:`SoccerDebugConfig`. Change these
    by defaults, constructor arguments, or ROS debug parameters instead of public env vars.
    """

    team_id: int = 1
    robot_names: tuple[str, ...] = ("robot1", "robot2", "robot3")
    opponent_robot_names: tuple[str, ...] = ()
    ready_slots: dict[int, ReadySlot] = field(
        default_factory=lambda: {
            1: ReadySlot.CENTER,
            2: ReadySlot.SIDE,
            3: ReadySlot.KEEPER,
        }
    )
    control_hz: float = 30.0
    game_controller_topic: str = "/soccer/game_controller"
    field_length: float = ADULT_FIELD_DIMENSIONS.length
    field_width: float = ADULT_FIELD_DIMENSIONS.width
    penalty_dist: float = ADULT_FIELD_DIMENSIONS.penalty_dist
    goal_width: float = ADULT_FIELD_DIMENSIONS.goal_width
    center_circle_radius: float = ADULT_FIELD_DIMENSIONS.circle_radius
    penalty_area_length: float = ADULT_FIELD_DIMENSIONS.penalty_area_length
    penalty_area_width: float = ADULT_FIELD_DIMENSIONS.penalty_area_width
    goal_area_length: float = ADULT_FIELD_DIMENSIONS.goal_area_length
    goal_area_width: float = ADULT_FIELD_DIMENSIONS.goal_area_width
    strategy: SoccerStrategyTuning = field(default_factory=SoccerStrategyTuning)
    debug: SoccerDebugConfig = field(default_factory=SoccerDebugConfig)

    def __post_init__(self) -> None:
        if not self.opponent_robot_names:
            self.opponent_robot_names = _default_opponent_robot_names(self.team_id)
        self.ready_slots = _complete_ready_slots(
            self.ready_slots,
            player_count=len(self.robot_names),
        )

    @property
    def player_ids(self) -> tuple[int, ...]:
        return tuple(range(1, len(self.robot_names) + 1))

    def ready_slot_for_player(self, player_id: int) -> ReadySlot:
        return self.ready_slots.get(player_id, ReadySlot.SIDE)

    def goalkeeper_player_id(self) -> int | None:
        for player_id in self.player_ids:
            if self.ready_slot_for_player(player_id) == ReadySlot.KEEPER:
                return player_id
        return None

    def opponent_team_id(self) -> int:
        return 2 if self.team_id == 1 else 1

    def field_dimensions(self) -> FieldDimensions:
        return FieldDimensions(
            length=self.field_length,
            width=self.field_width,
            penalty_dist=self.penalty_dist,
            goal_width=self.goal_width,
            circle_radius=self.center_circle_radius,
            penalty_area_length=self.penalty_area_length,
            penalty_area_width=self.penalty_area_width,
            goal_area_length=self.goal_area_length,
            goal_area_width=self.goal_area_width,
        )

    @classmethod
    def from_env(cls, environ: Mapping[str, str] | None = None) -> "SoccerConfig":
        """Load per-match fields from environment variables; keep dataclass defaults for the rest.

        Public environment variables are limited to per-match identity:

        ``SOCCER_TEAM_ID``
        ``SOCCER_ROBOT_NAMES``

        Speed limits, kick hysteresis, and other tactic knobs live in
        :class:`SoccerStrategyTuning` and are no longer env vars; adjust defaults or pass custom tuning.
        """

        env = os.environ if environ is None else environ
        base = cls()
        team_id = _parse_int(env.get("SOCCER_TEAM_ID"), base.team_id)
        robot_names = _parse_robot_names(
            env.get("SOCCER_ROBOT_NAMES"),
            default=base.robot_names,
        )
        return cls(
            team_id=team_id,
            robot_names=robot_names,
        )


def _parse_robot_names(value: str | None, default: tuple[str, ...]) -> tuple[str, ...]:
    if value is None or not value.strip():
        return default
    names = tuple(_normalize_robot_name(item) for item in _split_csv(value))
    if not names:
        return default
    return names


def _normalize_robot_name(value: str) -> str:
    normalized = value.strip()
    if normalized.lower() in {"default", "<default>", "none", "null"}:
        return ""
    return normalized


def _split_csv(value: str) -> list[str]:
    return [item.strip() for item in value.split(",") if item.strip()]


def _parse_int(value: str | None, default: int) -> int:
    if value is None or not value.strip():
        return default
    return int(value)


def _default_opponent_robot_names(team_id: int) -> tuple[str, ...]:
    if team_id == 1:
        return ("robot4", "robot5", "robot6")
    return ("robot1", "robot2", "robot3")


def _complete_ready_slots(
    ready_slots: Mapping[int, ReadySlot],
    player_count: int,
) -> dict[int, ReadySlot]:
    completed: dict[int, ReadySlot] = {}
    for player_id in range(1, player_count + 1):
        default_slot = (
            DEFAULT_READY_SLOT_SEQUENCE[player_id - 1]
            if player_id <= len(DEFAULT_READY_SLOT_SEQUENCE)
            else ReadySlot.SIDE
        )
        completed[player_id] = ready_slots.get(player_id, default_slot)
    return completed
