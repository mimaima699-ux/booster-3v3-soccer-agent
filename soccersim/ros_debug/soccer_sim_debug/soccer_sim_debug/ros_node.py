"""Standalone ROS2 debug node entry equivalent to SoccerSimAgent."""

from __future__ import annotations

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from src.runtime import SoccerTeamRuntime
from src.soccer_framework import (
    ReadySlot,
    SoccerConfig,
    SoccerDebugConfig,
    SoccerStrategyTuning,
)
from src.soccer_framework.telemetry import SoccerLogger, create_soccer_logger


class SoccerSimNode(Node):
    """ROS2 node wrapper for the same team runtime used by SoccerSimAgent."""

    def __init__(self):
        super().__init__("soccer_sim_node")
        self.config = self._config_from_parameters()
        self.soccer_logger: SoccerLogger = create_soccer_logger(
            self.get_logger(),
            source=f"soccersim.debug.team{self.config.team_id}",
        )
        self._structured_logger_closed = False
        self.runtime = SoccerTeamRuntime(
            logger=self.soccer_logger,
            config=self.config,
        )
        self._log_structured_log_path()
        self.soccer_logger.info(
            "Soccer debug node initialized",
            event="debug_node_initialized",
            console=False,
            team_id=self.config.team_id,
            robot_names=list(self.config.robot_names),
            opponent_robot_names=list(self.config.opponent_robot_names),
            control_hz=self.config.control_hz,
        )
        try:
            self.runtime.start()
        except Exception:
            self.runtime.stop()
            self.soccer_logger.error(
                "Soccer debug node start failed",
                event="debug_node_start_failed",
                console=False,
                team_id=self.config.team_id,
            )
            self._close_structured_logger()
            raise

    def destroy_node(self) -> None:
        runtime = getattr(self, "runtime", None)
        if runtime is not None:
            runtime.stop()
        self._close_structured_logger()
        super().destroy_node()

    def _log_structured_log_path(self) -> None:
        path = self.soccer_logger.path
        if path is not None:
            self.get_logger().info(f"Soccer structured log: {path}")
        pretty_path = self.soccer_logger.pretty_path
        if pretty_path is not None:
            self.get_logger().info(f"Soccer structured pretty log: {pretty_path}")

    def _close_structured_logger(self) -> None:
        if self._structured_logger_closed:
            return
        self._structured_logger_closed = True
        self.soccer_logger.info(
            "Soccer debug node closed",
            event="debug_node_closed",
            console=False,
            team_id=self.config.team_id,
        )
        self.soccer_logger.close()

    def _config_from_parameters(self) -> SoccerConfig:
        base = SoccerConfig.from_env()
        self._declare_config_parameter("team_id", base.team_id)
        self._declare_config_parameter("robot_names", ",".join(base.robot_names))
        self._declare_config_parameter(
            "opponent_robot_names",
            ",".join(base.opponent_robot_names),
        )
        self._declare_config_parameter(
            "ready_slots",
            ",".join(
                base.ready_slot_for_player(player_id).value
                for player_id in base.player_ids
            ),
        )

        self._declare_config_parameter("control_hz", base.control_hz)
        self._declare_config_parameter(
            "game_controller_topic",
            base.game_controller_topic,
        )
        self._declare_config_parameter(
            "debug_bt_trace_ticks",
            base.debug.bt_trace_ticks,
        )
        self._declare_config_parameter(
            "debug_bt_trace_sample_sec",
            base.debug.bt_trace_sample_sec,
        )

        strategy = base.strategy
        self._declare_config_parameter("max_linear_speed", strategy.max_linear_speed)
        self._declare_config_parameter("max_angular_speed", strategy.max_angular_speed)
        self._declare_config_parameter(
            "soccer_kick_enter_distance",
            strategy.soccer_kick_enter_distance,
        )
        self._declare_config_parameter(
            "soccer_kick_exit_distance",
            strategy.soccer_kick_exit_distance,
        )
        self._declare_config_parameter("soccer_kick_power", strategy.soccer_kick_power)
        self._declare_config_parameter(
            "soccer_kick_min_active_sec",
            strategy.soccer_kick_min_active_sec,
        )
        self._declare_config_parameter(
            "soccer_kick_exit_delay_sec",
            strategy.soccer_kick_exit_delay_sec,
        )

        self._declare_config_parameter(
            "restart_touch_distance",
            strategy.restart_touch_distance,
        )
        self._declare_config_parameter(
            "opponent_restart_avoid_distance_m",
            strategy.opponent_restart_avoid_distance_m,
        )
        self._declare_config_parameter(
            "opponent_obstacle_radius",
            strategy.opponent_obstacle_radius,
        )
        self._declare_config_parameter(
            "teammate_obstacle_radius",
            strategy.teammate_obstacle_radius,
        )
        self._declare_config_parameter(
            "obstacle_safety_margin",
            strategy.obstacle_safety_margin,
        )
        self._declare_config_parameter(
            "obstacle_start_ignore_distance",
            strategy.obstacle_start_ignore_distance,
        )
        self._declare_config_parameter(
            "obstacle_target_ignore_distance",
            strategy.obstacle_target_ignore_distance,
        )
        self._declare_config_parameter("pass_enabled", strategy.pass_enabled)
        self._declare_config_parameter("pass_min_score", strategy.pass_min_score)
        self._declare_config_parameter(
            "pass_min_forward_m",
            strategy.pass_min_forward_m,
        )
        self._declare_config_parameter(
            "pass_lane_clearance",
            strategy.pass_lane_clearance,
        )
        self._declare_config_parameter("dribble_advance_m", strategy.dribble_advance_m)
        self._declare_config_parameter("dribble_center_pull", strategy.dribble_center_pull)
        self._declare_config_parameter("support_depth_m", strategy.support_depth_m)
        self._declare_config_parameter("support_lateral_m", strategy.support_lateral_m)
        self._declare_config_parameter(
            "support_min_spacing_m",
            strategy.support_min_spacing_m,
        )
        self._declare_config_parameter(
            "goalkeeper_challenge_margin_m",
            strategy.goalkeeper_challenge_margin_m,
        )
        self._declare_config_parameter(
            "sideline_recovery_margin_m",
            strategy.sideline_recovery_margin_m,
        )
        self._declare_config_parameter(
            "sideline_recovery_infield_m",
            strategy.sideline_recovery_infield_m,
        )
        self._declare_config_parameter(
            "sideline_recovery_advance_m",
            strategy.sideline_recovery_advance_m,
        )
        self._declare_config_parameter(
            "goal_line_recovery_margin_m",
            strategy.goal_line_recovery_margin_m,
        )
        self._declare_config_parameter(
            "teammate_challenge_tie_margin_m",
            strategy.teammate_challenge_tie_margin_m,
        )
        self._declare_config_parameter(
            "yaw_avoid_horizon_sec",
            strategy.yaw_avoid_horizon_sec,
        )
        self._declare_config_parameter(
            "yaw_avoid_min_distance_m",
            strategy.yaw_avoid_min_distance_m,
        )
        self._declare_config_parameter(
            "yaw_avoid_bias_max",
            strategy.yaw_avoid_bias_max,
        )

        robot_names = tuple(
            _normalize_robot_name(item) for item in self._parameter_list("robot_names")
        )
        if not robot_names:
            raise ValueError("robot_names must not be empty")
        opponent_robot_names = tuple(
            _normalize_robot_name(item)
            for item in self._parameter_list("opponent_robot_names")
        )
        ready_slots = _ready_slots_from_parameter(
            self._parameter_value("ready_slots"),
            player_count=len(robot_names),
        )

        return SoccerConfig(
            team_id=self._parameter_int("team_id"),
            robot_names=robot_names,
            opponent_robot_names=opponent_robot_names,
            ready_slots=ready_slots,
            control_hz=self._parameter_float("control_hz"),
            game_controller_topic=self._parameter_str("game_controller_topic"),
            debug=SoccerDebugConfig(
                bt_trace_ticks=self._parameter_str("debug_bt_trace_ticks"),
                bt_trace_sample_sec=self._parameter_float("debug_bt_trace_sample_sec"),
            ),
            strategy=SoccerStrategyTuning(
                max_linear_speed=self._parameter_float("max_linear_speed"),
                max_angular_speed=self._parameter_float("max_angular_speed"),
                soccer_kick_enter_distance=self._parameter_float(
                    "soccer_kick_enter_distance"
                ),
                soccer_kick_exit_distance=self._parameter_float(
                    "soccer_kick_exit_distance"
                ),
                soccer_kick_power=self._parameter_float("soccer_kick_power"),
                soccer_kick_min_active_sec=self._parameter_float(
                    "soccer_kick_min_active_sec"
                ),
                soccer_kick_exit_delay_sec=self._parameter_float(
                    "soccer_kick_exit_delay_sec"
                ),
                restart_touch_distance=self._parameter_float(
                    "restart_touch_distance"
                ),
                opponent_restart_avoid_distance_m=self._parameter_float(
                    "opponent_restart_avoid_distance_m"
                ),
                opponent_obstacle_radius=self._parameter_float(
                    "opponent_obstacle_radius"
                ),
                teammate_obstacle_radius=self._parameter_float(
                    "teammate_obstacle_radius"
                ),
                obstacle_safety_margin=self._parameter_float(
                    "obstacle_safety_margin"
                ),
                obstacle_start_ignore_distance=self._parameter_float(
                    "obstacle_start_ignore_distance"
                ),
                obstacle_target_ignore_distance=self._parameter_float(
                    "obstacle_target_ignore_distance"
                ),
                pass_enabled=self._parameter_bool("pass_enabled"),
                pass_min_score=self._parameter_float("pass_min_score"),
                pass_min_forward_m=self._parameter_float("pass_min_forward_m"),
                pass_lane_clearance=self._parameter_float("pass_lane_clearance"),
                dribble_advance_m=self._parameter_float("dribble_advance_m"),
                dribble_center_pull=self._parameter_float("dribble_center_pull"),
                support_depth_m=self._parameter_float("support_depth_m"),
                support_lateral_m=self._parameter_float("support_lateral_m"),
                support_min_spacing_m=self._parameter_float("support_min_spacing_m"),
                goalkeeper_challenge_margin_m=self._parameter_float(
                    "goalkeeper_challenge_margin_m"
                ),
                sideline_recovery_margin_m=self._parameter_float(
                    "sideline_recovery_margin_m"
                ),
                sideline_recovery_infield_m=self._parameter_float(
                    "sideline_recovery_infield_m"
                ),
                sideline_recovery_advance_m=self._parameter_float(
                    "sideline_recovery_advance_m"
                ),
                goal_line_recovery_margin_m=self._parameter_float(
                    "goal_line_recovery_margin_m"
                ),
                teammate_challenge_tie_margin_m=self._parameter_float(
                    "teammate_challenge_tie_margin_m"
                ),
                yaw_avoid_horizon_sec=self._parameter_float(
                    "yaw_avoid_horizon_sec"
                ),
                yaw_avoid_min_distance_m=self._parameter_float(
                    "yaw_avoid_min_distance_m"
                ),
                yaw_avoid_bias_max=self._parameter_float(
                    "yaw_avoid_bias_max"
                ),
            ),
        )

    def _declare_config_parameter(self, name: str, value: object) -> None:
        self.declare_parameter(
            name,
            value,
            descriptor=ParameterDescriptor(dynamic_typing=True),
        )

    def _parameter_value(self, name: str) -> object:
        value = self.get_parameter(name).value
        if value is None:
            raise ValueError(f"parameter {name!r} must not be unset")
        return value

    def _parameter_int(self, name: str) -> int:
        return int(self._parameter_value(name))

    def _parameter_float(self, name: str) -> float:
        return float(self._parameter_value(name))

    def _parameter_str(self, name: str) -> str:
        return str(self._parameter_value(name))

    def _parameter_bool(self, name: str) -> bool:
        return _parameter_bool(self._parameter_value(name))

    def _parameter_list(self, name: str) -> list[str]:
        return _parameter_list(self._parameter_value(name))


def _parameter_list(value: object) -> list[str]:
    if isinstance(value, str):
        return [item.strip() for item in value.split(",") if item.strip()]
    if isinstance(value, (list, tuple)):
        return [str(item).strip() for item in value if str(item).strip()]
    return [str(value).strip()] if str(value).strip() else []


def _parameter_bool(value: object) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, int):
        return value != 0
    normalized = str(value).strip().lower()
    if normalized in {"1", "true", "yes", "y", "on"}:
        return True
    if normalized in {"0", "false", "no", "n", "off"}:
        return False
    raise ValueError(f"invalid bool parameter value: {value!r}")


def _normalize_robot_name(value: str) -> str:
    normalized = value.strip()
    if normalized.lower() in {"default", "<default>", "none", "null"}:
        return ""
    return normalized


def _ready_slots_from_parameter(
    value: object,
    player_count: int,
) -> dict[int, ReadySlot]:
    slot_values = _parameter_list(value)
    if len(slot_values) != player_count:
        raise ValueError("ready_slots must have the same item count as robot_names")
    return {
        player_id: ReadySlot(slot_values[player_id - 1].strip().lower())
        for player_id in range(1, player_count + 1)
    }


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: SoccerSimNode | None = None
    try:
        node = SoccerSimNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
