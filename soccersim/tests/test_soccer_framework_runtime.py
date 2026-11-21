from __future__ import annotations

import sys
import unittest
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from types import ModuleType, SimpleNamespace


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


@dataclass
class FakeFallDownState:
    state: str = "normal"
    recoverable: bool = False

    @property
    def is_normal(self) -> bool:
        return self.state == "normal"

    @property
    def has_fallen(self) -> bool:
        return self.state == "fallen"


class FakeBoosterRobot:
    instances: list["FakeBoosterRobot"] = []

    def __init__(
        self,
        virtual_robot_name: str,
        enable_tf_listener: bool,
        timeout: float | None = None,
    ) -> None:
        self.virtual_robot_name = virtual_robot_name
        self.enable_tf_listener = enable_tf_listener
        self.timeout = timeout
        self.mode = "prepare"
        self.fall_down_state = FakeFallDownState()
        self.gait_calls: list[str] = []
        self.mode_calls: list[str] = []
        self.get_mode_calls = 0
        self.get_fall_down_state_calls = 0
        self.get_up_calls = 0
        self.velocity_calls: list[tuple[float, float, float]] = []
        self.velocity_errors: list[Exception] = []
        FakeBoosterRobot.instances.append(self)

    def list_gaits(self) -> list[str]:
        return ["robocup"]

    def get_mode(self) -> str:
        self.get_mode_calls += 1
        return self.mode

    def get_fall_down_state(self) -> FakeFallDownState:
        self.get_fall_down_state_calls += 1
        return self.fall_down_state

    def get_up(self) -> None:
        self.get_up_calls += 1
        self.fall_down_state = FakeFallDownState(
            state="getting_up",
            recoverable=False,
        )

    def set_gait(self, gait: str) -> None:
        self.gait_calls.append(gait)

    def set_mode(self, mode: str) -> None:
        self.mode_calls.append(mode)
        self.mode = mode

    def set_velocity(self, vx: float, vy: float, vyaw: float) -> None:
        if self.velocity_errors:
            raise self.velocity_errors.pop(0)
        self.velocity_calls.append((vx, vy, vyaw))

    def _close(self) -> None:
        return None


class FakeSoccerKickManager:
    instances: list["FakeSoccerKickManager"] = []

    def __init__(self, robot: FakeBoosterRobot) -> None:
        self.robot = robot
        self.started = False
        self.command_calls: list[tuple[float, float]] = []
        self.ball_calls: list[tuple[float, float]] = []
        FakeSoccerKickManager.instances.append(self)

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.started = False

    def update_command(self, direction: float, power: float) -> None:
        self.command_calls.append((direction, power))

    def update_ball(self, x: float, y: float) -> None:
        self.ball_calls.append((x, y))


class FakeRosPose2D:
    pass


class FakeRosString:
    def __init__(self) -> None:
        self.data = ""


class FakeQoSProfile:
    def __init__(
        self,
        history: object,
        depth: int,
        durability: object,
        reliability: object,
    ) -> None:
        self.history = history
        self.depth = depth
        self.durability = durability
        self.reliability = reliability


class FakeQoSHistoryPolicy:
    KEEP_LAST = "keep_last"


class FakeQoSDurabilityPolicy:
    VOLATILE = "volatile"


class FakeQoSReliabilityPolicy:
    RELIABLE = "reliable"


class FakeRclpyContext:
    def __init__(self) -> None:
        self.initialized = False
        self.shutdown_called = False
        self.init_args: object = None
        self.initialize_logging = True

    def init(self, args: object = None, initialize_logging: bool = False) -> None:
        self.init_args = args
        self.initialize_logging = initialize_logging
        self.initialized = True

    def shutdown(self) -> None:
        self.shutdown_called = True
        self.initialized = False


class FakeExternalShutdownException(Exception):
    pass


class FakeSingleThreadedExecutor:
    instances: list["FakeSingleThreadedExecutor"] = []

    def __init__(self, context: object) -> None:
        self.context = context
        self.nodes: list[object] = []
        self.shutdown_called = False
        FakeSingleThreadedExecutor.instances.append(self)

    def add_node(self, node: object) -> None:
        self.nodes.append(node)

    def spin(self) -> None:
        return None

    def shutdown(self) -> None:
        self.shutdown_called = True


def _install_runtime_dependency_fakes() -> None:
    boosteros_module = ModuleType("boosteros")
    robots_module = ModuleType("boosteros.robots")
    booster_module = ModuleType("boosteros.robots.booster")
    setattr(booster_module, "BoosterRobot", FakeBoosterRobot)
    setattr(booster_module, "RobotGaitName", str)
    setattr(booster_module, "RobotModeName", str)
    setattr(booster_module, "SoccerKickManager", FakeSoccerKickManager)
    setattr(robots_module, "booster", booster_module)
    setattr(boosteros_module, "robots", robots_module)
    sys.modules["boosteros"] = boosteros_module
    sys.modules["boosteros.robots"] = robots_module
    sys.modules["boosteros.robots.booster"] = booster_module

    geometry_module = ModuleType("geometry_msgs")
    geometry_msg_module = ModuleType("geometry_msgs.msg")
    setattr(geometry_msg_module, "Pose2D", FakeRosPose2D)
    setattr(geometry_module, "msg", geometry_msg_module)
    sys.modules["geometry_msgs"] = geometry_module
    sys.modules["geometry_msgs.msg"] = geometry_msg_module

    rclpy_module = ModuleType("rclpy")
    rclpy_node_module = ModuleType("rclpy.node")
    rclpy_executors_module = ModuleType("rclpy.executors")
    qos_module = ModuleType("rclpy.qos")
    default_context = FakeRclpyContext()
    setattr(rclpy_module, "get_default_context", lambda: default_context)
    setattr(
        rclpy_module,
        "ok",
        lambda context=None: (
            default_context.initialized
            if context is None
            else bool(getattr(context, "initialized", False))
        ),
    )
    setattr(rclpy_module, "create_node", lambda *args, **kwargs: object())
    setattr(rclpy_node_module, "Node", object)
    setattr(
        rclpy_executors_module,
        "ExternalShutdownException",
        FakeExternalShutdownException,
    )
    setattr(
        rclpy_executors_module,
        "SingleThreadedExecutor",
        FakeSingleThreadedExecutor,
    )
    setattr(qos_module, "QoSDurabilityPolicy", FakeQoSDurabilityPolicy)
    setattr(qos_module, "QoSHistoryPolicy", FakeQoSHistoryPolicy)
    setattr(qos_module, "QoSProfile", FakeQoSProfile)
    setattr(qos_module, "QoSReliabilityPolicy", FakeQoSReliabilityPolicy)
    setattr(rclpy_module, "qos", qos_module)
    sys.modules["rclpy"] = rclpy_module
    sys.modules["rclpy.node"] = rclpy_node_module
    sys.modules["rclpy.executors"] = rclpy_executors_module
    sys.modules["rclpy.qos"] = qos_module

    std_msgs_module = ModuleType("std_msgs")
    std_msgs_msg_module = ModuleType("std_msgs.msg")
    setattr(std_msgs_msg_module, "String", FakeRosString)
    setattr(std_msgs_module, "msg", std_msgs_msg_module)
    sys.modules["std_msgs"] = std_msgs_module
    sys.modules["std_msgs.msg"] = std_msgs_msg_module


_install_runtime_dependency_fakes()

from src.soccer_framework import (  # noqa: E402
    BallState,
    GameControlState,
    GameState,
    KickIntent,
    MoveIntent,
    NoopIntent,
    Penalty,
    PlayerState,
    Pose2D,
    RobotCommand,
    RobotRuntimeStatus,
    RobotState,
    ReadySlot,
    SoccerConfig,
    TeamState,
    PlayContext,
    PlayContextProvider,
)
from src.runtime import SoccerTeamRuntime  # noqa: E402
from src.soccer_framework.game_controller import (  # noqa: E402
    GameControllerRosProvider,
)
import src.soccer_framework.robot as robot_module  # noqa: E402
from src.soccer_framework.ros_adapter import SoccerRosAdapter  # noqa: E402
import src.soccer_framework.ros_adapter as ros_adapter_module  # noqa: E402
from src.soccer_framework.robot import TeamRobotManager  # noqa: E402
from src.soccer_framework.ros_truth import RosTruthProvider  # noqa: E402

ros_adapter_module.rclpy = sys.modules["rclpy"]
ros_adapter_module.ExternalShutdownException = FakeExternalShutdownException
ros_adapter_module.SingleThreadedExecutor = FakeSingleThreadedExecutor


@dataclass
class FakeSubscription:
    msg_type: object
    topic: str
    callback: Callable[[object], None]
    qos: object


class FakeNode:
    def __init__(self) -> None:
        self.subscriptions: list[FakeSubscription] = []
        self.destroyed = False

    def create_subscription(
        self,
        msg_type: object,
        topic: str,
        callback: Callable[[object], None],
        qos: object,
    ) -> FakeSubscription:
        subscription = FakeSubscription(msg_type, topic, callback, qos)
        self.subscriptions.append(subscription)
        return subscription

    def destroy_subscription(self, subscription: FakeSubscription) -> None:
        if subscription in self.subscriptions:
            self.subscriptions.remove(subscription)

    def destroy_node(self) -> None:
        self.destroyed = True


class FakeLogger:
    def __init__(self) -> None:
        self.messages: list[str] = []
        self.events: list[dict[str, object]] = []

    def info(self, message: str, **fields: object) -> None:
        self.messages.append(message)
        event = fields.get("event")
        if isinstance(event, str):
            self.events.append({"level": "INFO", "message": message, **fields})

    def warn(self, message: str, **fields: object) -> None:
        self.messages.append(f"WARN: {message}")
        event = fields.get("event")
        if isinstance(event, str):
            self.events.append({"level": "WARN", "message": message, **fields})


class FakePlayContextProvider(PlayContextProvider):
    def __init__(self) -> None:
        self.context = PlayContext()

    def start(self) -> None:
        return None

    def stop(self) -> None:
        return None

    def get_snapshot(self) -> PlayContext:
        return self.context

    def set_game_state(self, game_state: GameControlState) -> None:
        self.context.game_state = game_state


class SoccerRuntimeTests(unittest.TestCase):
    def setUp(self) -> None:
        FakeBoosterRobot.instances.clear()
        FakeSoccerKickManager.instances.clear()
        FakeSingleThreadedExecutor.instances.clear()

    def test_runtime_creates_internal_ros_node_after_team_robot_manager(self) -> None:
        order: list[str] = []
        original_manager = robot_module.TeamRobotManager
        original_create_node = ros_adapter_module.rclpy.create_node

        class RecordingTeamRobotManager(TeamRobotManager):
            def __init__(self, config: SoccerConfig, logger: object) -> None:
                order.append("team_robot_manager")
                super().__init__(config, logger)

        def create_node(name: str, context: FakeRclpyContext) -> FakeNode:
            self.assertEqual(name, "sim_bridge")
            self.assertIsInstance(context, FakeRclpyContext)
            order.append("ros_node")
            return FakeNode()

        robot_module.TeamRobotManager = RecordingTeamRobotManager
        ros_adapter_module.rclpy.create_node = create_node
        runtime: SoccerTeamRuntime | None = None
        try:
            runtime = SoccerTeamRuntime(
                logger=FakeLogger(),
                config=SoccerConfig(
                    robot_names=("robot1",),
                    ready_slots={1: ReadySlot.CENTER},
                ),
            )
            self.assertEqual(order, ["team_robot_manager", "ros_node"])
            self.assertIsInstance(runtime.ros_adapter.node, FakeNode)
        finally:
            robot_module.TeamRobotManager = original_manager
            ros_adapter_module.rclpy.create_node = original_create_node
            if runtime is not None:
                runtime.stop()

    def test_runtime_start_initializes_robot_manager_before_ros_adapter(self) -> None:
        order: list[str] = []
        runtime = SoccerTeamRuntime(
            logger=FakeLogger(),
            config=SoccerConfig(
                robot_names=("robot1",),
                ready_slots={1: ReadySlot.CENTER},
            ),
        )

        runtime.robot_manager.start = (  # type: ignore[method-assign]
            lambda: order.append("robot_manager")
        )
        runtime.ros_adapter.start = (  # type: ignore[method-assign]
            lambda: order.append("ros_adapter")
        )
        runtime.control_loop = lambda: None  # type: ignore[method-assign]

        runtime.start()

        self.assertEqual(order, ["robot_manager", "ros_adapter"])
        runtime.stop()

    def test_framework_package_exposes_public_data_and_adapter_interfaces(
        self,
    ) -> None:
        config = SoccerConfig(robot_names=("robot1",))
        command = RobotCommand(
            intent=MoveIntent(vx=0.1),
            reason="example move",
        )
        context = PlayContext()

        self.assertEqual(config.robot_names, ("robot1",))
        self.assertIsInstance(command.intent, MoveIntent)
        self.assertIsInstance(context, PlayContext)

    def test_ground_truth_topics_and_callbacks_use_field_coordinates(self) -> None:
        node = FakeNode()
        logger = FakeLogger()
        provider = RosTruthProvider(
            node=node,
            config=SoccerConfig(
                team_id=2,
                robot_names=("robot4", "robot5"),
                ready_slots={1: ReadySlot.CENTER, 2: ReadySlot.KEEPER},
            ),
            logger=logger,
        )

        provider.start()

        self.assertEqual(
            [subscription.topic for subscription in node.subscriptions],
            [
                "/team2/robot4/soccer/sim/ground_truth/robot_pose",
                "/team2/robot5/soccer/sim/ground_truth/robot_pose",
                "/team2/robot1/soccer/sim/ground_truth/robot_pose",
                "/team2/robot2/soccer/sim/ground_truth/robot_pose",
                "/team2/robot3/soccer/sim/ground_truth/robot_pose",
                "/team2/soccer/sim/ground_truth/ball",
            ],
        )
        self.assertEqual(
            logger.messages,
            [
                "Truth provider team_id=2 topic_prefix=/team2 "
                "members=[p1:robot4/center, p2:robot5/keeper]",
                "Truth subscription p1 robot=robot4 ready_slot=center "
                "pose_topic=/team2/robot4/soccer/sim/ground_truth/robot_pose",
                "Truth subscription p2 robot=robot5 ready_slot=keeper "
                "pose_topic=/team2/robot5/soccer/sim/ground_truth/robot_pose",
                "Opponent truth subscription p1 robot=robot1 "
                "pose_topic=/team2/robot1/soccer/sim/ground_truth/robot_pose",
                "Opponent truth subscription p2 robot=robot2 "
                "pose_topic=/team2/robot2/soccer/sim/ground_truth/robot_pose",
                "Opponent truth subscription p3 robot=robot3 "
                "pose_topic=/team2/robot3/soccer/sim/ground_truth/robot_pose",
                "Truth subscription team_id=2 "
                "ball_topic=/team2/soccer/sim/ground_truth/ball",
            ],
        )

        node.subscriptions[0].callback(SimpleNamespace(x=1.0, y=2.0, theta=0.25))
        node.subscriptions[2].callback(SimpleNamespace(x=-1.0, y=-2.0, theta=0.0))
        node.subscriptions[5].callback(SimpleNamespace(x=3.5, y=-0.5, theta=0.0))

        context = provider.get_snapshot()
        robot = context.teammates[1]
        robot_pose = robot.pose
        self.assertIsNotNone(robot_pose)
        assert robot_pose is not None
        self.assertAlmostEqual(robot_pose.x, 1.0)
        self.assertAlmostEqual(robot_pose.y, 2.0)
        self.assertAlmostEqual(robot_pose.theta, 0.25)

        opponent_pose = context.opponents[1].pose
        self.assertIsNotNone(opponent_pose)
        assert opponent_pose is not None
        self.assertAlmostEqual(opponent_pose.x, -1.0)
        self.assertAlmostEqual(opponent_pose.y, -2.0)

        ball = context.ball
        self.assertIsNotNone(ball)
        assert ball is not None
        self.assertAlmostEqual(ball.x, 3.5)
        self.assertAlmostEqual(ball.y, -0.5)

    def test_game_controller_provider_last_topic_at_tracks_reception(
        self,
    ) -> None:
        states: list[GameControlState] = []
        provider = GameControllerRosProvider(
            node=FakeNode(),
            config=SoccerConfig(),
            logger=FakeLogger(),
            on_state=states.append,
        )

        self.assertEqual(provider.last_topic_at, 0.0)
        self.assertEqual(states, [])

    def test_ros_adapter_starts_ros_providers_and_internal_executor(self) -> None:
        node = FakeNode()
        original_create_node = ros_adapter_module.rclpy.create_node

        def create_node(name: str, context: FakeRclpyContext) -> FakeNode:
            self.assertEqual(name, "sim_bridge")
            self.assertIsInstance(context, FakeRclpyContext)
            return node

        ros_adapter_module.rclpy.create_node = create_node
        ros_adapter: SoccerRosAdapter | None = None
        try:
            ros_adapter = SoccerRosAdapter(
                config=SoccerConfig(
                    robot_names=("robot1",),
                    ready_slots={1: ReadySlot.CENTER},
                ),
                logger=FakeLogger(),
            )

            ros_adapter.start()

            self.assertEqual(ros_adapter.node, node)
            self.assertEqual(
                [subscription.topic for subscription in node.subscriptions],
                [
                    "/team1/robot1/soccer/sim/ground_truth/robot_pose",
                    "/team1/robot4/soccer/sim/ground_truth/robot_pose",
                    "/team1/robot5/soccer/sim/ground_truth/robot_pose",
                    "/team1/robot6/soccer/sim/ground_truth/robot_pose",
                    "/team1/soccer/sim/ground_truth/ball",
                    "/soccer/game_controller",
                ],
            )
            self.assertEqual(len(FakeSingleThreadedExecutor.instances), 1)
            self.assertEqual(FakeSingleThreadedExecutor.instances[0].nodes, [node])
        finally:
            ros_adapter_module.rclpy.create_node = original_create_node
            if ros_adapter is not None:
                ros_adapter.stop()

        self.assertEqual(node.subscriptions, [])
        self.assertTrue(node.destroyed)

    def test_control_once_stops_when_game_state_is_stale(self) -> None:
        runtime = SoccerTeamRuntime(
            logger=FakeLogger(),
            config=SoccerConfig(
                robot_names=("robot1",),
                ready_slots={1: ReadySlot.CENTER},
            ),
        )
        runtime.robot_manager.start()
        robot = FakeBoosterRobot.instances[0]
        robot.mode = "walk"
        robot.velocity_calls.clear()
        robot.get_mode_calls = 0
        # Build a stale game_state: last_seen_at=7.0, current now=10.0, which
        # exceeds the 2.0 s freshness threshold.
        gs = GameControlState(state=GameState.PLAYING)
        gs.last_seen_at = 7.0
        runtime.ros_adapter.context_provider.set_game_state(gs)

        runtime.tree.tick(now=10.0, executor=runtime)

        self.assertEqual(robot.velocity_calls, [(0.0, 0.0, 0.0)])
        self.assertEqual(robot.get_mode_calls, 1)

    def test_control_once_skips_stop_velocity_when_non_walk_in_terminal_states(
        self,
    ) -> None:
        for game_state in (None, GameControlState(state=GameState.FINISHED)):
            with self.subTest(game_state=game_state.state if game_state else "NONE"):
                logger = FakeLogger()
                runtime = SoccerTeamRuntime(
                    logger=logger,
                    config=SoccerConfig(
                        robot_names=("robot1",),
                        ready_slots={1: ReadySlot.CENTER},
                    ),
                )
                runtime.robot_manager.start()
                robot = FakeBoosterRobot.instances[-1]
                robot.mode = "prepare"
                robot.velocity_calls.clear()
                robot.velocity_errors.append(RuntimeError("API call failed, code = 400"))
                robot.get_mode_calls = 0
                if game_state is not None:
                    runtime.ros_adapter.context_provider.set_game_state(game_state)

                runtime.tree.tick(now=10.0, executor=runtime)

                self.assertEqual(robot.get_mode_calls, 1)
                self.assertIsInstance(
                    runtime.tree.last_executed_commands[1].intent,
                    NoopIntent,
                )
                self.assertEqual(robot.velocity_calls, [])
                self.assertFalse(
                    any("set_velocity failed" in message for message in logger.messages)
                )

    def test_control_once_runs_strategy_and_logs_commands(self) -> None:
        logger = FakeLogger()
        runtime = SoccerTeamRuntime(logger=logger)

        runtime.tree.tick(now=10.0, executor=runtime)
        runtime._log_commands(
            10.0, runtime.tree.last_context, runtime.tree.last_executed_commands,
        )

        self.assertTrue(
            any(
                message.startswith("Soccer control state=NONE")
                for message in logger.messages
            )
        )

    def test_manager_applies_active_velocity_without_status_polling(self) -> None:
        manager = TeamRobotManager(
            config=SoccerConfig(
                robot_names=("robot1",),
                ready_slots={1: ReadySlot.CENTER},
            ),
            logger=FakeLogger(),
        )
        manager.start()
        robot = FakeBoosterRobot.instances[0]
        robot.mode = "prepare"
        robot.mode_calls.clear()
        robot.get_mode_calls = 0
        robot.get_fall_down_state_calls = 0

        manager.apply_command(
            1,
            RobotCommand(
                intent=MoveIntent(vx=0.2, vy=0.0, vyaw=0.1),
                reason="center approach ball",
            ),
        )

        self.assertEqual(robot.get_mode_calls, 0)
        self.assertEqual(robot.get_fall_down_state_calls, 0)
        self.assertEqual(robot.mode_calls, [])
        self.assertEqual(robot.velocity_calls[-1], (0.2, 0.0, 0.1))

    def test_manager_start_does_not_enter_soccer_mode(self) -> None:
        manager = TeamRobotManager(
            config=SoccerConfig(
                robot_names=("robot1",),
                ready_slots={1: ReadySlot.CENTER},
            ),
            logger=FakeLogger(),
        )

        manager.start()

        robot = FakeBoosterRobot.instances[0]
        self.assertEqual(robot.gait_calls, [])
        self.assertEqual(robot.mode_calls, [])
        self.assertEqual(robot.mode, "prepare")

    def test_manager_sends_kick_command(self) -> None:
        manager = TeamRobotManager(
            config=SoccerConfig(
                robot_names=("robot1",),
                ready_slots={1: ReadySlot.CENTER},
            ),
            logger=FakeLogger(),
        )
        manager.start()

        manager.apply_command(
            1,
            RobotCommand(
                intent=KickIntent(
                    direction=0.25,
                    power=2.5,
                    ball_x=1.2,
                    ball_y=-0.3,
                ),
                reason="test kick",
            ),
        )

        kick_manager = FakeSoccerKickManager.instances[0]
        self.assertEqual(kick_manager.command_calls, [(0.25, 2.5)])
        self.assertEqual(kick_manager.ball_calls, [(1.2, -0.3)])

    def test_manager_clamps_kick_power_to_minimum(self) -> None:
        manager = TeamRobotManager(
            config=SoccerConfig(
                robot_names=("robot1",),
                ready_slots={1: ReadySlot.CENTER},
            ),
            logger=FakeLogger(),
        )
        manager.start()

        manager.apply_command(
            1,
            RobotCommand(
                intent=KickIntent(
                    direction=-0.1,
                    power=0.4,
                    ball_x=0.8,
                    ball_y=0.15,
                ),
                reason="test clamped kick",
            ),
        )

        kick_manager = FakeSoccerKickManager.instances[0]
        self.assertEqual(kick_manager.command_calls, [(-0.1, 1.0)])
        self.assertEqual(kick_manager.ball_calls, [(0.8, 0.15)])

    def test_manager_does_not_recover_walk_for_penalized_stop(self) -> None:
        manager = TeamRobotManager(
            config=SoccerConfig(
                robot_names=("robot1",),
                ready_slots={1: ReadySlot.CENTER},
            ),
            logger=FakeLogger(),
        )
        manager.start()
        robot = FakeBoosterRobot.instances[0]
        robot.mode = "walk"
        robot.velocity_calls.clear()
        robot.mode_calls.clear()
        robot.get_mode_calls = 0

        manager.apply_command(1, RobotCommand.stop("inactive"))

        self.assertEqual(robot.get_mode_calls, 0)
        self.assertEqual(robot.mode_calls, [])
        self.assertEqual(robot.velocity_calls[-1], (0.0, 0.0, 0.0))

    def test_manager_repeats_stop_velocity(self) -> None:
        manager = TeamRobotManager(
            config=SoccerConfig(
                robot_names=("robot1",),
                ready_slots={1: ReadySlot.CENTER},
            ),
            logger=FakeLogger(),
        )
        manager.start()
        robot = FakeBoosterRobot.instances[0]
        robot.mode = "walk"
        robot.velocity_calls.clear()

        manager.apply_command(1, RobotCommand.stop("inactive"))
        manager.apply_command(1, RobotCommand.stop("inactive"))

        self.assertEqual(
            robot.velocity_calls,
            [(0.0, 0.0, 0.0), (0.0, 0.0, 0.0)],
        )

    def test_manager_stop_all_skips_non_walk_velocity(self) -> None:
        manager = TeamRobotManager(
            config=SoccerConfig(
                robot_names=("robot1",),
                ready_slots={1: ReadySlot.CENTER},
            ),
            logger=FakeLogger(),
        )
        manager.start()
        robot = FakeBoosterRobot.instances[0]
        robot.mode = "prepare"
        robot.velocity_calls.clear()
        robot.velocity_errors.append(RuntimeError("API call failed, code = 400"))
        robot.get_mode_calls = 0

        manager.stop_all("test stop all")

        self.assertEqual(robot.get_mode_calls, 1)
        self.assertEqual(robot.velocity_calls, [])

    def test_manager_retries_failed_stop_velocity(self) -> None:
        logger = FakeLogger()
        manager = TeamRobotManager(
            config=SoccerConfig(
                robot_names=("robot1",),
                ready_slots={1: ReadySlot.CENTER},
            ),
            logger=logger,
        )
        manager.start()
        robot = FakeBoosterRobot.instances[0]
        robot.mode = "walk"
        robot.velocity_errors.append(RuntimeError("API call failed, code = 400"))
        robot.mode_calls.clear()
        robot.get_mode_calls = 0

        manager.apply_command(1, RobotCommand.stop("inactive"))
        manager.apply_command(1, RobotCommand.stop("inactive"))
        manager.apply_command(1, RobotCommand.stop("runtime closing"))

        warnings = [
            message for message in logger.messages if "set_velocity failed" in message
        ]
        self.assertEqual(
            warnings,
            [
                "WARN: set_velocity failed for player 1: "
                "RuntimeError: API call failed, code = 400"
            ],
        )
        self.assertEqual(
            robot.velocity_calls,
            [(0.0, 0.0, 0.0), (0.0, 0.0, 0.0)],
        )
        self.assertEqual(robot.get_mode_calls, 0)
        self.assertEqual(robot.mode_calls, [])

    def test_manager_polls_and_returns_runtime_status(self) -> None:
        config = SoccerConfig(
            robot_names=("robot1",),
            ready_slots={1: ReadySlot.CENTER},
        )
        manager = TeamRobotManager(config=config, logger=FakeLogger())
        manager.start()
        robot = FakeBoosterRobot.instances[0]
        robot.mode = "walk"
        robot.fall_down_state = FakeFallDownState(
            state="normal",
            recoverable=False,
        )
        robot.get_mode_calls = 0
        robot.get_fall_down_state_calls = 0

        status = manager.poll_runtime_status(1, now=10.0)

        self.assertEqual(status.mode, "walk")
        self.assertEqual(status.fall_down_state, "normal")
        self.assertFalse(status.fall_down_recoverable)
        self.assertEqual(status.updated_at, 10.0)
        self.assertEqual(robot.get_mode_calls, 1)
        self.assertEqual(robot.get_fall_down_state_calls, 0)

    def test_manager_polls_mode_every_tick_and_skips_walk_fall_down(self) -> None:
        config = SoccerConfig(
            robot_names=("robot1",),
            ready_slots={1: ReadySlot.CENTER},
        )
        manager = TeamRobotManager(config=config, logger=FakeLogger())
        manager.start()
        robot = FakeBoosterRobot.instances[0]
        robot.mode = "walk"
        robot.get_mode_calls = 0
        robot.get_fall_down_state_calls = 0

        manager.poll_runtime_status(1, now=10.0)
        manager.poll_runtime_status(1, now=10.05)

        self.assertEqual(robot.get_mode_calls, 2)
        self.assertEqual(robot.get_fall_down_state_calls, 0)

    def test_manager_polls_fall_down_only_outside_walk(self) -> None:
        config = SoccerConfig(
            robot_names=("robot1",),
            ready_slots={1: ReadySlot.CENTER},
        )
        manager = TeamRobotManager(config=config, logger=FakeLogger())
        manager.start()
        robot = FakeBoosterRobot.instances[0]
        robot.mode = "prepare"
        robot.fall_down_state = FakeFallDownState(
            state="fallen",
            recoverable=True,
        )
        robot.get_mode_calls = 0
        robot.get_fall_down_state_calls = 0

        status = manager.poll_runtime_status(1, now=10.0)

        self.assertEqual(status.mode, "prepare")
        self.assertEqual(status.fall_down_state, "fallen")
        self.assertTrue(status.fall_down_recoverable)
        self.assertEqual(robot.get_mode_calls, 1)
        self.assertEqual(robot.get_fall_down_state_calls, 1)

    def test_manager_trigger_get_up_throttles_retries(self) -> None:
        config = SoccerConfig(
            robot_names=("robot1",),
            ready_slots={1: ReadySlot.CENTER},
        )
        manager = TeamRobotManager(config=config, logger=FakeLogger())
        manager.start()
        robot = FakeBoosterRobot.instances[0]

        first = manager.trigger_get_up(1, now=10.0)
        # Calls within the same second should be throttled
        # (``_GET_UP_RETRY_INTERVAL_SEC`` = 1.0).
        second = manager.trigger_get_up(1, now=10.5)
        third = manager.trigger_get_up(1, now=11.5)

        self.assertTrue(first)
        self.assertFalse(second)
        self.assertTrue(third)
        self.assertEqual(robot.get_up_calls, 2)

    def test_manager_ensure_walk_mode_sets_mode_and_recheck(self) -> None:
        config = SoccerConfig(
            robot_names=("robot1",),
            ready_slots={1: ReadySlot.CENTER},
        )
        manager = TeamRobotManager(config=config, logger=FakeLogger())
        manager.start()
        robot = FakeBoosterRobot.instances[0]
        robot.mode = "prepare"
        robot.mode_calls.clear()
        robot.get_mode_calls = 0

        manager.ensure_walk_mode(1, "test recovery")

        self.assertEqual(robot.mode_calls, ["walk"])
        # Optimistically update to walk so this BT tick does not retrigger;
        # the next poll will confirm again.
        cached = manager.poll_runtime_status(1, now=20.0)
        self.assertEqual(cached.mode, robot.mode)  # The robot has switched to walk too.

    def test_status_monitor_gets_up_and_blocks_command_via_tree(self) -> None:
        """Integration: SafetyOverrides triggers get-up and blocks commands."""

        runtime = SoccerTeamRuntime(
            logger=FakeLogger(),
            config=SoccerConfig(
                robot_names=("robot1",),
                ready_slots={1: ReadySlot.CENTER},
            ),
        )
        runtime.robot_manager.start()
        robot = FakeBoosterRobot.instances[0]
        robot.mode = "prepare"
        robot.fall_down_state = FakeFallDownState(
            state="fallen",
            recoverable=True,
        )
        robot.velocity_calls.clear()
        runtime.ros_adapter.context_provider.set_game_state(
            GameControlState(state=GameState.PLAYING),
        )
        runtime.ros_adapter.game_controller.last_topic_at = 9.5

        runtime.tree.tick(now=10.0, executor=runtime)

        # When fallen, BT should trigger get_up once; in prepare mode it should
        # not call the velocity API.
        self.assertEqual(robot.get_up_calls, 1)
        self.assertIsInstance(
            runtime.tree.last_executed_commands[1].intent,
            NoopIntent,
        )
        self.assertEqual(robot.velocity_calls, [])

    def test_status_monitor_blocks_penalized_player_via_tree(self) -> None:
        """Integration: SafetyOverrides blocks penalized players independently."""

        runtime = SoccerTeamRuntime(
            logger=FakeLogger(),
            config=SoccerConfig(
                robot_names=("robot1",),
                ready_slots={1: ReadySlot.CENTER},
            ),
        )
        runtime.robot_manager.start()
        robot = FakeBoosterRobot.instances[0]
        robot.mode = "walk"
        robot.velocity_calls.clear()
        runtime.ros_adapter.game_controller.last_topic_at = 9.5

        runtime.ros_adapter.context_provider.set_game_state(
            GameControlState(state=GameState.PLAYING),
        )
        snapshot = runtime.ros_adapter.context_provider.get_snapshot()
        snapshot.teammates[1] = RobotState(
            player_id=1,
            pose=Pose2D(x=-1.0, y=0.0, theta=0.0),
            last_seen_at=10.0,
        )
        snapshot.ball = BallState(x=0.0, y=0.0, last_seen_at=10.0, confidence=1.0)
        snapshot.game_state = GameControlState(
            state=GameState.PLAYING,
            teams=[
                TeamState(
                    team_number=1,
                    goalkeeper=1,
                    players=[
                        PlayerState(
                            penalty=Penalty.SENT_OFF,
                            secs_till_unpenalised=0,
                        )
                    ],
                ),
                TeamState(team_number=2, goalkeeper=1, players=[]),
            ],
        )
        runtime.ros_adapter.context_provider.get_snapshot = lambda: snapshot  # type: ignore[method-assign]

        runtime.tree.tick(now=10.0, executor=runtime)

        self.assertEqual(robot.velocity_calls[-1], (0.0, 0.0, 0.0))

    def test_status_monitor_recovers_walk_in_ready_via_tree(self) -> None:
        """SafetyOverrides recovers walk mode for non-stop READY commands."""

        runtime = SoccerTeamRuntime(
            logger=FakeLogger(),
            config=SoccerConfig(
                robot_names=("robot1",),
                ready_slots={1: ReadySlot.CENTER},
            ),
        )
        runtime.robot_manager.start()
        robot = FakeBoosterRobot.instances[0]
        robot.mode = "prepare"
        robot.mode_calls.clear()
        robot.get_mode_calls = 0

        snapshot = runtime.ros_adapter.context_provider.get_snapshot()
        snapshot.game_state = GameControlState(state=GameState.READY)
        snapshot.teammates[1] = RobotState(
            player_id=1,
            pose=Pose2D(x=2.0, y=2.0, theta=0.0),
            last_seen_at=10.0,
        )
        runtime.ros_adapter.context_provider.get_snapshot = lambda: snapshot  # type: ignore[method-assign]

        runtime.tree.tick(now=10.0, executor=runtime)

        self.assertEqual(robot.get_mode_calls, 1)
        self.assertIn("walk", robot.mode_calls)

    def test_status_monitor_does_not_recover_walk_in_set_via_tree(self) -> None:
        """Non READY/PLAYING branches only produce stop and do not recover walk."""

        runtime = SoccerTeamRuntime(
            logger=FakeLogger(),
            config=SoccerConfig(
                robot_names=("robot1",),
                ready_slots={1: ReadySlot.CENTER},
            ),
        )
        runtime.robot_manager.start()
        robot = FakeBoosterRobot.instances[0]
        robot.mode = "prepare"
        robot.velocity_calls.clear()
        robot.mode_calls.clear()
        robot.get_mode_calls = 0

        snapshot = runtime.ros_adapter.context_provider.get_snapshot()
        snapshot.game_state = GameControlState(state=GameState.SET)
        snapshot.teammates[1] = RobotState(
            player_id=1,
            pose=Pose2D(x=2.0, y=2.0, theta=0.0),
            last_seen_at=10.0,
        )
        runtime.ros_adapter.context_provider.get_snapshot = lambda: snapshot  # type: ignore[method-assign]

        runtime.tree.tick(now=10.0, executor=runtime)

        self.assertEqual(robot.get_mode_calls, 1)
        self.assertEqual(robot.mode_calls, [])
        self.assertIsInstance(
            runtime.tree.last_executed_commands[1].intent,
            NoopIntent,
        )
        self.assertEqual(robot.velocity_calls, [])

    def test_status_monitor_recovers_walk_via_tree(self) -> None:
        """Integration: non-walk mode plus a non-stop BT command recovers walk.

        Build PLAYING plus a known ball so the center slot emits a non-stop
        ApproachBall command; only then should SafetyOverrides call
        ensure_walk_mode.
        """

        runtime = SoccerTeamRuntime(
            logger=FakeLogger(),
            config=SoccerConfig(
                robot_names=("robot1",),
                ready_slots={1: ReadySlot.CENTER},
            ),
        )
        runtime.robot_manager.start()
        robot = FakeBoosterRobot.instances[0]
        robot.mode = "prepare"
        robot.mode_calls.clear()

        # Feed PLAYING state, ball, and robot pose so BT enters the PLAYING
        # branch.
        runtime.ros_adapter.context_provider.set_game_state(
            GameControlState(state=GameState.PLAYING),
        )
        snapshot = runtime.ros_adapter.context_provider.get_snapshot()
        snapshot.teammates[1] = RobotState(
            player_id=1,
            pose=Pose2D(x=-1.0, y=0.0, theta=0.0),
            last_seen_at=10.0,
        )
        snapshot.ball = BallState(x=0.0, y=0.0, last_seen_at=10.0, confidence=1.0)
        runtime.ros_adapter.context_provider.get_snapshot = lambda: snapshot  # type: ignore[method-assign]
        runtime.ros_adapter.game_controller.last_topic_at = 9.5
        # Poll once so the manager cache sees prepare mode.
        runtime.robot_manager.poll_runtime_status(1, now=10.0)

        runtime.tree.tick(now=10.5, executor=runtime)

        self.assertIn("walk", robot.mode_calls)


if __name__ == "__main__":
    unittest.main()
