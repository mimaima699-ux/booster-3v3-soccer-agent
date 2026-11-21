"""Behavior-level SoccerSim tests.

Design principles:
- **Black box**: feed :class:`PlayContext` plus ``now`` and assert the
  resulting :class:`RobotCommand`. Do not poke ``_xxx`` private methods or
  depend on concrete behavior-tree node names.
- **Readable**: each test states one behavior-level meaning so competitors can
  follow the suite.
- **Isolated**: data type, configuration, parsing, and strategy behavior tests
  live in separate TestCase classes.

If the strategy internals change again, this suite should only need input or
assertion updates, not structural rewrites.
"""

from __future__ import annotations

import ast
import sys
import time
import unittest
from dataclasses import fields
from pathlib import Path

# Allow offline pytest runs to import py_trees.
ROS_SITE = "/opt/ros/humble/lib/python3.10/site-packages"
if ROS_SITE not in sys.path:
    sys.path.append(ROS_SITE)

import types as _types

_pydot_stub = _types.ModuleType("pydot")
_pydot_stub.Dot = type("Dot", (), {})  # type: ignore[attr-defined]
sys.modules.setdefault("pydot", _pydot_stub)

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from src.soccer_framework import (  # noqa: E402
    BallState,
    GameControlState,
    GamePhase,
    GameState,
    KickIntent,
    MoveIntent,
    Penalty,
    PlayerState,
    Pose2D,
    RobotCommand,
    RobotState,
    ReadySlot,
    SetPlay,
    SoccerConfig,
    SoccerDebugConfig,
    SoccerStrategyTuning,
    StopIntent,
    TeamState,
    PlayContext,
    PlayContextProvider,
)
from src.soccer_framework.types import (  # noqa: E402
    KICKING_TEAM_NONE,
    MAX_NUM_PLAYERS,
)
from src.soccer_framework.game_state import (  # noqa: E402
    game_control_state_from_json,
    game_control_state_to_json,
)
from src.runtime import SoccerKit  # noqa: E402
from src.behavior_tree import TeamStrategyTree  # noqa: E402
from src.tactics.kick_hysteresis import KickHysteresis  # noqa: E402
from src.play import DefaultPlaybook, PLAYBOOKS, RoleAssignment  # noqa: E402


# --- Factory helpers ------------------------------------------------------


def _default_tree(
    config: SoccerConfig | None = None,
    provider: PlayContextProvider | None = None,
) -> TeamStrategyTree:
    """Build a :class:`TeamStrategyTree` with the default Playbook.

    Pass ``provider`` to reuse an existing fake. When omitted, this creates an
    empty :class:`_FakePlayContextProvider`; callers can later feed context via
    ``tree.context_provider.set_context(...)``.
    """
    cfg = SoccerConfig() if config is None else config
    fake = _FakePlayContextProvider() if provider is None else provider
    kit = SoccerKit(cfg)
    return TeamStrategyTree(kit, PLAYBOOKS.create_default(kit), fake)


class _FakePlayContextProvider(PlayContextProvider):
    """Test provider that stores one ``PlayContext`` and replaces it on set."""

    def __init__(self, context: PlayContext | None = None):
        self._context = PlayContext() if context is None else context

    def set_context(self, context: PlayContext) -> None:
        self._context = context

    def start(self) -> None:
        return None

    def stop(self) -> None:
        return None

    def get_snapshot(self) -> PlayContext:
        return self._context

    def set_game_state(self, game_state: GameControlState) -> None:
        self._context.game_state = game_state


_GAME_STATE_BY_ID = {
    0: GameState.INITIAL,
    1: GameState.READY,
    2: GameState.SET,
    3: GameState.PLAYING,
    4: GameState.FINISHED,
}

_SET_PLAY_BY_ID = {
    0: SetPlay.NONE,
    1: SetPlay.DIRECT_FREE_KICK,
    2: SetPlay.INDIRECT_FREE_KICK,
    3: SetPlay.PENALTY_KICK,
    4: SetPlay.THROW_IN,
    5: SetPlay.GOAL_KICK,
    6: SetPlay.CORNER_KICK,
}

_PENALTY_BY_ID = {
    0: Penalty.NONE,
    1: Penalty.ILLEGAL_POSITIONING,
    2: Penalty.MOTION_IN_SET,
    3: Penalty.LOCAL_GAME_STUCK,
    4: Penalty.INCAPABLE_ROBOT,
    5: Penalty.PICKED_UP,
    6: Penalty.BALL_HOLDING,
    7: Penalty.LEAVING_THE_FIELD,
    8: Penalty.PLAYING_WITH_ARMS_HANDS,
    9: Penalty.PUSHING,
    10: Penalty.SENT_OFF,
    11: Penalty.SUBSTITUTE,
}


def _team_state(
    team_number: int,
    goalkeeper: int = 3,
    penalty_p1: int = 0,
    secs_till_unpenalised_p1: int = 0,
) -> TeamState:
    players: list[PlayerState] = []
    for player_id in range(1, MAX_NUM_PLAYERS + 1):
        penalty = penalty_p1 if player_id == 1 else 0
        secs = secs_till_unpenalised_p1 if player_id == 1 else 0
        players.append(
            PlayerState(
                penalty=_PENALTY_BY_ID[penalty],
                secs_till_unpenalised=secs,
            )
        )
    return TeamState(
        team_number=team_number,
        goalkeeper=goalkeeper,
        players=players,
    )


def _game_state(
    state: int = 3,
    kicking_team: int = 1,
    set_play: int = 0,
    secondary_time: int = 0,
    first_half: int = 1,
    penalty_p1: int = 0,
    secs_till_unpenalised_p1: int = 0,
    stopped: bool = False,
) -> GameControlState:
    return GameControlState(
        packet_number=7,
        players_per_team=3,
        state=_GAME_STATE_BY_ID[state],
        set_play=_SET_PLAY_BY_ID[set_play],
        first_half=bool(first_half),
        kicking_team=kicking_team,
        secs_remaining=600,
        secondary_time=secondary_time,
        stopped=stopped,
        teams=[
            _team_state(
                1,
                penalty_p1=penalty_p1,
                secs_till_unpenalised_p1=secs_till_unpenalised_p1,
            ),
            _team_state(2),
        ],
    )


def _context(game_state: GameControlState, now: float) -> PlayContext:
    """Basic scene with a three-player team and a stationary ball."""

    return PlayContext(
        game_state=game_state,
        teammates={
            1: RobotState(1, Pose2D(-1.0, 0.0, 0.0), last_seen_at=now),
            2: RobotState(2, Pose2D(-1.8, 0.8, 0.0), last_seen_at=now),
            3: RobotState(3, Pose2D(-3.4, 0.0, 0.0), last_seen_at=now),
        },
        ball=BallState(0.0, 0.0, last_seen_at=now),
    )


def _tick(tree: TeamStrategyTree, context: PlayContext, now: float) -> dict[int, RobotCommand]:
    provider = tree.context_provider
    assert isinstance(provider, _FakePlayContextProvider)
    provider.set_context(context)
    tree.tick(now=now)
    return tree.last_commands


class _RecordingExecutor:
    def __init__(self) -> None:
        self.commands: dict[int, RobotCommand] = {}

    def execute_team_commands(
        self,
        commands: dict[int, RobotCommand],
    ) -> dict[int, RobotCommand]:
        self.commands = dict(commands)
        return commands


class _TraceLogger:
    def __init__(self) -> None:
        self.events: list[dict[str, object]] = []

    def info(self, message: str, **fields: object) -> None:
        event = fields.get("event")
        if isinstance(event, str):
            self.events.append({"message": message, **fields})

    def warn(self, message: str, **fields: object) -> None:
        event = fields.get("event")
        if isinstance(event, str):
            self.events.append({"message": message, **fields})


# --- Data types, parsing, and configuration ------------------------------


class CoreTypeTests(unittest.TestCase):
    def test_game_controller_json_round_trip(self):
        original = _game_state(state=3, kicking_team=2, set_play=1, secondary_time=12)
        encoded = game_control_state_to_json(original)
        parsed = game_control_state_from_json(encoded)
        self.assertEqual(parsed.state, GameState.PLAYING)
        self.assertEqual(parsed.kicking_team, 2)
        self.assertEqual(parsed.set_play, SetPlay.DIRECT_FREE_KICK)
        self.assertEqual(parsed.secondary_time, 12)


class ConfigTests(unittest.TestCase):
    def test_default_config_uses_sim_topic(self):
        config = SoccerConfig()
        self.assertEqual(config.game_controller_topic, "/soccer/game_controller")

    def test_config_from_env_supports_single_robot(self):
        env = {"SOCCER_ROBOT_NAMES": "robot_a", "SOCCER_TEAM_ID": "2"}
        config = SoccerConfig.from_env(env)
        self.assertEqual(config.team_id, 2)
        self.assertEqual(config.robot_names, ("robot_a",))
        self.assertEqual(config.opponent_robot_names, ("robot1", "robot2", "robot3"))
        self.assertIn(1, config.ready_slots)

    def test_config_from_env_ignores_non_identity_overrides(self):
        env = {
            "SOCCER_OPPONENT_ROBOT_NAMES": "custom1,custom2,custom3",
            "SOCCER_CONTROL_HZ": "60",
            "SOCCER_GAME_CONTROLLER_TOPIC": "/custom/game_controller",
        }
        config = SoccerConfig.from_env(env)
        self.assertEqual(config.opponent_robot_names, ("robot4", "robot5", "robot6"))
        self.assertEqual(config.control_hz, 30.0)
        self.assertEqual(config.game_controller_topic, "/soccer/game_controller")

    def test_config_from_env_ignores_removed_tuning_overrides(self):
        # Tactical knobs for speed, kicking, and similar behavior now live in
        # SoccerStrategyTuning instead of env vars. Legacy env vars should be
        # ignored and tuning should stay at defaults.
        env = {
            "SOCCER_KICK_ENTER_DISTANCE": "1.8",
            "SOCCER_KICK_POWER": "2.5",
            "SOCCER_MAX_LINEAR_SPEED": "0.6",
        }
        config = SoccerConfig.from_env(env)
        self.assertEqual(config.strategy, SoccerStrategyTuning())

    def test_config_from_env_ignores_removed_debug_overrides(self):
        env = {
            "SOCCER_TRACE_TICKS": "on",
            "SOCCER_TRACE_SAMPLE_SEC": "0.1",
        }
        config = SoccerConfig.from_env(env)
        self.assertEqual(config.debug, SoccerDebugConfig())

    def test_ros_node_parameters_match_nested_config_fields(self):
        ros_node_path = (
            Path(__file__).resolve().parents[1]
            / "ros_debug"
            / "soccer_sim_debug"
            / "soccer_sim_debug"
            / "ros_node.py"
        )
        tree = ast.parse(ros_node_path.read_text())
        tuning_fields = {field.name for field in fields(SoccerStrategyTuning)}
        debug_fields = {field.name for field in fields(SoccerDebugConfig)}

        unknown_strategy_attrs = sorted(
            {
                node.attr
                for node in ast.walk(tree)
                if isinstance(node, ast.Attribute)
                and isinstance(node.value, ast.Name)
                and node.value.id == "strategy"
            }
            - tuning_fields
        )
        self.assertEqual(unknown_strategy_attrs, [])

        unknown_debug_attrs = sorted(
            {
                node.attr
                for node in ast.walk(tree)
                if isinstance(node, ast.Attribute)
                and isinstance(node.value, ast.Attribute)
                and node.value.attr == "debug"
            }
            - debug_fields
        )
        self.assertEqual(unknown_debug_attrs, [])

        constructor_keywords: list[str] = []
        debug_constructor_keywords: list[str] = []
        for node in ast.walk(tree):
            if (
                isinstance(node, ast.Call)
                and isinstance(node.func, ast.Name)
                and node.func.id == "SoccerStrategyTuning"
            ):
                for keyword in node.keywords:
                    self.assertIsNotNone(keyword.arg)
                    constructor_keywords.append(keyword.arg)
            if (
                isinstance(node, ast.Call)
                and isinstance(node.func, ast.Name)
                and node.func.id == "SoccerDebugConfig"
            ):
                for keyword in node.keywords:
                    self.assertIsNotNone(keyword.arg)
                    debug_constructor_keywords.append(keyword.arg)

        self.assertTrue(constructor_keywords)
        unknown_constructor_keywords = sorted(set(constructor_keywords) - tuning_fields)
        self.assertEqual(unknown_constructor_keywords, [])
        self.assertTrue(debug_constructor_keywords)
        unknown_debug_constructor_keywords = sorted(
            set(debug_constructor_keywords) - debug_fields
        )
        self.assertEqual(unknown_debug_constructor_keywords, [])


# --- Behavior: match phases ----------------------------------------------


class TeamFlowTests(unittest.TestCase):
    """Cover team behavior under each main GameController state."""

    def test_no_game_controller_state_stops_team(self):
        commands = _tick(_default_tree(), PlayContext(), now=0.0)
        for cmd in commands.values():
            self.assertEqual(cmd.reason, "no game controller state")
            self.assertIsInstance(cmd.intent, StopIntent)

    def test_set_state_stops_team(self):
        now = time.monotonic()
        context = _context(_game_state(state=2), now)
        commands = _tick(_default_tree(), context, now)
        for cmd in commands.values():
            self.assertEqual(cmd.reason, "non playing state")
            self.assertIsInstance(cmd.intent, StopIntent)

    def test_initial_and_finished_stop_team(self):
        now = time.monotonic()
        for state in (0, 4):
            commands = _tick(
                _default_tree(),
                _context(_game_state(state=state), now),
                now,
            )
            self.assertEqual(commands[1].reason, "non playing state")

    def test_stopped_flag_holds_position(self):
        now = time.monotonic()
        context = _context(_game_state(state=3, stopped=True), now)
        commands = _tick(_default_tree(), context, now)
        for cmd in commands.values():
            self.assertEqual(cmd.reason, "game stopped")

    def test_penalized_player_gets_inactive_stop(self):
        now = time.monotonic()
        context = _context(
            _game_state(state=3, kicking_team=KICKING_TEAM_NONE, penalty_p1=1),
            now,
        )
        commands = _tick(_default_tree(), context, now)
        self.assertEqual(commands[1].reason, "inactive or penalized")
        self.assertIsInstance(commands[1].intent, StopIntent)

    def test_all_players_inactive_stops_team(self):
        now = time.monotonic()
        # Mark all players as SUBSTITUTE.
        gs = _game_state(state=3)
        team = gs.teams[0]
        for player in team.players[:3]:
            player.penalty = Penalty.SUBSTITUTE
        context = _context(gs, now)
        commands = _tick(_default_tree(), context, now)
        for cmd in commands.values():
            self.assertEqual(cmd.reason, "all players inactive")

    def test_ready_phase_walks_every_player_to_ready_target(self):
        now = time.monotonic()
        context = _context(_game_state(state=1), now)
        commands = _tick(_default_tree(), context, now)
        self.assertTrue(commands[1].reason.startswith("ready center"))
        self.assertTrue(commands[2].reason.startswith("ready side"))
        self.assertTrue(commands[3].reason.startswith("ready keeper"))

    def test_ready_phase_does_not_require_ball(self):
        now = time.monotonic()
        context = _context(_game_state(state=1), now)
        context.ball = None
        commands = _tick(_default_tree(), context, now)
        self.assertTrue(commands[1].reason.startswith("ready center"))
        self.assertTrue(commands[2].reason.startswith("ready side"))
        self.assertTrue(commands[3].reason.startswith("ready keeper"))

    def test_runtime_executor_receives_commands(self):
        now = time.monotonic()
        context = _context(_game_state(state=2), now)
        provider = _FakePlayContextProvider(context)
        tree = _default_tree(provider=provider)
        executor = _RecordingExecutor()
        tree.tick(now=now, executor=executor)
        self.assertEqual(tree.last_commands, executor.commands)


# --- Behavior: PLAYING phase ---------------------------------------------


class PlayingTests(unittest.TestCase):
    def test_center_slot_kicks_when_within_kick_distance(self):
        now = time.monotonic()
        context = _context(_game_state(state=3, kicking_team=KICKING_TEAM_NONE), now)
        context.teammates[1].pose = Pose2D(-1.5, 0.0, 0.0)
        context.ball = BallState(0.5, 0.0, last_seen_at=now)
        commands = _tick(_default_tree(), context, now)
        self.assertIsInstance(commands[1].intent, KickIntent)

    def test_center_slot_approaches_ball_when_far(self):
        now = time.monotonic()
        context = _context(_game_state(state=3, kicking_team=KICKING_TEAM_NONE), now)
        # Put the center slot closest to the ball but outside kick distance;
        # move the side slot farther away so it does not claim the attack.
        context.teammates[1].pose = Pose2D(-5.0, 0.0, 0.0)
        context.teammates[2].pose = Pose2D(-7.0, 1.5, 0.0)
        context.ball = BallState(-1.0, 0.0, last_seen_at=now)
        commands = _tick(_default_tree(), context, now)
        self.assertNotIsInstance(commands[1].intent, KickIntent)
        self.assertIn("approach ball", commands[1].reason)

    def test_kick_hysteresis_keeps_kick_active_in_exit_window(self):
        # Same chaser across two frames: enter kick mode first, then move out
        # instantly while still inside the exit delay.
        now = time.monotonic()
        config = SoccerConfig(
            strategy=SoccerStrategyTuning(soccer_kick_exit_delay_sec=2.0)
        )
        strategy = _default_tree(config)
        gs = _game_state(state=3, kicking_team=KICKING_TEAM_NONE)
        # Move the side and keeper slots far away so the chaser always comes
        # from the center slot.
        context = PlayContext(
            game_state=gs,
            teammates={
                1: RobotState(1, Pose2D(-1.0, 0.0, 0.0), last_seen_at=now),
                2: RobotState(2, Pose2D(-6.0, 2.0, 0.0), last_seen_at=now),
                3: RobotState(3, Pose2D(-6.5, 0.0, 0.0), last_seen_at=now),
            },
            ball=BallState(0.5, 0.0, last_seen_at=now),
        )
        first = _tick(strategy, context, now)
        self.assertIsInstance(first[1].intent, KickIntent)

        # Frame 2: the ball jumps farther away, but kick mode stays active
        # because the exit delay has not expired.
        context.teammates[1].pose = Pose2D(-1.5, 0.0, 0.0)
        context.ball = BallState(2.0, 0.0, last_seen_at=now + 0.5)
        second = _tick(strategy, context, now + 0.5)
        # distance=3.5 > exit=3.0, but still within the exit delay.
        self.assertTrue(strategy.kit.kicker.is_active(1))
        self.assertIsInstance(second[1].intent, KickIntent)

    def test_side_slot_supports_when_center_slot_chases(self):
        now = time.monotonic()
        context = _context(_game_state(state=3, kicking_team=KICKING_TEAM_NONE), now)
        context.teammates[1].pose = Pose2D(-1.2, 0.0, 0.0)
        context.ball = BallState(-0.3, 0.0, last_seen_at=now)
        commands = _tick(_default_tree(), context, now)
        # The side slot should not kick; it should support or guard.
        self.assertNotIsInstance(commands[2].intent, KickIntent)
        self.assertTrue("support" in commands[2].reason or "guard" in commands[2].reason)

    def test_goalkeeper_intercepts_dangerous_ball(self):
        now = time.monotonic()
        context = _context(_game_state(state=3, kicking_team=KICKING_TEAM_NONE), now)
        # Ball is in a dangerous position near our penalty area.
        context.ball = BallState(-5.5, 0.5, last_seen_at=now)
        context.teammates[3].pose = Pose2D(-6.0, 0.0, 0.0)
        commands = _tick(_default_tree(), context, now)
        self.assertIn("goalkeeper", commands[3].reason)

    def test_goalkeeper_guards_when_ball_is_far(self):
        now = time.monotonic()
        context = _context(_game_state(state=3, kicking_team=KICKING_TEAM_NONE), now)
        # Ball is in the opponent half, so the goalkeeper only guards.
        context.ball = BallState(4.0, 0.0, last_seen_at=now)
        commands = _tick(_default_tree(), context, now)
        self.assertNotIsInstance(commands[3].intent, KickIntent)

    def test_playing_without_ball_stops_in_safety_guards(self):
        """Temporary ball loss during PLAYING is handled before playbook tactics.

        When the referee detects a corner or throw-in, the truth provider may
        briefly have no ball. SafetyGuards should stop the team directly so
        playbook code can assume fresh ball data.
        """

        now = time.monotonic()
        context = _context(_game_state(state=3, kicking_team=KICKING_TEAM_NONE), now)
        context.ball = None
        commands = _tick(_default_tree(), context, now)
        for cmd in commands.values():
            self.assertEqual(cmd.reason, "waiting for ball")

    def test_safety_guard_skips_assign_roles_when_ball_missing(self):
        class BallRequiredPlaybook(DefaultPlaybook):
            def assign_roles(self, context: PlayContext) -> RoleAssignment:
                if context.ball is None:
                    raise AssertionError("assign_roles should be guarded by SafetyGuards")
                return super().assign_roles(context)

        now = time.monotonic()
        kit = SoccerKit(SoccerConfig())
        tree = TeamStrategyTree(
            kit,
            BallRequiredPlaybook(kit),
            _FakePlayContextProvider(),
        )
        context = _context(_game_state(state=3, kicking_team=KICKING_TEAM_NONE), now)
        context.ball = None
        commands = _tick(tree, context, now)
        for cmd in commands.values():
            self.assertIn("waiting for ball", cmd.reason)


# --- Behavior: restarts ---------------------------------------------------


class RestartTests(unittest.TestCase):
    def test_opponent_restart_keeps_distance_while_set_play_active(self):
        now = time.monotonic()
        gs = _game_state(state=3, kicking_team=2, set_play=4, secondary_time=45)
        context = _context(gs, now)
        commands = _tick(_default_tree(), context, now)
        self.assertTrue(commands[1].reason.startswith("avoid opponent restart"))
        self.assertEqual(commands[2].reason, "avoid opponent restart: clear of ball")
        self.assertEqual(commands[3].reason, "avoid opponent restart: clear of ball")
        self.assertIsInstance(commands[2].intent, StopIntent)
        self.assertIsInstance(commands[3].intent, StopIntent)

    def test_stopped_restart_holds_for_throwin_goalkick_cornerkick(self):
        now = time.monotonic()
        for set_play in (4, 5, 6):
            with self.subTest(set_play=set_play):
                gs = _game_state(state=3, kicking_team=2, set_play=set_play, stopped=True)
                context = _context(gs, now)
                commands = _tick(_default_tree(), context, now)
                self.assertEqual(commands[1].reason, "game stopped")

    def test_kickoff_for_own_team_keeps_normal_play(self):
        now = time.monotonic()
        gs = _game_state(state=3, kicking_team=1, set_play=0)
        context = _context(gs, now)
        context.teammates[1].pose = Pose2D(-1.5, 0.0, 0.0)
        context.ball = BallState(0.0, 0.0, last_seen_at=now)
        commands = _tick(_default_tree(), context, now)
        # Own kickoff: the forward should play the ball normally.
        self.assertIsInstance(commands[1].intent, KickIntent)

    def test_opponent_kickoff_holds_ready_positions(self):
        now = time.monotonic()
        gs = _game_state(state=3, kicking_team=2, set_play=0, secondary_time=10)
        context = _context(gs, now)
        commands = _tick(_default_tree(), context, now)
        for cmd in commands.values():
            self.assertTrue(cmd.reason.startswith("ready"))

    def test_opponent_restart_releases_when_set_play_clears(self):
        now = time.monotonic()
        gs = _game_state(state=3, kicking_team=2, set_play=4, secondary_time=45)
        context = _context(gs, now)
        strategy = _default_tree()
        _tick(strategy, context, now)
        commands = _tick(strategy, context, now + 0.1)
        self.assertTrue(commands[1].reason.startswith("avoid opponent restart"))
        gs.set_play = SetPlay.NONE
        gs.kicking_team = KICKING_TEAM_NONE
        commands = _tick(strategy, context, now + 0.2)
        self.assertFalse(commands[1].reason.startswith("avoid opponent restart"))


# --- Behavior: avoidance --------------------------------------------------


class AvoidanceTests(unittest.TestCase):
    def test_ready_phase_avoids_static_teammate(self):
        # READY phase: the forward should walk to the ready position
        # (attack_line_x), but an opponent blocks the path. The old
        # move_to_ready_target path skipped navigation and walked into it.
        now = time.monotonic()
        gs = _game_state(state=1)
        # Forward ready x is -1.68 when own_side=-1. Spawn the forward at -4.5,
        # target -1.68, and place an opponent on the line at -3.0.
        context = PlayContext(
            game_state=gs,
            teammates={
                1: RobotState(1, Pose2D(-4.5, 0.0, 0.0), last_seen_at=now),
                2: RobotState(2, Pose2D(-4.0, 1.5, 0.0), last_seen_at=now),
                3: RobotState(3, Pose2D(-5.5, 0.0, 0.0), last_seen_at=now),
            },
            opponents={
                4: RobotState(4, Pose2D(-3.0, 0.0, 0.0), last_seen_at=now),
            },
            ball=BallState(0.0, 0.0, last_seen_at=now),
        )
        commands = _tick(_default_tree(), context, now)
        # Navigation appends "via obstacle" when avoidance triggers.
        self.assertIn("via obstacle", commands[1].reason)

    def test_ready_phase_avoids_goal_post(self):
        # The keeper ready x is roughly -field_length/2 + goal_area_length
        # + 0.25, or -5.75. If the path grazes a goal post
        # (+/-field_length/2, +/-goal_width/2 = +/-7.0, +/-1.3), navigation
        # should route around it. Spawn the keeper outside the post so it must
        # route back inside the field.
        now = time.monotonic()
        config = SoccerConfig()
        gs = _game_state(state=1)
        # Keeper starts outside our post at (-7.5, 1.4), targeting (-5.75, 0);
        # the straight line crosses the post at (-7.0, 1.3).
        context = PlayContext(
            game_state=gs,
            teammates={
                1: RobotState(1, Pose2D(-3.0, 0.0, 0.0), last_seen_at=now),
                2: RobotState(2, Pose2D(-3.5, 1.5, 0.0), last_seen_at=now),
                3: RobotState(3, Pose2D(-7.5, 1.4, 0.0), last_seen_at=now),
            },
            ball=BallState(0.0, 0.0, last_seen_at=now),
        )
        commands = _tick(_default_tree(config), context, now)
        self.assertIn("via obstacle", commands[3].reason)

    def test_navigation_treats_goal_back_net_as_solid(self):
        # Goal-net avoidance: a straight path from behind our goal
        # (-8.0, 0.0) toward field center crosses the back net around
        # x=-7.6, y=0. The forward ready target is (-1.68, 0), so spawning the
        # forward at (-8.0, 0.0) should route around the back net.
        now = time.monotonic()
        gs = _game_state(state=1)
        context = PlayContext(
            game_state=gs,
            teammates={
                1: RobotState(1, Pose2D(-8.0, 0.0, 0.0), last_seen_at=now),
                2: RobotState(2, Pose2D(-3.5, 1.5, 0.0), last_seen_at=now),
                3: RobotState(3, Pose2D(-5.5, 0.0, 0.0), last_seen_at=now),
            },
            ball=BallState(0.0, 0.0, last_seen_at=now),
        )
        commands = _tick(_default_tree(), context, now)
        self.assertIn("via obstacle", commands[1].reason)

    def test_ready_yaw_avoid_reacts_to_close_opponent(self):
        # READY phase: an opponent close in front of the forward should map
        # the avoidance direction into a vyaw bias. Biped constraints require
        # leaving vy at zero.
        now = time.monotonic()
        gs = _game_state(state=1)  # READY
        # Forward spawns at (-3.0, 0), heading 0. The opponent is 0.30 m ahead
        # at (-2.7, 0). Without the opponent, angle error is 0 and vyaw should
        # be 0; with the opponent, vyaw should be biased away from zero.
        context = PlayContext(
            game_state=gs,
            teammates={
                1: RobotState(1, Pose2D(-3.0, 0.0, 0.0), last_seen_at=now),
                2: RobotState(2, Pose2D(-3.5, 1.5, 0.0), last_seen_at=now),
                3: RobotState(3, Pose2D(-5.5, 0.0, 0.0), last_seen_at=now),
            },
            opponents={
                4: RobotState(4, Pose2D(-2.7, 0.0, 0.0), last_seen_at=now),
            },
            ball=BallState(0.0, 0.0, last_seen_at=now),
        )
        commands = _tick(_default_tree(), context, now)
        self.assertIn("yaw avoid", commands[1].reason)
        # vyaw should be biased and nonzero; vy must remain 0 for the unicycle
        # constraint.
        intent = commands[1].intent
        assert isinstance(intent, MoveIntent)
        self.assertGreater(abs(intent.vyaw), 1e-6)
        self.assertAlmostEqual(intent.vy, 0.0)

    def test_play_yaw_avoid_does_not_react_to_opponent(self):
        # PLAY phase: with the same close-opponent setup, move_to_target should
        # not enable opponent yaw avoidance. Otherwise the chaser gets pushed
        # away from opponents and cannot play. Put the ball 1.5 m ahead and an
        # opponent 0.5 m ahead; the chasing path must not add opponent yaw
        # avoidance, so the command reason must not contain "yaw avoid".
        # Path-detour via avoidance may still trigger, but not the yaw layer.
        now = time.monotonic()
        gs = _game_state(state=3)  # PLAYING
        # Forward at (0, 0), ball at (1.5, 0), opponent at (1.0, 0) in the path.
        # Teammates stay far away so teammate yaw avoidance does not trigger.
        context = PlayContext(
            game_state=gs,
            teammates={
                1: RobotState(1, Pose2D(0.0, 0.0, 0.0), last_seen_at=now),
                2: RobotState(2, Pose2D(-4.0, 2.5, 0.0), last_seen_at=now),
                3: RobotState(3, Pose2D(-5.5, 0.0, 0.0), last_seen_at=now),
            },
            opponents={
                4: RobotState(4, Pose2D(1.0, 0.0, 0.0), last_seen_at=now),
            },
            ball=BallState(1.5, 0.0, last_seen_at=now),
        )
        commands = _tick(_default_tree(), context, now)
        # Path-detour via may trigger because the opponent blocks the path.
        # That is allowed. Absence of "yaw avoid" proves the yaw layer ignored
        # the opponent.
        self.assertNotIn("yaw avoid", commands[1].reason)

    def test_all_movement_commands_respect_unicycle_constraint(self):
        # Biped base: vx and vy must not be commanded together because gait
        # synthesis cannot handle that combination. Check representative
        # scenarios and assert every movement command has at least one of
        # vx/vy equal to zero.
        now = time.monotonic()
        scenarios = [
            # READY, with an opponent on the forward's path.
            PlayContext(
                game_state=_game_state(state=1),
                teammates={
                    1: RobotState(1, Pose2D(-4.0, 0.0, 0.5), last_seen_at=now),
                    2: RobotState(2, Pose2D(-3.5, 1.5, 0.0), last_seen_at=now),
                    3: RobotState(3, Pose2D(-5.5, 0.0, 0.0), last_seen_at=now),
                },
                opponents={
                    4: RobotState(4, Pose2D(-2.5, 0.0, 0.0), last_seen_at=now),
                },
                ball=BallState(0.0, 0.0, last_seen_at=now),
            ),
            # PLAY, with the forward chasing the ball.
            PlayContext(
                game_state=_game_state(state=3),
                teammates={
                    1: RobotState(1, Pose2D(-1.0, 0.5, 0.3), last_seen_at=now),
                    2: RobotState(2, Pose2D(-2.0, 1.0, 0.0), last_seen_at=now),
                    3: RobotState(3, Pose2D(-5.5, 0.0, 0.0), last_seen_at=now),
                },
                ball=BallState(1.5, 0.0, last_seen_at=now),
            ),
        ]
        for context in scenarios:
            commands = _tick(_default_tree(), context, now)
            for player_id, cmd in commands.items():
                # Kick and stop intents are handled by the SDK or ignored here.
                intent = cmd.intent
                if not isinstance(intent, MoveIntent):
                    continue
                self.assertTrue(
                    abs(intent.vx) < 1e-6 or abs(intent.vy) < 1e-6,
                    f"player {player_id} violates unicycle: "
                    f"vx={intent.vx} vy={intent.vy} reason={cmd.reason}",
                )


# --- Factory --------------------------------------------------------------


class StrategyFactoryTests(unittest.TestCase):
    def test_team_tree_exposes_kit_playbook_context_provider(self):
        kit = SoccerKit(SoccerConfig())
        playbook = PLAYBOOKS.create_default(kit)
        provider = _FakePlayContextProvider()
        tree = TeamStrategyTree(kit, playbook, provider)
        self.assertIs(tree.kit, kit)
        self.assertIs(tree.playbook, playbook)
        self.assertIs(tree.context_provider, provider)

    def test_default_playbook_registered(self):
        # The default Playbook must be explicitly registered at the end of
        # play/__init__.py.
        self.assertIn("default", PLAYBOOKS.names())

    def test_debug_tree_ascii_includes_top_level_phases(self):
        tree = _default_tree()
        ascii_tree = tree.ascii_tree()
        # Assert only the top-level structure, not concrete node-name details.
        for phrase in ("TeamRoot", "DataLayer", "MatchControl", "SafetyGuards",
                       "ReadyPhase", "PlayingPhase", "CommitTeamCommands"):
            self.assertIn(phrase, ascii_tree)

    def test_default_tree_excludes_defender_branch(self):
        tree = _default_tree()
        ascii_tree = tree.ascii_tree()
        self.assertIn("AsChaser(1)", ascii_tree)
        self.assertIn("AsSupporter(1)", ascii_tree)
        self.assertIn("AsGoalkeeper(1)", ascii_tree)
        self.assertNotIn("AsDefender", ascii_tree)

    def test_tick_trace_records_nodes_blackboard_and_commands(self):
        logger = _TraceLogger()
        context = _context(_game_state(state=3), now=10.0)
        provider = _FakePlayContextProvider(context)

        config = SoccerConfig(debug=SoccerDebugConfig(bt_trace_ticks="on"))
        kit = SoccerKit(config)
        tree = TeamStrategyTree(
            kit, PLAYBOOKS.create_default(kit), provider, logger=logger,
        )
        tree.tick(now=10.0)

        bt_events = [event for event in logger.events if event.get("event") == "bt_tick"]
        self.assertEqual(len(bt_events), 1)
        event = bt_events[0]
        self.assertEqual(event["root_status"], "SUCCESS")
        nodes = event["nodes"]
        self.assertIsInstance(nodes, list)
        assert isinstance(nodes, list)
        self.assertTrue(
            any(
                isinstance(node, dict) and node.get("name") == "TeamRoot"
                for node in nodes
            )
        )
        blackboard = event["blackboard"]
        self.assertIsInstance(blackboard, dict)
        assert isinstance(blackboard, dict)
        self.assertIn("/game/state", blackboard)
        commands = event["commands_committed"]
        self.assertIsInstance(commands, dict)
        assert isinstance(commands, dict)
        self.assertIn("1", commands)


# --- KickHysteresis: core model with stable unit-test semantics -----------


class KickHysteresisTests(unittest.TestCase):
    def test_enters_inside_enter_distance(self):
        h = KickHysteresis(enter=2.5, exit=3.0, exit_delay=1.5)
        self.assertTrue(h.in_kick_range(1, 2.4, 0.0))
        self.assertTrue(h.is_active(1))

    def test_does_not_enter_outside_enter_distance(self):
        h = KickHysteresis(enter=2.5, exit=3.0, exit_delay=1.5)
        self.assertFalse(h.in_kick_range(1, 2.6, 0.0))

    def test_holds_in_exit_window_then_releases(self):
        h = KickHysteresis(enter=2.5, exit=3.0, exit_delay=1.5)
        h.in_kick_range(1, 2.0, 0.0)
        self.assertTrue(h.in_kick_range(1, 3.4, 0.5))  # Still inside the exit delay.
        self.assertFalse(h.in_kick_range(1, 3.4, 2.5))  # Exit delay has expired.
        self.assertFalse(h.is_active(1))

    def test_clear_player_drops_state(self):
        h = KickHysteresis(enter=2.5, exit=3.0, exit_delay=1.5)
        h.in_kick_range(1, 1.0, 0.0)
        h.clear_player(1)
        self.assertFalse(h.is_active(1))


if __name__ == "__main__":
    unittest.main()
