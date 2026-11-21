#!/usr/bin/env python3
"""Automated deploy/control/observe loop for SoccerSim debug matches."""

from __future__ import annotations

import argparse
import asyncio
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import signal
import shutil
import subprocess
import struct
import sys
import threading
import time
from types import FrameType
from typing import TextIO, TypeAlias
import urllib.error
import urllib.request


JsonValue: TypeAlias = (
    None | bool | int | float | str | list["JsonValue"] | dict[str, "JsonValue"]
)
JsonObject: TypeAlias = dict[str, JsonValue]

DEFAULT_CONTAINER = "c972daf7bf19"
DEFAULT_CONFIG_FILE = "ros_debug/.soccer_sim.env"
DEFAULT_WORKDIR = "/root/workspace/SoccerSim"
DEFAULT_ROS_SETUP = "/opt/ros/humble/setup.bash"
DEFAULT_FASTDDS_PROFILE = "/opt/booster/BoosterRos2/fastdds_profile.xml"
DEFAULT_CONTROL_URL = "http://127.0.0.1:38383"
DEFAULT_WORLD_WS_URL = "ws://127.0.0.1:8788"
DEFAULT_GAME_CONTROL_ARTIFACT_DIR = "/usr/local/booster_robot/booster_robocup_sim/logs/game-control"
DEFAULT_ARTIFACT_ROOT = Path("artifacts/auto_debug")
DEFAULT_READY_SLOTS = "center,side,keeper"
SCRIPT_IN_CONTAINER = "/tmp/soccer_auto_debug_match.py"


@dataclass(frozen=True)
class TeamSpec:
    team_id: int
    robot_names: str
    ready_slots: str = DEFAULT_READY_SLOTS
    opponent_robot_names: str = ""


@dataclass(frozen=True)
class RunnerConfig:
    local_container: bool
    container: str
    workdir: str
    ros_setup: str
    extra_setup: str
    fastdds_profile: str
    control_url: str
    game_control_artifact_dir: str
    artifact_root: Path
    run_id: str
    build: bool
    start_nodes: bool
    keep_nodes: bool
    auto_match: bool
    control_hz: float
    ready_wait_sec: float
    play_sec: float
    observe_hz: float
    min_score: float
    team1: TeamSpec
    team2: TeamSpec


@dataclass
class ManagedProcess:
    name: str
    process: subprocess.Popen[str]
    log_path: Path
    cleanup_command: Sequence[str] | None = None
    output_thread: threading.Thread | None = None
    output_file: TextIO | None = None


@dataclass(frozen=True)
class GameControlArtifacts:
    events: list[JsonObject]
    readiness: list[JsonObject]
    copied_files: list[str]


def main() -> int:
    if "--inside-control" in sys.argv:
        return _inside_control_main(sys.argv[1:])
    if "--inside-bridge" in sys.argv:
        return _inside_bridge_main(sys.argv[1:])
    if "--inside-observer" in sys.argv:
        return _observer_main(sys.argv[1:])
    return _host_main(sys.argv[1:])


def _host_main(argv: Sequence[str]) -> int:
    config = _parse_host_args(argv)
    run_dir = (config.artifact_root / config.run_id).resolve()
    run_dir.mkdir(parents=True, exist_ok=True)
    print(f"[auto-debug] run_id={config.run_id}")
    print(f"[auto-debug] artifacts={run_dir}")

    managed: list[ManagedProcess] = []
    bridge: ManagedProcess | None = None
    observer: ManagedProcess | None = None
    try:
        _write_json(run_dir / "config.json", _config_record(config))
        if not config.local_container:
            _require_docker_container(config.container)
            _copy_observer_script(config.container)
        if config.build:
            _run_debug_build(config, run_dir)
        _wait_for_game_control(config, timeout_sec=15.0)
        bridge = _maybe_start_topic_bridge(config, run_dir)
        if bridge is not None:
            managed.append(bridge)
        if config.start_nodes:
            managed.extend(_start_team_nodes(config, run_dir))
            _wait_for_debug_logs(config, timeout_sec=20.0)
        observer = _start_observer(config, run_dir)
        _run_episode(config, run_dir)
        stopped_observer = observer
        _terminate_process(stopped_observer)
        observer = None
        report_processes = list(managed)
        if not config.keep_nodes:
            for managed_process in report_processes:
                _terminate_process(managed_process)
            managed = []
        report = _build_report(config, run_dir, report_processes, stopped_observer)
        _write_json(run_dir / "report.json", report)
        _write_markdown_report(run_dir / "report.md", report)
        print(f"[auto-debug] report={run_dir / 'report.md'}")
        score = _number(report.get("score"), 0.0)
        print(f"[auto-debug] score={score:.1f}/100")
        if score < config.min_score:
            return 2
        return 0
    finally:
        if observer is not None:
            _terminate_process(observer)
        if not config.keep_nodes:
            for managed_process in managed:
                _terminate_process(managed_process)


def _parse_host_args(argv: Sequence[str]) -> RunnerConfig:
    local_config = _load_local_config_env()
    parser = argparse.ArgumentParser(
        description="Run one automated SoccerSim deploy/debug/observe episode.",
    )
    parser.add_argument(
        "--local-container",
        action="store_true",
        help="Run from inside the simulator container without nested docker calls.",
    )
    parser.add_argument(
        "--container",
        default=_setting(local_config, "SOCCER_DOCKER_CONTAINER", DEFAULT_CONTAINER),
    )
    parser.add_argument(
        "--workdir",
        default=_setting(local_config, "SOCCER_DOCKER_WORKDIR", DEFAULT_WORKDIR),
    )
    parser.add_argument(
        "--ros-setup",
        default=_setting(local_config, "SOCCER_DOCKER_ROS_SETUP", DEFAULT_ROS_SETUP),
    )
    parser.add_argument(
        "--extra-setup",
        default=_setting(local_config, "SOCCER_DOCKER_EXTRA_SETUP", ""),
    )
    parser.add_argument(
        "--fastdds-profile",
        default=_setting(local_config, "SOCCER_FASTDDS_PROFILE", DEFAULT_FASTDDS_PROFILE),
    )
    parser.add_argument(
        "--control-url",
        default=_setting(local_config, "SOCCER_GAME_CONTROL_URL", DEFAULT_CONTROL_URL),
    )
    parser.add_argument(
        "--game-control-artifact-dir",
        default=_setting(
            local_config,
            "SOCCER_GAME_CONTROL_ARTIFACT_DIR",
            DEFAULT_GAME_CONTROL_ARTIFACT_DIR,
        ),
        help="Directory inside the simulator container where GameControl writes events.jsonl/status.json.",
    )
    parser.add_argument("--artifact-root", type=Path, default=DEFAULT_ARTIFACT_ROOT)
    parser.add_argument("--run-id", default=_default_run_id())
    parser.add_argument("--build", action="store_true", help="Run scripts/debug_build before starting nodes.")
    parser.add_argument("--no-start-nodes", action="store_true", help="Only control and observe an already running setup.")
    parser.add_argument("--keep-nodes", action="store_true", help="Leave started debug nodes running after the episode.")
    parser.add_argument(
        "--auto-match",
        action="store_true",
        help=(
            "Run the real simulator GameControl automatic match flow. "
            "Current GameControl no longer supports manual ready/set/playing control."
        ),
    )
    parser.add_argument("--control-hz", type=float, default=10.0)
    parser.add_argument("--ready-wait-sec", type=float, default=8.0)
    parser.add_argument("--play-sec", type=float, default=20.0)
    parser.add_argument("--observe-hz", type=float, default=5.0)
    parser.add_argument(
        "--min-score",
        type=float,
        default=40.0,
        help="Exit with code 2 when the generated quality score is below this value.",
    )
    parser.add_argument("--team1-robots", default="robot1,robot2,robot3")
    parser.add_argument("--team2-robots", default="robot4,robot5,robot6")
    parser.add_argument("--team1-ready-slots", default=DEFAULT_READY_SLOTS)
    parser.add_argument("--team2-ready-slots", default=DEFAULT_READY_SLOTS)
    args = parser.parse_args(argv)
    team1_robots = str(args.team1_robots)
    team2_robots = str(args.team2_robots)
    return RunnerConfig(
        container=str(args.container),
        local_container=bool(args.local_container),
        workdir=str(args.workdir),
        ros_setup=str(args.ros_setup),
        extra_setup=str(args.extra_setup),
        fastdds_profile=str(args.fastdds_profile),
        control_url=str(args.control_url).rstrip("/"),
        game_control_artifact_dir=str(args.game_control_artifact_dir),
        artifact_root=args.artifact_root,
        run_id=str(args.run_id),
        build=bool(args.build),
        start_nodes=not bool(args.no_start_nodes),
        keep_nodes=bool(args.keep_nodes),
        auto_match=bool(args.auto_match),
        control_hz=float(args.control_hz),
        ready_wait_sec=float(args.ready_wait_sec),
        play_sec=float(args.play_sec),
        observe_hz=float(args.observe_hz),
        min_score=float(args.min_score),
        team1=TeamSpec(
            1,
            team1_robots,
            str(args.team1_ready_slots),
            team2_robots,
        ),
        team2=TeamSpec(
            2,
            team2_robots,
            str(args.team2_ready_slots),
            team1_robots,
        ),
    )


def _load_local_config_env() -> dict[str, str]:
    config_file = os.environ.get("SOCCER_SIM_CONFIG_FILE")
    if config_file is not None:
        config_path = Path(config_file).expanduser()
        if not config_path.is_absolute():
            config_path = Path.cwd() / config_path
    else:
        config_path = _repo_root() / DEFAULT_CONFIG_FILE
    return _read_env_file(config_path)


def _repo_root() -> Path:
    script_path = Path(__file__).resolve()
    if script_path.parent.name == "scripts":
        return script_path.parents[1]
    return Path.cwd()


def _setting(config: Mapping[str, str], name: str, default: str) -> str:
    env_value = os.environ.get(name)
    if env_value is not None:
        return env_value
    return config.get(name, default)


def _read_env_file(path: Path) -> dict[str, str]:
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except FileNotFoundError:
        return {}

    result: dict[str, str] = {}
    for raw_line in lines:
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("export "):
            line = line[7:].strip()
        key, separator, raw_value = line.partition("=")
        if not separator:
            continue
        normalized_key = key.strip()
        if not _is_env_key(normalized_key):
            continue
        result[normalized_key] = _normalize_env_value(raw_value.strip())
    return result


def _is_env_key(value: str) -> bool:
    if not value:
        return False
    first = value[0]
    if not (first == "_" or first.isalpha()):
        return False
    return all(char == "_" or char.isalnum() for char in value)


def _normalize_env_value(value: str) -> str:
    if len(value) >= 2 and value[0] == value[-1] and value[0] in ("'", '"'):
        return value[1:-1]
    return value


def _default_run_id() -> str:
    return time.strftime("auto_%Y%m%d_%H%M%S", time.localtime())


def _config_record(config: RunnerConfig) -> JsonObject:
    return {
        "container": config.container,
        "local_container": config.local_container,
        "workdir": config.workdir,
        "control_url": config.control_url,
        "game_control_artifact_dir": config.game_control_artifact_dir,
        "run_id": config.run_id,
        "build": config.build,
        "start_nodes": config.start_nodes,
        "auto_match": config.auto_match,
        "control_hz": config.control_hz,
        "ready_wait_sec": config.ready_wait_sec,
        "play_sec": config.play_sec,
        "observe_hz": config.observe_hz,
        "min_score": config.min_score,
        "teams": [
            {
                "team_id": config.team1.team_id,
                "robot_names": config.team1.robot_names,
                "ready_slots": config.team1.ready_slots,
                "opponent_robot_names": config.team1.opponent_robot_names,
            },
            {
                "team_id": config.team2.team_id,
                "robot_names": config.team2.robot_names,
                "ready_slots": config.team2.ready_slots,
                "opponent_robot_names": config.team2.opponent_robot_names,
            },
        ],
    }


def _require_docker_container(container: str) -> None:
    result = _run(
        ["docker", "inspect", "-f", "{{.State.Running}}", container],
        capture_output=True,
        check=False,
    )
    if result.returncode != 0 or result.stdout.strip() != "true":
        raise RuntimeError(f"Docker container is not running: {container}")


def _copy_observer_script(container: str) -> None:
    script_path = Path(__file__).resolve()
    _run(["docker", "cp", str(script_path), f"{container}:{SCRIPT_IN_CONTAINER}"])


def _maybe_start_topic_bridge(
    config: RunnerConfig,
    run_dir: Path,
) -> ManagedProcess | None:
    required_topics = _required_truth_topics(config)
    missing_topics = _missing_ros_topics(config, required_topics)
    if not missing_topics:
        print("[auto-debug] native ROS GameControl/ground-truth topics detected")
        return None
    print(
        "[auto-debug] missing ROS topics; starting bridge for "
        + ", ".join(missing_topics)
    )
    bridge = _start_topic_bridge(config, run_dir)
    _wait_for_ros_topics(config, required_topics, timeout_sec=15.0)
    return bridge


def _run_debug_build(config: RunnerConfig, run_dir: Path) -> None:
    log_path = run_dir / "debug_build.log"
    print("[auto-debug] building debug package")
    with log_path.open("w", encoding="utf-8") as fp:
        if config.local_container:
            process = subprocess.run(
                _local_ros_command(
                    config,
                    [
                        f"cd {_shell_quote(config.workdir)}",
                        f"source {_shell_quote(config.ros_setup)}",
                        _optional_source(config.extra_setup),
                        "colcon build --base-paths ros_debug --packages-select soccer_sim_debug",
                    ],
                ),
                cwd=_repo_root(),
                text=True,
                stdout=fp,
                stderr=subprocess.STDOUT,
                check=False,
            )
        else:
            env = {
                **os.environ,
                "SOCCER_DOCKER_CONTAINER": config.container,
                "SOCCER_DOCKER_WORKDIR": config.workdir,
                "SOCCER_DOCKER_ROS_SETUP": config.ros_setup,
                "SOCCER_DOCKER_EXTRA_SETUP": config.extra_setup,
                "SOCCER_FASTDDS_PROFILE": config.fastdds_profile,
            }
            process = subprocess.run(
                ["scripts/debug_build"],
                cwd=_repo_root(),
                env=env,
                text=True,
                stdout=fp,
                stderr=subprocess.STDOUT,
                check=False,
            )
    if process.returncode != 0:
        raise RuntimeError(f"debug build failed; see {log_path}")


def _wait_for_game_control(config: RunnerConfig, timeout_sec: float) -> None:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        try:
            status = _game_control_get(config, "/status", timeout_sec=2.0)
        except RuntimeError:
            time.sleep(0.5)
            continue
        if isinstance(status.get("game"), dict):
            return
        time.sleep(0.5)
    raise RuntimeError(f"GameControl HTTP API did not become ready: {config.control_url}")


def _start_team_nodes(config: RunnerConfig, run_dir: Path) -> list[ManagedProcess]:
    return [
        _start_team_node(config, config.team1, run_dir),
        _start_team_node(config, config.team2, run_dir),
    ]


def _start_team_node(config: RunnerConfig, team: TeamSpec, run_dir: Path) -> ManagedProcess:
    log_path = run_dir / f"team{team.team_id}.node.log"
    remote_log_dir = f"/tmp/booster_agent/soccer_auto_debug/{config.run_id}"
    ros_args = [
        "--ros-args",
        "-r",
        f"__node:=soccer_auto_debug_team{team.team_id}",
        "-p",
        f"team_id:={team.team_id}",
        "-p",
        f"robot_names:={team.robot_names}",
        "-p",
        f"opponent_robot_names:={team.opponent_robot_names}",
        "-p",
        f"ready_slots:={team.ready_slots}",
        "-p",
        f"control_hz:={config.control_hz}",
    ]
    command = _runtime_command(
        config,
        [
            f"cd {_shell_quote(config.workdir)}",
            f"source {_shell_quote(config.ros_setup)}",
            _optional_source(config.extra_setup),
            "test -f install/setup.bash",
            "source install/setup.bash",
            _fastdds_export(config.fastdds_profile),
            f"export SOCCER_RUN_ID={_shell_quote(config.run_id)}",
            f"export SOCCER_LOG_DIR={_shell_quote(remote_log_dir)}",
            "export SOCCER_PRETTY_LOG=off",
            "export SOCCER_TELEMETRY=jsonl",
            "exec ros2 run soccer_sim_debug soccer_sim_node "
            + " ".join(_shell_quote(arg) for arg in ros_args),
        ],
    )
    fp = log_path.open("w", encoding="utf-8")
    process = subprocess.Popen(
        command,
        cwd=_repo_root(),
        text=True,
        stdout=fp,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    fp.close()
    print(f"[auto-debug] started team{team.team_id} node pid={process.pid}")
    cleanup_needle = f"__node:=soccer_auto_debug_team{team.team_id}"
    cleanup_script = (
        "import os,signal,sys,time\n"
        "needle=sys.argv[1]\n"
        "own={os.getpid(),os.getppid()}\n"
        "matches=[]\n"
        "for name in os.listdir('/proc'):\n"
        "    if not name.isdigit():\n"
        "        continue\n"
        "    pid=int(name)\n"
        "    if pid in own:\n"
        "        continue\n"
        "    try:\n"
        "        raw=open(f'/proc/{pid}/cmdline','rb').read()\n"
        "    except OSError:\n"
        "        continue\n"
        "    parts=[p.decode('utf-8','replace') for p in raw.split(b'\\0') if p]\n"
        "    if needle in ' '.join(parts) and any('soccer_sim_node' in p for p in parts):\n"
        "        matches.append(pid)\n"
        "for pid in matches:\n"
        "    try:\n"
        "        os.kill(pid,signal.SIGTERM)\n"
        "    except ProcessLookupError:\n"
        "        pass\n"
        "time.sleep(0.5)\n"
        "for pid in matches:\n"
        "    try:\n"
        "        os.kill(pid,0)\n"
        "    except ProcessLookupError:\n"
        "        continue\n"
        "    try:\n"
        "        os.kill(pid,signal.SIGKILL)\n"
        "    except ProcessLookupError:\n"
        "        pass\n"
    )
    cleanup_command = _runtime_command(
        config,
        [f"python3 -c {_shell_quote(cleanup_script)} {_shell_quote(cleanup_needle)}"],
    )
    return ManagedProcess(f"team{team.team_id}", process, log_path, cleanup_command)


def _wait_for_debug_logs(config: RunnerConfig, timeout_sec: float) -> None:
    remote_dir = f"/tmp/booster_agent/soccer_auto_debug/{config.run_id}/{config.run_id}"
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        result = subprocess.run(
            _runtime_command(
                config,
                [f"find {_shell_quote(remote_dir)} -name '*.jsonl' 2>/dev/null | wc -l"],
            ),
            cwd=_repo_root(),
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        try:
            count = int(result.stdout.strip() or "0")
        except ValueError:
            count = 0
        if count >= 2:
            return
        time.sleep(0.5)
    print("[auto-debug] warning: debug node logs were not visible before episode start")


def _start_topic_bridge(config: RunnerConfig, run_dir: Path) -> ManagedProcess:
    log_path = run_dir / "topic_bridge.log"
    command = _runtime_command(
        config,
        [
            f"cd {_shell_quote(config.workdir)} || cd /tmp",
            f"source {_shell_quote(config.ros_setup)}",
            _optional_source(config.extra_setup),
            _fastdds_export(config.fastdds_profile),
            "exec python3 "
            + _shell_quote(_runtime_script_path(config))
            + " --inside-bridge"
            + f" --control-url {_shell_quote(config.control_url)}"
            + f" --world-ws-url {_shell_quote(DEFAULT_WORLD_WS_URL)}"
            + f" --team1-id {_shell_quote(str(config.team1.team_id))}"
            + f" --team2-id {_shell_quote(str(config.team2.team_id))}",
        ],
    )
    fp = log_path.open("w", encoding="utf-8")
    process = subprocess.Popen(
        command,
        cwd=_repo_root(),
        text=True,
        stdout=fp,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    fp.close()
    print(f"[auto-debug] started topic bridge pid={process.pid}")
    return ManagedProcess("topic_bridge", process, log_path)


def _start_observer(config: RunnerConfig, run_dir: Path) -> ManagedProcess:
    log_path = run_dir / "observer.log"
    sample_path = run_dir / "observer.samples.jsonl"
    duration = _observer_duration_sec(config)
    command = _runtime_command(
        config,
        [
            f"cd {_shell_quote(config.workdir)} || cd /tmp",
            f"source {_shell_quote(config.ros_setup)}",
            _optional_source(config.extra_setup),
            _fastdds_export(config.fastdds_profile),
            "exec python3 "
            + _shell_quote(_runtime_script_path(config))
            + " --inside-observer"
            + f" --duration-sec {_shell_quote(str(duration))}"
            + f" --hz {_shell_quote(str(config.observe_hz))}",
        ],
    )
    fp = log_path.open("w", encoding="utf-8")
    sample_fp = sample_path.open("w", encoding="utf-8")
    process = subprocess.Popen(
        command,
        cwd=_repo_root(),
        text=True,
        stdout=subprocess.PIPE,
        stderr=fp,
        start_new_session=True,
    )
    fp.close()
    stdout = process.stdout
    if stdout is None:
        sample_fp.close()
        raise RuntimeError("observer stdout pipe was not created")
    output_thread = threading.Thread(
        target=_copy_json_lines,
        args=(stdout, sample_fp),
        name=f"{config.run_id}-observer-json-filter",
        daemon=True,
    )
    output_thread.start()
    print(f"[auto-debug] started observer pid={process.pid}")
    return ManagedProcess(
        "observer",
        process,
        log_path,
        output_thread=output_thread,
        output_file=sample_fp,
    )


def _observer_duration_sec(config: RunnerConfig) -> float:
    if config.auto_match:
        return config.play_sec + 12.0
    return config.ready_wait_sec + config.play_sec + 12.0


def _required_truth_topics(config: RunnerConfig) -> list[str]:
    first_team_robot = _first_robot_name(config.team1.robot_names)
    second_team_robot = _first_robot_name(config.team2.robot_names)
    result = [
        "/soccer/game_controller",
        f"/team{config.team1.team_id}/soccer/sim/ground_truth/ball",
        f"/team{config.team2.team_id}/soccer/sim/ground_truth/ball",
    ]
    if first_team_robot is not None:
        result.append(
            f"/team{config.team1.team_id}/{first_team_robot}/soccer/sim/ground_truth/robot_pose"
        )
    if second_team_robot is not None:
        result.append(
            f"/team{config.team2.team_id}/{second_team_robot}/soccer/sim/ground_truth/robot_pose"
        )
    return result


def _first_robot_name(value: str) -> str | None:
    names = [item.strip() for item in value.split(",") if item.strip()]
    if not names:
        return None
    return names[0]


def _missing_ros_topics(
    config: RunnerConfig,
    required_topics: Sequence[str],
) -> list[str]:
    available = _ros_topic_list(config)
    return [topic for topic in required_topics if topic not in available]


def _wait_for_ros_topics(
    config: RunnerConfig,
    required_topics: Sequence[str],
    timeout_sec: float,
) -> None:
    deadline = time.monotonic() + timeout_sec
    last_missing = list(required_topics)
    while time.monotonic() < deadline:
        last_missing = _missing_ros_topics(config, required_topics)
        if not last_missing:
            return
        time.sleep(0.5)
    raise RuntimeError(
        "required ROS topics did not appear: " + ", ".join(last_missing)
    )


def _ros_topic_list(config: RunnerConfig) -> set[str]:
    result = subprocess.run(
        _runtime_command(
            config,
            [
                f"source {_shell_quote(config.ros_setup)}",
                _optional_source(config.extra_setup),
                _fastdds_export(config.fastdds_profile),
                "ros2 topic list",
            ],
        ),
        cwd=_repo_root(),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if result.returncode != 0:
        return set()
    return {line.strip() for line in result.stdout.splitlines() if line.strip()}


def _run_episode(config: RunnerConfig, run_dir: Path) -> None:
    timeline: list[JsonObject] = []
    match_started = False
    try:
        _match_control(config, "/match/start", timeline, "match_start")
        match_started = True
        if not config.auto_match:
            timeline.append(
                {
                    "label": "manual_control_unavailable",
                    "wall_time": time.time(),
                    "payload": {
                        "message": (
                            "Current GameControl only exposes automatic match "
                            "control; ready/set/playing requests are not sent."
                        )
                    },
                }
            )
        time.sleep(_episode_active_duration_sec(config))
        status = _game_control_get(config, "/status")
        timeline.append(
            {
                "label": "final_status",
                "wall_time": time.time(),
                "payload": _json_value(status),
            }
        )
    finally:
        if match_started:
            try:
                _match_control(config, "/match/end", timeline, "match_end")
            except RuntimeError as exc:
                timeline.append(
                    {
                        "label": "match_end_error",
                        "wall_time": time.time(),
                        "error": str(exc),
                    }
                )
        try:
            result = _game_control_get(config, "/result")
            timeline.append(
                {
                    "label": "result",
                    "wall_time": time.time(),
                    "payload": _json_value(result),
                }
            )
        except RuntimeError as exc:
            timeline.append(
                {
                    "label": "result_error",
                    "wall_time": time.time(),
                    "error": str(exc),
                }
            )
        _write_json(run_dir / "timeline.json", {"events": timeline})


def _episode_active_duration_sec(config: RunnerConfig) -> float:
    if config.auto_match:
        return config.play_sec
    return config.ready_wait_sec + config.play_sec


def _match_control(
    config: RunnerConfig,
    path: str,
    timeline: list[JsonObject],
    label: str,
) -> JsonObject:
    response = _game_control_post(config, path, {})
    timeline.append(
        {
            "label": label,
            "wall_time": time.time(),
            "request": {},
            "response": _json_value(response),
        }
    )
    game = response.get("game")
    if isinstance(game, dict):
        state = str(game.get("state"))
        set_play = str(game.get("setPlay"))
        stopped = bool(game.get("stopped"))
        print(f"[auto-debug] {label}: state={state} setPlay={set_play} stopped={stopped}")
    else:
        print(f"[auto-debug] {label}: ok")
    return response


def _game_control_get(
    config: RunnerConfig,
    path: str,
    timeout_sec: float = 5.0,
) -> JsonObject:
    return _container_http_json(
        config,
        "GET",
        f"{config.control_url}{path}",
        None,
        timeout_sec,
    )


def _game_control_post(
    config: RunnerConfig,
    path: str,
    payload: Mapping[str, JsonValue],
    timeout_sec: float = 5.0,
) -> JsonObject:
    return _container_http_json(
        config,
        "POST",
        f"{config.control_url}{path}",
        payload,
        timeout_sec,
    )


def _container_http_json(
    config: RunnerConfig,
    method: str,
    url: str,
    payload: Mapping[str, JsonValue] | None,
    timeout_sec: float,
) -> JsonObject:
    if config.local_container:
        return _http_json_direct(method, url, payload, timeout_sec)
    command = [
        "docker",
        "exec",
        config.container,
        "python3",
        SCRIPT_IN_CONTAINER,
        "--inside-control",
        "--method",
        method,
        "--url",
        url,
        "--timeout-sec",
        str(timeout_sec),
    ]
    if payload is not None:
        command.extend(
            [
                "--payload",
                json.dumps(payload, separators=(",", ":"), ensure_ascii=False),
            ]
        )
    result = _run(command, capture_output=True, check=False)
    if result.returncode != 0:
        detail = (result.stderr or result.stdout).strip()
        raise RuntimeError(f"container HTTP {method} {url} failed: {detail}")
    try:
        parsed = json.loads(result.stdout or "{}")
    except json.JSONDecodeError as exc:
        raise RuntimeError(
            f"container HTTP {method} {url} returned invalid JSON: {result.stdout!r}"
        ) from exc
    if not isinstance(parsed, dict):
        raise RuntimeError(f"container HTTP {method} {url} did not return an object")
    return _json_object(parsed)


def _http_json_direct(
    method: str,
    url: str,
    payload: Mapping[str, JsonValue] | None,
    timeout_sec: float,
) -> JsonObject:
    data: bytes | None = None
    headers: dict[str, str] = {}
    if payload is not None:
        data = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        headers["Content-Type"] = "application/json"
    request = urllib.request.Request(url, data=data, headers=headers, method=method)
    try:
        with urllib.request.urlopen(request, timeout=timeout_sec) as response:
            body = response.read().decode("utf-8")
    except (urllib.error.URLError, TimeoutError) as exc:
        raise RuntimeError(f"HTTP {method} {url} failed: {exc}") from exc
    parsed = json.loads(body or "{}")
    if not isinstance(parsed, dict):
        raise RuntimeError(f"HTTP {method} {url} did not return a JSON object")
    return _json_object(parsed)


def _build_report(
    config: RunnerConfig,
    run_dir: Path,
    managed: Sequence[ManagedProcess],
    observer: ManagedProcess | None,
) -> JsonObject:
    samples = _read_jsonl(run_dir / "observer.samples.jsonl")
    telemetry = _collect_remote_telemetry(config, run_dir)
    timeline = _read_json(run_dir / "timeline.json")
    final_status = _extract_final_status(timeline)
    result_status = _extract_timeline_payload(timeline, "result")
    game_control = _collect_game_control_artifacts(
        config,
        run_dir,
        final_status,
        result_status,
        timeline,
    )
    score, metrics, findings = _score_episode(
        samples,
        telemetry,
        final_status,
        game_control.events,
        timeline,
        auto_match_requested=config.auto_match,
    )
    processes = [
        _process_record(process)
        for process in ([*managed, observer] if observer is not None else list(managed))
    ]
    return {
        "run_id": config.run_id,
        "score": score,
        "metrics": metrics,
        "findings": findings,
        "artifacts": {
            "run_dir": str(run_dir),
            "observer_samples": str(run_dir / "observer.samples.jsonl"),
            "telemetry_dir": str(run_dir / "telemetry"),
            "timeline": str(run_dir / "timeline.json"),
            "game_control_dir": str(run_dir / "game_control"),
            "game_control_copied_files": game_control.copied_files,
        },
        "processes": processes,
        "final_status": _json_value(final_status),
        "result_status": _json_value(result_status),
    }


def _collect_remote_telemetry(config: RunnerConfig, run_dir: Path) -> list[JsonObject]:
    local_dir = run_dir / "telemetry"
    local_dir.mkdir(parents=True, exist_ok=True)
    remote_root = f"/tmp/booster_agent/soccer_auto_debug/{config.run_id}"
    if config.local_container:
        records: list[JsonObject] = []
        for remote_path in Path(remote_root).rglob("*.jsonl"):
            local_path = local_dir / remote_path.name
            try:
                shutil.copy2(remote_path, local_path)
            except OSError:
                continue
            records.extend(_read_jsonl(local_path))
        return records
    result = _run(
        [
            "docker",
            "exec",
            config.container,
            "bash",
            "-lc",
            f"find {_shell_quote(remote_root)} -name '*.jsonl' -print 2>/dev/null",
        ],
        capture_output=True,
        check=False,
    )
    records: list[JsonObject] = []
    for line in result.stdout.splitlines():
        remote_path = line.strip()
        if not remote_path:
            continue
        local_path = local_dir / Path(remote_path).name
        _run(["docker", "cp", f"{config.container}:{remote_path}", str(local_path)], check=False)
        records.extend(_read_jsonl(local_path))
    return records


def _collect_game_control_artifacts(
    config: RunnerConfig,
    run_dir: Path,
    final_status: JsonObject,
    result_status: JsonObject,
    timeline: JsonObject,
) -> GameControlArtifacts:
    local_dir = run_dir / "game_control"
    local_dir.mkdir(parents=True, exist_ok=True)
    remote_dir = _game_control_artifact_dir(config)
    copied_files: list[str] = []
    for name in ("events.jsonl", "readiness.jsonl", "status.json", "result.json", "gameControl.jsonl"):
        if _copy_game_control_file(config, remote_dir, name, local_dir / name):
            copied_files.append(name)
    raw_events = _read_jsonl(local_dir / "events.jsonl")
    readiness = _read_jsonl(local_dir / "readiness.jsonl")
    match_id = _match_id_from_status(final_status, result_status)
    events = _events_for_match(raw_events, match_id, timeline)
    if raw_events and events != raw_events:
        _write_jsonl(local_dir / "events.filtered.jsonl", events)
        copied_files.append("events.filtered.jsonl")
    return GameControlArtifacts(events=events, readiness=readiness, copied_files=copied_files)


def _game_control_artifact_dir(config: RunnerConfig) -> str:
    return config.game_control_artifact_dir.rstrip("/")


def _copy_game_control_file(
    config: RunnerConfig,
    remote_dir: str,
    name: str,
    local_path: Path,
) -> bool:
    remote_path = f"{remote_dir}/{name}"
    if config.local_container:
        try:
            shutil.copy2(Path(remote_path), local_path)
        except OSError:
            return False
        return True
    result = _run(
        ["docker", "cp", f"{config.container}:{remote_path}", str(local_path)],
        capture_output=True,
        check=False,
    )
    return result.returncode == 0


def _score_episode(
    samples: Sequence[JsonObject],
    telemetry: Sequence[JsonObject],
    final_status: JsonObject,
    game_control_events: Sequence[JsonObject] = (),
    timeline: JsonObject | None = None,
    *,
    auto_match_requested: bool = False,
) -> tuple[float, JsonObject, list[JsonObject]]:
    gc_samples = [sample for sample in samples if sample.get("type") == "sample"]
    states = _sample_states(gc_samples)
    ball_points = _ball_points(gc_samples)
    team_scores = _team_scores(final_status)
    player_penalties = _player_penalties(final_status)
    player_warnings = _player_warning_totals(final_status)
    referee_penalty_events = _events_of_type(game_control_events, "penalty")
    referee_warning_events = [
        event for event in game_control_events if str(event.get("severity")) == "warning"
    ]
    auto_match_enabled_seen = _auto_match_enabled_seen(timeline or {})
    telemetry_events = [str(record.get("event")) for record in telemetry]

    gc_seen = len(states) > 0
    ready_seen = "READY" in states
    playing_seen = "PLAYING" in states
    first_kick_sec = _first_event_offset_sec(telemetry, "soccer_kick_started")
    stale_count = telemetry_events.count("game_controller_stale")
    warn_count = sum(1 for record in telemetry if str(record.get("level")) in {"WARN", "ERROR"})
    telemetry_seen = bool(telemetry)
    max_ball_x = max((point[1] for point in ball_points), default=None)
    min_ball_x = min((point[1] for point in ball_points), default=None)
    ball_progress = (
        round(max_ball_x - ball_points[0][1], 3)
        if max_ball_x is not None and ball_points
        else None
    )

    score = 0.0
    findings: list[JsonObject] = []
    score += _award(gc_seen, 20.0, findings, "裁判 topic 可见", "未采集到 /soccer/game_controller")
    score += _award(ready_seen, 15.0, findings, "READY 状态可见", "未观察到 READY")
    score += _award(playing_seen, 15.0, findings, "PLAYING 状态可见", "未观察到 PLAYING")
    score += _award(first_kick_sec is not None, 20.0, findings, "检测到 soccer_kick_started", "未检测到自动踢球启动")
    score += _award(bool(ball_points), 10.0, findings, "采集到球真值", "未采集到球真值")
    if ball_progress is not None and ball_progress > 0.2:
        score += 10.0
        findings.append({"ok": True, "message": f"球向进攻方向推进 {ball_progress:.2f}m"})
    else:
        findings.append({"ok": False, "message": "球向进攻方向推进不足"})
    score += _award(
        telemetry_seen and stale_count == 0,
        5.0,
        findings,
        "无 GameController stale",
        f"GameController stale 次数={stale_count} telemetry_seen={telemetry_seen}",
    )
    score += _award(
        telemetry_seen and warn_count == 0,
        5.0,
        findings,
        "无 WARN/ERROR telemetry",
        f"WARN/ERROR telemetry 次数={warn_count} telemetry_seen={telemetry_seen}",
    )
    if auto_match_requested:
        if auto_match_enabled_seen:
            findings.append({"ok": True, "message": "真实 Auto Match 已启用"})
        else:
            score -= 10.0
            findings.append({"ok": False, "message": "未确认真实 Auto Match 启用"})
    if game_control_events:
        penalty_count = len(referee_penalty_events)
        warning_count = len(referee_warning_events)
        if penalty_count == 0:
            findings.append({"ok": True, "message": "无自动裁判 penalty 事件"})
        else:
            score -= min(20.0, 4.0 * penalty_count)
            findings.append({"ok": False, "message": f"自动裁判 penalty 事件={penalty_count}"})
        if warning_count == 0:
            findings.append({"ok": True, "message": "无自动裁判 warning 事件"})
        else:
            score -= min(10.0, 2.0 * warning_count)
            findings.append({"ok": False, "message": f"自动裁判 warning 事件={warning_count}"})
    if player_penalties:
        score -= min(10.0, 2.0 * len(player_penalties))
        findings.append({"ok": False, "message": f"最终仍有被罚机器人={len(player_penalties)}"})
    elif auto_match_requested or game_control_events:
        findings.append({"ok": True, "message": "最终无场上机器人处于 penalty"})

    metrics: JsonObject = {
        "sample_count": len(samples),
        "game_controller_sample_count": len(states),
        "states_seen": sorted(set(states)),
        "auto_match_enabled_seen": auto_match_enabled_seen,
        "first_soccer_kick_sec": first_kick_sec,
        "telemetry_seen": telemetry_seen,
        "game_controller_stale_count": stale_count,
        "warn_or_error_count": warn_count,
        "ball_sample_count": len(ball_points),
        "ball_min_x": min_ball_x,
        "ball_max_x": max_ball_x,
        "ball_progress_x": ball_progress,
        "team_scores": team_scores,
        "telemetry_event_counts": _event_counts(telemetry),
        "game_control_event_count": len(game_control_events),
        "game_control_event_counts": _event_counts(game_control_events),
        "referee_penalty_count": len(referee_penalty_events),
        "referee_warning_count": len(referee_warning_events),
        "referee_penalty_types": _nested_event_counts(referee_penalty_events, ("outcome", "penalty")),
        "referee_penalty_reasons": _nested_event_counts(referee_penalty_events, ("outcome", "reason")),
        "referee_penalty_rule_ids": _nested_event_counts(referee_penalty_events, ("evidence", "ruleId")),
        "referee_penalized_actors": _event_actors(referee_penalty_events),
        "final_player_penalty_count": len(player_penalties),
        "final_player_penalties": player_penalties,
        "final_player_warnings": player_warnings,
    }
    return round(min(100.0, max(0.0, score)), 1), metrics, findings


def _award(
    condition: bool,
    points: float,
    findings: list[JsonObject],
    ok_message: str,
    fail_message: str,
) -> float:
    findings.append({"ok": condition, "message": ok_message if condition else fail_message})
    return points if condition else 0.0


def _sample_states(samples: Sequence[JsonObject]) -> list[str]:
    states: list[str] = []
    for sample in samples:
        gc = sample.get("game_controller")
        if isinstance(gc, dict):
            state = gc.get("state")
            if isinstance(state, str):
                states.append(state)
    return states


def _ball_points(samples: Sequence[JsonObject]) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    for sample in samples:
        wall_time = _number(sample.get("wall_time"), 0.0)
        ball = sample.get("team1_ball")
        if isinstance(ball, dict):
            x_value = ball.get("x")
            y_value = ball.get("y")
            if isinstance(x_value, (int, float)) and isinstance(y_value, (int, float)):
                points.append((wall_time, float(x_value), float(y_value)))
    return points


def _team_scores(final_status: JsonObject) -> JsonObject:
    game = final_status.get("game")
    if not isinstance(game, dict):
        return {}
    teams = game.get("teams")
    if not isinstance(teams, dict):
        return {}
    scores: JsonObject = {}
    for side in ("home", "away"):
        team = teams.get(side)
        if isinstance(team, dict):
            score = team.get("score")
            if isinstance(score, (int, float)):
                scores[side] = int(score)
    return scores


def _first_event_offset_sec(records: Sequence[JsonObject], event_name: str) -> float | None:
    matching = [
        _number(record.get("monotonic_sec"), 0.0)
        for record in records
        if record.get("event") == event_name
    ]
    all_times = [_number(record.get("monotonic_sec"), 0.0) for record in records]
    if not matching or not all_times:
        return None
    return round(min(matching) - min(all_times), 3)


def _event_counts(records: Sequence[JsonObject]) -> JsonObject:
    counts: dict[str, int] = {}
    for record in records:
        event = record.get("event")
        if not isinstance(event, str):
            event = record.get("type")
        if isinstance(event, str):
            counts[event] = counts.get(event, 0) + 1
    return dict(sorted(counts.items()))


def _nested_event_counts(records: Sequence[JsonObject], path: Sequence[str]) -> JsonObject:
    counts: dict[str, int] = {}
    for record in records:
        value: object = record
        for key in path:
            if not isinstance(value, dict):
                value = None
                break
            value = value.get(key)
        if isinstance(value, str) and value:
            counts[value] = counts.get(value, 0) + 1
    return dict(sorted(counts.items()))


def _events_of_type(records: Sequence[JsonObject], event_type: str) -> list[JsonObject]:
    return [record for record in records if record.get("type") == event_type]


def _event_actors(records: Sequence[JsonObject]) -> list[JsonObject]:
    actors: list[JsonObject] = []
    seen: set[tuple[str, str, str]] = set()
    for record in records:
        actor = record.get("actor")
        if not isinstance(actor, dict):
            continue
        side = actor.get("side")
        robot_name = actor.get("robotName")
        player_number = actor.get("playerNumber")
        if not isinstance(side, str):
            continue
        robot_text = str(robot_name) if robot_name is not None else ""
        player_text = str(player_number) if player_number is not None else ""
        key = (side, robot_text, player_text)
        if key in seen:
            continue
        seen.add(key)
        item: JsonObject = {"side": side}
        if isinstance(robot_name, str):
            item["robotName"] = robot_name
        if isinstance(player_number, (int, float)):
            item["playerNumber"] = int(player_number)
        actors.append(item)
    return actors


def _extract_final_status(timeline: JsonObject) -> JsonObject:
    return _extract_timeline_payload(timeline, "final_status")


def _extract_timeline_payload(timeline: JsonObject, label: str) -> JsonObject:
    events = timeline.get("events")
    if not isinstance(events, list):
        return {}
    for event in reversed(events):
        if isinstance(event, dict) and event.get("label") == label:
            payload = event.get("payload")
            if isinstance(payload, dict):
                return _json_object(payload)
    return {}


def _match_id_from_status(final_status: JsonObject, result_status: JsonObject) -> str | None:
    match = final_status.get("match")
    if isinstance(match, dict):
        match_id = match.get("matchId")
        if isinstance(match_id, str) and match_id:
            return match_id
    result_match_id = result_status.get("matchId")
    if isinstance(result_match_id, str) and result_match_id:
        return result_match_id
    return None


def _events_for_match(
    events: Sequence[JsonObject],
    match_id: str | None,
    timeline: JsonObject | None = None,
) -> list[JsonObject]:
    wall_window = _timeline_wall_time_window(timeline or {})
    filtered: list[JsonObject] = []
    for event in events:
        event_match_id = event.get("matchId")
        if match_id is not None and event_match_id != match_id:
            continue
        if wall_window is not None and not _event_inside_wall_window(
            event,
            wall_window,
        ):
            continue
        if match_id is not None or wall_window is not None:
            filtered.append(event)
    if match_id is None and wall_window is None:
        return list(events)
    return filtered


def _timeline_wall_time_window(timeline: JsonObject) -> tuple[float, float] | None:
    events = timeline.get("events")
    if not isinstance(events, list):
        return None
    start: float | None = None
    end: float | None = None
    for event in events:
        if not isinstance(event, dict):
            continue
        label = event.get("label")
        wall_time = _number(event.get("wall_time"), -1.0)
        if wall_time < 0.0:
            continue
        if label == "match_start" and start is None:
            start = wall_time
        if label in {"match_end", "final_status", "result"}:
            end = wall_time
    if start is None:
        return None
    if end is None or end < start:
        end = start
    return (start - 1.0, end + 1.0)


def _event_inside_wall_window(
    event: JsonObject,
    wall_window: tuple[float, float],
) -> bool:
    wall_time = _number(event.get("wallTime"), -1.0)
    if wall_time < 0.0:
        return False
    return wall_window[0] <= wall_time <= wall_window[1]


def _auto_match_enabled_seen(timeline: JsonObject) -> bool:
    events = timeline.get("events")
    if not isinstance(events, list):
        return False
    for event in events:
        if not isinstance(event, dict):
            continue
        if event.get("label") == "match_start" and isinstance(event.get("response"), dict):
            return True
        response = event.get("response")
        if isinstance(response, dict) and response.get("autoMatchEnabled") is True:
            return True
        payload = event.get("payload")
        if isinstance(payload, dict) and payload.get("autoMatchEnabled") is True:
            return True
    return False


def _player_penalties(final_status: JsonObject) -> list[JsonObject]:
    penalties: list[JsonObject] = []
    teams = _status_teams(final_status)
    for side, team in teams:
        players = team.get("players")
        if not isinstance(players, list):
            continue
        for index, player in enumerate(players, start=1):
            if not isinstance(player, dict):
                continue
            penalty = player.get("penalty")
            penalty_text = str(penalty) if penalty is not None else ""
            if _is_no_player_penalty(penalty_text):
                continue
            item: JsonObject = {
                "side": side,
                "playerNumber": index,
                "penalty": penalty_text,
            }
            secs = player.get("secsTillUnpenalized")
            if isinstance(secs, (int, float)):
                item["secsTillUnpenalized"] = round(float(secs), 3)
            penalties.append(item)
    return penalties


def _player_warning_totals(final_status: JsonObject) -> JsonObject:
    totals: dict[str, int] = {}
    teams = _status_teams(final_status)
    for side, team in teams:
        warnings = 0
        cautions = 0
        players = team.get("players")
        if isinstance(players, list):
            for player in players:
                if not isinstance(player, dict):
                    continue
                penalty = player.get("penalty")
                penalty_text = str(penalty) if penalty is not None else ""
                if penalty_text == "substitute":
                    continue
                warning_value = player.get("warnings")
                caution_value = player.get("cautions")
                if isinstance(warning_value, (int, float)):
                    warnings += int(warning_value)
                if isinstance(caution_value, (int, float)):
                    cautions += int(caution_value)
        totals[f"{side}_warnings"] = warnings
        totals[f"{side}_cautions"] = cautions
    return dict(sorted(totals.items()))


def _status_teams(final_status: JsonObject) -> list[tuple[str, JsonObject]]:
    game = final_status.get("game")
    if not isinstance(game, dict):
        return []
    teams = game.get("teams")
    if not isinstance(teams, dict):
        return []
    result: list[tuple[str, JsonObject]] = []
    for side in ("home", "away"):
        team = teams.get(side)
        if isinstance(team, dict):
            result.append((side, _json_object(team)))
    return result


def _is_no_player_penalty(penalty: str) -> bool:
    return penalty in {"", "0", "none", "NONE", "noPenalty", "substitute"}


def _process_record(managed: ManagedProcess | None) -> JsonObject:
    if managed is None:
        return {}
    return {
        "name": managed.name,
        "pid": managed.process.pid,
        "returncode": managed.process.poll(),
        "log_path": str(managed.log_path),
    }


def _write_markdown_report(path: Path, report: JsonObject) -> None:
    metrics = report.get("metrics")
    metrics_obj = metrics if isinstance(metrics, dict) else {}
    findings = report.get("findings")
    finding_items = findings if isinstance(findings, list) else []
    lines = [
        f"# Soccer Auto Debug Report",
        "",
        f"- run_id: `{report.get('run_id')}`",
        f"- score: `{report.get('score')}/100`",
        f"- states_seen: `{metrics_obj.get('states_seen')}`",
        f"- auto_match_enabled_seen: `{metrics_obj.get('auto_match_enabled_seen')}`",
        f"- first_soccer_kick_sec: `{metrics_obj.get('first_soccer_kick_sec')}`",
        f"- ball_progress_x: `{metrics_obj.get('ball_progress_x')}`",
        f"- stale_count: `{metrics_obj.get('game_controller_stale_count')}`",
        f"- warn_or_error_count: `{metrics_obj.get('warn_or_error_count')}`",
        f"- referee_penalty_count: `{metrics_obj.get('referee_penalty_count')}`",
        f"- referee_warning_count: `{metrics_obj.get('referee_warning_count')}`",
        f"- final_player_penalty_count: `{metrics_obj.get('final_player_penalty_count')}`",
        "",
        "## Findings",
        "",
    ]
    for item in finding_items:
        if isinstance(item, dict):
            marker = "PASS" if item.get("ok") else "FAIL"
            lines.append(f"- {marker}: {item.get('message')}")
    lines.extend(
        [
            "",
            "## Artifacts",
            "",
        ]
    )
    artifacts = report.get("artifacts")
    if isinstance(artifacts, dict):
        for key, value in artifacts.items():
            lines.append(f"- {key}: `{value}`")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _terminate_process(managed: ManagedProcess) -> None:
    print(f"[auto-debug] stopping {managed.name} pid={managed.process.pid}")
    _stop_process(managed)
    _join_output_thread(managed)
    if managed.cleanup_command is not None:
        subprocess.run(
            list(managed.cleanup_command),
            cwd=_repo_root(),
            text=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )


def _stop_process(managed: ManagedProcess) -> None:
    if managed.process.poll() is None:
        _signal_process_group(managed.process, signal.SIGTERM)
        try:
            managed.process.wait(timeout=8.0)
        except subprocess.TimeoutExpired:
            _signal_process_group(managed.process, signal.SIGKILL)
            managed.process.wait(timeout=5.0)


def _join_output_thread(managed: ManagedProcess) -> None:
    if managed.output_thread is not None:
        managed.output_thread.join(timeout=3.0)
    if managed.output_file is not None and not managed.output_file.closed:
        managed.output_file.close()


def _copy_json_lines(source: TextIO, destination: TextIO) -> None:
    try:
        for line in source:
            if _is_json_object_line(line):
                destination.write(line)
    finally:
        source.close()
        destination.flush()
        destination.close()


def _is_json_object_line(line: str) -> bool:
    stripped = line.strip()
    if not stripped.startswith("{"):
        return False
    try:
        parsed = json.loads(stripped)
    except json.JSONDecodeError:
        return False
    return isinstance(parsed, dict)


def _signal_process_group(process: subprocess.Popen[str], signum: int) -> None:
    try:
        os.killpg(process.pid, signum)
    except ProcessLookupError:
        return
    except PermissionError:
        if process.poll() is None:
            if signum == signal.SIGTERM:
                process.terminate()
            else:
                process.kill()


def _docker_ros_command(config: RunnerConfig, commands: Sequence[str]) -> list[str]:
    inner = "set -eo pipefail\n" + "\n".join(command for command in commands if command)
    return ["docker", "exec", "-i", config.container, "bash", "-lc", inner]


def _local_ros_command(config: RunnerConfig, commands: Sequence[str]) -> list[str]:
    inner = "set -eo pipefail\n" + "\n".join(command for command in commands if command)
    return ["bash", "-lc", inner]


def _runtime_command(config: RunnerConfig, commands: Sequence[str]) -> list[str]:
    if config.local_container:
        return _local_ros_command(config, commands)
    return _docker_ros_command(config, commands)


def _runtime_script_path(config: RunnerConfig) -> str:
    if config.local_container:
        return str(Path(__file__).resolve())
    return SCRIPT_IN_CONTAINER


def _optional_source(path: str) -> str:
    if not path:
        return ""
    return f"source {_shell_quote(path)}"


def _fastdds_export(path: str) -> str:
    return (
        f"if [ -f {_shell_quote(path)} ]; then "
        f"export FASTRTPS_DEFAULT_PROFILES_FILE=${{FASTRTPS_DEFAULT_PROFILES_FILE:-{_shell_quote(path)}}}; "
        "fi"
    )


def _shell_quote(value: str) -> str:
    return "'" + value.replace("'", "'\"'\"'") + "'"


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _run(
    args: Sequence[str],
    *,
    capture_output: bool = False,
    check: bool = True,
) -> subprocess.CompletedProcess[str]:
    result = subprocess.run(
        list(args),
        cwd=_repo_root(),
        text=True,
        stdout=subprocess.PIPE if capture_output else None,
        stderr=subprocess.PIPE if capture_output else None,
        check=False,
    )
    if check and result.returncode != 0:
        message = f"command failed ({result.returncode}): {' '.join(args)}"
        if capture_output and result.stderr:
            message += f"\n{result.stderr}"
        raise RuntimeError(message)
    return result


def _read_json(path: Path) -> JsonObject:
    try:
        parsed = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    if isinstance(parsed, dict):
        return _json_object(parsed)
    return {}


def _read_jsonl(path: Path) -> list[JsonObject]:
    records: list[JsonObject] = []
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError:
        return records
    for line in lines:
        if not line.strip():
            continue
        try:
            parsed = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(parsed, dict):
            records.append(_json_object(parsed))
    return records


def _write_json(path: Path, payload: Mapping[str, JsonValue]) -> None:
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def _write_jsonl(path: Path, records: Sequence[Mapping[str, JsonValue]]) -> None:
    lines = [
        json.dumps(record, ensure_ascii=False, separators=(",", ":"))
        for record in records
    ]
    path.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")


def _json_object(value: Mapping[object, object]) -> JsonObject:
    result: JsonObject = {}
    for key, item in value.items():
        result[str(key)] = _json_value(item)
    return result


def _json_value(value: object) -> JsonValue:
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if isinstance(value, Mapping):
        return _json_object(value)
    if isinstance(value, Sequence) and not isinstance(value, (bytes, bytearray, str)):
        return [_json_value(item) for item in value]
    return str(value)


def _number(value: object, default: float) -> float:
    if isinstance(value, (int, float)):
        return float(value)
    if isinstance(value, str):
        try:
            return float(value)
        except ValueError:
            return default
    return default


def _wrap_to_pi(angle: float) -> float:
    return float((angle + math.pi) % (2.0 * math.pi) - math.pi)


def _mirror_xy_theta(x: float, y: float, theta: float) -> tuple[float, float, float]:
    return (-x, -y, _wrap_to_pi(theta + math.pi))


def _bridge_control_message_json(message: Mapping[str, object]) -> str:
    packet = {
        "version": int(message.get("version", 19)),
        "packetNumber": int(message.get("packetNumber", 0)),
        "playersPerTeam": int(message.get("playersPerTeam", 0)),
        "competitionType": _bridge_enum_name(
            _BRIDGE_COMPETITION_TYPE,
            int(message.get("competitionType", 0)),
            "competitionType",
        ),
        "stopped": bool(message.get("stopped", 0)),
        "gamePhase": _bridge_enum_name(
            _BRIDGE_GAME_PHASE,
            int(message.get("gamePhase", 0)),
            "gamePhase",
        ),
        "state": _bridge_enum_name(
            _BRIDGE_GAME_STATE,
            int(message.get("state", 0)),
            "state",
        ),
        "setPlay": _bridge_enum_name(
            _BRIDGE_SET_PLAY,
            int(message.get("setPlay", 0)),
            "setPlay",
        ),
        "firstHalf": bool(message.get("firstHalf", 0)),
        "kickingTeam": int(message.get("kickingTeam", 255)),
        "secsRemaining": int(message.get("secsRemaining", 0)),
        "secondaryTime": int(message.get("secondaryTime", 0)),
        "teams": [
            _bridge_team_json(team)
            for team in list(message.get("teams") or [])[:2]
        ],
    }
    return json.dumps(packet, ensure_ascii=False, separators=(",", ":"))


def _bridge_team_json(team: object) -> JsonObject:
    if not isinstance(team, dict):
        team = {}
    return {
        "teamNumber": int(team.get("number", 0)),
        "fieldPlayerColour": int(team.get("fieldPlayerColor", 0)),
        "goalkeeperColour": int(team.get("goalkeeperColor", 0)),
        "goalkeeper": int(team.get("goalkeeper", 0)),
        "score": int(team.get("score", 0)),
        "penaltyShot": int(team.get("penaltyShot", 0)),
        "singleShots": int(team.get("singleShots", 0)),
        "messageBudget": int(team.get("messageBudget", 0)),
        "players": [
            _bridge_player_json(player)
            for player in list(team.get("players") or [])
        ],
    }


def _bridge_player_json(player: object) -> JsonObject:
    if not isinstance(player, dict):
        player = {}
    return {
        "penalty": _bridge_enum_name(
            _BRIDGE_PENALTY,
            int(player.get("penalty", 0)),
            "penalty",
        ),
        "secsTillUnpenalised": int(player.get("secsTillUnpenalized", 0)),
        "warnings": int(player.get("warnings", 0)),
        "cautions": int(player.get("cautions", 0)),
    }


def _bridge_enum_name(
    mapping: Mapping[int, str],
    value: int,
    field: str,
) -> str:
    try:
        return mapping[value]
    except KeyError as exc:
        raise ValueError(f"unknown {field} value: {value}") from exc


_BRIDGE_COMPETITION_TYPE = {
    0: "SMALL",
    1: "MIDDLE",
    2: "LARGE",
}
_BRIDGE_GAME_PHASE = {
    0: "NORMAL",
    1: "PENALTY_SHOOT_OUT",
    2: "EXTRA_TIME",
    3: "TIMEOUT",
}
_BRIDGE_GAME_STATE = {
    0: "INITIAL",
    1: "READY",
    2: "SET",
    3: "PLAYING",
    4: "FINISHED",
}
_BRIDGE_SET_PLAY = {
    0: "NONE",
    1: "DIRECT_FREE_KICK",
    2: "INDIRECT_FREE_KICK",
    3: "PENALTY_KICK",
    4: "THROW_IN",
    5: "GOAL_KICK",
    6: "CORNER_KICK",
}
_BRIDGE_PENALTY = {
    0: "NONE",
    1: "ILLEGAL_POSITIONING",
    2: "MOTION_IN_SET",
    3: "LOCAL_GAME_STUCK",
    4: "INCAPABLE_ROBOT",
    5: "PICKED_UP",
    6: "BALL_HOLDING",
    7: "LEAVING_THE_FIELD",
    8: "PLAYING_WITH_ARMS_HANDS",
    9: "PUSHING",
    10: "SENT_OFF",
    11: "SUBSTITUTE",
}


def _observer_main(argv: Sequence[str]) -> int:
    filtered = [arg for arg in argv if arg != "--inside-observer"]
    parser = argparse.ArgumentParser(description="Container-side ROS observer")
    parser.add_argument("--duration-sec", type=float, required=True)
    parser.add_argument("--hz", type=float, default=5.0)
    args = parser.parse_args(filtered)
    return _run_ros_observer(duration_sec=float(args.duration_sec), hz=float(args.hz))


def _inside_bridge_main(argv: Sequence[str]) -> int:
    filtered = [arg for arg in argv if arg != "--inside-bridge"]
    parser = argparse.ArgumentParser(description="Container-side ROS topic bridge")
    parser.add_argument("--control-url", required=True)
    parser.add_argument("--world-ws-url", default=DEFAULT_WORLD_WS_URL)
    parser.add_argument("--team1-id", type=int, default=1)
    parser.add_argument("--team2-id", type=int, default=2)
    parser.add_argument("--publish-hz", type=float, default=15.0)
    args = parser.parse_args(filtered)
    return _run_ros_topic_bridge(
        control_url=str(args.control_url),
        world_ws_url=str(args.world_ws_url),
        team1_id=int(args.team1_id),
        team2_id=int(args.team2_id),
        publish_hz=float(args.publish_hz),
    )


def _inside_control_main(argv: Sequence[str]) -> int:
    filtered = [arg for arg in argv if arg != "--inside-control"]
    parser = argparse.ArgumentParser(description="Container-side HTTP control helper")
    parser.add_argument("--method", choices=["GET", "POST"], required=True)
    parser.add_argument("--url", required=True)
    parser.add_argument("--payload")
    parser.add_argument("--timeout-sec", type=float, default=5.0)
    args = parser.parse_args(filtered)
    payload = None
    if args.payload is not None:
        parsed = json.loads(str(args.payload))
        if not isinstance(parsed, dict):
            raise ValueError("--payload must be a JSON object")
        payload = _json_object(parsed)
    result = _http_json_direct(
        str(args.method),
        str(args.url),
        payload,
        float(args.timeout_sec),
    )
    print(json.dumps(result, ensure_ascii=False, separators=(",", ":")), flush=True)
    return 0


def _run_ros_topic_bridge(
    *,
    control_url: str,
    world_ws_url: str,
    team1_id: int,
    team2_id: int,
    publish_hz: float,
) -> int:
    try:
        import msgpack
        import rclpy
        import websockets
        from geometry_msgs.msg import Pose2D
        from rclpy.executors import ExternalShutdownException
        from rclpy.node import Node
        from std_msgs.msg import String
    except ImportError as exc:
        print(f"[auto-debug-bridge] import failed: {exc}", file=sys.stderr, flush=True)
        return 1

    class BridgeNode(Node):
        def __init__(self) -> None:
            super().__init__("soccer_auto_debug_bridge")
            self._lock = threading.RLock()
            self._stop_threads = threading.Event()
            self._latest_gc_json: str | None = None
            self._latest_world: dict[str, object] | None = None
            self._gc_publisher = self.create_publisher(String, "/soccer/game_controller", 10)
            self._ball_publishers = {
                team1_id: self.create_publisher(
                    Pose2D,
                    f"/team{team1_id}/soccer/sim/ground_truth/ball",
                    10,
                ),
                team2_id: self.create_publisher(
                    Pose2D,
                    f"/team{team2_id}/soccer/sim/ground_truth/ball",
                    10,
                ),
            }
            self._robot_publishers: dict[str, object] = {}
            self._publish_timer = self.create_timer(
                1.0 / max(1.0, publish_hz),
                self._publish_latest,
            )
            self._gc_thread = threading.Thread(
                target=self._game_control_loop,
                name="soccer-auto-debug-gc-bridge",
                daemon=True,
            )
            self._world_thread = threading.Thread(
                target=self._world_loop,
                name="soccer-auto-debug-world-bridge",
                daemon=True,
            )
            self._gc_thread.start()
            self._world_thread.start()

        def close(self) -> None:
            self._stop_threads.set()
            self._gc_thread.join(timeout=2.0)
            self._world_thread.join(timeout=2.0)

        def _publish_latest(self) -> None:
            with self._lock:
                gc_json = self._latest_gc_json
                world = self._latest_world
            if gc_json is not None:
                gc_msg = String()
                gc_msg.data = gc_json
                self._gc_publisher.publish(gc_msg)
            if not isinstance(world, dict):
                return
            ball = world.get("ballPosition")
            if isinstance(ball, (list, tuple)) and len(ball) >= 2:
                self._publish_ball(float(ball[0]), float(ball[1]))
            robots = world.get("robots")
            if isinstance(robots, list):
                self._publish_robots(robots)

        def _publish_ball(self, x: float, y: float) -> None:
            for team_id, mirror in ((team1_id, False), (team2_id, True)):
                publish_x = x
                publish_y = y
                if mirror:
                    publish_x, publish_y, _theta = _mirror_xy_theta(x, y, 0.0)
                msg = Pose2D()
                msg.x = float(publish_x)
                msg.y = float(publish_y)
                msg.theta = 0.0
                self._ball_publishers[team_id].publish(msg)

        def _publish_robots(self, robots: Sequence[object]) -> None:
            for robot in robots:
                if not isinstance(robot, dict):
                    continue
                name = robot.get("name")
                position = robot.get("position")
                yaw_value = robot.get("yaw", 0.0)
                if not isinstance(name, str):
                    continue
                if not isinstance(position, (list, tuple)) or len(position) < 2:
                    continue
                x = float(position[0])
                y = float(position[1])
                theta = float(yaw_value) if isinstance(yaw_value, (int, float)) else 0.0
                for team_id, mirror in ((team1_id, False), (team2_id, True)):
                    topic = f"/team{team_id}/{name}/soccer/sim/ground_truth/robot_pose"
                    publisher = self._robot_publishers.get(topic)
                    if publisher is None:
                        publisher = self.create_publisher(Pose2D, topic, 10)
                        self._robot_publishers[topic] = publisher
                    publish_x = x
                    publish_y = y
                    publish_theta = theta
                    if mirror:
                        publish_x, publish_y, publish_theta = _mirror_xy_theta(
                            x,
                            y,
                            theta,
                        )
                    msg = Pose2D()
                    msg.x = float(publish_x)
                    msg.y = float(publish_y)
                    msg.theta = float(publish_theta)
                    publisher.publish(msg)

        def _game_control_loop(self) -> None:
            while not self._stop_threads.is_set():
                try:
                    request = urllib.request.Request(f"{control_url}/events", method="GET")
                    with urllib.request.urlopen(request, timeout=10.0) as response:
                        event_name: str | None = None
                        data_parts: list[str] = []
                        while not self._stop_threads.is_set():
                            raw_line = response.readline()
                            if not raw_line:
                                break
                            line = raw_line.decode("utf-8", "replace").rstrip("\r\n")
                            if not line:
                                self._consume_sse_event(event_name, data_parts)
                                event_name = None
                                data_parts = []
                                continue
                            if line.startswith("event:"):
                                event_name = line[6:].strip()
                                continue
                            if line.startswith("data:"):
                                data_parts.append(line[5:].lstrip())
                except Exception as exc:
                    self.get_logger().warn(f"GameControl bridge reconnecting after error: {exc}")
                    self._stop_threads.wait(0.5)

        def _consume_sse_event(
            self,
            event_name: str | None,
            data_parts: Sequence[str],
        ) -> None:
            if event_name != "gameControlPublication" or not data_parts:
                return
            data_text = "\n".join(data_parts)
            try:
                payload = json.loads(data_text)
            except json.JSONDecodeError as exc:
                self.get_logger().warn(f"Ignore invalid GameControl SSE JSON: {exc}")
                return
            if not isinstance(payload, dict):
                return
            control_message = payload.get("controlMessage")
            if not isinstance(control_message, dict):
                return
            try:
                gc_json = _bridge_control_message_json(control_message)
            except ValueError as exc:
                self.get_logger().warn(f"Ignore unsupported GameControl payload: {exc}")
                return
            with self._lock:
                self._latest_gc_json = gc_json

        def _world_loop(self) -> None:
            asyncio.run(self._world_loop_async())

        async def _world_loop_async(self) -> None:
            while not self._stop_threads.is_set():
                try:
                    async with websockets.connect(
                        world_ws_url,
                        max_size=10 * 1024 * 1024,
                    ) as websocket:
                        await websocket.send("subscribe:game_control_world")
                        while not self._stop_threads.is_set():
                            raw_message = await asyncio.wait_for(
                                websocket.recv(),
                                timeout=1.0,
                            )
                            if not isinstance(raw_message, bytes) or len(raw_message) < 8:
                                continue
                            msg_type, payload_len = struct.unpack("!II", raw_message[:8])
                            if msg_type != 13 or len(raw_message) < 8 + payload_len:
                                continue
                            frame = msgpack.unpackb(
                                raw_message[8 : 8 + payload_len],
                                raw=False,
                            )
                            if not isinstance(frame, dict):
                                continue
                            world = frame.get("world")
                            if isinstance(world, dict):
                                with self._lock:
                                    self._latest_world = dict(world)
                except TimeoutError:
                    continue
                except Exception as exc:
                    self.get_logger().warn(f"World bridge reconnecting after error: {exc}")
                    await asyncio.sleep(0.5)

    stop_requested = False

    def request_stop(_signum: int, _frame: FrameType | None) -> None:
        nonlocal stop_requested
        stop_requested = True

    previous_sigterm = signal.signal(signal.SIGTERM, request_stop)
    previous_sigint = signal.signal(signal.SIGINT, request_stop)
    rclpy.init(args=None)
    node = BridgeNode()
    return_code = 0
    try:
        while rclpy.ok() and not stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        if not stop_requested:
            print(
                f"[auto-debug-bridge] runtime error: {exc.__class__.__name__}: {exc}",
                file=sys.stderr,
                flush=True,
            )
            return_code = 1
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGTERM, previous_sigterm)
        signal.signal(signal.SIGINT, previous_sigint)
    return return_code


def _run_ros_observer(duration_sec: float, hz: float) -> int:
    try:
        import rclpy
        from geometry_msgs.msg import Pose2D
        from rclpy.executors import ExternalShutdownException
        from rclpy.node import Node
        from rclpy.qos import QoSProfile
        from std_msgs.msg import String
    except ImportError as exc:
        print(json.dumps({"type": "observer_error", "error": str(exc)}), flush=True)
        return 1

    class ObserverNode(Node):
        def __init__(self) -> None:
            super().__init__("soccer_auto_debug_observer")
            self.latest_gc: JsonObject | None = None
            self.team1_ball: JsonObject | None = None
            self.team2_ball: JsonObject | None = None
            self.robot_poses: dict[str, JsonObject] = {}
            qos = QoSProfile(depth=10)
            self.create_subscription(String, "/soccer/game_controller", self._gc_cb, qos)
            self.create_subscription(Pose2D, "/team1/soccer/sim/ground_truth/ball", self._team1_ball_cb, qos)
            self.create_subscription(Pose2D, "/team2/soccer/sim/ground_truth/ball", self._team2_ball_cb, qos)
            for team_id, names in ((1, ("robot1", "robot2", "robot3")), (2, ("robot4", "robot5", "robot6"))):
                for name in names:
                    topic = f"/team{team_id}/{name}/soccer/sim/ground_truth/robot_pose"
                    self.create_subscription(Pose2D, topic, self._pose_cb(topic), qos)

        def _gc_cb(self, msg: object) -> None:
            data = getattr(msg, "data", "")
            try:
                payload = json.loads(str(data))
            except json.JSONDecodeError:
                return
            if isinstance(payload, dict):
                self.latest_gc = _json_object(payload)

        def _team1_ball_cb(self, msg: object) -> None:
            self.team1_ball = _pose_msg(msg)

        def _team2_ball_cb(self, msg: object) -> None:
            self.team2_ball = _pose_msg(msg)

        def _pose_cb(self, topic: str):
            def callback(msg: object) -> None:
                self.robot_poses[topic] = _pose_msg(msg)

            return callback

    def emit_sample(node: ObserverNode) -> None:
        print(
            json.dumps(
                {
                    "type": "sample",
                    "wall_time": time.time(),
                    "game_controller": node.latest_gc,
                    "team1_ball": node.team1_ball,
                    "team2_ball": node.team2_ball,
                    "robot_poses": node.robot_poses,
                },
                ensure_ascii=False,
                separators=(",", ":"),
            ),
            flush=True,
        )

    stop_requested = False

    def request_stop(_signum: int, _frame: FrameType | None) -> None:
        nonlocal stop_requested
        stop_requested = True

    previous_sigterm = signal.signal(signal.SIGTERM, request_stop)
    previous_sigint = signal.signal(signal.SIGINT, request_stop)
    rclpy.init(args=None)
    node = ObserverNode()
    stop_at = time.monotonic() + duration_sec
    period = 1.0 / max(hz, 0.1)
    next_emit = time.monotonic()
    return_code = 0
    try:
        while time.monotonic() < stop_at and rclpy.ok() and not stop_requested:
            now = time.monotonic()
            if now >= next_emit:
                emit_sample(node)
                while next_emit <= now:
                    next_emit += period
                continue
            timeout_sec = min(next_emit - now, 0.1, max(0.0, stop_at - now))
            rclpy.spin_once(node, timeout_sec=timeout_sec)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        if not stop_requested:
            print(
                json.dumps(
                    {
                        "type": "observer_error",
                        "error_type": exc.__class__.__name__,
                        "error": str(exc),
                    },
                    ensure_ascii=False,
                    separators=(",", ":"),
                ),
                flush=True,
            )
            return_code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGTERM, previous_sigterm)
        signal.signal(signal.SIGINT, previous_sigint)
    return return_code


def _pose_msg(msg: object) -> JsonObject:
    return {
        "x": round(float(getattr(msg, "x", 0.0)), 4),
        "y": round(float(getattr(msg, "y", 0.0)), 4),
        "theta": round(float(getattr(msg, "theta", 0.0)), 4),
        "wall_time": time.time(),
    }


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
    except Exception as exc:
        print(f"[auto-debug] error: {exc}", file=sys.stderr)
        raise SystemExit(1)
