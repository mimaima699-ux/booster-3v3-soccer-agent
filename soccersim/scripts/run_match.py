#!/usr/bin/env python3
from __future__ import annotations

import argparse
import atexit
from dataclasses import dataclass
import fcntl
import os
import signal
import shutil
import sys
import threading
import time
import subprocess
from pathlib import Path

from sandbox import SandboxError, SandboxManager

AGENT_ROOT = Path("/opt/booster/booster_agent_data/data/agents/extract")


TEAM_COLORS = ("\033[34m", "\033[31m")
MAX_TEAMS = 2
TEAM_ROBOT_NAMES = {
    "1": "robot1,robot2,robot3",
    "2": "robot4,robot5,robot6",
}


@dataclass
class SandboxParam:
    sandbox_id: str
    agent_id: str
    team: str
    soccer_robot_names: str
    color: str


SANDBOX_PARAMS: list[SandboxParam] = []

BASE_DIR = Path("/run/3v3_runner")
EXEC_FILE = Path(__file__).with_name("supervisor.py")
SANDBOX_EXEC_FILE = Path("/tmp/booster_agent/3v3_runner/supervisor.py")
FLOCK_RELA_PATH = Path("data/tmp/booster_agent/run.lock")
START_TIME = time.monotonic()
RUN_ID = time.strftime("match_%Y%m%d_%H%M%S", time.localtime())
STRUCTURED_LOG_DIR = "/tmp/booster_agent/soccer_logs"

sandbox_manager = SandboxManager(BASE_DIR)
stopping = False


def log_prefix(level: str) -> str:
    elapsed = time.monotonic() - START_TIME
    return f"[{elapsed:06.1f}s/{level}/MAIN]"


def info(message: str) -> None:
    print(f"{log_prefix('INFO')} {message}", flush=True)


def error(message: str) -> None:
    print(f"{log_prefix('ERROR')} {message}", file=sys.stderr, flush=True)


def sandbox_structured_log_file(param: SandboxParam) -> str:
    return f"{STRUCTURED_LOG_DIR}/{RUN_ID}/soccersim.team{param.team}.jsonl"


def host_structured_log_file(param: SandboxParam) -> Path:
    return (
        BASE_DIR
        / param.sandbox_id
        / "data/tmp/booster_agent/soccer_logs"
        / RUN_ID
        / f"soccersim.team{param.team}.jsonl"
    )


def host_pretty_log_file(param: SandboxParam) -> Path:
    return host_structured_log_file(param).with_suffix(".pretty.log")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run soccer agent sandboxes (at most 2 teams).",
    )
    parser.add_argument(
        "--teams",
        nargs="+",
        required=True,
        metavar="AGENT_ID",
        help=(
            "Agent ids under .../agents/extract/, in team order "
            "(1st -> team 1, 2nd -> team 2; at most 2)"
        ),
    )
    return parser.parse_args()


def validate_agent_id(agent_id: str) -> Path:
    agent_dir = AGENT_ROOT / agent_id
    meta_file = agent_dir / "agent.json"
    if not meta_file.is_file():
        raise RuntimeError(f"agent.json not found: {meta_file}")
    return agent_dir


def build_sandbox_params(args: argparse.Namespace) -> list[SandboxParam]:
    agent_ids = list(args.teams)
    if len(agent_ids) > MAX_TEAMS:
        raise SystemExit(
            f"--teams accepts at most {MAX_TEAMS} agent ids, got {len(agent_ids)}"
        )

    params: list[SandboxParam] = []
    for index, agent_id in enumerate(agent_ids, start=1):
        validate_agent_id(agent_id)
        team = str(index)
        soccer_robot_names = TEAM_ROBOT_NAMES[team]
        color = TEAM_COLORS[index - 1] if index <= len(TEAM_COLORS) else ""
        params.append(
            SandboxParam(
                f"team_{team}",
                agent_id,
                team,
                soccer_robot_names,
                color,
            )
        )
    return params


def load_runtime_env() -> None:
    result = subprocess.run(
        ["pgrep", "-f", "booster_agent_manager"], capture_output=True, text=True
    )
    pids = [line.strip() for line in result.stdout.splitlines() if line.strip()]
    pid = pids[0] if pids and Path(f"/proc/{pids[0]}/environ").is_file() else None
    if not pid:
        raise RuntimeError("booster_agent_manager process not found")
    environ = Path(f"/proc/{pid}/environ")
    env: dict[str, str] = {}
    for item in environ.read_bytes().split(b"\0"):
        if not item or b"=" not in item:
            continue
        key, value = item.split(b"=", 1)
        env[key.decode("utf-8", "replace")] = value.decode("utf-8", "replace")
    os.environ.update(env)


def stream_sandbox_log(param: SandboxParam) -> None:
    log_path = BASE_DIR / param.sandbox_id / "sandbox.log"
    with log_path.open("r", encoding="utf-8", errors="replace") as fp:
        while True:
            line = fp.readline()
            if line:
                print(
                    f"{param.color}[{param.sandbox_id}]\033[0m {line}",
                    end="",
                    flush=True,
                )
                continue
            time.sleep(0.2)


def cleanup() -> None:
    global stopping
    if stopping:
        return
    stopping = True
    for sandbox in SANDBOX_PARAMS:
        try:
            sandbox_manager.destroy(sandbox.sandbox_id)
            info("destroyed sandbox: " + sandbox.sandbox_id)
        except Exception as ex:
            error(f"cleanup sandbox {sandbox.sandbox_id} failed: {ex}")


def handle_signal(signum: int, _frame) -> None:
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    signal.signal(signal.SIGTSTP, signal.SIG_IGN)
    raise SystemExit(128 + signum)


def create_sandbox(param: SandboxParam) -> None:
    try:
        sandbox_manager.destroy(param.sandbox_id)
        shutil.rmtree(BASE_DIR / param.sandbox_id, ignore_errors=True)
    except SandboxError:
        pass
    sandbox_manager.create(param.sandbox_id)
    info(
        f"created sandbox {param.sandbox_id} "
        f"team={param.team} agent={param.agent_id} "
        f"SOCCER_TEAM_ID={param.team} SOCCER_ROBOT_NAMES={param.soccer_robot_names}"
    )
    threading.Thread(target=stream_sandbox_log, args=(param,), daemon=True).start()
    host_exec_dir = BASE_DIR / param.sandbox_id / "data/tmp/booster_agent/3v3_runner"
    host_exec_dir.mkdir(parents=True, exist_ok=True)
    host_exec_file = host_exec_dir / "supervisor.py"
    shutil.copy2(EXEC_FILE, host_exec_file)
    host_exec_file.chmod(0o755)
    cmd = [
        "env",
        f"SOCCER_TEAM_ID={param.team}",
        f"SOCCER_ROBOT_NAMES={param.soccer_robot_names}",
        f"SOCCER_RUN_ID={RUN_ID}",
        f"SOCCER_LOG_DIR={STRUCTURED_LOG_DIR}",
        f"SOCCER_LOG_FILE={sandbox_structured_log_file(param)}",
        "python3",
        "-u",
        str(SANDBOX_EXEC_FILE),
        param.agent_id,
    ]
    sandbox_manager.exec_async(param.sandbox_id, cmd)
    info(f"started execute: {param.sandbox_id}")
    info(f"structured log {param.sandbox_id}: {host_structured_log_file(param)}")
    info(f"pretty log {param.sandbox_id}: {host_pretty_log_file(param)}")


def probe_sandbox_run_flock(id: str) -> bool:
    lock_path = BASE_DIR / id / FLOCK_RELA_PATH
    try:
        with lock_path.open("r+") as fp:
            try:
                fcntl.flock(fp.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                return True
            fcntl.flock(fp.fileno(), fcntl.LOCK_UN)
            return False
    except FileNotFoundError:
        return False


def wait_sandboxes() -> None:
    alives = [True] * len(SANDBOX_PARAMS)
    while True:
        for index in range(0, len(SANDBOX_PARAMS)):
            sandbox_id = SANDBOX_PARAMS[index].sandbox_id
            state = probe_sandbox_run_flock(sandbox_id)
            if (not state) and alives[index]:
                error(f"sandbox flock lost: {sandbox_id}")
            if state and not alives[index]:
                error(f"sandbox flock appear: {sandbox_id}")
            alives[index] = state
            if not any(alives):
                error("all sandbox flocks lost, exiting")
                return
        time.sleep(3)


def main() -> None:
    global SANDBOX_PARAMS
    args = parse_args()
    SANDBOX_PARAMS = build_sandbox_params(args)
    os.system("clear")
    info(f"begin 3v3_runner ({len(SANDBOX_PARAMS)} sandbox(es))")
    info(f"structured logs run_id={RUN_ID} log_dir={STRUCTURED_LOG_DIR}")
    for param in SANDBOX_PARAMS:
        info(f"  team {param.team} ({param.sandbox_id}): {AGENT_ROOT / param.agent_id}")
    load_runtime_env()
    assert EXEC_FILE.is_file(), f"sandbox exec file not found: {EXEC_FILE}"
    atexit.register(cleanup)
    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)
    signal.signal(signal.SIGTSTP, handle_signal)
    try:
        for sandbox in SANDBOX_PARAMS:
            create_sandbox(sandbox)
        time.sleep(1)
        wait_sandboxes()
    except SandboxError as exc:
        error(f"error: {exc}")
    finally:
        cleanup()


if __name__ == "__main__":
    main()
    raise SystemExit(0)
