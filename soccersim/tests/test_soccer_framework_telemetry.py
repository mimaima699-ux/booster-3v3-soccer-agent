from __future__ import annotations

import json
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from src.soccer_framework.telemetry import (  # noqa: E402
    DEFAULT_LOG_DIR,
    JsonlTelemetry,
    NullTelemetry,
    SoccerLogger,
    create_soccer_logger,
    create_soccer_telemetry,
)


class _FakeConsoleLogger:
    def __init__(self) -> None:
        self.messages: list[str] = []

    def info(self, message: str) -> None:
        self.messages.append(message)

    def warn(self, message: str) -> None:
        self.messages.append(f"WARN: {message}")

    def error(self, message: str) -> None:
        self.messages.append(f"ERROR: {message}")


class SoccerTelemetryTests(unittest.TestCase):
    def test_jsonl_telemetry_writes_structured_events(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            path = Path(tmp_dir) / "events.jsonl"
            pretty_path = Path(tmp_dir) / "events.pretty.log"
            telemetry = JsonlTelemetry(
                path=path,
                run_id="run-1",
                source="test",
                pretty_path=pretty_path,
            )

            telemetry.event(
                "control_summary",
                team_id=1,
                players=[
                    {
                        "player_id": 1,
                        "command": {"vx": 0.1, "reason": "center approach ball"},
                    }
                ],
            )
            telemetry.close()
            telemetry.event("ignored_after_close")

            lines = path.read_text(encoding="utf-8").splitlines()
            pretty_text = pretty_path.read_text(encoding="utf-8")

        self.assertEqual(len(lines), 1)
        record = json.loads(lines[0])
        self.assertEqual(record["run_id"], "run-1")
        self.assertEqual(record["source"], "test")
        self.assertEqual(record["level"], "INFO")
        self.assertEqual(record["event"], "control_summary")
        self.assertEqual(record["team_id"], 1)
        self.assertEqual(
            record["players"][0]["command"]["reason"],
            "center approach ball",
        )
        self.assertIn('\n  "event": "control_summary"', pretty_text)
        self.assertIn('\n  "players": [', pretty_text)

    def test_soccer_logger_records_event_without_console_noise(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            console = _FakeConsoleLogger()
            with patch.dict(
                os.environ,
                {
                    "SOCCER_LOG": "jsonl",
                    "SOCCER_LOG_DIR": tmp_dir,
                    "SOCCER_RUN_ID": "run1",
                },
            ):
                logger = create_soccer_logger(console, source="soccersim.team1")
                logger.info(
                    "Soccer BT tick trace",
                    event="bt_tick",
                    console=False,
                    tick_id=3,
                    root_status="SUCCESS",
                )
                logger.warn("failed", event="control_loop_failed", error="oops")
                path = logger.path
                logger.close()

            self.assertEqual(console.messages, ["WARN: failed"])
            self.assertIsNotNone(path)
            assert path is not None
            records = [
                json.loads(line)
                for line in path.read_text(encoding="utf-8").splitlines()
            ]

        self.assertEqual(records[0]["event"], "bt_tick")
        self.assertEqual(records[0]["tick_id"], 3)
        self.assertEqual(records[0]["root_status"], "SUCCESS")
        self.assertEqual(records[1]["event"], "control_loop_failed")
        self.assertEqual(records[1]["level"], "WARN")
        self.assertEqual(records[1]["message"], "failed")

    def test_factory_uses_run_id_and_log_dir(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            with patch.dict(
                os.environ,
                {
                    "SOCCER_LOG": "jsonl",
                    "SOCCER_LOG_DIR": tmp_dir,
                    "SOCCER_RUN_ID": "match 1",
                },
            ):
                telemetry = create_soccer_telemetry(source="soccersim/team 1")
                path = telemetry.path
                pretty_path = telemetry.pretty_path
                telemetry.event("runtime_starting", team_id=1)
                telemetry.close()

            self.assertEqual(
                path,
                Path(tmp_dir) / "match_1" / "soccersim_team_1.jsonl",
            )
            self.assertEqual(
                pretty_path,
                Path(tmp_dir) / "match_1" / "soccersim_team_1.pretty.log",
            )
            self.assertIsNotNone(path)
            assert path is not None
            self.assertIsNotNone(pretty_path)
            assert pretty_path is not None
            records = [
                json.loads(line)
                for line in path.read_text(encoding="utf-8").splitlines()
            ]
            pretty_text = pretty_path.read_text(encoding="utf-8")

        self.assertEqual(records[0]["run_id"], "match_1")
        self.assertEqual(records[0]["event"], "runtime_starting")
        self.assertIn('\n  "event": "runtime_starting"', pretty_text)

    def test_factory_defaults_log_dir_under_tmp(self) -> None:
        run_id = f"default_path_test_{os.getpid()}"
        with patch.dict(
            os.environ,
            {
                "SOCCER_LOG": "jsonl",
                "SOCCER_RUN_ID": run_id,
            },
            clear=True,
        ):
            telemetry = create_soccer_telemetry(source="soccersim.team1")
            path = telemetry.path
            pretty_path = telemetry.pretty_path
            telemetry.close()

        self.assertEqual(
            path,
            DEFAULT_LOG_DIR / run_id / "soccersim.team1.jsonl",
        )
        self.assertEqual(
            pretty_path,
            DEFAULT_LOG_DIR / run_id / "soccersim.team1.pretty.log",
        )
        if path is not None:
            path.unlink(missing_ok=True)
        if pretty_path is not None:
            pretty_path.unlink(missing_ok=True)
        if path is not None:
            try:
                path.parent.rmdir()
            except OSError:
                pass

    def test_factory_can_disable_telemetry(self) -> None:
        with patch.dict(os.environ, {"SOCCER_LOG": "off"}):
            telemetry = create_soccer_telemetry()

        self.assertIsInstance(telemetry, NullTelemetry)

    def test_legacy_alias_uses_soccer_telemetry_env(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            with patch.dict(
                os.environ,
                {
                    "SOCCER_TELEMETRY": "jsonl",
                    "SOCCER_LOG_DIR": tmp_dir,
                    "SOCCER_RUN_ID": "legacy",
                },
                clear=False,
            ):
                telemetry = create_soccer_telemetry("legacy.team")
                telemetry.event("legacy_event")
                path = telemetry.path
                telemetry.close()

            self.assertIsNotNone(path)
            assert path is not None
            records = [
                json.loads(line)
                for line in path.read_text(encoding="utf-8").splitlines()
            ]

        self.assertEqual(records[0]["event"], "legacy_event")


if __name__ == "__main__":
    unittest.main()
