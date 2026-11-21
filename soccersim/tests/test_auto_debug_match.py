from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scripts.auto_debug_match import (  # noqa: E402
    RunnerConfig,
    TeamSpec,
    _bridge_control_message_json,
    _container_http_json,
    _config_record,
    _events_for_match,
    _match_id_from_status,
    _is_json_object_line,
    _parse_host_args,
    _read_env_file,
    _required_truth_topics,
    _run_episode,
    _score_episode,
    _start_team_node,
    _write_markdown_report,
)


class AutoDebugMatchTests(unittest.TestCase):
    def test_score_episode_rewards_control_observation_and_kick(self) -> None:
        samples = [
            {
                "type": "sample",
                "game_controller": {"state": "READY"},
                "team1_ball": {"x": 0.0, "y": 0.0},
                "wall_time": 1.0,
            },
            {
                "type": "sample",
                "game_controller": {"state": "PLAYING"},
                "team1_ball": {"x": 0.5, "y": 0.0},
                "wall_time": 2.0,
            },
        ]
        telemetry = [
            {
                "event": "runtime_starting",
                "level": "INFO",
                "monotonic_sec": 10.0,
            },
            {
                "event": "soccer_kick_started",
                "level": "INFO",
                "monotonic_sec": 12.5,
            },
        ]
        final_status = {
            "game": {
                "teams": {
                    "home": {"score": 1},
                    "away": {"score": 0},
                }
            }
        }

        score, metrics, findings = _score_episode(samples, telemetry, final_status)

        self.assertEqual(score, 100.0)
        self.assertEqual(metrics["states_seen"], ["PLAYING", "READY"])
        self.assertEqual(metrics["first_soccer_kick_sec"], 2.5)
        self.assertEqual(metrics["ball_progress_x"], 0.5)
        self.assertEqual(metrics["team_scores"], {"home": 1, "away": 0})
        self.assertTrue(all(bool(item["ok"]) for item in findings))

    def test_score_episode_reports_missing_signals(self) -> None:
        score, metrics, findings = _score_episode([], [], {})

        self.assertEqual(score, 0.0)
        self.assertEqual(metrics["states_seen"], [])
        self.assertIsNone(metrics["first_soccer_kick_sec"])
        self.assertFalse(any(bool(item["ok"]) for item in findings[:6]))

    def test_is_json_object_line_filters_ros_noise(self) -> None:
        self.assertTrue(_is_json_object_line('{"type":"sample"}\n'))
        self.assertFalse(_is_json_object_line("[RTPS_TRANSPORT_SHM Error]\n"))
        self.assertFalse(_is_json_object_line("[1, 2, 3]\n"))

    def test_container_http_json_uses_inside_control_helper(self) -> None:
        config = RunnerConfig(
            local_container=False,
            container="container1",
            workdir="/work",
            ros_setup="/opt/ros/humble/setup.bash",
            extra_setup="",
            fastdds_profile="/profile.xml",
            control_url="http://127.0.0.1:38383",
            game_control_artifact_dir="/game-control",
            artifact_root=Path("artifacts/auto_debug"),
            run_id="run1",
            build=False,
            start_nodes=True,
            keep_nodes=False,
            auto_match=False,
            control_hz=10.0,
            ready_wait_sec=1.0,
            play_sec=1.0,
            observe_hz=2.0,
            min_score=40.0,
            team1=TeamSpec(1, "robot1,robot2,robot3"),
            team2=TeamSpec(2, "robot4,robot5,robot6"),
        )

        with patch("scripts.auto_debug_match._run") as run:
            run.return_value.returncode = 0
            run.return_value.stdout = '{"ok":true}'
            run.return_value.stderr = ""
            result = _container_http_json(
                config,
                "POST",
                "http://127.0.0.1:38383/match/start",
                {},
                3.0,
            )

        self.assertEqual(result, {"ok": True})
        command = run.call_args.args[0]
        self.assertEqual(command[:4], ["docker", "exec", "container1", "python3"])
        self.assertIn("--inside-control", command)
        self.assertIn("--payload", command)
        self.assertIn("{}", command)

    def test_run_episode_uses_current_match_control_api(self) -> None:
        config = RunnerConfig(
            local_container=True,
            container="container1",
            workdir="/work",
            ros_setup="/opt/ros/humble/setup.bash",
            extra_setup="",
            fastdds_profile="/profile.xml",
            control_url="http://127.0.0.1:38383",
            game_control_artifact_dir="/game-control",
            artifact_root=Path("artifacts/auto_debug"),
            run_id="run1",
            build=False,
            start_nodes=True,
            keep_nodes=False,
            auto_match=True,
            control_hz=10.0,
            ready_wait_sec=1.0,
            play_sec=0.01,
            observe_hz=2.0,
            min_score=40.0,
            team1=TeamSpec(1, "robot1,robot2,robot3"),
            team2=TeamSpec(2, "robot4,robot5,robot6"),
        )
        post_calls: list[str] = []

        def fake_post(
            request_config: RunnerConfig,
            path: str,
            payload: object,
            timeout_sec: float = 5.0,
        ) -> dict[str, object]:
            self.assertIs(request_config, config)
            self.assertIsInstance(payload, dict)
            self.assertEqual(timeout_sec, 5.0)
            post_calls.append(path)
            return {"game": {"state": "ready", "setPlay": "kickOff", "stopped": False}}

        def fake_get(
            request_config: RunnerConfig,
            path: str,
            timeout_sec: float = 5.0,
        ) -> dict[str, object]:
            self.assertIs(request_config, config)
            self.assertEqual(timeout_sec, 5.0)
            return {"path": path, "game": {"state": "playing"}}

        with tempfile.TemporaryDirectory() as tmp_dir:
            with patch("scripts.auto_debug_match._game_control_post", fake_post):
                with patch("scripts.auto_debug_match._game_control_get", fake_get):
                    _run_episode(config, Path(tmp_dir))

        self.assertEqual(post_calls, ["/match/start", "/match/end"])

    def test_parse_host_args_supports_ready_slots_and_opponents(self) -> None:
        config = _parse_host_args(
            [
                "--local-container",
                "--team1-robots",
                "robot1,robot2,robot3",
                "--team2-robots",
                "robot4,robot5,robot6",
                "--team1-ready-slots",
                "center,side,keeper",
                "--team2-ready-slots",
                "keeper,side,center",
            ]
        )
        record = _config_record(config)

        self.assertEqual(config.team1.ready_slots, "center,side,keeper")
        self.assertEqual(config.team1.opponent_robot_names, "robot4,robot5,robot6")
        self.assertEqual(config.team2.ready_slots, "keeper,side,center")
        self.assertEqual(config.team2.opponent_robot_names, "robot1,robot2,robot3")
        teams = record["teams"]
        self.assertIsInstance(teams, list)
        self.assertEqual(teams[0]["ready_slots"], "center,side,keeper")
        self.assertEqual(teams[1]["ready_slots"], "keeper,side,center")

    def test_required_truth_topics_use_team_ids_and_first_robot_names(self) -> None:
        config = _parse_host_args(
            [
                "--local-container",
                "--team1-robots",
                "alpha,beta,gamma",
                "--team2-robots",
                "delta,epsilon,zeta",
            ]
        )

        topics = _required_truth_topics(config)

        self.assertIn("/soccer/game_controller", topics)
        self.assertIn("/team1/soccer/sim/ground_truth/ball", topics)
        self.assertIn("/team2/soccer/sim/ground_truth/ball", topics)
        self.assertIn("/team1/alpha/soccer/sim/ground_truth/robot_pose", topics)
        self.assertIn("/team2/delta/soccer/sim/ground_truth/robot_pose", topics)

    def test_bridge_control_message_json_matches_runtime_parser_shape(self) -> None:
        payload = {
            "version": 19,
            "packetNumber": 7,
            "playersPerTeam": 3,
            "competitionType": 2,
            "stopped": 0,
            "gamePhase": 0,
            "state": 3,
            "setPlay": 0,
            "firstHalf": 1,
            "kickingTeam": 255,
            "secsRemaining": 599,
            "secondaryTime": 0,
            "teams": [
                {
                    "number": 1,
                    "fieldPlayerColor": 1,
                    "goalkeeperColor": 0,
                    "goalkeeper": 3,
                    "score": 0,
                    "players": [
                        {
                            "penalty": 0,
                            "secsTillUnpenalized": 0,
                            "warnings": 0,
                            "cautions": 0,
                        }
                    ],
                }
            ],
        }

        encoded = _bridge_control_message_json(payload)
        decoded = json.loads(encoded)

        self.assertEqual(decoded["state"], "PLAYING")
        self.assertEqual(decoded["setPlay"], "NONE")
        self.assertEqual(decoded["competitionType"], "LARGE")
        self.assertEqual(decoded["teams"][0]["teamNumber"], 1)
        self.assertEqual(decoded["teams"][0]["goalkeeper"], 3)
        self.assertEqual(decoded["teams"][0]["players"][0]["penalty"], "NONE")
        self.assertIn("secsTillUnpenalised", decoded["teams"][0]["players"][0])

    def test_parse_host_args_reads_local_container_config(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            config_path = Path(tmp_dir) / "soccer.env"
            config_path.write_text(
                "SOCCER_DOCKER_CONTAINER=config-container\n"
                "SOCCER_DOCKER_WORKDIR=/configured/workdir\n",
                encoding="utf-8",
            )

            with patch.dict(
                "os.environ",
                {"SOCCER_SIM_CONFIG_FILE": str(config_path)},
                clear=True,
            ):
                config = _parse_host_args(["--local-container"])

        self.assertEqual(config.container, "config-container")
        self.assertEqual(config.workdir, "/configured/workdir")

    def test_parse_host_args_prefers_env_and_cli_over_local_config(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            config_path = Path(tmp_dir) / "soccer.env"
            config_path.write_text(
                "SOCCER_DOCKER_CONTAINER=config-container\n",
                encoding="utf-8",
            )

            with patch.dict(
                "os.environ",
                {
                    "SOCCER_SIM_CONFIG_FILE": str(config_path),
                    "SOCCER_DOCKER_CONTAINER": "env-container",
                },
                clear=True,
            ):
                env_config = _parse_host_args(["--local-container"])
                cli_config = _parse_host_args(
                    ["--local-container", "--container", "cli-container"]
                )

        self.assertEqual(env_config.container, "env-container")
        self.assertEqual(cli_config.container, "cli-container")

    def test_read_env_file_accepts_export_and_quotes(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            config_path = Path(tmp_dir) / "soccer.env"
            config_path.write_text(
                "# comment\n"
                "export SOCCER_DOCKER_CONTAINER='quoted-container'\n"
                "SOCCER_DOCKER_WORKDIR=\"/configured/workdir\"\n"
                "not a setting\n",
                encoding="utf-8",
            )

            values = _read_env_file(config_path)

        self.assertEqual(values["SOCCER_DOCKER_CONTAINER"], "quoted-container")
        self.assertEqual(values["SOCCER_DOCKER_WORKDIR"], "/configured/workdir")

    def test_match_id_prefers_current_status_match(self) -> None:
        match_id = _match_id_from_status(
            {"match": {"matchId": "current-match"}},
            {"matchId": "previous-result"},
        )

        self.assertEqual(match_id, "current-match")

    def test_events_for_match_uses_timeline_window_for_reused_match_id(self) -> None:
        events = [
            {"type": "penalty", "matchId": "local-match", "wallTime": 50.0},
            {"type": "match_started", "matchId": "local-match", "wallTime": 100.1},
            {"type": "state_changed", "matchId": "local-match", "wallTime": 110.0},
            {"type": "penalty", "matchId": "other-match", "wallTime": 110.0},
            {"type": "match_finished", "matchId": "local-match", "wallTime": 121.0},
        ]
        timeline = {
            "events": [
                {"label": "match_start", "wall_time": 100.0},
                {"label": "match_end", "wall_time": 120.0},
            ],
        }

        filtered = _events_for_match(events, "local-match", timeline)

        self.assertEqual(
            [item["type"] for item in filtered],
            ["match_started", "state_changed", "match_finished"],
        )

    def test_start_team_node_passes_team_ros_parameters(self) -> None:
        config = RunnerConfig(
            local_container=True,
            container="container1",
            workdir="/work",
            ros_setup="/opt/ros/humble/setup.bash",
            extra_setup="",
            fastdds_profile="/profile.xml",
            control_url="http://127.0.0.1:38383",
            game_control_artifact_dir="/game-control",
            artifact_root=Path("artifacts/auto_debug"),
            run_id="run1",
            build=False,
            start_nodes=True,
            keep_nodes=False,
            auto_match=False,
            control_hz=10.0,
            ready_wait_sec=1.0,
            play_sec=1.0,
            observe_hz=2.0,
            min_score=40.0,
            team1=TeamSpec(
                1,
                "robot1,robot2,robot3",
                "center,side,keeper",
                "robot4,robot5,robot6",
            ),
            team2=TeamSpec(
                2,
                "robot4,robot5,robot6",
                "center,side,keeper",
                "robot1,robot2,robot3",
            ),
        )
        with tempfile.TemporaryDirectory() as tmp_dir:
            with patch("scripts.auto_debug_match.subprocess.Popen") as popen:
                popen.return_value = SimpleNamespace(pid=123)
                _start_team_node(config, config.team1, Path(tmp_dir))

        command = popen.call_args.args[0]
        command_text = "\n".join(command)
        self.assertIn("team_id:=1", command_text)
        self.assertIn("robot_names:=robot1,robot2,robot3", command_text)
        self.assertIn("ready_slots:=center,side,keeper", command_text)
        self.assertIn("opponent_robot_names:=robot4,robot5,robot6", command_text)

    def test_score_episode_reports_referee_penalties(self) -> None:
        samples = [
            {
                "type": "sample",
                "game_controller": {"state": "READY"},
                "team1_ball": {"x": 0.0, "y": 0.0},
                "wall_time": 1.0,
            },
            {
                "type": "sample",
                "game_controller": {"state": "PLAYING"},
                "team1_ball": {"x": 0.6, "y": 0.0},
                "wall_time": 2.0,
            },
        ]
        telemetry = [
            {"event": "runtime_starting", "level": "INFO", "monotonic_sec": 10.0},
            {"event": "soccer_kick_started", "level": "INFO", "monotonic_sec": 11.0},
        ]
        final_status = {
            "game": {
                "teams": {
                    "home": {
                        "score": 0,
                        "players": [
                            {
                                "penalty": "illegalPosition",
                                "secsTillUnpenalized": 29,
                                "warnings": 1,
                                "cautions": 0,
                            }
                        ],
                    },
                    "away": {"score": 0, "players": [{"penalty": "noPenalty"}]},
                }
            }
        }
        game_control_events = [
            {
                "type": "penalty",
                "severity": "penalty",
                "actor": {"side": "home", "robotName": "robot1", "playerNumber": 1},
                "outcome": {"penalty": "illegalPosition", "reason": "illegalPosition"},
                "evidence": {"ruleId": "illegalPosition"},
            },
            {"type": "illegal_reentry_warning", "severity": "warning"},
        ]
        timeline = {"events": [{"label": "match_start", "response": {"game": {}}}]}

        score, metrics, findings = _score_episode(
            samples,
            telemetry,
            final_status,
            game_control_events,
            timeline,
            auto_match_requested=True,
        )

        self.assertEqual(score, 92.0)
        self.assertTrue(metrics["auto_match_enabled_seen"])
        self.assertEqual(metrics["referee_penalty_count"], 1)
        self.assertEqual(metrics["referee_warning_count"], 1)
        self.assertEqual(metrics["referee_penalty_rule_ids"], {"illegalPosition": 1})
        self.assertEqual(metrics["final_player_penalty_count"], 1)
        self.assertTrue(any("自动裁判 penalty 事件=1" in str(item["message"]) for item in findings))

    def test_markdown_report_writes_summary(self) -> None:
        report = {
            "run_id": "run1",
            "score": 75.0,
            "metrics": {
                "states_seen": ["READY", "PLAYING"],
                "auto_match_enabled_seen": True,
                "first_soccer_kick_sec": 3.0,
                "ball_progress_x": 0.4,
                "game_controller_stale_count": 0,
                "warn_or_error_count": 1,
                "referee_penalty_count": 2,
                "referee_warning_count": 1,
                "final_player_penalty_count": 1,
            },
            "findings": [
                {"ok": True, "message": "裁判 topic 可见"},
                {"ok": False, "message": "WARN/ERROR telemetry 次数=1"},
            ],
            "artifacts": {"run_dir": "/tmp/run1"},
        }
        with tempfile.TemporaryDirectory() as tmp_dir:
            path = Path(tmp_dir) / "report.md"
            _write_markdown_report(path, report)
            text = path.read_text(encoding="utf-8")

        self.assertIn("score: `75.0/100`", text)
        self.assertIn("PASS: 裁判 topic 可见", text)
        self.assertIn("FAIL: WARN/ERROR telemetry 次数=1", text)


if __name__ == "__main__":
    unittest.main()
