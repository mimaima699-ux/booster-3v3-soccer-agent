"""Data-layer leaves that write external inputs and shared state to the blackboard each tick.

``UpdateGameState``
The tree root chains these nodes in a ``Sequence`` because they depend on each other:
``UpdateClock`` writes time, ``UpdatePlayContext`` reads callbacks, freshness filters run,
and ``UpdateRobotStatus`` pulls hardware status.

PLAY-stage role assignment used to live here (old ``UpdateChaser``), but now
belongs to :class:`src.play.nodes.AssignRoles` so it only runs when play actually starts.

To add a new global input, add a ``_DataLeaf`` subclass and mount it in the DataLayer ``Sequence``.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import TYPE_CHECKING

import py_trees

from ...soccer_framework import RobotRuntimeStatus, PlayContext, PlayContextProvider
from ..blackboard import BlackboardKeys, BlackboardClient, robot_status_key

if TYPE_CHECKING:
    from ...runtime import SoccerKit, SoccerLoggerProtocol


_BALL_STALE_SEC = 3.0
_ROBOT_POSE_STALE_SEC = 4.0
_GAME_STATE_STALE_SEC = 4.0


def _log_data_layer_none(
    kit: "SoccerKit",
    field: str,
    now: float,
    reason: str,
) -> None:
    """Log when a data field is None from the provider; perception is missing, so warn level."""
    logger = _get_logger(kit)
    if logger is not None:
        logger.warn(
            f"DataLayer: {field} is None ({reason})",
            event="data_layer_none",
            console=False,
            field=field,
            now=round(now, 3),
            reason=reason,
        )


def _log_data_layer_cleared(
    kit: "SoccerKit",
    field: str,
    now: float,
    last_seen_at: float,
    age_sec: float,
    threshold_sec: float,
) -> None:
    """Log when a data field is cleared due to staleness; a runtime regression, so warn level."""
    logger = _get_logger(kit)
    if logger is not None:
        logger.warn(
            f"DataLayer: clearing stale {field} (age={age_sec:.2f}s > threshold={threshold_sec:.1f}s)",
            event="data_layer_cleared",
            console=False,
            field=field,
            now=round(now, 3),
            last_seen_at=round(last_seen_at, 3),
            age_sec=round(age_sec, 3),
            threshold_sec=threshold_sec,
        )


def _get_logger(kit: "SoccerKit") -> "SoccerLoggerProtocol | None":
    """Return kit's bound logger, or None for tests and dry-run."""
    return kit.logger()


class _DataLeaf(py_trees.behaviour.Behaviour):
    """Shared base for data-layer leaves; always returns SUCCESS."""

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = BlackboardClient(name=name)


class UpdateClock(_DataLeaf):
    """Write current time to ``/clock/now`` and reset frame-level flags."""

    def __init__(self, get_now: Callable[[], float]):
        super().__init__("UpdateClock")
        self._get_now = get_now

    def update(self) -> py_trees.common.Status:
        self.blackboard.write(BlackboardKeys.NOW, self._get_now())
        self.blackboard.write(BlackboardKeys.SAFETY_ACTIVE, False)
        return py_trees.common.Status.SUCCESS


class UpdatePlayContext(_DataLeaf):
    """Snapshot the context from :class:`PlayContextProvider` directly into ``/play_context``.

    Holding the provider here avoids an extra callback chain through runtime,
    strategy, tree, and a temporary cached context.
    """

    def __init__(self, provider: PlayContextProvider):
        super().__init__("UpdatePlayContext")
        self._provider = provider

    def update(self) -> py_trees.common.Status:
        context = self._provider.get_snapshot()
        self.blackboard.write(BlackboardKeys.PLAY_CONTEXT, context)
        return py_trees.common.Status.SUCCESS


class UpdateGameState(_DataLeaf):
    """Clear stale GameControlState in place using the data-layer watchdog.

    Freshness filtering is centralized in the data layer and mutates
    ``context.game_state`` in place, so strategy code reads the filtered value without a separate key.
    These thresholds protect against stopped publishers or blocked callbacks; they are
    not tactic tuning knobs.

    ``last_seen_at == 0.0`` means no topic callback has written it yet, often in
    tests, so it is not filtered to match the old ``last_topic_at <= 0.0`` behavior.
    """

    def __init__(self):
        super().__init__("UpdateGameState")

    def update(self) -> py_trees.common.Status:
        context = self.blackboard.read(BlackboardKeys.PLAY_CONTEXT)
        now = self.blackboard.read(BlackboardKeys.NOW)
        if not isinstance(context, PlayContext) or now is None:
            return py_trees.common.Status.SUCCESS
        gs = context.game_state
        if gs is not None and not gs.is_recent(now, _GAME_STATE_STALE_SEC):
            context.game_state = None
        return py_trees.common.Status.SUCCESS


class UpdateRecentBall(_DataLeaf):
    """Clear a stale ball in place using the data-layer watchdog.

    Like game and robot filtering, this mutates ``context.ball`` directly and does
    not use a separate blackboard key.

    Logs both provider-None (ball never observed) and staleness-cleared cases
    to help diagnose "waiting for ball" downstream failures.
    """

    def __init__(self, kit: "SoccerKit"):
        super().__init__("UpdateRecentBall")
        self._kit = kit

    def update(self) -> py_trees.common.Status:
        context = self.blackboard.read(BlackboardKeys.PLAY_CONTEXT)
        now = self.blackboard.read(BlackboardKeys.NOW)
        if not isinstance(context, PlayContext) or now is None:
            return py_trees.common.Status.SUCCESS

        ball = context.ball
        if ball is None:
            # Provider returned None: no ball observation at all
            _log_data_layer_none(self._kit, "ball", now, reason="provider returned None")
        elif not ball.is_recent(now, _BALL_STALE_SEC):
            # Ball exists but stale: clear it
            age_sec = now - ball.last_seen_at
            _log_data_layer_cleared(
                self._kit, "ball", now,
                last_seen_at=ball.last_seen_at,
                age_sec=age_sec,
                threshold_sec=_BALL_STALE_SEC,
            )
            context.ball = None
        return py_trees.common.Status.SUCCESS


class UpdateRobotPoses(_DataLeaf):
    """Clear stale teammate/opponent poses in place using the data-layer watchdog.

    ``context.teammates[i].pose``
    The snapshot is deep-copied by the provider, so these in-place edits do not
    pollute provider internals.

    Once stale poses become None, existing ``if robot.pose is None`` checks in
    strategy and motion code work without extra ``last_seen_at`` checks.

    Logs both provider-None (pose never received) and staleness-cleared cases
    to help diagnose "waiting for pose" downstream failures.
    """

    def __init__(self, kit: "SoccerKit"):
        super().__init__("UpdateRobotPoses")
        self._kit = kit

    def update(self) -> py_trees.common.Status:
        context = self.blackboard.read(BlackboardKeys.PLAY_CONTEXT)
        now = self.blackboard.read(BlackboardKeys.NOW)
        if not isinstance(context, PlayContext) or now is None:
            return py_trees.common.Status.SUCCESS

        for player_id, robot in context.teammates.items():
            if robot.pose is None:
                _log_data_layer_none(
                    self._kit, f"teammate_{player_id}_pose", now,
                    reason="provider returned None",
                )
            elif not robot.is_recent(now, _ROBOT_POSE_STALE_SEC):
                age_sec = now - robot.last_seen_at
                _log_data_layer_cleared(
                    self._kit, f"teammate_{player_id}_pose", now,
                    last_seen_at=robot.last_seen_at,
                    age_sec=age_sec,
                    threshold_sec=_ROBOT_POSE_STALE_SEC,
                )
                robot.pose = None

        for opponent_id, opponent in context.opponents.items():
            if opponent.pose is None:
                _log_data_layer_none(
                    self._kit, f"opponent_{opponent_id}_pose", now,
                    reason="provider returned None",
                )
            elif not opponent.is_recent(now, _ROBOT_POSE_STALE_SEC):
                age_sec = now - opponent.last_seen_at
                _log_data_layer_cleared(
                    self._kit, f"opponent_{opponent_id}_pose", now,
                    last_seen_at=opponent.last_seen_at,
                    age_sec=age_sec,
                    threshold_sec=_ROBOT_POSE_STALE_SEC,
                )
                opponent.pose = None

        return py_trees.common.Status.SUCCESS


class UpdateRobotStatus(_DataLeaf):
    """Pull one player's hardware status onto the blackboard.

    This is read-only: it calls throttled ``poll_runtime_status`` via
    :class:`RobotServices`, writes ``/robot_status/{player_id}``, and lets SafetyOverrides own side effects.

    Without bound services, such as tests or dry-run, it writes a default
    :class:`RobotRuntimeStatus` so guards naturally take the normal branch.
    """

    def __init__(self, kit: "SoccerKit", player_id: int):
        super().__init__(f"UpdateRobotStatus({player_id})")
        self._kit = kit
        self._player_id = player_id

    def update(self) -> py_trees.common.Status:
        now = self.blackboard.read(BlackboardKeys.NOW)
        services = self._kit.robot_services()
        if services is None or now is None:
            status = RobotRuntimeStatus()
        else:
            status = services.poll_runtime_status(self._player_id, now)
        self.blackboard.write(robot_status_key(self._player_id), status)
        return py_trees.common.Status.SUCCESS
