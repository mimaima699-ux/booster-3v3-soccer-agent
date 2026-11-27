"""Team Playbook: stable attack/defense mode machine on top of DefaultPlaybook.

Baseline roles equal :class:`DefaultPlaybook`. On top:

- Team mode ATTACK/DEFENSE with separate enter/exit thresholds, so a ball
  bouncing around halfway does not flip tactics every frame. Transitions are
  logged with the triggering ball position.
- DEFENSE: the non-chaser field player becomes the cover defender instead of
  the attacking supporter, giving the compact "one pressures, one covers"
  shape while the goalkeeper tracks the ball near goal.
- ATTACK: roles stay chaser + supporter; the supporter switches to a forward
  passing offer (see :class:`SupporterRole`).
- The losing-scoreline press (keeper -> supporter) is config-gated and
  disabled by default: the dedicated-keeper strategy never abandons the goal.

Decisions are based on the current match snapshot.
"""

from __future__ import annotations

import math

from ..runtime import SoccerKit
from ..soccer_framework import PlayContext
from .default_roles import DefenderRole
from .playbook import (
    ROLE_DEFENDER,
    ROLE_GOAL_COVER,
    ROLE_SUPPORTER,
    DefaultPlaybook,
    RoleAssignment,
)

_MODE_ATTACK = "ATTACK"
_MODE_DEFENSE = "DEFENSE"


class TeamPlaybook(DefaultPlaybook):
    """Default dynamic roles plus a stable attack/defense mode machine."""

    def __init__(self, kit: SoccerKit) -> None:
        super().__init__(kit)
        self.register_role(DefenderRole())
        self._mode: str = _MODE_ATTACK

    def assign_roles(self, context: PlayContext) -> RoleAssignment:
        self._update_mode(context)
        base = super().assign_roles(context)
        mapping = dict(base.by_player)

        if self._mode == _MODE_DEFENSE:
            # Compact defense: the second field player covers instead of
            # offering attack support.
            for player_id, role in mapping.items():
                if role == ROLE_SUPPORTER:
                    mapping[player_id] = ROLE_DEFENDER

        if ROLE_GOAL_COVER not in mapping.values() and self._should_press(context):
            goalkeeper = self._configured_goalkeeper()
            if goalkeeper is not None:
                mapping[goalkeeper] = ROLE_SUPPORTER

        return RoleAssignment(mapping)

    def _update_mode(self, context: PlayContext) -> None:
        ball = context.ball
        if ball is None:
            return
        strategy = self.kit.config.strategy
        previous = self._mode
        nearest_opponent = min(
            (
                math.hypot(robot.pose.x - ball.x, robot.pose.y - ball.y)
                for robot in context.opponents.values() if robot.pose is not None
            ),
            default=math.inf,
        )
        game = context.game
        nearest_teammate = min(
            (
                math.hypot(robot.pose.x - ball.x, robot.pose.y - ball.y)
                for player_id, robot in context.teammates.items()
                if robot.pose is not None
                and game is not None and self.kit.is_player_allowed(game, player_id)
            ),
            default=math.inf,
        )
        opponent_controls = (
            nearest_opponent <= strategy.defense_opponent_control_range_m
            and nearest_opponent + strategy.defense_control_margin_m < nearest_teammate
        )
        if opponent_controls or ball.x < strategy.defense_enter_ball_x:
            self._mode = _MODE_DEFENSE
        elif ball.x > strategy.defense_exit_ball_x:
            self._mode = _MODE_ATTACK
        self.kit.team_mode = self._mode
        if self._mode != previous:
            self._log(f"team mode -> {self._mode} (ball.x={ball.x:.2f})")

    def _should_press(self, context: PlayContext) -> bool:
        if not self.kit.config.strategy.press_enabled:
            return False
        game = context.game_state
        if game is None:
            return False
        strategy = self.kit.config.strategy
        own = game.get_team_state(self.kit.config.team_id)
        opponent = next(
            (
                team
                for team in game.teams
                if team.team_number != self.kit.config.team_id
            ),
            None,
        )
        if own is None or opponent is None:
            return False
        deficit = opponent.score - own.score
        return (
            deficit >= strategy.press_min_goal_deficit
            and 0 < game.secs_remaining <= strategy.press_when_losing_after_sec
        )

    def _log(self, message: str) -> None:
        logger = self.kit.logger()
        if logger is not None:
            logger.info(
                message,
                event="team_mode",
                team=self.kit.config.team_id,
            )
