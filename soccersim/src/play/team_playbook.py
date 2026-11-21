"""Team Playbook: :class:`DefaultPlaybook` plus scoreline awareness.

Baseline behavior equals :class:`DefaultPlaybook`. One override: when the team
trails by at least ``press_min_goal_deficit`` and the GameController clock has
dropped to ``press_when_losing_after_sec``, the goalkeeper joins the attack as
a supporter so all three players press for the equalizer. The decision reads
only the referee score and ``secs_remaining``; it never uses local timers, and
the default protective shape is kept while leading or tied.
"""

from __future__ import annotations

from ..runtime import SoccerKit
from ..soccer_framework import PlayContext
from .playbook import ROLE_SUPPORTER, DefaultPlaybook, RoleAssignment


class TeamPlaybook(DefaultPlaybook):
    """Default dynamic roles plus an all-out press when losing late."""

    def assign_roles(self, context: PlayContext) -> RoleAssignment:
        base = super().assign_roles(context)
        if not self._should_press(context):
            return base
        goalkeeper = self._configured_goalkeeper()
        if goalkeeper is None:
            return base
        mapping = dict(base.by_player)
        mapping[goalkeeper] = ROLE_SUPPORTER
        return RoleAssignment(mapping)

    def _should_press(self, context: PlayContext) -> bool:
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
