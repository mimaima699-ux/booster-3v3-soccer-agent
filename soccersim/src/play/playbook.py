"""PLAY-stage strategy entry point; competitors usually edit this file first.

:class:`RoleAssignment` snapshots "who does what" once per tick and stores a
``player_id -> role_name`` mapping that can hold custom roles. :class:`Playbook`
centralizes competitor-overridable PLAY decisions and explicitly registers roles
through :meth:`register_role`.

:class:`DefaultPlaybook` is the template default. It registers the chaser,
supporter, and goalkeeper roles; fixed starting slots use ``ReadySlot`` for
non-PLAY branches. To change tactics, override ``assign_roles``, customize
``select_chaser`` or ``kick_target``, or register a new role after
``super().__init__(kit)``.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import TYPE_CHECKING

from ..soccer_framework import PlayContext, ReadySlot, RobotCommand
from ..runtime import SoccerKit

if TYPE_CHECKING:
    from .role import RoleRegistry, RoleStrategy


# Default role labels kept as constants for ``assign_roles``; they no longer restrict the string set.
ROLE_CHASER = "chaser"
ROLE_SUPPORTER = "supporter"
ROLE_GOALKEEPER = "goalkeeper"
ROLE_DEFENDER = "defender"
ROLE_GOAL_COVER = "goal_cover"
ROLE_NONE = "none"


@dataclass(frozen=True)
class RoleAssignment:
    """Snapshot of this tick's dynamic role assignment.

    The storage is ``by_player: Mapping[int, str]`` where keys are player IDs
    and values are role labels. ``role_of`` returns :data:`ROLE_NONE` when absent.

    Construction is direct: ``RoleAssignment({1: "chaser", 2: "supporter"})``.

    Use :meth:`players_of` for reverse lookup by role; specialized attributes
    such as chaser/supporters are intentionally not provided.
    """

    by_player: Mapping[int, str] = field(default_factory=dict)

    def __post_init__(self) -> None:
        # Freeze as a read-only view so external by_player edits cannot change ``role_of`` behavior.
        object.__setattr__(self, "by_player", MappingProxyType(dict(self.by_player)))

    def role_of(self, player_id: int) -> str:
        return self.by_player.get(player_id, ROLE_NONE)

    def players_of(self, name: str) -> tuple[int, ...]:
        return tuple(pid for pid, role in self.by_player.items() if role == name)


class Playbook:
    """Entry point for all PLAY-stage decisions competitors may override.

    Leaf nodes hold both :class:`SoccerKit` for tools and :class:`Playbook`
    for decisions. In the PLAY subtree, ``AssignRoles`` writes
    :meth:`assign_roles` output, per-player branches choose by
    :class:`RoleAssignment` plus ``role_registry``, kick leaves call
    :meth:`kick_target`, hold-style roles own their targets, and
    ``WaitForBall`` calls :meth:`waiting_command`.

    Subclasses add or remove roles in ``__init__`` with :meth:`register_role` and
    override :meth:`assign_roles` to choose this tick's assignment.
    """

    def __init__(self, kit: SoccerKit):
        # Delayed import breaks the role -> kit.blackboard -> kit -> play -> playbook cycle.
        from .role import RoleRegistry

        self.kit = kit
        self._registry = RoleRegistry()

    # Role registry

    def register_role(self, role: RoleStrategy) -> "Playbook":
        """Register a role on this Playbook and return self for chaining.

        Subclasses call this in ``__init__`` after ``super().__init__(kit)``.
        Registration order determines PLAY Selector branch priority.
        """

        self._registry.register(role)
        return self

    @property
    def role_registry(self) -> RoleRegistry:
        return self._registry

    # Role assignment, the core strategy node

    def assign_roles(
        self,
        context: PlayContext,
    ) -> RoleAssignment:
        """Return one :class:`RoleAssignment` per tick; subclasses decide the actual assignment."""

        raise NotImplementedError

    # Cross-role cooperative targets

    def waiting_command(
        self,
        player_id: int,
        context: PlayContext,
    ) -> RobotCommand:
        """Fallback command when no role is assigned.

        The default is a stop tagged by ReadySlot. Competitors can override this
        for custom idle positioning; the node has already cleared :class:`KickHysteresis`.
        """

        slot = self.kit.config.ready_slot_for_player(player_id)
        return RobotCommand.stop(f"{slot.value} waiting for ball")


# ----------------------------------------------------------------------
# Default implementation: fixed ReadySlot starts plus PLAY dynamic roles
# ----------------------------------------------------------------------


class DefaultPlaybook(Playbook):
    """Default SoccerSim playbook: chaser/supporter/goalkeeper dynamic roles plus Targeting scores.

    Subclasses can selectively override one method, for example:

    .. code-block:: python

    class AggressivePlaybook(DefaultPlaybook):
    def assign_roles(self, context):
    base = super().assign_roles(context)
    Move more players to supporters when trailing.
    """

    def __init__(self, kit: SoccerKit):
        super().__init__(kit)
        # Explicitly register default PLAY dynamic roles; competitor subclasses can
        # call register_role(...) after super().__init__(kit). DefenderRole is reserved for explicit custom Playbook registration.
        from .default_roles import (
            ChaserRole,
            GoalCoverRole,
            GoalkeeperRole,
            SupporterRole,
        )

        self.register_role(ChaserRole())
        self.register_role(SupporterRole())
        self._goalkeeper = GoalkeeperRole()
        self.register_role(self._goalkeeper)
        self.register_role(GoalCoverRole())
        self._chaser_id: int | None = None
        self._goal_cover_id: int | None = None

    def assign_roles(self, context: PlayContext) -> RoleAssignment:
        chaser_id = self.select_chaser(context)
        goalkeeper_id = self._configured_goalkeeper()

        mapping: dict[int, str] = {}
        for player_id in self.kit.config.player_ids:
            if player_id == goalkeeper_id:
                mapping[player_id] = ROLE_GOALKEEPER
            elif player_id == chaser_id:
                mapping[player_id] = ROLE_CHASER
            else:
                mapping[player_id] = ROLE_SUPPORTER

        keeper = self.role_registry.get(ROLE_GOALKEEPER)
        if goalkeeper_id is not None and keeper is self._goalkeeper:
            decision = self._goalkeeper.prepare(self.kit, goalkeeper_id, context)
            if decision.claims_ball:
                if chaser_id is not None:
                    mapping[chaser_id] = ROLE_SUPPORTER
                cover_target = self.kit.ready_stance.goalkeeper_guard_target(context.known_ball)
                game = context.known_game
                distances = {
                    mate_id: math.hypot(robot.pose.x - cover_target.x, robot.pose.y - cover_target.y)
                    for mate_id, robot in context.teammates.items()
                    if mate_id in mapping and mate_id != goalkeeper_id
                    and robot.pose is not None and self.kit.is_player_allowed(game, mate_id)
                }
                if distances:
                    cover_id = self._goal_cover_id
                    if cover_id is None or cover_id not in distances:
                        cover_id = min(distances, key=lambda mate_id: (distances[mate_id], mate_id))
                    self._goal_cover_id = cover_id
                    mapping[cover_id] = ROLE_GOAL_COVER
                    return RoleAssignment(mapping)

        self._goal_cover_id = None
        return RoleAssignment(mapping)

    # Internals

    def _configured_goalkeeper(self) -> int | None:
        return self.kit.config.goalkeeper_player_id()

    def select_chaser(self, context: PlayContext) -> int | None:
        """Keep the current field player on the ball until another has a clear advantage."""
        config = self.kit.config
        ball = context.known_ball
        game = context.known_game
        goalkeeper_id = self._configured_goalkeeper()

        scores: dict[int, float] = {}
        for player_id in config.player_ids:
            if player_id == goalkeeper_id or not self.kit.is_player_allowed(game, player_id):
                continue
            robot = context.teammates.get(player_id)
            if robot is None or robot.pose is None:
                continue
            scores[player_id] = self.kit.targeting.ball_claim_score(
                config.ready_slot_for_player(player_id), robot.pose, ball,
            )

        if not scores:
            self._chaser_id = None
            return None

        best_id = min(scores, key=lambda player_id: (scores[player_id], player_id))
        current_id = self._chaser_id
        margin = config.strategy.teammate_challenge_tie_margin_m
        if current_id is not None and scores.get(current_id, float("inf")) <= scores[best_id] + margin:
            return current_id
        self._chaser_id = best_id
        return best_id
