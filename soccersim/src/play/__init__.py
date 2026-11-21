"""PLAY-stage strategy package and template core.

Competitors mainly inspect this package for the default Playbook, role strategy
base classes, shared utility nodes, dynamic roles, the Playbook registry, and
the PLAY subtree shape with rule guards and role branches.

All roles extend :class:`RoleStrategy`; the required contract is
:meth:`RoleStrategy.build_subtree`. Pure positioning roles can use a single
:class:`MoveToTarget` leaf, and conditional kicking roles can use
:func:`build_attack_subtree`.
"""

from ..soccer_framework import PlayContext
from .default_roles import (
    ChaserRole,
    DefenderRole,
    GoalkeeperRole,
    SupporterRole,
)
from .nodes import (
    AssignRoles,
    AttackSubtreeConfig,
    IsRole,
    IsKickWanted,
    KickAction,
    MoveToTarget,
    WaitForBall,
    build_attack_subtree,
)
from .playbook import (
    DefaultPlaybook,
    Playbook,
    ROLE_CHASER,
    ROLE_GOALKEEPER,
    ROLE_NONE,
    ROLE_SUPPORTER,
    RoleAssignment,
)
from .registry import PLAYBOOKS, PlaybookRegistry
from .role import (
    RoleRegistry,
    RoleStrategy,
)
from .team_playbook import TeamPlaybook
from .play_subtree import create_play_subtree


# ----------------------------------------------------------------------
# Built-in Playbook registration is visible and uses the same API as custom Playbooks.
# TeamPlaybook is the team's active default; the plain default stays creatable by name.
# ----------------------------------------------------------------------
PLAYBOOKS.register("default", DefaultPlaybook)
PLAYBOOKS.register("team", TeamPlaybook, default=True)

__all__ = [
    "AssignRoles",
    "AttackSubtreeConfig",
    "ChaserRole",
    "DefaultPlaybook",
    "DefenderRole",
    "GoalkeeperRole",
    "IsRole",
    "IsKickWanted",
    "KickAction",
    "MoveToTarget",
    "PLAYBOOKS",
    "PlayContext",
    "Playbook",
    "PlaybookRegistry",
    "ROLE_CHASER",
    "ROLE_GOALKEEPER",
    "ROLE_NONE",
    "ROLE_SUPPORTER",
    "RoleAssignment",
    "RoleRegistry",
    "RoleStrategy",
    "SupporterRole",
    "TeamPlaybook",
    "WaitForBall",
    "build_attack_subtree",
    "create_play_subtree",
]
