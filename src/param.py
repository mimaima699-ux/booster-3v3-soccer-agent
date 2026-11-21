"""Central configuration point for strategy tuning.

This module contains tunable strategy, movement, kicking, and obstacle
avoidance parameters. Existing values were moved here from their original
modules; make future parameter adjustments here first.
"""

from __future__ import annotations

import math


# ======================================================================
# Kick power
# ======================================================================

# Player.kick() clamps power to this range. Unsupported values may cause a fall.
KICK_POWER_MIN = 1.0
KICK_POWER_MAX = 10.0

# Kick power during normal play.
KICK_POWER_DEFAULT = 5.0
KICK_POWER_BACKFIELD = 7.0


# ======================================================================
# Player movement control
# ======================================================================

ARRIVE_DIST = 0.15             # Target arrival threshold (m)
OMNI_DIST = 1.0                # Use omnidirectional control below this distance
TURN_THRESHOLD = 0.5           # Turn in place above this heading error (rad)
MAX_LINEAR = 2.0               # Maximum commanded forward speed (m/s)
MAX_ANGULAR = 2.0              # Maximum commanded angular speed (rad/s)
LINEAR_GAIN = 1.5              # Translation proportional gain
ANGULAR_GAIN = 2.0             # Rotation proportional gain


# ======================================================================
# Player kicking and shot planning
# ======================================================================

KICK_ENTER_M = 2.0             # Enter kicking state below this ball distance
KICK_EXIT_M = 2.5              # Leave kicking state above this ball distance
CHASE_BEHIND_M = 0.35          # Distance behind the ball while chasing


# ======================================================================
# Player skill parameters: guarding and support
# ======================================================================

GUARD_FACE_BALL = True

# Guard active interception: when the ball enters our half, the guard moves
# to intercept along the ball-to-goal line at this fraction of the distance
# from our goal, providing active defense instead of passively standing at
# the goal-area center.
GUARD_INTERCEPT_FRACTION = 0.4          # Fraction along ball-to-goal line (0 = own goal, 1 = ball)
GUARD_INTERCEPT_MIN_DIST_M = 0.8        # Minimum distance from our goal line
GUARD_THREAT_ENTER_X = -1.0             # Ball crosses this x to trigger active interception
GUARD_THREAT_EXIT_X = -0.7              # Ball crosses this x to resume passive guarding

SUPPORT_DIST_M = 3.0

# M-05 inactivity protection: any robot within BALL_NEAR_RADIUS_M of the ball
# must show real displacement within INACTIVITY_ALERT_S seconds, otherwise the
# referee may penalize the robot for local game stuck (10 s rule).
# The strategy repositions robots whose last significant movement is too old.
BALL_NEAR_RADIUS_M = 3.0               # Radius around ball where M-05 applies
INACTIVITY_ALERT_S = 6.0                # Alert threshold well under the 10 s detection line
INACTIVITY_MIN_DISPLACEMENT_M = 0.5     # Minimum displacement to count as "active"

# ======================================================================
# Normal-phase strategy
# ======================================================================

ATTACKER_KEEP_DIST_MARGIN_M = 0.3  # Prevent attacker-selection oscillation

FALLEN_COST = 10.0  # Distance penalty assigned to a fallen player (m)


# ======================================================================
# Kickoff and set-play strategy
# ======================================================================

# Kickoff
KICKOFF_STAGE_M = 2.0
KICKOFF_FRONT_MARGIN = 0.1
KICKOFF_LATERAL_TOL = 0.35

# Two-touch kickoff rule: a goal scored straight from the kickoff spot does
# not count. The ball must first leave this radius around the center spot
# (referee detection line is 0.5 m; this value keeps a margin) before a
# shot at the opponent goal is valid.
KICKOFF_CLEAR_RADIUS_M = 0.8

# First kickoff touch: a soft push along a line that cannot enter the
# opponent goal directly, so the ball leaves the kickoff zone without
# risking an invalid direct goal.
KICK_POWER_KICKOFF_PUSH = 2.5
KICKOFF_PUSH_SCAN_STEP_RAD = math.radians(15)  # Direction scan resolution
KICKOFF_PUSH_SCAN_STEPS = 8                    # Max scan offset: 8 * 15 deg

CENTER_LEAVE_DIST_M = 0.15 # Distance from center at which the ball counts as moved

OPP_SET_WALL_DIST_M = 2.0 # Blocking distance during an opponent kickoff

# Opponent set plays: retreat threshold and field boundary margin (m).
OPP_SET_PLAY_CLEARANCE_M = 1.65
OPP_SET_PLAY_FIELD_MARGIN_M = 0.3


# ======================================================================
# Positioning and avoidance
# ======================================================================

OPPONENT_RESTART_AVOID_M = 1.65
CIRCLE_MARGIN_M = 0.3


# ======================================================================
# Kick-target geometry
# ======================================================================

GOAL_TARGET_DEPTH_M = 0.25              # Kick target depth inside the goal (m)

# ======================================================================
# Obstacle geometry
# ======================================================================

BALL_OBSTACLE_RADIUS = 0.5              # Ball obstacle radius (m)
OPPONENT_RADIUS = 0.55                  # Opponent robot radius (m)
TEAMMATE_RADIUS = 0.48                  # Teammate robot radius (m)
SAFETY_MARGIN = 0.22                    # General safety margin (m)

GOAL_DEPTH = 0.6                        # Goal depth used for obstacle modeling (m)
POST_RADIUS = 0.18                      # Goal-post radius (m)
NET_RADIUS = 0.20                       # Goal-net sample radius (m)
NET_STEP = 0.35                         # Goal-net sampling interval (m)

START_IGNORE = 0.0                      # Ignore obstacles near the start (m)
TARGET_IGNORE = 0.0                     # Ignore obstacles near the target (m)

# ======================================================================
# Global path planner (A* grid planner)
# ======================================================================

USE_GLOBAL_PATH_PLANNER = True          # Fall back to local VFH when disabled
GLOBAL_GRID_RESOLUTION_M = 0.35         # Grid cell size; smaller is finer (m)
GLOBAL_FIELD_MARGIN_M = 0.25            # Extra field margin for boundary paths (m)
GLOBAL_OBSTACLE_MARGIN_M = 0.10         # Additional obstacle inflation radius (m)
GLOBAL_PATH_LOOKAHEAD_M = 0.9           # Lookahead distance along the path (m)

# ======================================================================
# Local path planner (VFH direction scan)
# ======================================================================

PLAN_LOOKAHEAD = 1.2                    # Forward probe length (m)
PLAN_CLEARANCE = 0.35                   # Minimum clearance for a candidate (m)
PLAN_STEP = math.radians(15)            # Candidate direction scan step (rad)
PLAN_MAX_OFFSET = math.radians(100)     # Maximum offset from target heading (rad)

# ======================================================================
# Visualization
# ======================================================================

KICK_TARGET_MARK_SIZE_M = 0.18
