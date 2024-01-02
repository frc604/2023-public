import numpy as np

# Since our arm is off center, we want to offset + on blue and - on red.
SCORING_OFFSET = 0.1

# Slower speed when traversing bump
SLOW_BUMP_SPEED = 3.0  # m/s

### TOP
TOP_START_POS = (
    2.0,
    (5.0, 5.0 - SCORING_OFFSET),
    (np.deg2rad(175.0), 0.0),
)  # Cone
TOP_START_POS_PC = (1.9, 5.0, (1.5 * np.pi, 0.5 * np.pi))  # Cube
TOP_BACKUP_SLIGHTLY = (2.25, 5.0, (np.pi, 0.0))  # Cone
# TOP_SECOND_SCORING_POS = (2.15, 3.9, (np.pi, 0.0))  # Cone
TOP_THIRD_SCORING_POS = (
    2.25,
    (4.4 + SCORING_OFFSET, 4.4 - SCORING_OFFSET),
    (np.pi, 0.0),
)  # Cube

TOP_COMMUNITY_EXIT = (4.4, (4.7, 4.8), (np.deg2rad(175.0), 0.0))
TOP_COMMUNITY_ENTRANCE = (4.4, (4.7, 4.8), (np.pi, 0.0))
TOP_COMMUNITY_ENTRANCE_PC = (4.0, (4.7, 4.8), (1.5 * np.pi, 0.5 * np.pi))
TOP_INTAKE_DEPLOY_PC = (2.0, 4.8, (1.5 * np.pi, 0.5 * np.pi))
# TOP_COMMUNITY_ENTRANCE_FAKE = (2.5, 4.8, (np.pi, 0.0))

TOP_FIRST_PIECE_APPROACH = (5.6, (4.5, 4.7), (1.5 * np.pi, 0.5 * np.pi))
# TOP_FIRST_PIECE_APPROACH_FAKE = (3.0, 4.6, (1.5 * np.pi, 0.5 * np.pi))
TOP_FIRST_PIECE = (6.8, (4.5, 4.7), (1.5 * np.pi, 0.5 * np.pi))
# TOP_FIRST_PIECE_FAKE = (3.5, 4.6, (1.5 * np.pi, 0.5 * np.pi))
TOP_SECOND_PIECE_APPROACH = (6.1, 4.6, (1.25 * np.pi, 0.75 * np.pi))
TOP_SECOND_PIECE = (6.8, 3.7, (1.25 * np.pi, 0.75 * np.pi))
# Init waypoint to get around charge station corner
TOP_SCORING_INIT = (2.3, 4.6, (np.pi, 0.0))
TOP_CHARGE_STATION = (5.7, 2.7, (np.pi, 0.0))

TOP_INSIDE_CHARGE_INIT = (2.2, 3.3, (np.pi, 0.0))
TOP_CHARGE_STATION_INSIDE = (2.2, 3.1, (np.pi, 0.0))

### Bottom
BOTTOM_START_POS = (2.0, (0.5 + SCORING_OFFSET, 0.5), (np.pi, np.deg2rad(-5.0)))  # Cone
BOTTOM_START_POS_PC = (1.9, 0.5, (1.5 * np.pi, 0.5 * np.pi))  # Cube
BOTTOM_BACKUP_SLIGHTLY = (2.25, 0.5, (np.pi, 0.0))  # Cone
# BOTTOM_SECOND_SCORING_POS = (2.15, 1.6, (np.pi, 0.0))  # Cone
BOTTOM_THIRD_SCORING_POS = (
    2.15,
    (1.20 + SCORING_OFFSET, 1.1 - SCORING_OFFSET),
    (np.pi, 0.0),
)  # Cube

BOTTOM_COMMUNITY_EXIT = (5.0, 0.7, (np.pi, np.deg2rad(-5.0)))
BOTTOM_COMMUNITY_ENTRANCE = (5.0, 0.7, (np.pi, 0.0))
BOTTOM_COMMUNITY_ENTRANCE_PC = (5.0, 0.7, (1.5 * np.pi, 0.5 * np.pi))
BOTTOM_INTAKE_DEPLOY_PC = (3.0, 0.7, (1.5 * np.pi, 0.5 * np.pi))

BOTTOM_FIRST_PIECE_APPROACH = (5.6, (0.8, 1.05), (1.5 * np.pi, 0.5 * np.pi))
BOTTOM_FIRST_PIECE = (6.8, (0.8, 1.05), (1.5 * np.pi, 0.5 * np.pi))
BOTTOM_SECOND_PIECE_APPROACH = (6.1, 1.4, (1.75 * np.pi, 0.25 * np.pi))
BOTTOM_SECOND_PIECE = (6.8, 1.8, (1.75 * np.pi, 0.25 * np.pi))

# Init waypoint to get around charge station corner
BOTTOM_SCORING_INIT = (2.3, 0.5, (np.pi, 0.0))
BOTTOM_CHARGE_STATION = (5.7, 2.7, (np.pi, 0.0))

BOTTOM_CHARGE_STATION_INSIDE = (2.2, 2.3, (np.pi, 0.0))
BOTTOM_INSIDE_CHARGE_INIT = (2.2, 0.5, (np.pi, 0.0))

# Mid start pos
MID_START_POS = (
    2.15,
    (2.2 + SCORING_OFFSET, 2.2 - SCORING_OFFSET),
    (np.pi, 0.0),
)  # Cone
MID_BACKUP_SLIGHTLY = (2.25, 2.2, (np.pi, 0.0))  # Cone
MID_CHARGE_STATION_INSIDE = (2.25, 2.75, (np.pi, 0.0))
