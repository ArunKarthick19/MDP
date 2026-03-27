from enum import Enum


class Direction(int, Enum):
    NORTH = 0
    EAST = 2
    SOUTH = 4
    WEST = 6
    SKIP = 8

    def __int__(self):
        return self.value

    @staticmethod
    def rotation_cost(d1, d2):
        diff = abs(d1 - d2)
        return min(diff, 8 - diff)

MOVE_DIRECTION = [
    (1, 0, Direction.EAST),
    (-1, 0, Direction.WEST),
    (0, 1, Direction.NORTH),
    (0, -1, Direction.SOUTH),
]

TURN_FACTOR = 1

# Grid resolution: 40x40 for 200cm x 200cm arena → 5cm per cell
# Obstacle: 10cm x 10cm = 2x2 cells
# Robot: 25cm x 25cm = 5x5 cells (half-width = 2.5 cells)
# EXPANDED_CELL = half-width of robot in cells, so 1 + EXPANDED_CELL*2 = 8 cells = 40cm clearance
EXPANDED_CELL = 2.5 # for both agent and obstacles

WIDTH = 40
HEIGHT = 40

ITERATIONS = 1000
TURN_RADIUS = 2  # Physical turn radius unchanged; 2 cells at 5cm/cell = 10cm arc step

SAFE_COST = 1000 # the cost for the turn in case there is a chance that the robot is touch some obstacle
SCREENSHOT_COST = 1 # the cost for the place where the picture is taken
CELL_SIZE_CM = 5 # physical size of one grid cell in centimetres