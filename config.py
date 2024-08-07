from os import name
CLEAR_CMD = "cls" if name == "nt" else "clear"

FINAL_SPEED    = -1
FINAL_ALTITUDE = 5
MAX_HOR_SPEED  = 5

GEAR_DELAY = 4

DRAG_COEFFICIENT = 0.4
DRAG_AREA        = 3.14 * (1.2)**2

USE_TRAJECTORY  = True
DRAW_TRAJECTORY = True

INTERVAL = 0.05