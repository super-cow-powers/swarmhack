from enum import Enum
import time


pipucks = {
    1:  "144.32.165.227",
    2:  "144.32.165.226",
    3:  "144.32.165.225",
    4:  "144.32.165.224",
    5:  "144.32.165.231",
    6:  "144.32.165.223",
    7:  "144.32.165.222",
    8:  "144.32.165.221",
    9:  "144.32.165.216",
    10: "144.32.165.220",
    11: "",
    12: "",
    13: "",
    14: "",
    15: "",
    16: "",
    17: "",
    18: "",
    19: "",
    20: "",
    21: "",
    22: "",
    23: "",
    24: "",
    25: "",
    26: "",
    27: "",
    28: "",
    29: "",
    30: ""
}

monas = {
    31: "144.32.165.235",
    32: "144.32.165.238",
    33: "144.32.165.234",
    34: "144.32.165.236",
    35: "144.32.165.237",
    36: "144.32.165.240",
    37: "144.32.165.244",
    38: "144.32.165.239",
    39: "144.32.165.232",
    40: "144.32.165.242",
    41: "144.32.165.212",
    42: "144.32.165.203",
    43: "144.32.165.202",
    44: "144.32.165.204",
    45: "144.32.165.208",
    46: "144.32.165.201",
    47: "144.32.165.200",
    48: "144.32.165.199",
    49: "144.32.165.198",
    50: "144.32.165.207",
    51: "",
    52: "",
    53: "",
    54: "",
    55: "",
    56: "",
    57: "",
    58: "",
    59: "",
    60: ""
}

robots = {**pipucks, **monas}

server_york = "144.32.165.233"
server_sheffield = ""
server_manchester = ""

class RobotState(Enum):
    FORWARDS = 1
    BACKWARDS = 2
    LEFT = 3
    RIGHT = 4
    STOP = 5


class Robot:

    BAT_LOW_VOLTAGE = 3.6
    MAX_SPEED = 100

    def __init__(self, id):
        self.id = id
        self.connection = None
        self.left = 0.0
        self.right = 0.0
        self.orientation = 0
        self.neighbours = {}
        self.tasks = {}

        self.teleop = False
        self.state = RobotState.STOP
        self.ir_readings = []
        self.battery_charging = False
        self.battery_voltage = 0
        self.battery_percentage = 0

        self.turn_time = time.time()

        if id < 31:
            self.ir_threshold = 200  # Pi-puck
        else:
            self.ir_threshold = 80  # Mona
