import time

from numpy import pi

from robots import Robot
from robots import RobotState
from scipy import stats, special


# leader_id = ""


def update_flock(robot: Robot, leader_id) -> Robot:
    # global leader_id
    # print(robot)
    # if robot.teleop:
    #     leader_id = robot.id
    #     print("----------------", leader_id)
    robot = auto_mode(robot, leader_id)

    return robot


# Move in Direction
# left: -1 -> 1
# right: -1 -> 1
def setMove(left: float, right: float, robot: Robot) -> Robot:
    if (left > 1) or (right > 1):
        return robot
    if left > right:
        robot.state = RobotState.LEFT
    elif left < right:
        robot.state = RobotState.RIGHT
    elif (left == right) and ((left + right) > 0):
        robot.state = RobotState.FORWARDS
    elif (left == right) and ((left + right) < 0):
        robot.state = RobotState.BACKWARDS
    elif (left + right) == 0:
        robot.state = RobotState.STOP
    # robot.left = 0
    robot.left = left * robot.MAX_SPEED
    # robot.right = 0
    robot.right = right * robot.MAX_SPEED
    return robot


# Avoid an obstacle
def avoid_obstacle(robot: Robot) -> Robot:
    if sum(robot.ir_readings[:3]) > sum(robot.ir_readings[2:]):
        robot = setMove(1, -1, robot)
    else:
        robot = setMove(-1, 1, robot)
    return robot


def check_fov(robot: Robot, bearing: int) -> Robot:
    if bearing > 15:
        robot.turn_time = time.time()
        robot = setMove(1, -0.5, robot)
    elif bearing < -15:
        robot.turn_time = time.time()
        robot = setMove(-0.5, 1, robot)
    else:
        robot = setMove(1, 1, robot)
    return robot


def orientate(robot: Robot, other_orientation: float) -> Robot:
    if (
        abs(other_orientation) - 15 < abs(robot.orientation)
        and abs(robot.orientation) < abs(other_orientation) + 15
    ):
        return setMove(1, 1, robot)
    elif robot.orientation * other_orientation >= 0:
        if robot.orientation < other_orientation:
            # RIGHT
            return setMove(1, -0.5, robot)
        else:
            # LEFT
            return setMove(-0.5, 1, robot)
    elif robot.orientation < 0:
        if abs(robot.orientation) + abs(other_orientation) < 180:
            # RIGHT
            return setMove(1, -0.5, robot)
        else:
            # LEFT
            return setMove(-0.5, 1, robot)
    else:
        if abs(robot.orientation) + abs(other_orientation) > 180:
            # RIGHT
            return setMove(1, -0.5, robot)
        else:
            # LEFT
            return setMove(-0.5, 1, robot)


def auto_mode(robot: Robot, leader_id) -> Robot:
    # global leader_id
    # Autonomous mode
    closest_target = None
    for target in robot.tasks.values():
        if closest_target is None:
            closest_target = target
        elif target["range"] < closest_target["range"]:
            closest_target = target

    distance_av = 0
    neighbour_bearings = []
    local_orientations = []
    has_leader = False
    for robot_id, neighbour in robot.neighbours.items():
        if int(robot_id) < 36 or int(robot_id) > 40:
            continue
        distance_av += neighbour["range"]
        neighbour_bearings.append((neighbour["bearing"] / 180) * pi)
        local_orientations.append((neighbour["orientation"] / 180) * pi)
        if str(robot_id) == str(leader_id):
            has_leader = True

    distance_av /= len(robot.neighbours.keys()) + 1e-20
    if closest_target is not None:
        for _ in range(2):
            neighbour_bearings.append((closest_target["bearing"] / 180) * pi)
    weighted_bearing = (stats.circmean(neighbour_bearings, high=pi, low=-pi) / pi) * 180
    average_orientation = (
        stats.circmean(local_orientations, high=pi, low=-pi) / pi
    ) * 180
    distance_threshold = 0.2

    if robot.state == RobotState.FORWARDS:
        robot = setMove(0.7, 0.7, robot)
        if any(ir > robot.ir_threshold for ir in robot.ir_readings):
            robot.turn_time = time.time()
            robot = avoid_obstacle(robot)
        elif has_leader:
            robot = check_fov(robot, robot.neighbours[str(leader_id)]["bearing"])

        elif distance_av > distance_threshold:
            print(robot.id, weighted_bearing)
            robot = check_fov(robot, weighted_bearing)

        elif closest_target is not None:
            robot = check_fov(robot, closest_target["bearing"])

        else:
            robot = orientate(robot, average_orientation)

    elif robot.state == RobotState.BACKWARDS:
        robot = setMove(-1, -1, robot)
        robot.turn_time = time.time()

    elif robot.state == RobotState.LEFT:
        if any(ir > robot.ir_threshold for ir in robot.ir_readings):
            robot.turn_time = time.time()
            robot = avoid_obstacle(robot)
        else:
            robot = setMove(-0.9, 1, robot)

        if time.time() - robot.turn_time > 0.08:
            robot.turn_time = time.time()
            robot = setMove(0.5, 0.5, robot)

    elif robot.state == RobotState.RIGHT:
        if any(ir > robot.ir_threshold for ir in robot.ir_readings):
            robot.turn_time = time.time()
            robot = avoid_obstacle(robot)
        else:
            robot = setMove(1, -0.9, robot)

        if time.time() - robot.turn_time > 0.08:
            robot.turn_time = time.time()
            robot = setMove(0.5, 0.5, robot)

    elif robot.state == RobotState.STOP:
        robot.turn_time = time.time()
        robot = setMove(0, 0, robot)
    print(robot.state)
    return robot
