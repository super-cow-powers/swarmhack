import time

from robots import Robot
from robots import RobotState
import random


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
    robot.left = left * robot.MAX_SPEED
    robot.right = right * robot.MAX_SPEED
    return robot


# Avoid an obstacle
def avoid_obstacle(robot: Robot) -> Robot:
    if sum(robot.ir_readings[:3]) > sum(robot.ir_readings[2:]):
        robot = setMove(1, -1, robot)
    else:
        robot = setMove(-1, 1, robot)
    return robot


# Move back towards CoM
def reorientate_to_flock(robot: Robot) -> Robot:
    if sum(robot.ir_readings[:3]) > sum(robot.ir_readings[2:]):
        robot = setMove(1, -1, robot)
    else:
        robot = setMove(-1, 1, robot)
    return robot


def check_fov(robot: Robot, bearing: int) -> Robot:
    if bearing > 20:
        robot.turn_time = time.time()
        robot = setMove(1, -1, robot)
    elif bearing < -20:
        robot.turn_time = time.time()
        robot = setMove(-1, 1, robot)
    else:
        robot = setMove(1, 1, robot)
    return robot


def auto_mode(robot: Robot, leader_id) -> Robot:
    # global leader_id
    # Autonomous mode
    distance_av = 0
    bearing_av = 0
    has_leader = False
    for robot_id, neighbour in robot.neighbours.items():
        if int(robot_id) < 36 or int(robot_id) > 40:
            continue
        distance_av += neighbour["range"]
        bearing_av += neighbour["bearing"]
        if robot_id == leader_id:
            has_leader = True

    distance_av /= len(robot.neighbours.keys()) + 1e-20
    bearing_av /= len(robot.neighbours.keys()) + 1e-20
    distance_threshold = 0.2

    closest_target = None
    for target in robot.tasks.values():
        if closest_target is None:
            closest_target = target
        elif target["range"] < closest_target["range"]:
            closest_target = target

    if robot.state == RobotState.FORWARDS:
        robot = setMove(1, 1, robot)
        if any(ir > robot.ir_threshold for ir in robot.ir_readings):
            robot.turn_time = time.time()
            robot = avoid_obstacle(robot)
        elif has_leader:
            print(leader_id + "-------------------")
            robot = check_fov(robot, robot.neighbours[leader_id]["bearing"])

        elif distance_av > distance_threshold:
            robot = check_fov(robot, bearing_av)

        elif closest_target is not None:
            pass
            # if closest_target["bearing"] > 15:
            #     robot.turn_time = time.time()
            #     robot = setMove(1,-1, robot)
            # if closest_target["bearing"] < 15:
            #     robot.turn_time = time.time()
            #     robot = setMove(-1,1, robot)

    elif robot.state == RobotState.BACKWARDS:
        robot = setMove(-1, -1, robot)
        robot.turn_time = time.time()

    elif robot.state == RobotState.LEFT:
        if any(ir > robot.ir_threshold for ir in robot.ir_readings):
            robot.turn_time = time.time()
            robot = avoid_obstacle(robot)
        else:
            robot = setMove(-0.9, 1, robot)

        if time.time() - robot.turn_time > 0.1:
            robot.turn_time = time.time()
            robot = setMove(0.5, 0.5, robot)

    elif robot.state == RobotState.RIGHT:
        if any(ir > robot.ir_threshold for ir in robot.ir_readings):
            robot.turn_time = time.time()
            robot = avoid_obstacle(robot)
        else:
            robot = setMove(1, -0.9, robot)

        if time.time() - robot.turn_time > 0.1:
            robot.turn_time = time.time()
            robot = setMove(0.5, 0.5, robot)

    elif robot.state == RobotState.STOP:
        robot.turn_time = time.time()
        robot = setMove(0, 0, robot)
    print(robot.state)
    return robot
