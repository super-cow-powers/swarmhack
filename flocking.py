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
    print("----------------", leader_id)
    robot = auto_mode(robot, leader_id)

    return robot


def avoid_obstacle(robot: Robot) -> RobotState:
    if sum(robot.ir_readings[:3]) > sum(robot.ir_readings[2:]):
        return RobotState.RIGHT
    else:
        return RobotState.LEFT


def reorientate_to_flock(robot: Robot) -> RobotState:
    if sum(robot.ir_readings[:3]) > sum(robot.ir_readings[2:]):
        return RobotState.RIGHT
    else:
        return RobotState.LEFT


def check_fov(robot: Robot, bearing: int) -> Robot:
    if bearing > 20:
        robot.turn_time = time.time()
        robot.state = RobotState.RIGHT
        left = robot.MAX_SPEED
        right = -robot.MAX_SPEED
    elif bearing < -20:
        robot.turn_time = time.time()
        robot.state = RobotState.LEFT
        left = -robot.MAX_SPEED
        right = robot.MAX_SPEED
    else:
        robot.state = RobotState.FORWARDS
        left = robot.MAX_SPEED
        right = robot.MAX_SPEED



def auto_mode(robot: Robot, leader_id) -> Robot:
    # global leader_id
    print("---------" + leader_id)
    left = right = 0
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
        left = right = robot.MAX_SPEED
        if any(ir > robot.ir_threshold for ir in robot.ir_readings):
            robot.turn_time = time.time()
            robot.state = avoid_obstacle(robot)
        elif has_leader:
            print(leader_id + "-------------------")
            robot = check_fov(robot, robot.neighbours[leader_id]['bearing'])

        elif distance_av > distance_threshold:
            pass
            # robot = check_fov(robot, bearing_av)

        elif closest_target is not None:
            pass
            # if closest_target["bearing"] > 15:
            #     robot.turn_time = time.time()
            #     robot.state = RobotState.RIGHT
            # if closest_target["bearing"] < 15:
            #     robot.turn_time = time.time()
            #     robot.state = RobotState.LEFT

    elif robot.state == RobotState.BACKWARDS:
        left = right = -robot.MAX_SPEED
        robot.turn_time = time.time()
        robot.state = RobotState.FORWARDS

    elif robot.state == RobotState.LEFT:
        if any(ir > robot.ir_threshold for ir in robot.ir_readings):
            robot.turn_time = time.time()
            robot.state = avoid_obstacle(robot)
        else:
            robot.state = RobotState.FORWARDS
        left = -robot.MAX_SPEED / 2
        right = robot.MAX_SPEED / 2
        if time.time() - robot.turn_time > random.uniform(0.5, 1.0):
            robot.turn_time = time.time()
            robot.state = RobotState.FORWARDS

    elif robot.state == RobotState.RIGHT:
        if any(ir > robot.ir_threshold for ir in robot.ir_readings):
            robot.turn_time = time.time()
            robot.state = avoid_obstacle(robot)
        else:
            robot.state = RobotState.FORWARDS
        left = robot.MAX_SPEED / 2
        right = -robot.MAX_SPEED / 2
        if time.time() - robot.turn_time > random.uniform(0.5, 1.0):
            robot.turn_time = time.time()
            robot.state = RobotState.FORWARDS

    elif robot.state == RobotState.STOP:
        left = right = 0
        robot.turn_time = time.time()
        robot.state = RobotState.FORWARDS
    robot.left = left
    robot.right = right
    print(robot.state)
    return robot
