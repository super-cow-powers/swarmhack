import time

from robots import Robot
from robots import RobotState
import random


def update_flock(robot: Robot) -> Robot:
    # print(robot)
    robot = auto_mode(robot)
    if any(ir > robot.ir_threshold for ir in robot.ir_readings):
        print(avoid_obstacle(robot))
    else:
        print(RobotState.FORWARDS)

    return robot


def avoid_obstacle(robot: Robot) -> RobotState:
    if sum(robot.ir_readings[:3]) > sum(robot.ir_readings[3:]):
        return RobotState.RIGHT
    else:
        return RobotState.LEFT


def reorientate_to_flock(robot: Robot) -> RobotState:
    if sum(robot.ir_readings[:3]) < sum(robot.ir_readings[3:]):
        return RobotState.RIGHT
    else:
        return RobotState.LEFT


def auto_mode(robot: Robot) -> Robot:
    # Autonomous mode
    distance_av = 0
    bearing_av = 0
    has_36 = False
    for robot_id, neighbour in robot.neighbours.items():
        distance_av += neighbour['range']
        bearing_av += neighbour['bearing']
        if robot_id == 36:
            has_36 = True

    distance_av /= (len(robot.neighbours.keys()) + 1e-20)
    bearing_av /= (len(robot.neighbours.keys()) + 1e-20)
    distance_threshold = 0.2

    closest_target = None
    for target in robot.tasks.values():
        if closest_target is None:
            closest_target = target
        elif target['range'] < closest_target['range']:
            closest_target = target

    if robot.state == RobotState.FORWARDS:
        left = right = robot.MAX_SPEED
        if any(ir > robot.ir_threshold for ir in robot.ir_readings):
            robot.turn_time = time.time()
            robot.state = avoid_obstacle(robot)

        elif (time.time() - robot.turn_time > 0.5) and distance_av > distance_threshold:
            if bearing_av > 15:
                robot.turn_time = time.time()
                robot.state = RobotState.RIGHT
                left = -robot.MAX_SPEED
                right = robot.MAX_SPEED
            if bearing_av < -15:
                robot.turn_time = time.time()
                robot.state = RobotState.LEFT
                left = robot.MAX_SPEED
                right = -robot.MAX_SPEED

        elif (time.time() - robot.turn_time > 0.5) and has_36:
            if robot.neighbours[36]['bearing'] > 15:
                robot.turn_time = time.time()
                robot.state = RobotState.RIGHT
            if robot.neighbours[36]['bearing'] < 15:
                robot.turn_time = time.time()
                robot.state = RobotState.LEFT

        elif closest_target is not None:
            if closest_target['bearing'] > 15:
                robot.turn_time = time.time()
                robot.state = RobotState.RIGHT
            if closest_target['bearing'] < 15:
                robot.turn_time = time.time()
                robot.state = RobotState.LEFT

    elif robot.state == RobotState.BACKWARDS:
        left = right = -robot.MAX_SPEED
        robot.turn_time = time.time()
        robot.state = RobotState.FORWARDS
    elif robot.state == RobotState.LEFT:
        left = -robot.MAX_SPEED / 2
        right = robot.MAX_SPEED / 2
        if time.time() - robot.turn_time > random.uniform(0.5, 1.0):
            robot.turn_time = time.time()
            robot.state = RobotState.FORWARDS
    elif robot.state == RobotState.RIGHT:
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
