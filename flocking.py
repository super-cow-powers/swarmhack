def update_flock(robit: Robot) -> Robot:
    robit = auto_mode(robit)
    return robit

def auto_mode(robot: Robot) -> Robot:
    # Autonomous mode
            if robot.state == RobotState.FORWARDS:
                left = right = robot.MAX_SPEED
                if (time.time() - robot.turn_time > 0.5) and any(ir > robot.ir_threshold for ir in robot.ir_readings):
                    robot.turn_time = time.time()
                    robot.state = random.choice((RobotState.LEFT, RobotState.RIGHT))
            elif robot.state == RobotState.BACKWARDS:
                left = right = -robot.MAX_SPEED
                robot.turn_time = time.time()
                robot.state = RobotState.FORWARDS
            elif robot.state == RobotState.LEFT:
                left = -robot.MAX_SPEED
                right = robot.MAX_SPEED
                if time.time() - robot.turn_time > random.uniform(0.5, 1.0):
                    robot.turn_time = time.time()
                    robot.state = RobotState.FORWARDS
            elif robot.state == RobotState.RIGHT:
                left = robot.MAX_SPEED
                right = -robot.MAX_SPEED
                if time.time() - robot.turn_time > random.uniform(0.5, 1.0):
                    robot.turn_time = time.time()
                    robot.state = RobotState.FORWARDS
            elif robot.state == RobotState.STOP:
                left = right = 0
                robot.turn_time = time.time()
                robot.state = RobotState.FORWARDS

