import math
import time


YAW_NORTH = math.pi / 2
YAW_SOUTH = -math.pi / 2
YAW_EAST = 0
YAW_WEST = math.pi

robot = None
simulator = None
travelled_path = None


def configure(robot_value, simulator_value, travelled_path_value):
    global robot, simulator, travelled_path
    robot = robot_value
    simulator = simulator_value
    travelled_path = travelled_path_value


def face_direction(yaw):
    wheel_base = robot["wheel_base"]
    wheel_radius = robot["wheel_radius"]
    deadline = time.monotonic() + 15.0

    try:
        while time.monotonic() < deadline:
            pose = simulator.read_state()["pose"]

            error = math.atan2(
                math.sin(yaw - pose["yaw"]),
                math.cos(yaw - pose["yaw"]),
            )

            if abs(error) < math.radians(3):
                return

            angular_speed = max(-1.0, min(1.0, 2.0 * error))
            wheel_speed = angular_speed * wheel_base / (2 * wheel_radius)

            simulator.set_wheel_speeds(-wheel_speed, wheel_speed)
            time.sleep(0.05)

        raise TimeoutError("Robot did not reach the desired heading")
    finally:
        simulator.stop_robot()


def face_north():
    face_direction(YAW_NORTH)


def face_south():
    face_direction(YAW_SOUTH)


def face_east():
    face_direction(YAW_EAST)


def face_west():
    face_direction(YAW_WEST)


def turn_45d_clockwise():
    pose = simulator.read_state()["pose"]
    face_direction(pose["yaw"] - math.pi / 4)


def turn_45d_anticlockwise():
    pose = simulator.read_state()["pose"]
    face_direction(pose["yaw"] + math.pi / 4)


def move_forward_one_robot_size(timeout=15):
    pose = simulator.read_state()["pose"]

    distance = 0.34
    target_x = pose["x"] + distance * math.cos(pose["yaw"])
    target_y = pose["y"] + distance * math.sin(pose["yaw"])

    command_id = simulator.goto(target_x, target_y)

    deadline = time.monotonic() + timeout

    try:
        while time.monotonic() < deadline:
            pose = simulator.read_state()["pose"]
            travelled_path.append((pose["x"], pose["y"]))

            distance_remaining = math.hypot(
                target_x - pose["x"],
                target_y - pose["y"],
            )

            if distance_remaining <= 0.03:
                return command_id

        raise TimeoutError("Robot did not reach the target")

    finally:
        simulator.stop_robot()
