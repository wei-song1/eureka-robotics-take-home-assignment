import math
import time

YAW_NORTH = math.pi / 2
YAW_SOUTH = -math.pi / 2
YAW_EAST = 0
YAW_WEST = math.pi

robot = None
simulator = None
travelled_path = None
map_render_callback = None

def configure(
    robot_value,
    simulator_value,
    travelled_path_value,
    map_render_callback_value=None,
):
    global robot, simulator, travelled_path, map_render_callback
    robot = robot_value
    simulator = simulator_value
    travelled_path = travelled_path_value
    map_render_callback = map_render_callback_value

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

def move_route(route, map_info, final_yaw=None, timeout=15):
    if not route:
        raise ValueError("Cannot move along an empty route")

    resolution = map_info["resolution"]
    origin_x, origin_y = map_info["origin"]
    height = map_info["height"]

    def grid_to_world(point):
        column, row = point
        return (
            origin_x + (column + 0.5) * resolution,
            origin_y + (height - row - 0.5) * resolution,
        )

    waypoints = []
    previous_direction = None

    for index, point in enumerate(route):
        if index == 0:
            continue

        direction = (
            point[0] - route[index - 1][0],
            point[1] - route[index - 1][1],
        )

        if (
            previous_direction is not None
            and direction != previous_direction
        ):
            waypoints.append(route[index - 1])

        previous_direction = direction

    waypoints.append(route[-1])

    try:
        for point in waypoints:
            target_x, target_y = grid_to_world(point)
            simulator.goto(target_x, target_y)
            deadline = time.monotonic() + timeout

            while time.monotonic() < deadline:
                pose = simulator.read_state()["pose"]
                travelled_path.append((pose["x"], pose["y"]))

                if math.hypot(
                    target_x - pose["x"],
                    target_y - pose["y"],
                ) <= max(0.03, resolution):
                    break

                time.sleep(0.05)
            else:
                print("Robot unable to reach route waypoint")

            if map_render_callback is not None:
                map_render_callback()

        if final_yaw is not None:
            face_direction(final_yaw)
    finally:
        simulator.stop_robot()