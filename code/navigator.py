import argparse
import io
import json
import math
import time

import numpy as np
import zmq
from PIL import Image, ImageDraw

# Constant Declarations
YAW_NORTH = math.pi / 2
YAW_SOUTH = -math.pi / 2
YAW_EAST = 0
YAW_WEST = math.pi

ROBOT_RADIUS_M = 0.17
ROBOT_DIAMETER_M = 2 * ROBOT_RADIUS_M

GOAL_POSITION_TOLERANCE_M = 0.05
GOAL_YAW_TOLERANCE_RAD = math.radians(5)

GOAL_COLOUR = "yellow"
TRAIL_COLOUR = "blue"
ROBOT_COLOUR = "green"
ROBOT_POINTER_COLOUR = "lime"
OUTLINE_COLOUR = "black"

ap = argparse.ArgumentParser()
ap.add_argument("--host", default="localhost")
ap.add_argument("--port-base", type=int, default=5555)
args = ap.parse_args()


def url(k):
    return f"tcp://{args.host}:{args.port_base + k}"


ctx = zmq.Context()
rpc = ctx.socket(zmq.REQ)
rpc.connect(url(0))
cmd = ctx.socket(zmq.PUSH)
cmd.connect(url(1))
state = ctx.socket(zmq.SUB)
state.connect(url(2))
state.setsockopt(zmq.SUBSCRIBE, b"")
camera = ctx.socket(zmq.SUB)
camera.setsockopt(zmq.RCVHWM, 2)
camera.connect(url(3))
camera.setsockopt(zmq.SUBSCRIBE, b"")



# Helper Functions ====================================
def call(method, **params):
    rpc.send_json({"method": method, "params": params})
    frames = rpc.recv_multipart()
    rep = json.loads(frames[0])
    if not rep["ok"]:
        raise RuntimeError(rep["error"])
    return rep["result"], frames[1:]


def latest(sock):
    """Most recent message: drop anything queued (streams are 'latest wins')."""
    msg = sock.recv_multipart()
    while sock.poll(0):
        msg = sock.recv_multipart()
    return msg

# ping 	— 	{"t": float}
def ping():
    result, _ = call("ping")
    return result

# list_scenes 	— 	[{"id": "01", "name": str, "difficulty": "easy"|"advanced"}]
def list_scenes():
    scenes, _ = call("list_scenes")
    return scenes

# load_scene 	id, seed? 	{"id", "start": {"x", "y", "yaw"}, "goals": [...]}; start is exact
def load_scene(scene_id, seed=None):
    params = {"id": scene_id}
    if seed is not None:
        params["seed"] = seed

    scene, _ = call("load_scene", **params)
    return scene

# reset 	seed? 	same as load_scene, current scene
def reset(seed=None):
    params = {}
    if seed is not None:
        params["seed"] = seed

    scene, _ = call("reset", **params)
    return scene

# get_goals 	— 	[{"name": str, "pose": [x, y, yaw?]}]
def get_goals():
    goals, _ = call("get_goals")
    return goals

# get_map 	— 	header + PNG frame (section 7)
def get_map():
    header, frames = call("get_map")
    return header, frames

# get_robot_info 	— 	dimensions, limits, ticks_per_rev, watchdog, rates
def get_robot_info():
    info, _ = call("get_robot_info")
    return info

# get_camera_info 	— 	{"width", "height", "fx", "fy", "cx", "cy", "depth_scale", "max_depth", "rate", "T_base_camera": 4×4}
def get_camera_info():
    info, _ = call("get_camera_info")
    return info

# mark_goal 	name 	{"t": float}: simulation time of the mark
def mark_goal(name):
    result, _ = call("mark_goal", name=name)
    return result

def choose(prompt, options):
    """Accept an option's name or its number."""
    print(f"\n{prompt}")
    for option in options:
        print(f"- {option}")

    while True:
        answer = input("> ").strip()

        for option in options:
            if answer.casefold() == option.casefold():
                return option

        if answer.isdigit() and 1 <= int(answer) <= len(options):
            return options[int(answer) - 1]

        print("Please enter one of the listed options.")
# Helper Functions ====================================



# Robot Movement Functions =============================
def face_direction(yaw):
    wheel_base = robot["wheel_base"]
    wheel_radius = robot["wheel_radius"]
    deadline = time.monotonic() + 15.0

    try:
        while time.monotonic() < deadline:
            if not state.poll(500):
                raise TimeoutError("No robot state received")

            pose = json.loads(latest(state)[0])["pose"]

            error = math.atan2(
                math.sin(yaw - pose["yaw"]),
                math.cos(yaw - pose["yaw"]),
            )

            if abs(error) < math.radians(3):
                return

            # Slow down as we approach the desired heading.
            angular_speed = max(-1.0, min(1.0, 2.0 * error))
            wheel_speed = angular_speed * wheel_base / (2 * wheel_radius)

            cmd.send_json({
                "type": "wheels",
                "left": -wheel_speed,
                "right": wheel_speed,
            })
            time.sleep(0.05)

        raise TimeoutError("Robot did not reach the desired heading")
    finally:
        cmd.send_json({"type": "wheels", "left": 0.0, "right": 0.0})

def face_north():
    face_direction(YAW_NORTH)

def face_south():
    face_direction(YAW_SOUTH)

def face_east():
    face_direction(YAW_EAST)

def face_west():
    face_direction(YAW_WEST)

def turn_45d_clockwise():
    pose = json.loads(latest(state)[0])["pose"]
    face_direction(pose["yaw"] - math.pi/4)

def turn_45d_anticlockwise():
    pose = json.loads(latest(state)[0])["pose"]
    face_direction(pose["yaw"] + math.pi/4)

def move_forward_one_robot_size(timeout=15):
    if not state.poll(500):
        raise TimeoutError("No robot state received")

    pose = json.loads(latest(state)[0])["pose"]

    distance = 0.34
    target_x = pose["x"] + distance * math.cos(pose["yaw"])
    target_y = pose["y"] + distance * math.sin(pose["yaw"])

    command_id = time.time_ns()

    cmd.send_json({
        "type": "goto",
        "id": command_id,
        "x": target_x,
        "y": target_y,
    })

    deadline = time.monotonic() + timeout

    try:
        while time.monotonic() < deadline:
            if not state.poll(500):
                raise TimeoutError("No robot state received")

            pose = json.loads(latest(state)[0])["pose"]

            # Record positions during movement for the blue trail.
            travelled_path.append((pose["x"], pose["y"]))

            distance_remaining = math.hypot(
                target_x - pose["x"],
                target_y - pose["y"],
            )

            if distance_remaining <= 0.03:  # Within 3 cm
                return command_id

        raise TimeoutError("Robot did not reach the target")

    finally:
        cmd.send_json({
            "type": "wheels",
            "left": 0.0,
            "right": 0.0,
        })
# Robot Movement Functions =============================



# Map Functions =========================================
def save_top_down_map(filename="top_down_map.png"):
    map_info, frames = call("get_map")
    goals, _ = call("get_goals")

    robot_state = json.loads(latest(state)[0])
    pose = robot_state["pose"]

    travelled_path.append((pose["x"], pose["y"]))

    resolution = map_info["resolution"]
    origin_x, origin_y = map_info["origin"]
    height = map_info["height"]

    # Convert world coordinates to image coordinates.
    def world_to_pixel(x, y):
        pixel_x = (x - origin_x) / resolution - 0.5
        pixel_y = height - 0.5 - (y - origin_y) / resolution
        return pixel_x, pixel_y

    image = Image.open(io.BytesIO(frames[0])).convert("RGB")
    draw = ImageDraw.Draw(image)

    # Convert all recorded positions into image coordinates.
    path_pixels = [
        world_to_pixel(x, y)
        for x, y in travelled_path
    ]

    # Connect consecutive positions with a blue line.
    if len(path_pixels) >= 2:
        draw.line(
            path_pixels,
            fill=TRAIL_COLOUR,
            width=3,
        )

    # Draw goals before the robot so the robot stays visible.
    for goal in goals:
        goal_pose = goal["pose"]
        goal_x, goal_y = goal_pose[:2]
        gx, gy = world_to_pixel(goal_x, goal_y)

        marker_radius = 5

        draw.ellipse(
            (
                gx - marker_radius,
                gy - marker_radius,
                gx + marker_radius,
                gy + marker_radius,
            ),
            fill=GOAL_COLOUR,
            outline=GOAL_COLOUR,
            width=1,
        )

        # Show the required final heading, when provided.
        if len(goal_pose) > 2 and goal_pose[2] is not None:
            goal_yaw = goal_pose[2]
            heading_length = 0.25  # Metres

            end_x = goal_x + heading_length * math.cos(goal_yaw)
            end_y = goal_y + heading_length * math.sin(goal_yaw)
            end_px, end_py = world_to_pixel(end_x, end_y)

            draw.line(
                (gx, gy, end_px, end_py),
                fill=GOAL_COLOUR,
                width=2,
            )

        draw.text(
            (gx + marker_radius + 3, gy - 8),
            goal["name"],
            fill=GOAL_COLOUR,
            stroke_width=1,
            stroke_fill=OUTLINE_COLOUR,
        )

    pixel_x, pixel_y = world_to_pixel(pose["x"], pose["y"])

    # Robot diameter is 0.34 m.
    robot_radius = 0.34 / 2
    radius_pixels = robot_radius / resolution

    # Draw the robot's footprint at its actual scale.
    draw.ellipse(
        (
            pixel_x - radius_pixels,
            pixel_y - radius_pixels,
            pixel_x + radius_pixels,
            pixel_y + radius_pixels,
        ),
        fill=ROBOT_COLOUR,
        outline=ROBOT_COLOUR,
        width=1,
    )

    # Heading arrow, extending slightly beyond the footprint.
    yaw = pose["yaw"]
    arrow_length = radius_pixels * 1.5

    # Negate sine because image Y points downward.
    direction_x = math.cos(yaw)
    direction_y = -math.sin(yaw)

    tip_x = pixel_x + arrow_length * direction_x
    tip_y = pixel_y + arrow_length * direction_y

    draw.line(
        (pixel_x, pixel_y, tip_x, tip_y),
        fill=ROBOT_POINTER_COLOUR,
        width=2,
    )

    # Draw a triangular arrowhead.
    head_length = radius_pixels * 0.5
    head_half_width = radius_pixels * 0.3

    base_x = tip_x - head_length * direction_x
    base_y = tip_y - head_length * direction_y

    perpendicular_x = -direction_y
    perpendicular_y = direction_x

    draw.polygon(
        [
            (tip_x, tip_y),
            (
                base_x + head_half_width * perpendicular_x,
                base_y + head_half_width * perpendicular_y,
            ),
            (
                base_x - head_half_width * perpendicular_x,
                base_y - head_half_width * perpendicular_y,
            ),
        ],
        fill=ROBOT_POINTER_COLOUR,
    )

    image.save(filename, format="PNG")

    print("\n=== Debug Info ===")
    print(f"Map saved to {filename}")
    print("Map Info:", map_info)
    print(
        f"Robot: x={pose['x']:.3f} m, "
        f"y={pose['y']:.3f} m, "
        f"yaw={math.degrees(yaw):.1f}°"
    )
    print(f"Robot diameter on image: {2 * radius_pixels:.1f} pixels")
    print("Goals:", [goal["name"] for goal in goals])
    print("===================\n")
# Map Functions =========================================


# Scene selection
scenes, _ = call("list_scenes")

scene_options = {
    f"{s['id']} ({s['difficulty'].lower()})": s["id"]
    for s in scenes
}

selected_scene = choose("Which scene do you want?", list(scene_options))
scene_id = scene_options[selected_scene]


# Seed selection
print("\nWhat seed?")
while True:
    try:
        seed = int(input("> ").strip())
        break
    except ValueError:
        print("Please enter a whole number.")


# Load the selected scene to get its goals.
scene, _ = call("load_scene", id=scene_id, seed=seed)

goal_names = [g["name"] for g in scene["goals"]]
selected_goal = choose("Which goal?", goal_names + ["Manual"])

# Automatic goal handling will be added later.
if selected_goal != "Manual":
    sys.exit(0)


# Manual mode initialisation
start = scene["start"]
robot, _ = call("get_robot_info")
cam, _ = call("get_camera_info")
travelled_path = []

face_north()
save_top_down_map()

controls = {
    "w": face_north,
    "s": face_south,
    "d": face_east,
    "a": face_west,
    "q": turn_45d_anticlockwise,
    "e": turn_45d_clockwise,
    "f": move_forward_one_robot_size,
}

print("\nManual Robot Controls")
print("W = North, S = South, D = East, A = West")
print("Q = Turn Anticlockwise, E = Turn Clockwise")
print("F = Move Forward, R <goal> = Mark Goal")
print("Type a letter and press Enter. M = Quit.")

try:
    while True:
        key = input("\nDirection> ").strip().lower()

        if key == "m":
            break

        parts = key.split(maxsplit=1)

        try:
            if parts and parts[0] == "r":
                if len(parts) < 2:
                    print("Usage: r <goal_name>, e.g. r kitchen")
                    continue

                mark_goal(parts[1])
            else:
                action = controls.get(key)

                if action is None:
                    print(
                        "Invalid input. Use W, A, S, D, E, Q, F, "
                        "R <goal>, or M to quit."
                    )
                    continue

                action()

            save_top_down_map()
            print("Finished action.")

        except TimeoutError as exc:
            print(f"Action failed: {exc}")

except (KeyboardInterrupt, EOFError):
    print("\nExiting.")

finally:
    cmd.send_json({"type": "wheels", "left": 0.0, "right": 0.0})
    travelled_path.clear()
    scene, _ = call("load_scene", id=scene_id, seed=seed)