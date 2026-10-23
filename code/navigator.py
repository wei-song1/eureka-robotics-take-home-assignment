"""Minimal Python client: RPC, map, wheels, goto, state and camera streams.

pip install pyzmq numpy pillow; python client.py [--host localhost]
"""
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

def move_forward_one_robot_size():
    robot_state = json.loads(latest(state)[0])
    pose = robot_state["pose"]

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

    return command_id

def save_top_down_map(filename="top_down_map.png"):
    map_info, frames = call("get_map")

    robot_state = json.loads(latest(state)[0])
    pose = robot_state["pose"]

    resolution = map_info["resolution"]
    origin_x, origin_y = map_info["origin"]
    height = map_info["height"]

    # Convert world coordinates to image coordinates.
    # Image Y increases downward; world Y increases upward.
    # The -0.5 aligns coordinates with pixel centres.
    pixel_x = (pose["x"] - origin_x) / resolution - 0.5
    pixel_y = height - 0.5 - (pose["y"] - origin_y) / resolution

    # Robot diameter is 0.34 m, according to the specification.
    robot_radius = 0.34 / 2
    radius_pixels = robot_radius / resolution

    image = Image.open(io.BytesIO(frames[0])).convert("RGB")
    draw = ImageDraw.Draw(image)

    # Draw the robot's footprint at its actual scale.
    draw.ellipse(
        (
            pixel_x - radius_pixels,
            pixel_y - radius_pixels,
            pixel_x + radius_pixels,
            pixel_y + radius_pixels,
        ),
        fill="dodgerblue",
        outline="blue",
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
        fill="red",
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
        fill="red",
    )

    image.save(filename, format="PNG")

    print(f"Map saved to {filename}")
    print("Map Info:", map_info)
    print(
        f"Robot: x={pose['x']:.3f} m, "
        f"y={pose['y']:.3f} m, "
        f"yaw={math.degrees(yaw):.1f}°"
    )
    print(f"Robot diameter on image: {2 * radius_pixels:.1f} pixels")

    return map_info

# Load a scene, read the robot and camera info, save the map.
print("scenes:", [s["id"] for s in call("list_scenes")[0]])
scene, _ = call("load_scene", id="01", seed=42)
print("start:", scene["start"], "goals:", [g["name"] for g in scene["goals"]])
robot, _ = call("get_robot_info")
cam, _ = call("get_camera_info")

save_top_down_map()
face_north()

# Terminal keyboard controls
controls = {
    "w": face_north,
    "s": face_south,
    "d": face_east,
    "a": face_west,
    "q": turn_45d_anticlockwise,
    "e": turn_45d_clockwise,
    "f": move_forward_one_robot_size,
    "r": save_top_down_map
}

print("\nManual Robot Controls")
print("Controls: W = North, S = South, D = East, A = West, ")
print("Q = Turn Clockwise, E = Turn Anticlockwise, F = Move Foward, R = Take Screenshot")
print("Type a letter and press Enter. M = Quit.")

try:
    while True:
        key = input("\nDirection> ").strip().lower()

        if key == "m":
            break

        action = controls.get(key)

        if action is None:
            print("Invalid input. Use W, A, S, D, E, Q, F, R, or M to quit")
            continue

        try:
            action()
            save_top_down_map()
            print("Finished turning.")
        except TimeoutError as exc:
            print(f"Turn failed: {exc}")

except (KeyboardInterrupt, EOFError):
    print("\nExiting.")

finally:
    cmd.send_json({"type": "wheels", "left": 0.0, "right": 0.0})

# # Wheels: rad/s per wheel, resend at 20 Hz (250 ms watchdog). Turn in place at 1 rad/s for 1.5 s.
# turn_rate = 1.0
# wheel_speed = turn_rate * robot["wheel_base"] / 2 / robot["wheel_radius"]
# for _ in range(30):
#     cmd.send_json({"type": "wheels", "left": -wheel_speed, "right": wheel_speed})
#     time.sleep(0.05)
# robot_state = json.loads(latest(state)[0])
# print("after turning:", robot_state["pose"], robot_state["wheels"])

# # goto 0.3 m straight ahead, then follow the state stream for 3 s.
# pose = robot_state["pose"]
# x = pose["x"] + 0.3 * math.cos(pose["yaw"])
# y = pose["y"] + 0.3 * math.sin(pose["yaw"])
# cmd.send_json({"type": "goto", "id": 1, "x": x, "y": y})
# end = time.monotonic() + 3.0
# while time.monotonic() < end:
#     robot_state = json.loads(latest(state)[0])
# print("goto:", robot_state["goto"], "pose:", robot_state["pose"], "bumper:", robot_state["bumper"])

# # Camera: header, JPEG, uint16 depth (mm).
# header, jpg, raw = latest(camera)
# header = json.loads(header)
# rgb = np.array(Image.open(io.BytesIO(jpg)))
# depth = np.frombuffer(raw, "<u2").reshape(header["height"], header["width"]) * cam["depth_scale"]

# # Back-project the centre pixel: pixel -> camera frame -> robot frame -> world frame.
# u, v = header["width"] // 2, header["height"] // 2
# z = depth[v, u]
# x_camera = (u - cam["cx"]) * z / cam["fx"]
# y_camera = (v - cam["cy"]) * z / cam["fy"]
# p_camera = np.array([x_camera, y_camera, z, 1.0])
# p_robot = np.array(cam["T_base_camera"]) @ p_camera
# pose = header["pose"]
# cos_yaw, sin_yaw = math.cos(pose["yaw"]), math.sin(pose["yaw"])
# x_world = pose["x"] + cos_yaw * p_robot[0] - sin_yaw * p_robot[1]
# y_world = pose["y"] + sin_yaw * p_robot[0] + cos_yaw * p_robot[1]
# p_world = (x_world, y_world, p_robot[2])
# print(f"camera seq {header['seq']}: rgb {rgb.shape}, centre depth {z:.3f} m -> world {np.round(p_world, 3)}")
