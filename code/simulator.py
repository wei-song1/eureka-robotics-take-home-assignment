import argparse
import json
import time

import zmq


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
    msg = sock.recv_multipart()
    while sock.poll(0):
        msg = sock.recv_multipart()
    return msg


def read_state(timeout_ms=500):
    if not state.poll(timeout_ms):
        raise TimeoutError("No robot state received")

    return json.loads(latest(state)[0])


def set_wheel_speeds(left, right):
    cmd.send_json({
        "type": "wheels",
        "left": left,
        "right": right,
    })


def goto(x, y):
    command_id = time.time_ns()
    cmd.send_json({
        "type": "goto",
        "id": command_id,
        "x": x,
        "y": y,
    })
    return command_id


def stop_robot():
    set_wheel_speeds(0.0, 0.0)


def ping():
    '''
    Returns {"t": float}
    '''
    result, _ = call("ping")
    return result


def list_scenes():
    '''
    Returns [{"id": "01", "name": str, "difficulty": "easy"|"advanced"}]
    '''
    scenes, _ = call("list_scenes")
    return scenes


def load_scene(scene_id, seed=None):
    '''
    Returns {"id", "start": {"x", "y", "yaw"}, "goals": [...]}; start is exact
    '''
    params = {"id": scene_id}
    if seed is not None:
        params["seed"] = seed

    scene, _ = call("load_scene", **params)
    return scene


def reset(seed=None):
    '''
    Same as load_scene, current scene
    '''
    params = {}
    if seed is not None:
        params["seed"] = seed

    scene, _ = call("reset", **params)
    return scene


def get_goals():
    '''
    Returns [{"name": str, "pose": [x, y, yaw?]}]
    '''
    goals, _ = call("get_goals")
    return goals


def get_map():
    '''
    Returns
        Frame 0: {"ok": true, "result": {"resolution": 0.02, "origin": [x, y], "width": W, "height": H}} 
        Frame 1: PNG, 8-bit grey, H rows × W columns: 0 = free, 255 = occupied.
    '''
    header, frames = call("get_map")
    return header, frames


def get_robot_info():
    '''
    Returns dimensions, limits, ticks_per_rev, watchdog, rates
    '''
    info, _ = call("get_robot_info")
    return info


def get_camera_info():
    '''
    Returns {"width", "height", "fx", "fy", "cx", "cy", "depth_scale", "max_depth", "rate", "T_base_camera": 4×4}
    '''
    info, _ = call("get_camera_info")
    return info


def mark_goal(name):
    '''
    Returns {"t": float}: simulation time of the mark
    '''
    result, _ = call("mark_goal", name=name)
    return result
