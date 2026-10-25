import io
import json
import threading
import time
from contextlib import contextmanager
from types import SimpleNamespace
import zmq
import math
import numpy as np
from PIL import Image, ImageDraw

simulator = None
travelled_path = None
_image_save_lock = threading.Lock()

def configure(simulator_value, travelled_path_value):
    global simulator, travelled_path
    simulator, travelled_path = simulator_value, travelled_path_value

def save_top_down_map(filename='top_down_map.png', debug=False, *, client=None):
    client = simulator if client is None else client
    map_info, frames = client.call('get_map')
    goals, _ = client.call('get_goals')
    robot_state = client.read_state()
    pose = robot_state['pose']
    travelled_path.append((pose['x'], pose['y']))
    resolution = map_info['resolution']
    origin_x, origin_y = map_info['origin']
    height = map_info['height']

    def world_to_pixel(x, y):
        pixel_x = (x - origin_x) / resolution - 0.5
        pixel_y = height - 0.5 - (y - origin_y) / resolution
        return (pixel_x, pixel_y)
    
    image = Image.open(io.BytesIO(frames[0])).convert('RGB')
    draw = ImageDraw.Draw(image)
    path_pixels = [world_to_pixel(x, y) for x, y in travelled_path]

    if len(path_pixels) >= 2:
        draw.line(path_pixels, fill='blue', width=3)

    for goal in goals:
        goal_pose = goal['pose']
        goal_x, goal_y = goal_pose[:2]
        gx, gy = world_to_pixel(goal_x, goal_y)
        marker_radius = 5
        draw.ellipse((gx - marker_radius, gy - marker_radius, gx + marker_radius, gy + marker_radius), fill='yellow', outline='yellow', width=1)

        if len(goal_pose) > 2 and goal_pose[2] is not None:
            goal_yaw = goal_pose[2]
            heading_length = 0.25
            end_x = goal_x + heading_length * math.cos(goal_yaw)
            end_y = goal_y + heading_length * math.sin(goal_yaw)
            end_px, end_py = world_to_pixel(end_x, end_y)
            draw.line((gx, gy, end_px, end_py), fill='yellow', width=2)

        draw.text((gx + marker_radius + 3, gy - 8), goal['name'], fill='yellow', stroke_width=1, stroke_fill='black')

    pixel_x, pixel_y = world_to_pixel(pose['x'], pose['y'])
    robot_radius = 0.34 / 2
    radius_pixels = robot_radius / resolution
    draw.ellipse((pixel_x - radius_pixels, pixel_y - radius_pixels, pixel_x + radius_pixels, pixel_y + radius_pixels), fill='dodgerblue', outline='blue', width=1)
    yaw = pose['yaw']
    arrow_length = radius_pixels * 1.5
    direction_x = math.cos(yaw)
    direction_y = -math.sin(yaw)
    tip_x = pixel_x + arrow_length * direction_x
    tip_y = pixel_y + arrow_length * direction_y
    draw.line((pixel_x, pixel_y, tip_x, tip_y), fill='red', width=2)
    head_length = radius_pixels * 0.5
    head_half_width = radius_pixels * 0.3
    base_x = tip_x - head_length * direction_x
    base_y = tip_y - head_length * direction_y
    perpendicular_x = -direction_y
    perpendicular_y = direction_x
    draw.polygon([(tip_x, tip_y), (base_x + head_half_width * perpendicular_x, base_y + head_half_width * perpendicular_y), (base_x - head_half_width * perpendicular_x, base_y - head_half_width * perpendicular_y)], fill='red')
    
    with _image_save_lock:
        image.save(filename, format='PNG')
        
    if debug:
        print('\n=== Debug Info ===')
        print(f'Map saved to {filename}')
        print('Map Info:', map_info)
        print(f"Robot: x={pose['x']:.3f} m, y={pose['y']:.3f} m, yaw={math.degrees(yaw):.1f}°")
        print(f'Robot diameter on image: {2 * radius_pixels:.1f} pixels')
        print('Goals:', [goal['name'] for goal in goals])
        print('===================\n')

def save_planned_map(planning_map, waypoints, filename='planned_map.png'):
    """Render a copy of the planning grid; never modify A* occupancy data.

    White = free, grey = hard robot clearance, orange = soft safety band,
    black = actual mapped obstacles,
    blue = planned route, green = current start, red = goal.
    """
    pixels = np.full((planning_map.height, planning_map.width, 3), 255, dtype=np.uint8)
    pixels[planning_map.blocked] = (190, 190, 190)
    pixels[planning_map.safety] = (255, 190, 80)
    pixels[planning_map.occupied] = (0, 0, 0)
    image = Image.fromarray(pixels)
    draw = ImageDraw.Draw(image)

    def world_to_pixel(point):
        return ((point[0] - planning_map.origin[0]) / planning_map.resolution - 0.5, planning_map.height - 0.5 - (point[1] - planning_map.origin[1]) / planning_map.resolution)
    path_pixels = [world_to_pixel(point) for point in waypoints]
    if len(path_pixels) >= 2:
        draw.line(path_pixels, fill='blue', width=2)
    if path_pixels:
        for point, colour in ((path_pixels[0], 'green'), (path_pixels[-1], 'red')):
            x, y = point
            draw.ellipse((x - 3, y - 3, x + 3, y + 3), fill=colour)
    image.save(filename, format='PNG')
    print(f'A* plan saved to {filename}')


@contextmanager
def periodic_screenshots():
    """Refresh top_down_map.png once per second until this scope exits."""
    stop_event = threading.Event()

    def take_screenshots():
        # Each thread owns its sockets; navigation keeps using its own.
        with simulator.ctx.socket(zmq.REQ) as rpc, simulator.ctx.socket(zmq.SUB) as state:
            rpc.linger = state.linger = 0
            rpc.rcvtimeo = rpc.sndtimeo = 1000
            rpc.setsockopt(zmq.REQ_RELAXED, 1)
            rpc.setsockopt(zmq.REQ_CORRELATE, 1)
            rpc.connect(simulator.url(0))
            state.setsockopt(zmq.SUBSCRIBE, b'')
            state.connect(simulator.url(2))

            def call(method, **params):
                rpc.send_json({'method': method, 'params': params})
                frames = rpc.recv_multipart()
                reply = json.loads(frames[0])
                if not reply['ok']:
                    raise RuntimeError(reply['error'])
                return reply['result'], frames[1:]

            def read_state():
                if not state.poll(500):
                    raise TimeoutError('No robot state received')
                return json.loads(simulator.latest(state)[0])

            client = SimpleNamespace(call=call, read_state=read_state)
            next_capture = time.monotonic()
            while not stop_event.is_set():
                try:
                    save_top_down_map('top_down_map.png', client=client)
                except (OSError, RuntimeError, TimeoutError, zmq.ZMQError) as exc:
                    print(f'Could not save map snapshot: {exc}')

                next_capture += 1.0
                now = time.monotonic()
                if next_capture < now:
                    # Skip missed ticks instead of taking a burst of images.
                    next_capture += math.ceil(now - next_capture)
                stop_event.wait(max(0.0, next_capture - time.monotonic()))

    thread = threading.Thread(target=take_screenshots, daemon=True)
    thread.start()

    def stop_screenshots():
        stop_event.set()
        thread.join()

    try:
        yield stop_screenshots
    finally:
        stop_screenshots()
