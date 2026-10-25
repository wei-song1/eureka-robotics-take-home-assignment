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
    robot, simulator, travelled_path = robot_value, simulator_value, travelled_path_value

def face_direction(yaw):
    wheel_base = robot['wheel_base']
    wheel_radius = robot['wheel_radius']
    deadline = time.monotonic() + 15.0

    try:
        while time.monotonic() < deadline:
            if not simulator.wait_for_state(500):
                raise TimeoutError('No robot state received')
            
            pose = simulator.read_state()['pose']
            error = math.atan2(math.sin(yaw - pose['yaw']), math.cos(yaw - pose['yaw']))

            if abs(error) < math.radians(3):
                return
            
            angular_speed = max(-1.0, min(1.0, 2.0 * error))
            wheel_speed = angular_speed * wheel_base / (2 * wheel_radius)
            simulator.send_command({'type': 'wheels', 'left': -wheel_speed, 'right': wheel_speed})
            time.sleep(0.05)

        raise TimeoutError('Robot did not reach the desired heading')
    
    finally:
        simulator.send_command({'type': 'wheels', 'left': 0.0, 'right': 0.0})

def face_north():
    face_direction(YAW_NORTH)

def face_south():
    face_direction(YAW_SOUTH)

def face_east():
    face_direction(YAW_EAST)

def face_west():
    face_direction(YAW_WEST)

def turn_45d_clockwise():
    pose = simulator.read_state()['pose']
    face_direction(pose['yaw'] - math.pi / 4)

def turn_45d_anticlockwise():
    pose = simulator.read_state()['pose']
    face_direction(pose['yaw'] + math.pi / 4)

def move_forward_one_robot_size(timeout=15):
    if not simulator.wait_for_state(500):
        raise TimeoutError('No robot state received')
    
    pose = simulator.read_state()['pose']
    distance = 0.34
    target_x = pose['x'] + distance * math.cos(pose['yaw'])
    target_y = pose['y'] + distance * math.sin(pose['yaw'])
    command_id = time.time_ns()
    simulator.send_command({'type': 'goto', 'id': command_id, 'x': target_x, 'y': target_y})
    deadline = time.monotonic() + timeout

    try:
        while time.monotonic() < deadline:
            if not simulator.wait_for_state(500):
                raise TimeoutError('No robot state received')
            
            pose = simulator.read_state()['pose']
            travelled_path.append((pose['x'], pose['y']))
            distance_remaining = math.hypot(target_x - pose['x'], target_y - pose['y'])

            if distance_remaining <= 0.03:
                return command_id
            
        raise TimeoutError('Robot did not reach the target')
    
    finally:
        simulator.send_command({'type': 'wheels', 'left': 0.0, 'right': 0.0})

def read_navigation_state():
    if not simulator.wait_for_state(1000):
        raise TimeoutError('No robot state received for one second')
    
    message = simulator.read_state()
    travelled_path.append((message['pose']['x'], message['pose']['y']))
    return message

def drive_to_waypoint(target, planning_map, timeout=30):
    message = read_navigation_state()
    pose = message['pose']

    if not planning_map.segment_is_free((pose['x'], pose['y']), target):
        raise ValueError('Robot drifted outside the checked route; stopping')
    
    command_id = time.time_ns()
    simulator.send_command({'type': 'goto', 'id': command_id, 'x': target[0], 'y': target[1]})
    deadline = time.monotonic() + timeout

    try:
        while time.monotonic() < deadline:
            message = read_navigation_state()
            pose = message['pose']
            progress = message.get('goto') or {}

            if message.get('bumper') or (progress.get('id') == command_id and progress.get('status') == 'bumped'):
                raise RuntimeError('Collision: the real world may differ from the map')
            
            if not planning_map.is_free(planning_map.world_to_cell(pose['x'], pose['y'])):
                raise RuntimeError('Robot entered the hard robot-radius exclusion zone')
            
            if progress.get('id') != command_id:
                continue

            status = progress.get('status')

            if status == 'reached':
                return
            
            if status in ('cancelled', 'preempted'):
                raise RuntimeError(f'Navigation command was {status}')
            
        raise TimeoutError('Robot did not finish its waypoint')
    finally:
        simulator.stop_robot()
