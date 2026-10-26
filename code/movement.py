import math
import time

YAW_NORTH = math.pi / 2
YAW_SOUTH = -math.pi / 2
YAW_EAST = 0
YAW_WEST = math.pi
robot = None
simulator = None
travelled_path = None


class RouteBlocked(RuntimeError):
    def __init__(self, reason, message, approach, collided=False):
        super().__init__(reason)
        self.message = message
        self.approach = approach
        self.collided = collided

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
            # Wrap the error so the robot takes the shortest turn across +/- pi.
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

def drive_to_waypoint(target, planning_map, timeout=8, obstacle_map=None):
    message = read_navigation_state()
    pose = message['pose']
    approach = (pose['x'], pose['y'])
    if obstacle_map is not None:
        obstacle_map.update(message)

    if not planning_map.segment_is_free((pose['x'], pose['y']), target):
        if obstacle_map is not None:
            raise RouteBlocked('Depth camera changed the checked route', message,
                               obstacle_map.approach or approach)
        raise ValueError('Robot drifted outside the checked route; stopping')

    if obstacle_map is not None:
        obstacle_map.approach = approach
    
    command_id = time.time_ns()
    simulator.send_command({'type': 'goto', 'id': command_id, 'x': target[0], 'y': target[1]})
    deadline = time.monotonic() + timeout

    try:
        while time.monotonic() < deadline:
            message = read_navigation_state()
            pose = message['pose']
            progress = message.get('goto') or {}

            if message.get('bumper') or (progress.get('id') == command_id and progress.get('status') == 'bumped'):
                raise RouteBlocked('Collision: the real world may differ from the map', message, approach, collided=True)

            if obstacle_map is not None:
                obstacle_map.update(message)
                if not planning_map.segment_is_free((pose['x'], pose['y']), target):
                    raise RouteBlocked('Depth camera found an obstacle on the route', message, approach)
            
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


def retreat_from_obstacle(blocked, obstacle_map, deadline=None):
    """Reverse slowly over the approach just travelled, checking clearance.

    The bumper remains latched until contact is released. Never turn its
    initial True into a second collision, or bypass normal A* exclusions.
    """
    grid = obstacle_map.planning_map
    message = read_navigation_state()
    pose = message['pose']
    start = (pose['x'], pose['y'])
    direction = (math.cos(pose['yaw']), math.sin(pose['yaw']))
    delta = tuple(a - b for a, b in zip(start, blocked.approach))
    distance_travelled = sum(a * b for a, b in zip(delta, direction))
    lateral = abs(delta[0] * direction[1] - delta[1] * direction[0])
    distance = min(0.18, distance_travelled - 0.01)
    if distance < 0.04 or lateral > 0.06:
        raise RuntimeError('Cannot retreat safely: no verified approach behind the robot')
    end = tuple(a - distance * b for a, b in zip(start, direction))
    if not grid.retreat_is_safe(start, end):
        raise RuntimeError('Cannot retreat safely: clearance behind the robot is insufficient')
    timeout = time.monotonic() + 4.0
    if deadline is not None:
        timeout = min(timeout, deadline)
    print(f'[navigation] Reversing up to {distance:.2f} m along the previous approach')
    speed = -0.06 / robot['wheel_radius']
    bumper_released = not message.get('bumper')
    try:
        while time.monotonic() < timeout:
            message = read_navigation_state()
            obstacle_map.update(message)
            pose = message['pose']
            if bumper_released and message.get('bumper'):
                raise RuntimeError('New bumper contact during retreat')
            bumper_released = bumper_released or not message.get('bumper')
            current = (pose['x'], pose['y'])
            backed = sum((a - b) * d for a, b, d in zip(start, current, direction))
            if backed < -0.03:
                raise RuntimeError('Robot moved toward the obstacle during retreat')
            if backed >= distance - 0.01:
                if not message.get('bumper') and grid.is_free(grid.world_to_cell(*current)):
                    return
                raise RuntimeError('Retreat ended without clearing the obstacle')
            yaw_error = abs(math.atan2(math.sin(pose['yaw'] - math.atan2(direction[1], direction[0])),
                                       math.cos(pose['yaw'] - math.atan2(direction[1], direction[0]))))
            lateral = abs((current[0] - start[0]) * direction[1] - (current[1] - start[1]) * direction[0])
            if yaw_error > math.radians(15) or lateral > 0.04:
                raise RuntimeError('Retreat drifted away from the verified approach')
            next_position = tuple(a - min(0.015, distance - backed) * b
                                  for a, b in zip(current, direction))
            if not grid.retreat_is_safe(current, next_position):
                raise RuntimeError('Retreat blocked by newly observed occupancy')
            simulator.set_wheel_speeds(speed, speed)
            time.sleep(0.05)
        raise TimeoutError('Robot did not clear the obstacle during retreat')
    finally:
        simulator.stop_robot()
