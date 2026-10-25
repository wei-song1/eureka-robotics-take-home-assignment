"""Earlier navigator behaviour, split into separate modules. Run this file."""
import math
import time
import zmq

import simulator
import movement
import planner
import map_renderer

def save_navigation_snapshot():
    """A failed debug image must not restart a successfully marked goal."""
    try:
        map_renderer.save_top_down_map()
    except (OSError, TimeoutError, zmq.ZMQError) as exc:
        print(f'Could not save map snapshot: {exc}')

def navigate_to_goal(goal_name, max_replans=5):
    """Retry failed navigation with fresh map data and a fresh robot pose.

    max_replans counts additional attempts after the initial attempt. Keep
    the scene loaded, so retries never reset the robot to its starting pose.
    A persistent obstacle/no-path condition is bounded rather than retried
    forever. Ctrl+C propagates immediately to the menu's interrupt handler.
    """
    if max_replans < 0:
        raise ValueError('max_replans must be nonnegative')
    deadline = time.monotonic() + 600
    for attempt in range(max_replans + 1):
        try:
            _navigate_to_goal_attempt(goal_name, deadline)
            return
        except (ValueError, RuntimeError, TimeoutError) as exc:
            simulator.stop_robot()
            if attempt == max_replans or time.monotonic() >= deadline:
                raise RuntimeError(f'Navigation could not finish after {attempt + 1} attempts: {exc}') from exc
            print(f'Navigation attempt {attempt + 1} failed: {exc}')
            print(f'Replanning from the current position ({attempt + 1}/{max_replans} retries)...')

def _navigate_to_goal_attempt(goal_name, deadline):
    goals, _ = simulator.call('get_goals')
    goal = next((g for g in goals if g['name'] == goal_name), None)
    if goal is None:
        raise ValueError(f'Unknown goal: {goal_name}')
    target = tuple(goal['pose'][:2])
    target_yaw = goal['pose'][2] if len(goal['pose']) > 2 else None
    planning_map = planner.get_planning_map()
    print(f'\nPlanning to {goal_name}: {target}; grid {planning_map.resolution:.3f} m/cell, robot diameter {2 * planner.ROBOT_RADIUS:.2f} m, safety margin {planner.SAFETY_MARGIN:.2f} m')
    attempts_near_goal = 0
    try:
        for step in range(300):
            if time.monotonic() >= deadline:
                raise TimeoutError('Navigation exceeded ten minutes')
            pose = movement.read_navigation_state()['pose']
            current = (pose['x'], pose['y'])
            distance = math.dist(current, target)
            if distance <= 0.04:
                if target_yaw is not None:
                    movement.face_direction(target_yaw)
                pose = movement.read_navigation_state()['pose']
                error = 0 if target_yaw is None else abs(math.atan2(math.sin(target_yaw - pose['yaw']), math.cos(target_yaw - pose['yaw'])))
                if math.dist((pose['x'], pose['y']), target) <= 0.04 and error <= math.radians(4):
                    simulator.mark_goal(goal_name)
                    print('\nGoal reached and marked.')
                    save_navigation_snapshot()
                    return
                current = (pose['x'], pose['y'])
                distance = math.dist(current, target)
            points = planner.plan_waypoints(planning_map, current, target)
            try:
                map_renderer.save_planned_map(planning_map, points)
            except OSError as exc:
                print(f'Could not save planned map: {exc}')
            waypoint = points[1]
            length = math.dist(current, waypoint)
            if length > 0.5:
                waypoint = tuple((a + (b - a) * 0.5 / length for a, b in zip(current, waypoint)))
            if distance < 0.1:
                attempts_near_goal += 1
                if attempts_near_goal > 5:
                    raise RuntimeError('Could not settle within goal tolerance; goal NOT marked')
            print(f'\nWaypoint {step + 1}: ({waypoint[0]:.2f}, {waypoint[1]:.2f}); goal distance {distance:.2f} m')
            movement.drive_to_waypoint(waypoint, planning_map)
            save_navigation_snapshot()
        raise RuntimeError('Waypoint limit exceeded; goal NOT marked')
    finally:
        simulator.stop_robot()

def choose(prompt, options):
    """Accept an option's name or its number."""
    print(f'\n{prompt}')
    for number, option in enumerate(options, start=1):
        print(f'{number}. {option}')
    while True:
        answer = input('> ').strip()
        for option in options:
            if answer.casefold() == option.casefold():
                return option
        if answer.isdigit() and 1 <= int(answer) <= len(options):
            return options[int(answer) - 1]
        print('Please enter one of the listed options.')

def main():
    global robot, cam, travelled_path
    simulator.initialise_client()
    scenes, _ = simulator.call('list_scenes')
    scene_options = {f"{s['id']} ({s['difficulty'].lower()})": s['id'] for s in scenes}
    selected_scene = choose('Which scene do you want?', list(scene_options))
    scene_id = scene_options[selected_scene]
    print('\nWhat seed?')
    while True:
        try:
            seed = int(input('> ').strip())
            break
        except ValueError:
            print('Please enter a whole number.')
    scene, _ = simulator.call('load_scene', id=scene_id, seed=seed)
    goal_names = [g['name'] for g in scene['goals']]
    selected_goal = choose('Which goal?', goal_names + ['Manual'])
    start = scene['start']
    robot, _ = simulator.call('get_robot_info')
    cam, _ = simulator.call('get_camera_info')
    travelled_path = []
    movement.configure(robot, simulator, travelled_path)
    map_renderer.configure(simulator, travelled_path)
    planner.configure(simulator)
    while selected_goal != 'Manual':
        if selected_goal == 'Quit':
            return
        try:
            navigate_to_goal(selected_goal)
            return
        except (ValueError, RuntimeError, TimeoutError, zmq.ZMQError) as exc:
            print(f'Navigation failed: {exc}')
        except KeyboardInterrupt:
            print('\nNavigation interrupted.')
        finally:
            simulator.stop_robot()
        selected_goal = choose('What next?', goal_names + ['Manual', 'Quit'])
    map_renderer.save_top_down_map()
    controls = {'w': movement.face_north, 's': movement.face_south, 'd': movement.face_east, 'a': movement.face_west, 'q': movement.turn_45d_anticlockwise, 'e': movement.turn_45d_clockwise, 'f': movement.move_forward_one_robot_size}
    print('\nManual Robot Controls')
    print('W = North, S = South, D = East, A = West')
    print('Q = Turn Anticlockwise, E = Turn Clockwise')
    print('F = Move Forward, R <goal> = Mark Goal')
    print('Type a letter and press Enter. M = Quit.')
    try:
        while True:
            key = input('\nDirection> ').strip().lower()
            if key == 'm':
                break
            parts = key.split(maxsplit=1)
            try:
                if parts and parts[0] == 'r':
                    if len(parts) < 2:
                        print('Usage: r <goal_name>, e.g. r kitchen')
                        continue
                    simulator.mark_goal(parts[1])
                else:
                    action = controls.get(key)
                    if action is None:
                        print('Invalid input. Use W, A, S, D, E, Q, F, R <goal>, or M to quit.')
                        continue
                    action()
                map_renderer.save_top_down_map()
                print('Finished action.')
            except TimeoutError as exc:
                print(f'Action failed: {exc}')
    except (KeyboardInterrupt, EOFError):
        print('\nExiting.')
    finally:
        simulator.send_command({'type': 'wheels', 'left': 0.0, 'right': 0.0})
        travelled_path.clear()
        scene, _ = simulator.call('load_scene', id=scene_id, seed=seed)

if __name__ == "__main__":
    main()
