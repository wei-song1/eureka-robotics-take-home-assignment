import math
import time
from pathlib import Path
import zmq

import simulator
import movement
import planner
import map_renderer
import camera_stream
import obstacle_mapping

def save_navigation_snapshot():
    try:
        map_renderer.save_top_down_map()

    except (OSError, TimeoutError, zmq.ZMQError) as exc:
        print(f'Could not save map snapshot: {exc}')

def navigate_to_goal(goal_name, max_replans=5):
    if max_replans < 0:
        raise ValueError('max_replans must be nonnegative')
    
    deadline = time.monotonic() + 600
    with obstacle_mapping.NavigationMap(simulator, cam, robot) as obstacle_map:
        map_renderer.set_obstacle_map(obstacle_map)
        for attempt in range(max_replans + 1):
            try:
                _navigate_to_goal_attempt(goal_name, deadline, obstacle_map)
                return

            except (ValueError, RuntimeError, TimeoutError) as exc:
                simulator.stop_robot()
                if isinstance(exc, movement.RouteBlocked):
                    if exc.collided:
                        obstacle_map.record_collision(exc)
                    pose = exc.message['pose']
                    if exc.collided or not obstacle_map.planning_map.is_free(
                            obstacle_map.planning_map.world_to_cell(pose['x'], pose['y'])):
                        movement.retreat_from_obstacle(exc, obstacle_map, deadline)
                    save_navigation_snapshot()
                if attempt == max_replans or time.monotonic() >= deadline:
                    raise RuntimeError(f'Navigation could not finish after {attempt + 1} attempts: {exc}') from exc

                print(f'Navigation attempt {attempt + 1} failed: {exc}')
                print(f'Replanning from the current position ({attempt + 1}/{max_replans} retries)...')

def _navigate_to_goal_attempt(goal_name, deadline, obstacle_map=None):
    goals, _ = simulator.call('get_goals')
    goal = next((g for g in goals if g['name'] == goal_name), None)
    if goal is None:
        raise ValueError(f'Unknown goal: {goal_name}')
    
    target = tuple(goal['pose'][:2])
    target_yaw = goal['pose'][2] if len(goal['pose']) > 2 else None
    planning_map = planner.get_planning_map() if obstacle_map is None else obstacle_map.planning_map
    print(f'\nPlanning to {goal_name}: {target}; grid {planning_map.resolution:.3f} m/cell, robot diameter {2 * planner.ROBOT_RADIUS:.2f} m, safety margin {planner.SAFETY_MARGIN:.2f} m')
    
    attempts_near_goal = 0
    try:
        for step in range(300):
            if time.monotonic() >= deadline:
                raise TimeoutError('Navigation exceeded ten minutes')
            
            message = movement.read_navigation_state()
            if obstacle_map is not None:
                obstacle_map.update(message)
            pose = message['pose']
            if obstacle_map is not None and not planning_map.is_free(
                    planning_map.world_to_cell(pose['x'], pose['y'])):
                raise movement.RouteBlocked('New obstacle clearance overlaps the robot',
                                            message, obstacle_map.approach or (pose['x'], pose['y']))
            current = (pose['x'], pose['y'])
            distance = math.dist(current, target)

            if distance <= 0.04:
                if target_yaw is not None:
                    movement.face_direction(target_yaw)

                pose = movement.read_navigation_state()['pose']
                error = 0 if target_yaw is None else abs(math.atan2(math.sin(target_yaw - pose['yaw']), math.cos(target_yaw - pose['yaw'])))

                if math.dist((pose['x'], pose['y']), target) <= 0.04 and error <= math.radians(4):
                    simulator.mark_goal(goal_name)
                    camera_stream.goal_marked(goal_name)
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
            try:
                movement.drive_to_waypoint(waypoint, planning_map, obstacle_map=obstacle_map)
            except movement.RouteBlocked as exc:
                if exc.collided or not planning_map.is_free(planning_map.world_to_cell(
                        exc.message['pose']['x'], exc.message['pose']['y'])):
                    raise
                print(f'[navigation] {exc}; replanning before contact')
            save_navigation_snapshot()

        raise RuntimeError('Waypoint limit exceeded; goal NOT marked')
    
    finally:
        simulator.stop_robot()

def choose(prompt, options, numbered=True):
    print(f'\n{prompt}')
    for number, option in enumerate(options, start=1):
        print(f'{number}. {option}' if numbered else option)

    while True:
        answer = input('> ').strip()
        for option in options:
            if answer.casefold() == option.casefold():
                return option
            
        if answer.isdigit() and 1 <= int(answer) <= len(options):
            return options[int(answer) - 1]
        
        print('Please enter one of the listed options.')

def run_session():
    global robot, cam, travelled_path
    scenes, _ = simulator.call('list_scenes')
    scene_options = {
        f"{s['id']}. {s['name'].title()} ({s['difficulty'].title()})": s['id']
        for s in scenes
    }

    selected_scene = choose('Which scene do you want?', list(scene_options), numbered=False)
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
    for filename in ('top_down_map.png', 'planned_map.png'):
        try:
            Path(filename).unlink(missing_ok=True)
        except OSError as exc:
            print(f'Could not clear old map {filename}: {exc}')
    robot, _ = simulator.call('get_robot_info')
    cam, _ = simulator.call('get_camera_info')
    travelled_path = []

    movement.configure(robot, simulator, travelled_path)
    map_renderer.configure(simulator, travelled_path)
    planner.configure(simulator)

    with camera_stream.camera_session(simulator, cam, scene_id=scene_id, seed=seed, scene_info=next(s for s in scenes if s['id'] == scene_id), target_goal=selected_goal, keep_ui_open=True) as camera_session, map_renderer.periodic_screenshots() as stop_screenshots:
        if selected_goal != 'Manual':
            # End navigation process
            if selected_goal == 'Quit':
                camera_session.request_close()
                return

            # A* Search
            camera_session.set_target(selected_goal)
            try:
                navigate_to_goal(selected_goal)

            except (ValueError, RuntimeError, TimeoutError, zmq.ZMQError) as exc:
                print(f'Navigation failed: {exc}')

            except KeyboardInterrupt:
                print('\nNavigation interrupted.')
                camera_session.request_close()
                return

            finally:
                simulator.stop_robot()

            return scene_id, seed, camera_session

        # Manual Process    
        camera_session.set_target(None)
        map_renderer.save_top_down_map()
        controls = {'w': movement.face_north, 's': movement.face_south, 'd': movement.face_east, 'a': movement.face_west, 'q': movement.turn_45d_anticlockwise, 'e': movement.turn_45d_clockwise, 'f': movement.move_forward_one_robot_size}
        print('\nManual Robot Controls')
        print('W = North, S = South, D = East, A = West')
        print('Q = Turn Anticlockwise, E = Turn Clockwise')
        print('F = Move Forward, R <goal> = Mark Goal')
        print('Type a letter and press Enter. M, quit, or exit = Quit.')

        try:
            while True:
                key = input('\nDirection> ').strip().lower()
                if key in ('m', 'quit', 'exit'):
                    camera_session.request_close()
                    return

                parts = key.split(maxsplit=1)
                try:
                    if parts and parts[0] == 'r':
                        if len(parts) < 2:
                            print('Usage: r <goal_name>, e.g. r kitchen')
                            continue

                        simulator.mark_goal(parts[1])
                        camera_stream.goal_marked(parts[1])
                        save_navigation_snapshot()
                        return scene_id, seed, camera_session

                    else:
                        action = controls.get(key)
                        if action is None:
                            print('Invalid input. Use W, A, S, D, E, Q, F, R <goal>, or M to quit.')
                            continue

                        action()

                    map_renderer.save_top_down_map()
                    print('Finished action.')

                except (ValueError, RuntimeError, TimeoutError, zmq.ZMQError) as exc:
                    print(f'Action failed: {exc}')
                    return scene_id, seed, camera_session

        except KeyboardInterrupt:
            print('\nExiting.')
            camera_session.request_close()

        except EOFError:
            camera_session.wait_for_close(allow_terminal=False)

        finally:
            simulator.send_command({'type': 'wheels', 'left': 0.0, 'right': 0.0})
            stop_screenshots()


def main():
    simulator.initialise_client()
    while True:
        result = run_session()
        if result is None:
            return

        # Restart Process
        scene_id, seed, camera_session = result
        simulator.stop_robot()
        camera_session.freeze_preview()
        try:
            simulator.call('reset', seed=seed)
        except (ValueError, RuntimeError, TimeoutError, zmq.ZMQError) as exc:
            print(f'Could not reset scene {scene_id}: {exc}')
        print('\nReady for a new run. Choose a scene, seed and goal again.')

if __name__ == "__main__":
    import os
    if os.environ.get('VACUUM_TERMINAL_MODE') == 'console' or os.environ.get('VACUUM_CAMERA_PREVIEW') == '0':
        main()
    else:
        from camera_viewer import main as ui_main
        raise SystemExit(ui_main(navigate=True))
