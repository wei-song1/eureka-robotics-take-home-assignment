import math
import sys

import map_renderer
import movement
import simulator
import planner


def choose(prompt, options):
    print(f"\n{prompt}")
    for option in options:
        print(f"- {option}")

    print("")
    while True:
        answer = input("> ").strip()

        for option in options:
            if answer.casefold() == option.casefold():
                return option

        if answer.isdigit() and 1 <= int(answer) <= len(options):
            return options[int(answer) - 1]

        print("Please enter one of the listed options.")


def main():
    scenes = simulator.list_scenes()

    scene_options = {
        f"{s['id']}. {s['name']} ({s['difficulty'].lower()})": s["id"]
        for s in scenes
    }

    selected_scene = choose("Which scene do you want?", list(scene_options))
    scene_id = scene_options[selected_scene]

    print("\nWhat seed?")
    while True:
        try:
            seed = int(input("> ").strip())
            break
        except ValueError:
            print("Please enter a whole number.")

    # Scene has [id, start, goals], start is exact, goals are approximate
    scene = simulator.load_scene(scene_id, seed)

    goal_options = {
        f"{index:02d}. {goal['name'].capitalize()}": goal
        for index, goal in enumerate(scene["goals"], start=1)
    }
    manual_option = f"{len(goal_options) + 1:02d}. Manual"
    goal_options[manual_option] = None

    selected_goal_option = choose("Which goal?", list(goal_options))
    selected_goal = goal_options[selected_goal_option]

    print("\nSelected scene: " + scene_id)
    print("Seleceted seed: ", seed)
    if selected_goal is None:
        print("Selected goal: Manual")
    else:
        print("Selected goal: " + selected_goal["name"])
        print("Selected goal pose:", selected_goal["pose"])

    # Start is coords and yaw of robot
    start = scene["start"]
    robot = simulator.get_robot_info()
    cam = simulator.get_camera_info()
    travelled_path = []

    map_renderer.configure(simulator, travelled_path)
    movement.configure(
        robot,
        simulator,
        travelled_path,
        map_renderer.save_top_down_map,
    )

    movement.face_north()
    map_renderer.save_top_down_map()
    map_renderer.get_planning_map()

    if selected_goal is None:
        controls = {
            "w": movement.face_north,
            "s": movement.face_south,
            "d": movement.face_east,
            "a": movement.face_west,
            "q": movement.turn_45d_anticlockwise,
            "e": movement.turn_45d_clockwise,
            "f": movement.move_forward_one_robot_size,
            "z": map_renderer.get_planning_map,
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

                        simulator.mark_goal(parts[1])
                    else:
                        action = controls.get(key)

                        if action is None:
                            print(
                                "Invalid input. Use W, A, S, D, E, Q, F, "
                                "R <goal>, or M to quit."
                            )
                            continue

                        action()

                    map_renderer.save_top_down_map()
                    print("Finished action.")

                except TimeoutError as exc:
                    print(f"Action failed: {exc}")

        except (KeyboardInterrupt, EOFError):
            print("\nExiting.")

        finally:
            simulator.stop_robot()
            travelled_path.clear()
            simulator.load_scene(scene_id, seed)

    if selected_goal is not None:
        goal_pose = selected_goal["pose"]
        while True:
            map_info, _ = simulator.get_map()
            planning_image = map_renderer.get_planning_map()
            pose = simulator.read_state()["pose"]

            route = planner.plan_route(map_info, planning_image, pose, goal_pose)
            if not route:
                raise RuntimeError("A* could not find a route to the goal.")

            print(route)
            movement.move_route(route, map_info)

            pose = simulator.read_state()["pose"]
            distance_to_goal = math.hypot(
                goal_pose[0] - pose["x"],
                goal_pose[1] - pose["y"],
            )
            if distance_to_goal <= 0.05:
                goal_yaw = goal_pose[2] if len(goal_pose) > 2 else None

                if goal_yaw is not None:
                    movement.face_direction(goal_yaw)

                simulator.mark_goal(selected_goal["name"])
                print("Goal Reached")
                break


if __name__ == "__main__":
    main()
