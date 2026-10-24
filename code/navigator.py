import sys

import map_renderer
import movement
import simulator


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

    goal_names = [g["name"].capitalize() for g in scene["goals"]]
    goal_options = {
        f"{index:02d}. {goal_name}": goal_name
        for index, goal_name in enumerate(goal_names, start=1)
    }
    manual_option = f"{len(goal_options) + 1:02d}. Manual"
    goal_options[manual_option] = "Manual"

    selected_goal = choose("Which goal?", list(goal_options))
    selected_goal = goal_options[selected_goal]

    if selected_goal != "Manual":
        sys.exit(0)

    # Start is coords and yaw of robot
    start = scene["start"]
    robot = simulator.get_robot_info()
    cam = simulator.get_camera_info()
    travelled_path = []

    movement.configure(robot, simulator, travelled_path)
    map_renderer.configure(simulator, travelled_path)

    movement.face_north()
    map_renderer.save_top_down_map()

    controls = {
        "w": movement.face_north,
        "s": movement.face_south,
        "d": movement.face_east,
        "a": movement.face_west,
        "q": movement.turn_45d_anticlockwise,
        "e": movement.turn_45d_clockwise,
        "f": movement.move_forward_one_robot_size,
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


if __name__ == "__main__":
    main()
