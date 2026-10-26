# Architecture

## Project Files Outline
```
├── code                        # All code is stored here
│   ├── app.py                  # Application entry point, launches the Qt viewer and navigator
│   ├── camera_stream.py        # Receives and validates RGBD camera frames
│   ├── camera_viewer.py        # Qt interface for camera feeds, maps, status, and controls
│   ├── map_renderer.py         # Renders maps, planned paths, goals, and robot trails
│   ├── movement.py             # Provides manual motion, waypoint control, and obstacle checks
│   ├── navigator_runner.py     # Runs the navigator as a UI-connected subprocess
│   ├── navigator.py            # Coordinates planning, navigation, replanning, and goal marking
│   ├── obstacle_mapping.py     # Detects unseen obstacles from depth data and updates the planning map
│   ├── planner.py              # Builds clearance maps and computes A* routes and waypoints
│   ├── recorder.py             # Saves RGBD frames, metadata, and RGB video recordings
│   ├── simulator.py            # Wraps ZeroMQ RPC, robot commands, state, and camera sockets
│   └── terminal_bridge.py      # Connects the Qt terminal to the navigator subprocess
├── docs                        # Contains all documentation
│   ├── assignment-brief.md
│   ├── design.md
│   └── todo.md
├── examples                    # Files provided by assignment-brief
│   ├── client.cpp
│   ├── client.py
│   └── CMakeLists.txt
├── recordings                  # Video recordings from RGBD camera
├── planned_map.png             # Image to show what A* Search planning is trying to do
├── top_down_map.png            # Image to show what's on screen and what is currently happening for users
└── README.md
```

# Key Decision

# Trade-offs

## Using `goto` commands instead of direct wheel controls

A* Search's movement was primarily chosen to use the simulator's `goto` commands, while manual control movement was chosen to use wheel commands instead. This is cause using `goto` is a simpler way to move a waypoint and makes it easier to implement A* Search. However, using it also provides less control over velocity profiles and turning behaviour than a dedicated trajectory controller would.

## Extremely conservative clearance versus shorter routes

The obstacles were expanded by the robot's radius in the planning map to prevent collisions, with an additional safety margin to account for potential drift and unpredictable movements during execution of the `goto` command. While this approach improves safety, it increases the cost of traversing nearby routes, potentially resulting in longer paths or rendering some routes inaccessible. This trade-off was intentional, as collision avoidance was prioritised over minimising travel time. If the robot becomes stuck, it may be unable to recover, making reliable navigation more important than maximising speed.

# Limitations

- Using `goto` commands and hoping for the best
    - The robot is controlled by sending waypoint commands and relying on the simulator's `goto` behaviour, rather than continuously tracking a planned trajectory. As a result, the robot's actual movement may differ from the planned path.
- Dependence on map accuracy
    - The planner assumes that the occupancy map reasonably reflects the real environment. It can handle some discrepancies by detecting collisions through the bumper and replanning the route. However, if the map differs significantly from the actual environment, collision-free navigation cannot be guaranteed.
- Limited testing in complex environments
    - Development and testing have primarily focused on simpler scenes. More challenging environments, such as those with narrow passages or complex obstacle arrangements, may reveal weaknesses in the planner or controller that require further improvements.
- Software and recording dependencies
    - The program has only been tested on my personal laptop, and may not work on other devices. Additionally, the user interface requires PySide6, while RGB video recording requires FFmpeg to be installed and accessible through PATH. The application may not support these features if the required dependencies are missing or incorrectly configured.
- Development time

# What I would do with more time

Unfortunately, I had to balance my internship and school commitments and assignments at the same time, so I could only dedicate around two days to this assignment. If I had more time, there are several things I would have done differently:

- Rely less on AI and write more code myself. This would allow me to develop a deeper understanding of the implementation and learn more from the process.
- Spend more time understanding and refining the solution instead of relying heavily on AI to complete the assignment within the limited time.
- Write more of the code manually. The initial navigation program for manual control, terminal commands, and the first draft of the A* search algorithm were the main parts I wrote myself. Most of the remaining implementation was generated with AI, with me reviewing the code, testing it, and ensuring that it worked as intended.
- Further develop my own implementation of A.* My original version had issues with zigzagging paths and continuously sending commands, so I decided to use ChatGPT's implementation instead. With more time, I would have worked on debugging and improving my own version to better understand the algorithm and solve these issues myself, like penalising turning too much.
- Improve the UI design. I would spend more time refining the layout and overall appearance to make the interface cleaner, more intuitive, and more user-friendly.
- Add automated unit tests to ensure that everything is working, creating regression tests that tests scenes/seeds to ensure that updating/changing the A* Search algorithm doesn't result in worse results.
- Add stronger linting rules to ensure that formatting is all the same.

Overall, although I managed to get the assignment working within the limited time available, I believe I could have gained a much deeper understanding of the code and produced a more polished result if I had more time.
