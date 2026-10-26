# Architecture

## Project Files Outline
```
├── code                        # All code is stored here
│   ├── app.py                  # Application entry point, launches the Qt viewer and navigator
│   ├── camera_display.py       # Formats camera status, depth previews, and video overlays
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
├── requirements-camera.txt     # Python dependencies
└── README.md
```

## System Architecture

The application combines the supplied map with local observations, executes short motion commands and records RGBD data. World units are metres and radians, displayed yaw is in degrees. All robot access uses ZeroMQ.


Viewing, recording and mapping have independent camera subscriptions, each with
a receiver-owned socket/thread. The navigator subprocess keeps terminal/RPC
work outside the Qt event loop. Atomic JSON files exchange calibration, status
and run transitions.

| Modules | Responsibility |
| --- | --- |
| `navigator.py`, `simulator.py` | Selection, run lifecycle, RPC/commands and navigation orchestration |
| `planner.py`, `obstacle_mapping.py` | Grid costs, A*, depth observations and learned occupancy |
| `movement.py` | Waypoint monitoring, encoder checks, heading control and retreat |
| `map_renderer.py` | Annotated map copies and periodic screenshots |
| `camera_stream.py`, `recorder.py` | Camera reception, per-run metadata and asynchronous storage |
| `camera_viewer.py`, `camera_display.py` | Qt presentation and shared video/status formatting |
| `app.py`, `terminal_bridge.py`, `navigator_runner.py` | GUI launch, terminal subprocess transport and shutdown |

# Key Decision

## Planning and execution

The clean `get_map` grid uses native resolution; row zero is the largest world y.
Rendering never modifies planning input. Inflation is the 0.17 m robot radius
plus `sqrt(2) * resolution` for pixel areas. Hard inflation is impassable. A
further 0.17 m safety band costs six times free space but remains traversable,
allowing escape after drift into that band.

A* uses eight neighbours, an octile heuristic and length-weighted mean endpoint
costs, prohibiting diagonal corner cutting. Shortcuts must pass grid traversal
checks without increasing cost. The navigator executes at most 0.5 m per goto,
then replans from a fresh pose. Repeated searches simplify correction but produce
stop/turn motion rather than continuous tracking.

Newly blocked segments stop and replan. Both encoder counters unchanged for
0.35 s also trigger replanning after a 0.5 s startup grace; either wheel moving
resets the check. The fallback timeout is eight seconds. Failures allow ten
retries within 600 s, with 300 steps per attempt and bounded near-goal settling.
Automatic marking requires at most 0.04 m position error and, when specified,
4 degrees yaw error, below the assignment limits.

## Missing obstacles and recovery

Mapping samples alternate depth rows/columns at up to 10 Hz. Intrinsics,
`T_base_camera` and capture-time pose project optical z-depth into the world.
Filtering accepts heights from 0.025 m to robot height and optical depth of
0.05–3 m, excluding invalid/self returns. One-cell support and two observations
within 0.5 simulation seconds confirm occupancy.

Learned cells incrementally update inflation/costs and persist across retries,
resetting between runs. Original occupancy and occluded space are never cleared.
This suits static scenes but can retain false detections. Stale depth,
camera/state timing mismatch or unexpected reset stops navigation.

Contact adds an inferred front arc, assuming forward motion because the bumper
has no bearing. Recovery reverses along an aligned recent approach at 0.06 m/s,
for at most 0.18 m/four seconds, checking clearance, occupancy, depth, drift and
new contacts. This permits escape from newly added inflation while normal A*
still forbids it. Recovery must release contact and exit hard inflation;
otherwise the robot stops. Encoder stops alone never create obstacles.

## Recording and presentation

Recording uses a bounded 90-frame queue; viewing keeps only the latest frame.
A worker feeds two FFmpeg encoders and stores annotated RGB/depth MP4s, original
JPEGs, lossless uint16 depth and a manifest of capture time/pose, sequence,
receipt times, filenames and video indices. Calibration supports offline use.

Shared formatting matches UI/video Info. Submission-time state snapshots and
mark timestamps keep queued captures' status accurate. Depth video is a
visualization; nominal-FPS playback requires manifests for timing analysis.
Resets create new segments. Overflow/encoder failure marks recording incomplete
while navigation can continue. Hard kills can leave unfinished MP4s.

After success/final failure, workers finish, the simulator resets and selection
restarts. Final images remain until the next complete selection. Maps show
supplied occupancy and observations, not ground truth.

## Validation, limitations and further work

There are 91 passing tests, including real video encode/decode and four synthetic
navigation scenarios. Native Qt, real PUB/SUB and supplied-simulator scene
completion remain unverified in the development environment.

There is no SLAM, pose filtering, moving-object tracking or obstacle clearing.
Inflation can reject passable gaps; camera blind spots and approximate contact
bearing limit recovery. Manual marking relies on the operator. Thresholds are
heuristics, not measured optimal settings.

Further work starts with multi-seed scene benchmarks and collision/arrival
metrics, then confidence decay/free-space clearing and better pose estimates.
Continuous tracking could improve smoothness; capture-paced video and recording
throughput benchmarks would improve replay/evaluation. Simulator testing must
precede claims of greater robustness.

## Manual controls

Choose `Manual` instead of a goal, then type a command and press Enter:

| Command | Action |
| --- | --- |
| `W`, `A`, `S`, `D` | Face north, west, south or east |
| `Q`, `E` | Turn 45 degrees anticlockwise or clockwise |
| `F` | Move forward one robot diameter, 0.34 m |
| `R <goal_name>` | Manually call `mark_goal` and finish the run |
| `M`, `quit`, `exit` | End the session |

## AI assistance

OpenAI Codex was used interactively to help implement and revise A* planning,
clearance/safety costs, depth and bumper obstacle learning, movement recovery,
the RGBD recorder, Qt visualiser, embedded terminal, repeat-run behaviour,
automated tests and documentation. The supplied simulator API and example client
were used as the integration reference.


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
