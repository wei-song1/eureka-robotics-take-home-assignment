# Submission for Eureka Robotics

This project is a submission for Eureka Robotics' take-home assignment. This is a Python application for controlling and visualising a simulated differential-drive robot vacuum. It communicates with the simulator exclusively through its ZeroMQ network interface.

The application has four main responsibilities:

1. Connect to the simulator and expose a small Python client API.
2. Plan and execute paths to named goals using A* Search.
3. Stream and record RGBD camera data.
4. Provide a Qt visualiser showing the camera feeds, robot state, maps, planned routes, and navigation output.

The main entry point is [`code/app.py`](code/app.py). It launches the Qt visualiser with the navigation terminal embedded in the window.

## Tools Used

- Python
- Visual Studio Code
- ChatGPT-6.1 Sol Medium/High (https://chatgpt.com/share/e/6adf5331-d064-83ec-b0a0-deb13cf37a6a)

Python dependencies are listed in `requirements-camera.txt`:

| Dependency | Purpose |
| --- | --- |
| NumPy | Occupancy grids, geometry and depth arrays |
| Pillow | Image decoding, maps, depth PNGs and video overlays |
| pyzmq | Simulator RPC, commands and sensor streams |
| PySide6 | Qt interface |

## Submission Results

Navigation results using seed `1`:

| Level | Scene | Difficulty | Goals |
|---|---|---|---|
| 01 | Studio Apartment | Easy | 4/4 |
| 02 | Two-Room Flat With Office | Easy | 4/4 |
| 03 | Family Home | Easy | 5/5 |
| 04 | Loft With Shelving Divider | Advanced | 3/3 |
| 05 | Apartment With Hallway | Advanced | 2/5 |
| 06 | L-Shaped Family House | Advanced | 2/6 |

# Screen Recording

[![Screen Recording](https://img.youtube.com/vi/Pq1ceCspPrQ/hqdefault.jpg)](https://youtu.be/Pq1ceCspPrQ)

# Example Camera Recording

[![Watch the video](https://img.youtube.com/vi/sut1CBWdPtc/hqdefault.jpg)](https://youtu.be/sut1CBWdPtc)

# How to build, run and test it

There is no building or compilation step.

1. First, create and activate a virtual environment on Linux/macOS:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements-camera.txt
```

On Windows PowerShell:

```powershell
py -m venv .venv
.venv\Scripts\Activate.ps1
python -m pip install -r requirements-camera.txt
```

Install FFmpeg separately, for example with `sudo apt install ffmpeg` on
Ubuntu/Debian or `brew install ffmpeg` on macOS. On Windows, install an FFmpeg
build and add its `bin` directory to `PATH`. Check it in the same terminal:

```bash
ffmpeg -version
```

2. Start the simulator in one terminal:

```bash
docker pull ghcr.io/eurekarobotics/vacuum-sim:1.0.0
docker run --rm -p 5555-5558:5555-5558 -p 8080:8080 ghcr.io/eurekarobotics/vacuum-sim:1.0.0 --scene 01 --viewer
```

3. In another terminal, activate the virtual environment and launch the application:


```bash
python code/app.py
```

A GUI will open with a terminal with 2 camera views and 2 map views. Follow the instructions given in the terminal.

> Alternatively, open http://localhost:8080 to watch the simulated world

# Documentation

For more details, see:
- For architecture, key designs, trade-offs, limitations and more, see [Architecture Design](docs/design.md)
- For assignment brief, see [Assignment Brief](docs/assignment-brief.md)