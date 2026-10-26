# Submission for Eureka Robotics

This project is a Python application submission for controlling and visualising a simulated differential-drive robot vacuum. It communicates with the simulator exclusively through its ZeroMQ network interface.

The application has four main responsibilities:

1. Connect to the simulator and expose a small Python client API.
2. Plan and execute paths to named goals.
3. Stream and record RGB-D camera data.
4. Provide a Qt visualiser showing the camera feeds, robot state, maps, planned routes, and navigation output.

The main entry point is [`code/app.py`](code/app.py). It launches the Qt visualiser with the navigation terminal embedded in the window. For more information, please see [Architecture Design](docs/design.md)

## Tools Used

- Python
- Visual Studio Code
- ChatGPT-6.1 Sol Medium ()

# Screen Recording

# Example Camera Recording

# How to build, run and test it

```bash
docker pull ghcr.io/eurekarobotics/vacuum-sim:1.0.0
docker run --rm -p 5555-5558:5555-5558 -p 8080:8080 ghcr.io/eurekarobotics/vacuum-sim:1.0.0 --scene 01 --viewer
```

Open http://localhost:8080 to watch the simulated world, then run one of the example clients.

# Documentation

For more details, see:
- For architecture, key designs, trade-offs, limitations and more, see [Architecture Design](docs/design.md)
- For assignment brief, see [Assignment Brief](docs/assignment-brief.md)