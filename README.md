# Submission for Eureka Robotics

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