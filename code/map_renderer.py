import io
import math

from PIL import Image, ImageDraw

GOAL_COLOUR = "yellow"
TRAIL_COLOUR = "blue"
ROBOT_COLOUR = "green"
ROBOT_POINTER_COLOUR = "lime"
OUTLINE_COLOUR = "black"

simulator = None
travelled_path = None

def configure(simulator_value, travelled_path_value):
    global simulator, travelled_path
    simulator = simulator_value
    travelled_path = travelled_path_value

def save_top_down_map(filename="top_down_map.png", debug=False):
    map_info, frames = simulator.get_map()
    goals = simulator.get_goals()

    robot_state = simulator.read_state()
    pose = robot_state["pose"]

    travelled_path.append((pose["x"], pose["y"]))

    resolution = map_info["resolution"]
    origin_x, origin_y = map_info["origin"]
    height = map_info["height"]

    def world_to_pixel(x, y):
        pixel_x = (x - origin_x) / resolution - 0.5
        pixel_y = height - 0.5 - (y - origin_y) / resolution
        return pixel_x, pixel_y

    image = Image.open(io.BytesIO(frames[0])).convert("RGB")
    draw = ImageDraw.Draw(image)

    path_pixels = [
        world_to_pixel(x, y)
        for x, y in travelled_path
    ]

    if len(path_pixels) >= 2:
        draw.line(
            path_pixels,
            fill=TRAIL_COLOUR,
            width=3,
        )

    for goal in goals:
        goal_pose = goal["pose"]
        goal_x, goal_y = goal_pose[:2]
        gx, gy = world_to_pixel(goal_x, goal_y)

        marker_radius = 5

        draw.ellipse(
            (
                gx - marker_radius,
                gy - marker_radius,
                gx + marker_radius,
                gy + marker_radius,
            ),
            fill=GOAL_COLOUR,
            outline=GOAL_COLOUR,
            width=1,
        )

        if len(goal_pose) > 2 and goal_pose[2] is not None:
            goal_yaw = goal_pose[2]
            heading_length = 0.25

            end_x = goal_x + heading_length * math.cos(goal_yaw)
            end_y = goal_y + heading_length * math.sin(goal_yaw)
            end_px, end_py = world_to_pixel(end_x, end_y)

            draw.line(
                (gx, gy, end_px, end_py),
                fill=GOAL_COLOUR,
                width=2,
            )

        draw.text(
            (gx + marker_radius + 3, gy - 8),
            goal["name"],
            fill=GOAL_COLOUR,
            stroke_width=1,
            stroke_fill=OUTLINE_COLOUR,
        )

    pixel_x, pixel_y = world_to_pixel(pose["x"], pose["y"])

    robot_radius = 0.34 / 2
    radius_pixels = robot_radius / resolution

    draw.ellipse(
        (
            pixel_x - radius_pixels,
            pixel_y - radius_pixels,
            pixel_x + radius_pixels,
            pixel_y + radius_pixels,
        ),
        fill=ROBOT_COLOUR,
        outline=ROBOT_COLOUR,
        width=1,
    )

    yaw = pose["yaw"]
    arrow_length = radius_pixels * 1.5

    direction_x = math.cos(yaw)
    direction_y = -math.sin(yaw)

    tip_x = pixel_x + arrow_length * direction_x
    tip_y = pixel_y + arrow_length * direction_y

    draw.line(
        (pixel_x, pixel_y, tip_x, tip_y),
        fill=ROBOT_POINTER_COLOUR,
        width=2,
    )

    head_length = radius_pixels * 0.5
    head_half_width = radius_pixels * 0.3

    base_x = tip_x - head_length * direction_x
    base_y = tip_y - head_length * direction_y

    perpendicular_x = -direction_y
    perpendicular_y = direction_x

    draw.polygon(
        [
            (tip_x, tip_y),
            (
                base_x + head_half_width * perpendicular_x,
                base_y + head_half_width * perpendicular_y,
            ),
            (
                base_x - head_half_width * perpendicular_x,
                base_y - head_half_width * perpendicular_y,
            ),
        ],
        fill=ROBOT_POINTER_COLOUR,
    )

    image.save(filename, format="PNG")

    if debug:
        print("\n=== Debug Info ===")
        print(f"Map saved to {filename}")
        print("Map Info:", map_info)
        print(
            f"Robot: x={pose['x']:.3f} m, "
            f"y={pose['y']:.3f} m, "
            f"yaw={math.degrees(yaw):.1f}°"
        )
        print(f"Robot diameter on image: {2 * radius_pixels:.1f} pixels")
        print("Goals:", [goal["name"] for goal in goals])
        print("===================\n")
