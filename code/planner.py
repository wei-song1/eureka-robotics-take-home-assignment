from heapq import heappop, heappush

import numpy as np
from PIL import Image


FREE = 0
SAFETY_MARGIN = 1
WALL = 2
SAFETY_MARGIN_COST = 5.0


def convert_map_to_array(image):
    # 0 = Free
    # 1 = Safety Margin
    # 2 = Wall
    gray = np.asarray(image.convert("L"))
    return np.digitize(gray, bins=[85, 170]).astype(np.uint8)


def world_to_grid(map_info, x, y):
    resolution = map_info["resolution"]
    origin_x, origin_y = map_info["origin"]
    height = map_info["height"]

    column = round((x - origin_x) / resolution - 0.5)
    row = round(height - 0.5 - (y - origin_y) / resolution)
    return column, row


def plan_route(map_info, planning_image, pose, goal_pose):
    grid = convert_map_to_array(planning_image)
    start = world_to_grid(map_info, pose["x"], pose["y"])
    goal = world_to_grid(map_info, goal_pose[0], goal_pose[1])
    return astar(grid, start, goal)


def astar(grid, start, goal):
    height, width = grid.shape

    def is_outside(point):
        column, row = point
        return not (
            0 <= column < width
            and 0 <= row < height
        )

    def is_blocked(point):
        return is_outside(point) or grid[point[1], point[0]] >= WALL

    if is_blocked(start) or is_blocked(goal):
        return []

    neighbours = (
        (0, -1),
        (0, 1),
        (-1, 0),
        (1, 0),
    )
    open_set = []
    sequence = 0
    heappush(open_set, (0.0, sequence, start))
    came_from = {}
    cost_so_far = {start: 0.0}

    while open_set:
        _, _, current = heappop(open_set)

        if current == goal:
            return _reconstruct_path(came_from, current)

        current_cost = cost_so_far[current]
        for dx, dy in neighbours:
            neighbour = (
                current[0] + dx,
                current[1] + dy,
            )

            if is_blocked(neighbour):
                continue

            cell_cost = (
                SAFETY_MARGIN_COST
                if grid[neighbour[1], neighbour[0]] == SAFETY_MARGIN
                else 1.0
            )
            new_cost = current_cost + cell_cost

            if new_cost >= cost_so_far.get(neighbour, float("inf")):
                continue

            cost_so_far[neighbour] = new_cost
            came_from[neighbour] = current
            heuristic = abs(goal[0] - neighbour[0]) + abs(
                goal[1] - neighbour[1],
            )
            sequence += 1
            heappush(
                open_set,
                (new_cost + heuristic, sequence, neighbour),
            )

    return []


def _reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path
