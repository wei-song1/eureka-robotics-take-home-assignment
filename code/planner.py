"""A* with hard robot clearance and a traversable, higher-cost safety band."""
import heapq
import io
import math
import numpy as np
from PIL import Image

ROBOT_RADIUS = 0.17
SAFETY_MARGIN = 0.17
SAFETY_COST = 6.0
simulator = None

def configure(simulator_value):
    global simulator
    simulator = simulator_value

class PlanningMap:

    def __init__(self, map_info, occupied, robot_radius=ROBOT_RADIUS,
                 safety_margin=SAFETY_MARGIN, safety_cost=SAFETY_COST):
        self.info = map_info
        self.resolution = float(map_info['resolution'])
        self.origin = map_info['origin']
        self.height, self.width = occupied.shape
        if (not all(math.isfinite(v) for v in
                    (self.resolution, robot_radius, safety_margin, safety_cost))
                or self.resolution <= 0 or robot_radius <= 0
                or safety_margin < 0 or safety_cost < 1):
            raise ValueError('Invalid map resolution or robot clearance')
        if (self.height, self.width) != (map_info['height'], map_info['width']):
            raise ValueError('Map dimensions disagree with PNG')
        self.occupied = occupied.copy()
        # Keep the physical exclusion distinct from the optional clearance band.
        # The diagonal allowance covers obstacle-pixel and centre-cell areas.
        hard_inflation = robot_radius + math.sqrt(2) * self.resolution
        inflation = hard_inflation + safety_margin
        self.robot_radius = robot_radius
        self.safety_cost = safety_cost
        self.hard_inflation = hard_inflation
        self.inflation = inflation
        cells = math.ceil(inflation / self.resolution)
        padded = np.pad(self.occupied, cells, constant_values=True)
        self.blocked = np.zeros_like(self.occupied, dtype=bool)
        expanded = np.zeros_like(self.occupied, dtype=bool)
        for dr in range(-cells, cells + 1):
            for dc in range(-cells, cells + 1):
                if math.hypot(dr, dc) * self.resolution <= inflation:
                    shifted = padded[cells + dr:cells + dr + self.height,
                                     cells + dc:cells + dc + self.width]
                    expanded |= shifted
                    if math.hypot(dr, dc) * self.resolution <= hard_inflation:
                        self.blocked |= shifted
        self.safety = expanded & ~self.blocked
        self.cost = np.ones(self.occupied.shape, dtype=float)
        # A* may cross the safety band, but hard-inflated cells remain forbidden.
        self.cost[self.safety] = safety_cost
        self.cost[self.blocked] = math.inf

    def add_obstacles(self, occupied):
        """Inflate only newly learned cells; preserve the original map and costs."""
        if occupied.shape != self.occupied.shape:
            raise ValueError('Obstacle layer dimensions disagree with map')
        rows, cols = np.nonzero(occupied & ~self.occupied)
        if not len(rows):
            return 0
        self.occupied[rows, cols] = True
        cells = math.ceil(self.inflation / self.resolution)
        for dr in range(-cells, cells + 1):
            for dc in range(-cells, cells + 1):
                distance = math.hypot(dr, dc) * self.resolution
                if distance > self.inflation:
                    continue
                rr, cc = rows + dr, cols + dc
                valid = (rr >= 0) & (rr < self.height) & (cc >= 0) & (cc < self.width)
                rr, cc = rr[valid], cc[valid]
                self.safety[rr, cc] = True
                self.cost[rr, cc] = self.safety_cost
                if distance <= self.hard_inflation:
                    self.blocked[rr, cc] = True
        self.safety[self.blocked] = False
        self.cost[self.blocked] = math.inf
        return len(rows)

    def retreat_is_safe(self, start, end):
        """Allow escape from new inflation only while clearance increases.

        A normal route cannot start in inflation. A short collision retreat
        may do so, but must stay inside the map and move away from occupancy.
        """
        rows, cols = np.nonzero(self.occupied)
        obstacle_x = self.origin[0] + (cols + 0.5) * self.resolution
        obstacle_y = self.origin[1] + (self.height - rows - 0.5) * self.resolution
        previous = None
        first = None
        count = max(2, math.ceil(math.dist(start, end) / (self.resolution / 2)) + 1)
        for fraction in np.linspace(0, 1, count):
            x, y = (a + fraction * (b - a) for a, b in zip(start, end))
            row, col = self.world_to_cell(x, y)
            if not (0 <= row < self.height and 0 <= col < self.width):
                return False
            if self.occupied[row, col]:
                return False
            clearance = float(np.min(np.hypot(obstacle_x - x, obstacle_y - y))) if len(rows) else math.inf
            if clearance < self.robot_radius - 0.03:
                return False
            if previous is not None and clearance < previous - 0.002:
                return False
            if first is None:
                first = clearance
            previous = clearance
        return self.is_free(self.world_to_cell(*end)) or previous > first + 0.001

    def world_to_cell(self, x, y):
        col = math.floor((x - self.origin[0]) / self.resolution)
        row = self.height - 1 - math.floor((y - self.origin[1]) / self.resolution)
        return (row, col)

    def cell_to_world(self, cell):
        row, col = cell
        return (self.origin[0] + (col + 0.5) * self.resolution, self.origin[1] + (self.height - row - 0.5) * self.resolution)

    def is_free(self, cell):
        row, col = cell
        return 0 <= row < self.height and 0 <= col < self.width and (not self.blocked[row, col])

    def segment_is_free(self, start, end):
        """Grid traversal checks every cell touched by a world-space segment.

        At an exact corner crossing, check both side cells too, preventing
        shortcuts diagonally between obstacles during path simplification.
        """
        x0 = (start[0] - self.origin[0]) / self.resolution
        y0 = (start[1] - self.origin[1]) / self.resolution
        x1 = (end[0] - self.origin[0]) / self.resolution
        y1 = (end[1] - self.origin[1]) / self.resolution
        col, bottom_row = (math.floor(x0), math.floor(y0))
        end_col, end_row = (math.floor(x1), math.floor(y1))
        dx, dy = (x1 - x0, y1 - y0)
        sx = 1 if dx > 0 else -1
        sy = 1 if dy > 0 else -1
        tx = (col + 1 - x0) / dx if dx > 0 else (col - x0) / dx if dx < 0 else math.inf
        ty = (bottom_row + 1 - y0) / dy if dy > 0 else (bottom_row - y0) / dy if dy < 0 else math.inf
        step_x = 1 / abs(dx) if dx else math.inf
        step_y = 1 / abs(dy) if dy else math.inf

        def free(c, r):
            return self.is_free((self.height - 1 - r, c))
        along_x_edge = dx == 0 and abs(x0 - round(x0)) < 1e-10
        along_y_edge = dy == 0 and abs(y0 - round(y0)) < 1e-10
        for _ in range(self.width + self.height + 4):
            if not free(col, bottom_row):
                return False
            if along_x_edge and (not free(col - 1, bottom_row)):
                return False
            if along_y_edge and (not free(col, bottom_row - 1)):
                return False
            if (col, bottom_row) == (end_col, end_row):
                return True
            if abs(tx - ty) < 1e-10:
                if not free(col + sx, bottom_row) or not free(col, bottom_row + sy):
                    return False
                col += sx
                bottom_row += sy
                tx += step_x
                ty += step_y
            elif tx < ty:
                col += sx
                tx += step_x
            else:
                bottom_row += sy
                ty += step_y
        return False

    def segment_cost(self, start, end):
        """Length-weighted cost through each crossed cell, in grid units.

        This is also used to compare shortcuts with the original A* path, so
        smoothing cannot silently replace a low-cost detour with a risky line.
        """
        if not self.segment_is_free(start, end):
            return math.inf
        x0 = (start[0] - self.origin[0]) / self.resolution
        y0 = (start[1] - self.origin[1]) / self.resolution
        x1 = (end[0] - self.origin[0]) / self.resolution
        y1 = (end[1] - self.origin[1]) / self.resolution
        dx, dy = x1 - x0, y1 - y0
        length = math.hypot(dx, dy)
        if length == 0:
            return 0.0
        col, row = math.floor(x0), math.floor(y0)
        sx, sy = (1 if dx > 0 else -1), (1 if dy > 0 else -1)
        tx = ((col + 1 - x0) / dx if dx > 0 else
              (col - x0) / dx if dx < 0 else math.inf)
        ty = ((row + 1 - y0) / dy if dy > 0 else
              (row - y0) / dy if dy < 0 else math.inf)
        dtx = 1 / abs(dx) if dx else math.inf
        dty = 1 / abs(dy) if dy else math.inf
        t, total = 0.0, 0.0
        for _ in range(self.width + self.height + 4):
            next_t = min(tx, ty, 1.0)
            total += max(0.0, next_t - t) * length * self.cost[self.height - 1 - row, col]
            if next_t >= 1:
                return total
            t = next_t
            if abs(tx - ty) < 1e-10:
                col, row = col + sx, row + sy
                tx, ty = tx + dtx, ty + dty
            elif tx < ty:
                col, tx = col + sx, tx + dtx
            else:
                row, ty = row + sy, ty + dty
        return math.inf

def get_planning_map():
    """Fetch a NEW, clean occupancy map directly from the simulator."""
    map_info, frames = simulator.call('get_map')
    pixels = np.array(Image.open(io.BytesIO(frames[0])).convert('L'))
    return PlanningMap(map_info, pixels != 0)

def astar_search(planning_map, start_xy, goal_xy):
    """Return a minimum-cost 8-connected grid path as (row, col) cells."""
    start = planning_map.world_to_cell(*start_xy)
    goal = planning_map.world_to_cell(*goal_xy)
    for label, cell in (('Robot', start), ('Goal', goal)):
        if not planning_map.is_free(cell):
            raise ValueError(f'{label} is outside the map or inside the hard robot-radius exclusion zone')

    def heuristic(cell):
        dr, dc = (abs(goal[0] - cell[0]), abs(goal[1] - cell[1]))
        return max(dr, dc) + (math.sqrt(2) - 1) * min(dr, dc)
    frontier = [(heuristic(start), 0.0, start)]
    costs = {start: 0.0}
    parent = {}
    while frontier:
        _, cost, current = heapq.heappop(frontier)
        if cost > costs[current]:
            continue
        if current == goal:
            path = [current]
            while current in parent:
                current = parent[current]
                path.append(current)
            return path[::-1]
        r, c = current
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)):
            neighbour = (r + dr, c + dc)
            if not planning_map.is_free(neighbour):
                continue
            if dr and dc and (not planning_map.is_free((r + dr, c)) or not planning_map.is_free((r, c + dc))):
                continue
            length = math.sqrt(2) if dr and dc else 1
            edge_cost = length * (planning_map.cost[current] + planning_map.cost[neighbour]) / 2
            new_cost = cost + edge_cost
            if new_cost < costs.get(neighbour, math.inf):
                costs[neighbour] = new_cost
                parent[neighbour] = current
                heapq.heappush(frontier, (new_cost + heuristic(neighbour), new_cost, neighbour))
    raise ValueError("A* found no route with the robot's required clearance")

def plan_waypoints(planning_map, start_xy, goal_xy):
    cells = astar_search(planning_map, start_xy, goal_xy)
    points = [tuple(start_xy)]
    points.extend((planning_map.cell_to_world(cell) for cell in cells))
    points.append(tuple(goal_xy))
    prefix_cost = [0.0]
    for a, b in zip(points, points[1:]):
        prefix_cost.append(prefix_cost[-1] + planning_map.segment_cost(a, b))
    waypoints = [points[0]]
    index = 0
    while index < len(points) - 1:
        candidate = len(points) - 1
        while candidate > index:
            shortcut_cost = planning_map.segment_cost(points[index], points[candidate])
            original_cost = prefix_cost[candidate] - prefix_cost[index]
            if math.isfinite(shortcut_cost) and shortcut_cost <= original_cost + 1e-8:
                break
            candidate -= 1
        if candidate == index:
            raise ValueError("No safe segment from the robot's actual position")
        waypoints.append(points[candidate])
        index = candidate
    return waypoints
