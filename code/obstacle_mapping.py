"""Per-run, additive obstacle observations for an otherwise static world."""
import math
import threading
import time

import numpy as np

from camera_stream import CameraStream
import planner


class NavigationMap:
    """Consume fresh depth on the navigation thread; receiver owns its socket."""

    MAX_FRAME_AGE = 0.75
    MIN_HEIGHT = 0.025
    MAX_RANGE = 3.0
    CONFIRMATIONS = 2

    def __init__(self, simulator, camera_info, robot_info=None, planning_map=None,
                 stream=None):
        self.simulator = simulator
        self.camera_info = dict(camera_info)
        self.planning_map = planning_map if planning_map is not None else planner.get_planning_map()
        self.base_occupied = self.planning_map.occupied.copy()
        self.learned = np.zeros_like(self.base_occupied)
        self.evidence = np.zeros(self.learned.shape, dtype=np.uint8)
        self.last_seen = np.full(self.learned.shape, -math.inf)
        self.lock = threading.Lock()
        self.stream = stream if stream is not None else CameraStream(simulator.url(3), camera_info)
        self.height = float((robot_info or {}).get('height', 0.09))
        self.transform = np.asarray(camera_info['T_base_camera'], dtype=float)
        if self.transform.shape != (4, 4) or not np.isfinite(self.transform).all():
            raise ValueError('Invalid camera-to-robot transform')
        self.last_key = None
        self.last_update = -math.inf
        self.last_report = -math.inf
        self.frames_processed = 0
        self.approach = None

    def __enter__(self):
        try:
            self.stream.start()
            deadline = time.monotonic() + 3
            while self.frames_processed < self.CONFIRMATIONS:
                if time.monotonic() >= deadline:
                    raise TimeoutError('No fresh depth frames for obstacle mapping')
                packet, error, _ = self.stream.snapshot()
                if error:
                    raise RuntimeError(f'Obstacle camera stopped: {error}')
                if packet is not None:
                    self.update(force=True)
                time.sleep(0.02)
            return self
        except BaseException:
            self.stream.stop()
            raise

    def __exit__(self, *args):
        self.stream.stop()

    def snapshot_obstacles(self):
        with self.lock:
            return self.learned.copy()

    def update(self, message=None, force=False):
        packet, error, _ = self.stream.snapshot()
        if error:
            raise RuntimeError(f'Obstacle camera stopped: {error}')
        if packet is None:
            raise TimeoutError('No depth frame available for obstacle mapping')
        now = time.monotonic()
        if now - packet.received_monotonic_ns / 1e9 > self.MAX_FRAME_AGE:
            raise TimeoutError('Depth camera is stale; robot stopped')
        if packet.segment != 0:
            raise RuntimeError('Simulator reset during navigation; choose the scene again')
        if message is not None and 't' in message:
            age = message['t'] - packet.header['t']
            if abs(age) > self.MAX_FRAME_AGE:
                raise TimeoutError('Depth capture and robot state are out of sync')
        key = (packet.segment, packet.header['seq'])
        if key == self.last_key or (not force and now - self.last_update < 0.1):
            return 0
        self.last_key = key
        self.last_update = now
        added = self.observe_depth(packet)
        self.frames_processed += 1
        if added and now - self.last_report >= 1:
            print(f'[map] Learned obstacles: {int(self.learned.sum())} cells', flush=True)
            self.last_report = now
        return added

    def observe_depth(self, packet):
        """Project optical z-depth using calibration and the capture-time pose.

        Sample every second pixel, rejecting floor, overhead furniture,
        invalid returns and the robot footprint. Each cell gets at most one
        vote per frame, requiring two nearby-in-time observations.
        """
        info, grid = self.camera_info, self.planning_map
        h, w = packet.header['height'], packet.header['width']
        if (h, w) != (info['height'], info['width']):
            raise ValueError('Depth dimensions changed during mapping')
        raw = np.frombuffer(packet.depth_bytes, '<u2').reshape(h, w)[::2, ::2]
        v, u = np.mgrid[0:h:2, 0:w:2]
        # The camera reports optical-axis depth, so project it before applying the pose.
        z = raw.astype(float) * info['depth_scale']
        valid = (raw != 0) & (z >= 0.05) & (z <= min(self.MAX_RANGE, info['max_depth']))
        z, u, v = z[valid], u[valid], v[valid]
        points = self.transform @ np.vstack(((u - info['cx']) * z / info['fx'],
                                             (v - info['cy']) * z / info['fy'],
                                             z, np.ones_like(z)))
        distance = np.hypot(points[0], points[1])
        valid = ((points[2] >= self.MIN_HEIGHT) & (points[2] <= self.height)
                 & (distance > grid.robot_radius + 0.03))
        x, y = points[0, valid], points[1, valid]
        pose = packet.header['pose']
        c, s = math.cos(pose['yaw']), math.sin(pose['yaw'])
        world_x = pose['x'] + c * x - s * y
        world_y = pose['y'] + s * x + c * y
        cols = np.floor((world_x - grid.origin[0]) / grid.resolution).astype(int)
        rows = grid.height - 1 - np.floor((world_y - grid.origin[1]) / grid.resolution).astype(int)
        valid = (rows >= 0) & (rows < grid.height) & (cols >= 0) & (cols < grid.width)
        hits = np.zeros_like(self.learned)
        hits[rows[valid], cols[valid]] = True
        # One-cell support tolerates small quantisation/pose differences.
        supported = hits.copy()
        supported[1:] |= hits[:-1]
        supported[:-1] |= hits[1:]
        supported[:, 1:] |= hits[:, :-1]
        supported[:, :-1] |= hits[:, 1:]
        rows, cols = np.nonzero(supported & ~self.base_occupied & ~self.learned)
        capture_t = packet.header['t']
        recent = capture_t - self.last_seen[rows, cols] <= 0.5
        self.evidence[rows, cols] = np.where(recent, self.evidence[rows, cols] + 1, 1)
        self.last_seen[rows, cols] = capture_t
        confirmed = np.zeros_like(hits)
        keep = self.evidence[rows, cols] >= self.CONFIRMATIONS
        confirmed[rows[keep], cols[keep]] = True
        return self._add(confirmed)

    def _add(self, occupied):
        with self.lock:
            new = occupied & ~self.planning_map.occupied
            count = self.planning_map.add_obstacles(new)
            self.learned |= new
        return count

    def record_collision(self, collision):
        """Infer a short front contact arc; bumper has no contact bearing."""
        pose = collision.message['pose']
        grid = self.planning_map
        occupied = np.zeros_like(self.learned)
        for angle in np.linspace(pose['yaw'] - math.radians(20),
                                 pose['yaw'] + math.radians(20), 9):
            distance = grid.robot_radius + 0.02
            row, col = grid.world_to_cell(pose['x'] + distance * math.cos(angle),
                                          pose['y'] + distance * math.sin(angle))
            if 0 <= row < grid.height and 0 <= col < grid.width:
                occupied[row, col] = True
        added = self._add(occupied)
        print(f'[map] Bumper contact added {added} obstacle cells', flush=True)
        return added
