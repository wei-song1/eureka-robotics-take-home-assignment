"""Continuous RGBD reception, independent of navigation and disk writing."""
from contextlib import contextmanager
from dataclasses import dataclass
from datetime import datetime, timezone
import importlib.util
import io
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
import uuid

import numpy as np
from PIL import Image

from recorder import RGBDRecorder, write_json


@dataclass(frozen=True)
class CameraPacket:
    header: dict
    jpeg: bytes
    depth_bytes: bytes
    received_wall_time_ns: int
    received_monotonic_ns: int
    segment: int = 0


def parse_camera_message(frames, camera_info=None, segment=0):
    if len(frames) != 3:
        raise ValueError('Camera message must contain header, JPEG and depth')
    header = json.loads(frames[0])
    width, height = header['width'], header['height']
    if type(width) is not int or type(height) is not int or width <= 0 or height <= 0:
        raise ValueError('Invalid camera dimensions')
    if header['rgb'] != 'jpeg' or header['depth'] != 'u16':
        raise ValueError('Unsupported camera encoding')
    if type(header['seq']) is not int or header['seq'] < 0:
        raise ValueError('Invalid camera sequence number')
    if not isinstance(header['t'], (float, int)) or not math.isfinite(header['t']):
        raise ValueError('Invalid simulation time')
    for key in ('x', 'y', 'yaw'):
        if not math.isfinite(header['pose'][key]):
            raise ValueError('Invalid capture-time pose')
    if len(frames[2]) != width * height * 2:
        raise ValueError('Depth payload has the wrong length')
    if camera_info and (width, height) != (camera_info['width'], camera_info['height']):
        raise ValueError('Camera dimensions changed; restart with new calibration')
    return CameraPacket(header, bytes(frames[1]), bytes(frames[2]),
                        time.time_ns(), time.monotonic_ns(), segment)


def decode_camera_packet(packet, camera_info):
    rgb = np.array(Image.open(io.BytesIO(packet.jpeg)).convert('RGB'))
    height, width = packet.header['height'], packet.header['width']
    if rgb.shape != (height, width, 3):
        raise ValueError('JPEG dimensions disagree with camera header')
    depth_u16 = np.frombuffer(packet.depth_bytes, dtype='<u2').reshape(height, width)
    depth_metres = depth_u16.astype(np.float32) * camera_info['depth_scale']
    depth_metres[depth_u16 == 0] = np.nan
    return rgb, depth_u16, depth_metres


def depth_preview(depth_u16, camera_info):
    """Fixed 0..max_depth grayscale: near is white, far/invalid is black."""
    metres = depth_u16.astype(np.float32) * camera_info['depth_scale']
    brightness = np.clip(1 - metres / camera_info['max_depth'], 0, 1)
    image = (brightness * 255).astype(np.uint8)
    image[depth_u16 == 0] = 0
    return image


class CameraStream:
    def __init__(self, endpoint, camera_info, recorder=None):
        self.endpoint = endpoint
        self.camera_info = dict(camera_info)
        self.recorder = recorder
        self.stop_event = threading.Event()
        self.ready = threading.Event()
        self.lock = threading.Lock()
        self.latest = None
        self.error = None
        self.received = 0
        self.source_gaps = 0
        self.invalid_messages = 0
        self.segment = 0
        self.last_seq = None
        self.last_t = None
        self.thread = None

    def start(self):
        self.thread = threading.Thread(target=self._run, name='rgbd-receiver', daemon=True)
        self.thread.start()
        if not self.ready.wait(3):
            raise TimeoutError('Camera receiver did not start')
        if self.error:
            raise RuntimeError(self.error)
        return self

    def snapshot(self):
        with self.lock:
            return self.latest, self.error, self.source_gaps

    def _accept(self, frames):
        packet = parse_camera_message(frames, self.camera_info)
        seq, capture_t = packet.header['seq'], packet.header['t']
        # A reset restarts simulation time/sequence. Keep its files and video
        # separate so replay never mixes two time origins or overwrites frames.
        if self.last_seq is not None:
            if seq < self.last_seq or capture_t < self.last_t:
                self.segment += 1
            elif seq == self.last_seq and capture_t == self.last_t:
                return
            else:
                self.source_gaps += max(0, seq - self.last_seq - 1)
        self.last_seq, self.last_t = seq, capture_t
        if self.segment:
            packet = CameraPacket(packet.header, packet.jpeg, packet.depth_bytes,
                                  packet.received_wall_time_ns, packet.received_monotonic_ns,
                                  self.segment)
        with self.lock:
            self.latest = packet
            self.received += 1
        if self.recorder:
            self.recorder.submit(packet)

    def _run(self):
        context, socket = None, None
        try:
            import zmq
            # Both the socket and its context are created/closed by this worker.
            # Do not use simulator.camera or simulator.latest() for recording.
            context = zmq.Context()
            socket = context.socket(zmq.SUB)
            socket.setsockopt(zmq.LINGER, 0)
            socket.setsockopt(zmq.RCVHWM, 120)
            socket.setsockopt(zmq.SUBSCRIBE, b'')
            socket.connect(self.endpoint)
            self.ready.set()
            while not self.stop_event.is_set():
                if not socket.poll(200):
                    continue
                try:
                    self._accept(socket.recv_multipart())
                except (ValueError, TypeError, KeyError, OverflowError) as exc:
                    self.invalid_messages += 1
                    if self.invalid_messages <= 3:
                        print(f'[camera] Invalid message: {exc}', flush=True)
        except Exception as exc:
            with self.lock:
                self.error = str(exc)
            print(f'[camera] Receiver stopped: {exc}', flush=True)
        finally:
            self.ready.set()
            if socket is not None:
                socket.close(linger=0)
            if context is not None:
                context.term()

    def stop(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=3)
            if self.thread.is_alive():
                print('[camera] Receiver did not stop within three seconds', flush=True)


class CameraSession:
    def __init__(self, simulator, camera_info, scene_id=None, seed=None):
        self.endpoint = simulator.url(3)
        self.camera_info = dict(camera_info)
        self.scene_id, self.seed = scene_id, seed
        self.recorder = self.stream = self.viewer = None
        self.viewer_log = None
        self.directory = None
        self.stopped = False

    def start(self):
        record = os.environ.get('VACUUM_CAMERA_RECORD', '1').lower() not in ('0', 'false', 'no')
        preview = os.environ.get('VACUUM_CAMERA_PREVIEW', '1').lower() not in ('0', 'false', 'no')
        if not record and not preview:
            return self
        root = Path(os.environ.get('VACUUM_RECORDINGS_DIR', 'recordings'))
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S')
        self.directory = root / f'{stamp}_{uuid.uuid4().hex[:8]}'
        self.directory.mkdir(parents=True, exist_ok=False)
        write_json(self.directory / 'camera_info.json', self.camera_info)
        if record:
            try:
                self.recorder = RGBDRecorder(self.directory, self.camera_info,
                    {'requested_scene': self.scene_id, 'seed': self.seed,
                     'camera_endpoint': self.endpoint, 'created_utc': stamp}).start()
                self.stream = CameraStream(self.endpoint, self.camera_info, self.recorder)
                self.stream.start()
                print(f'[camera] Recording to {self.directory.resolve()}', flush=True)
            except Exception as exc:
                print(f'[camera] Recording unavailable: {exc}', flush=True)
        if preview:
            if importlib.util.find_spec('PySide6') is None:
                print('[camera] Preview unavailable: install PySide6', flush=True)
            elif (sys.platform.startswith('linux') and not os.environ.get('DISPLAY')
                  and not os.environ.get('WAYLAND_DISPLAY')
                  and os.environ.get('QT_QPA_PLATFORM') not in ('offscreen', 'minimal')):
                print('[camera] No desktop display found; recording can continue without a preview', flush=True)
            else:
                self.viewer_log = (self.directory / 'viewer.log').open('wb')
                self.viewer = subprocess.Popen([
                    sys.executable, str(Path(__file__).with_name('camera_viewer.py')),
                    '--endpoint', self.endpoint,
                    '--camera-info', str((self.directory / 'camera_info.json').resolve()),
                    '--stop-file', str((self.directory / 'viewer.stop').resolve()),
                ], stdout=self.viewer_log, stderr=subprocess.STDOUT)
        return self

    def stop(self):
        if self.stopped:
            return
        self.stopped = True
        if self.stream:
            self.stream.stop()
        if self.recorder:
            if self.stream:
                self.recorder.source_gaps = self.stream.source_gaps
                self.recorder.invalid_messages = self.stream.invalid_messages
                if self.stream.error:
                    self.recorder._fail(f'Receiver: {self.stream.error}')
            self.recorder.stop()
        if self.viewer:
            try:
                (self.directory / 'viewer.stop').touch()
                self.viewer.wait(timeout=4)
            except (OSError, subprocess.TimeoutExpired):
                self.viewer.terminate()
                try:
                    self.viewer.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    self.viewer.kill()
                    self.viewer.wait()
            if self.viewer.returncode not in (0, -15):
                print(f'[camera] Preview exited with code {self.viewer.returncode}; see viewer.log', flush=True)
        if self.viewer_log:
            self.viewer_log.close()


@contextmanager
def camera_session(simulator, camera_info, scene_id=None, seed=None):
    session = CameraSession(simulator, camera_info, scene_id, seed)
    try:
        try:
            session.start()
        except Exception as exc:
            print(f'[camera] Camera startup failed: {exc}', flush=True)
        yield session
    finally:
        session.stop()
