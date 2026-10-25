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

_active_session = None


def goal_marked(name):
    """Notify the viewer after the simulator has accepted a mark_goal call."""
    if _active_session is not None:
        _active_session.goal_marked(name)


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
    def __init__(self, simulator, camera_info, scene_id=None, seed=None,
                 scene_info=None, target_goal=None):
        self.endpoint = simulator.url(3)
        self.camera_info = dict(camera_info)
        self.scene_id, self.seed = scene_id, seed
        self.recorder = self.stream = self.viewer = None
        self.viewer_log = None
        self.directory = None
        self.stopped = False
        self.close_requested = False
        self._terminal_available = True
        self._command_buffer = ''
        self.scene_info = dict(scene_info or {'id': scene_id, 'name': 'Unknown', 'difficulty': 'Unknown'})
        self.target_goal = target_goal
        self.marked_goals = set()

    def _write_viewer_state(self):
        if self.directory is None:
            return
        path = self.directory / 'viewer_state.json'
        try:
            temporary = path.with_suffix('.tmp')
            write_json(temporary, {'scene': self.scene_info, 'target_goal': self.target_goal,
                                   'marked_goals': sorted(self.marked_goals)})
            temporary.replace(path)
        except OSError as exc:
            print(f'[camera] Could not update viewer status: {exc}', flush=True)

    def set_target(self, name):
        self.target_goal = None if name in (None, 'Manual') else name
        self._write_viewer_state()

    def goal_marked(self, name):
        self.marked_goals.add(name)
        if self.target_goal in (None, 'Manual'):
            self.target_goal = name
        self._write_viewer_state()

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
        self._write_viewer_state()
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
                    '--planned-map', str(Path('planned_map.png').resolve()),
                    '--top-down-map', str(Path('top_down_map.png').resolve()),
                    '--viewer-state', str((self.directory / 'viewer_state.json').resolve()),
                ], stdout=self.viewer_log, stderr=subprocess.STDOUT)
        return self

    def request_close(self):
        self.close_requested = True
        if self.viewer and self.directory:
            try:
                (self.directory / 'viewer.stop').touch()
            except OSError as exc:
                print(f'[camera] Could not send viewer quit command: {exc}', flush=True)

    def _poll_terminal_command(self):
        if not self._terminal_available:
            return None
        try:
            if sys.platform == 'win32':
                if not sys.stdin.isatty():
                    self._terminal_available = False
                    return None
                import msvcrt
                if not msvcrt.kbhit():
                    return None
                char = msvcrt.getwch()
                if char == '\x03':
                    raise KeyboardInterrupt
                if char in ('\x00', '\xe0'):
                    msvcrt.getwch()
                elif char in ('\r', '\n'):
                    print('', flush=True)
                    command, self._command_buffer = self._command_buffer, ''
                    return command
                elif char == '\b':
                    if self._command_buffer:
                        self._command_buffer = self._command_buffer[:-1]
                        print('\b \b', end='', flush=True)
                else:
                    self._command_buffer += char
                    print(char, end='', flush=True)
            else:
                import select
                ready, _, _ = select.select([sys.stdin], [], [], 0)
                if ready:
                    line = sys.stdin.readline()
                    if line == '':
                        self._terminal_available = False
                    else:
                        return line.strip()
        except (OSError, ValueError, AttributeError):
            self._terminal_available = False
        return None

    def wait_for_close(self, allow_terminal=True):
        """Keep live viewing/recording until the window closes or quit is requested."""
        if self.viewer is None or self.viewer.poll() is not None:
            return
        print('[camera] Viewer remains open. Close the window, press Ctrl+C, '
              'or type quit then Enter.', flush=True)
        try:
            while self.viewer.poll() is None and not self.close_requested:
                command = self._poll_terminal_command() if allow_terminal else None
                if command is not None:
                    if command.casefold().strip() in ('quit', 'exit'):
                        self.request_close()
                        break
                    if command.strip():
                        print('[camera] Type quit then Enter to close the viewer.', flush=True)
                time.sleep(.1)
        except KeyboardInterrupt:
            self.request_close()
            raise

    def stop(self):
        if self.stopped:
            return
        self.stopped = True
        self.request_close()
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
def camera_session(simulator, camera_info, scene_id=None, seed=None,
                   scene_info=None, target_goal=None):
    global _active_session
    session = CameraSession(simulator, camera_info, scene_id, seed, scene_info, target_goal)
    previous_session = _active_session
    try:
        try:
            session.start()
        except Exception as exc:
            print(f'[camera] Camera startup failed: {exc}', flush=True)
        _active_session = session
        yield session
    finally:
        _active_session = previous_session
        session.stop()
