"""Asynchronous RGBD recording. Numerical depth and capture timing are preserved."""
import io
import json
from copy import deepcopy
from pathlib import Path
import queue
import shutil
import subprocess
import threading
import time

import numpy as np
from PIL import Image
from camera_display import depth_preview, format_viewer_status, overlay_video_info


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2), encoding='utf-8')


class RGBDRecorder:
    def __init__(self, directory, camera_info, session_info=None, queue_size=90):
        if queue_size < 1:
            raise ValueError('Recording queue size must be positive')
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.camera_info = dict(camera_info)
        self.session_info = dict(session_info or {})
        self.ffmpeg = shutil.which('ffmpeg')
        if self.ffmpeg is None:
            raise RuntimeError('FFmpeg is required for RGB/depth videos; install ffmpeg and add it to PATH')
        self.queue = queue.Queue(maxsize=queue_size)
        self.finished = threading.Event()
        self.lock = threading.Lock()
        self.error = None
        self.accepting = True
        self.queue_rejections = 0
        self.source_gaps = 0
        self.invalid_messages = 0
        self.written = 0
        self.segments = []
        self._thread = None
        self._videos = {}
        self._video_logs = {}
        self._manifest = None
        self._segment = None
        self._video_index = 0
        self.viewer_state = {
            'scene': self.session_info.get('scene') or {'id': self.session_info.get('requested_scene')},
            'target_goal': self.session_info.get('target_goal'), 'marked_goals': [],
        }
        self._status_frame = threading.Event()
        write_json(self.directory / 'camera_info.json', self.camera_info)
        write_json(self.directory / 'session.json', {
            **self.session_info,
            'format_version': 2,
            'depth_dtype': 'uint16',
            'depth_invalid': 0,
            'depth_scale_metres': self.camera_info['depth_scale'],
            'video_timing': 'constant rate; frames.jsonl contains original capture times and video indices',
            'videos': {'rgb': 'segment_*/rgb.mp4', 'depth': 'segment_*/depth.mp4',
                       'codec': 'H.264/yuv420p', 'overlay': 'UI info at bottom left',
                       'depth_display': 'fixed max_depth range; near white, far/invalid black'},
            'status_file': 'summary.json',
        })

    def start(self):
        self._thread = threading.Thread(target=self._run, name='rgbd-recorder', daemon=True)
        self._thread.start()
        return self

    def _fail(self, message):
        with self.lock:
            first = self.error is None
            if first:
                self.error = str(message)
            self.accepting = False
        if first:
            print(f'[camera] RECORDING ERROR: {message}', flush=True)

    def set_viewer_state(self, state):
        with self.lock:
            self.viewer_state = deepcopy(state)
            self._status_frame.clear()

    def wait_for_status_frame(self, timeout=0.4):
        """Allow a real post-mark capture before an immediately ended run closes."""
        return self._status_frame.wait(timeout)

    def submit(self, packet):
        # Never block the receiver or navigation when storage falls behind.
        with self.lock:
            if not self.accepting:
                self.queue_rejections += 1
                return False
            try:
                state = deepcopy(self.viewer_state)
                mark_times = state.pop('goal_mark_times', {})
                desired = state.get('marked_goals', [])
                state['marked_goals'] = [name for name in desired
                                         if packet.header['t'] >= mark_times.get(name, -float('inf'))]
                self.queue.put_nowait((packet, state))
                if state['marked_goals'] == desired:
                    self._status_frame.set()
                return True
            except queue.Full:
                self.queue_rejections += 1
        self._fail('Recording queue filled; recording stopped accepting frames. Preview/navigation continue.')
        return False

    def _open_segment(self, packet):
        self._close_segment()
        self._segment = packet.segment
        folder = self.directory / f'segment_{packet.segment:04d}'
        (folder / 'rgb').mkdir(parents=True, exist_ok=True)
        (folder / 'depth').mkdir(exist_ok=True)
        self._folder = folder
        self._manifest = (folder / 'frames.jsonl').open('w', encoding='utf-8')
        width, height = packet.header['width'], packet.header['height']
        if width % 2 or height % 2:
            raise ValueError('Video dimensions must be even for yuv420p recording')
        fps = float(self.camera_info.get('rate', 15.0))
        for name in ('rgb', 'depth'):
            log_name = 'ffmpeg.log' if name == 'rgb' else 'depth_ffmpeg.log'
            self._video_logs[name] = (folder / log_name).open('wb')
            command = [self.ffmpeg, '-hide_banner', '-loglevel', 'error', '-y',
                       '-f', 'rawvideo', '-pixel_format', 'rgb24',
                       '-video_size', f'{width}x{height}', '-framerate', str(fps),
                       '-i', 'pipe:0', '-an', '-c:v', 'libx264', '-preset', 'veryfast',
                       '-threads', '2', '-crf', '20', '-pix_fmt', 'yuv420p', '-movflags', '+faststart',
                       str(folder / f'{name}.mp4')]
            self._videos[name] = subprocess.Popen(command, stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL, stderr=self._video_logs[name])
        self._video_index = 0
        self.segments.append({'directory': folder.name, 'first_t': packet.header['t'],
                              'first_seq': packet.header['seq'],
                              'rgb_video': 'rgb.mp4', 'depth_video': 'depth.mp4'})

    def _close_segment(self):
        errors = []
        if self._manifest is not None:
            try:
                self._manifest.close()
            except OSError as exc:
                errors.append(f'Could not close frame metadata: {exc}')
            finally:
                self._manifest = None
        videos, self._videos = self._videos, {}
        for video in videos.values():
            try:
                video.stdin.close()
            except (BrokenPipeError, OSError) as exc:
                errors.append(f'Could not close video input: {exc}')
        deadline = time.monotonic() + 20
        for name, video in videos.items():
            try:
                if video.wait(timeout=max(0.01, deadline - time.monotonic())) != 0:
                    errors.append(f'FFmpeg failed to finalise {name} video; see the segment encoder logs')
            except subprocess.TimeoutExpired:
                video.kill()
                video.wait()
                errors.append(f'{name} FFmpeg did not finish within 20 seconds')
        for log in self._video_logs.values():
            log.close()
        self._video_logs = {}
        if errors:
            raise RuntimeError('; '.join(errors))

    def _write(self, packet, viewer_state):
        if packet.segment != self._segment:
            self._open_segment(packet)
        header = packet.header
        width, height = header['width'], header['height']
        image = Image.open(io.BytesIO(packet.jpeg)).convert('RGB')
        if image.size != (width, height):
            raise ValueError('JPEG dimensions disagree with camera header')
        depth = np.frombuffer(packet.depth_bytes, dtype='<u2').reshape(height, width)
        name = f'{self._video_index:08d}'
        rgb_file = f'rgb/{name}.jpg'
        depth_file = f'depth/{name}.png'
        (self._folder / rgb_file).write_bytes(packet.jpeg)
        Image.fromarray(depth.astype(np.uint16)).save(self._folder / depth_file, format='PNG')
        info_text = format_viewer_status(viewer_state, header)
        depth_image = Image.fromarray(depth_preview(depth, self.camera_info)).convert('RGB')
        self._videos['rgb'].stdin.write(overlay_video_info(image, info_text).tobytes())
        self._videos['depth'].stdin.write(overlay_video_info(depth_image, info_text).tobytes())
        self._manifest.write(json.dumps({
            'header': header,
            'video_frame_index': self._video_index,
            'viewer_state': viewer_state,
            'info_text': info_text,
            'rgb_file': rgb_file,
            'depth_file': depth_file,
            'received_wall_time_ns': packet.received_wall_time_ns,
            'received_monotonic_ns': packet.received_monotonic_ns,
        }) + '\n')
        self._manifest.flush()
        self._video_index += 1
        self.written += 1

    def _run(self):
        try:
            while not self.finished.is_set() or not self.queue.empty():
                try:
                    packet, viewer_state = self.queue.get(timeout=.1)
                except queue.Empty:
                    continue
                try:
                    self._write(packet, viewer_state)
                finally:
                    self.queue.task_done()
        except Exception as exc:
            self._fail(exc)
        finally:
            try:
                self._close_segment()
            except Exception as exc:
                self._fail(exc)
            self._write_summary()

    def _write_summary(self):
        try:
            write_json(self.directory / 'summary.json', {
                'status': 'error' if self.error else ('complete' if self.written else 'no_frames'),
                'error': self.error,
                'frames_written': self.written,
                'source_sequence_gaps': self.source_gaps,
                'invalid_messages': self.invalid_messages,
                'queue_rejections': self.queue_rejections,
                'unwritten_queued_frames': self.queue.qsize(),
                'segments': self.segments,
            })
        except OSError as exc:
            print(f'[camera] Could not write recording summary: {exc}', flush=True)

    def stop(self):
        with self.lock:
            self.accepting = False
        self.finished.set()
        if self._thread is not None:
            self._thread.join(timeout=30)
            if self._thread.is_alive():
                self._fail('Recorder is still finalising after 30 seconds; files may be incomplete')
                for video in tuple(self._videos.values()):
                    video.kill()
                self._thread.join(timeout=3)
            if not self._thread.is_alive():
                # Refresh counters after a failed writer while the receiver
                # was still running. No worker owns these files now.
                self._write_summary()
        return self.error
