"""Standalone Qt RGB/depth viewer; closing it does not stop robot navigation."""
import argparse
import json
import math
from pathlib import Path
import signal
import sys

import numpy as np

from camera_stream import CameraStream, decode_camera_packet, depth_preview


def format_viewer_status(state, header=None):
    scene = state.get('scene') or {}
    scene_id = scene.get('id') or '--'
    name = str(scene.get('name') or 'Unknown').title()
    difficulty = str(scene.get('difficulty') or 'Unknown').title()
    target = state.get('target_goal')
    target_text = target if target not in (None, 'Manual') else 'None (Manual)'
    marked = target is not None and any(
        str(goal).casefold() == str(target).casefold()
        for goal in state.get('marked_goals', []))
    if header is None:
        time_text = '--'
        pose_text = 'x - --, y - --, -- deg'
    else:
        pose = header['pose']
        time_text = f"{header['t']:.2f}"
        pose_text = f"x - {pose['x']:.3f}, y - {pose['y']:.3f}, {math.degrees(pose['yaw']):.1f} deg"
    return (f'Scene {scene_id} - {name} ({difficulty})\n'
            f'Target Goal: {target_text}\n'
            f'Time: {time_text} s\n'
            f'Pose: {pose_text}\n'
            f"Goal: {'Marked' if marked else 'Unmarked'}")


class MapImageFile:
    """Retry incomplete PNG writes and keep the last successfully shown image."""
    def __init__(self, path):
        self.path = Path(path)
        self.loaded_stamp = None

    def read_if_changed(self):
        try:
            stat = self.path.stat()
            stamp = stat.st_mtime_ns, stat.st_size
            if stamp == self.loaded_stamp:
                return None
            return self.path.read_bytes(), stamp
        except OSError:
            return None

    def mark_loaded(self, stamp):
        self.loaded_stamp = stamp


def load_camera_info(host, port):
    import zmq
    with zmq.Context() as context:
        with context.socket(zmq.REQ) as socket:
            socket.setsockopt(zmq.LINGER, 0)
            socket.setsockopt(zmq.RCVTIMEO, 3000)
            socket.setsockopt(zmq.SNDTIMEO, 3000)
            socket.connect(f'tcp://{host}:{port}')
            socket.send_json({'method': 'get_camera_info', 'params': {}})
            reply = json.loads(socket.recv_multipart()[0])
            if not reply['ok']:
                raise RuntimeError(reply['error'])
            return reply['result']


def main():
    parser = argparse.ArgumentParser(description='Live RGBD camera viewer')
    parser.add_argument('--host', default='localhost')
    parser.add_argument('--port-base', type=int, default=5555)
    parser.add_argument('--endpoint')
    parser.add_argument('--camera-info')
    parser.add_argument('--stop-file')
    parser.add_argument('--planned-map', default='planned_map.png')
    parser.add_argument('--top-down-map', default='top_down_map.png')
    parser.add_argument('--viewer-state')
    args = parser.parse_args()
    try:
        from PySide6.QtCore import Qt, QTimer
        from PySide6.QtGui import QImage, QPixmap
        from PySide6.QtWidgets import QApplication, QLabel, QMainWindow, QWidget, QGridLayout, QVBoxLayout, QPushButton
    except ImportError as exc:
        raise SystemExit('Install PySide6 to use the camera viewer: pip install PySide6') from exc

    camera_info = (json.loads(Path(args.camera_info).read_text(encoding='utf-8'))
                   if args.camera_info else load_camera_info(args.host, args.port_base))
    endpoint = args.endpoint or f'tcp://{args.host}:{args.port_base + 3}'
    stream = CameraStream(endpoint, camera_info).start()

    class ImagePanel(QLabel):
        def __init__(self, title, hover_callback=None):
            super().__init__(title)
            self.setMinimumSize(320, 240)
            self.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self.setStyleSheet('background: #111; color: white;')
            self.image = None
            self.depth = None
            self.hover_callback = hover_callback
            self.setMouseTracking(hover_callback is not None)

        def set_array(self, array, depth=None):
            array = np.ascontiguousarray(array)
            height, width = array.shape[:2]
            image_format = (QImage.Format.Format_Grayscale8 if array.ndim == 2
                            else QImage.Format.Format_RGB888)
            # Detach from NumPy's temporary buffer before the next camera frame.
            self.image = QImage(array.data, width, height, array.strides[0], image_format).copy()
            self.depth = depth
            self.render()

        def render(self):
            if self.image is not None:
                self.setPixmap(QPixmap.fromImage(self.image).scaled(
                    self.size(), Qt.AspectRatioMode.KeepAspectRatio,
                    Qt.TransformationMode.SmoothTransformation))

        def resizeEvent(self, event):
            super().resizeEvent(event)
            self.render()

        def mouseMoveEvent(self, event):
            if self.depth is not None and self.pixmap() is not None:
                pixmap = self.pixmap()
                left = (self.width() - pixmap.width()) / 2
                top = (self.height() - pixmap.height()) / 2
                x, y = event.position().x() - left, event.position().y() - top
                if 0 <= x < pixmap.width() and 0 <= y < pixmap.height():
                    u = min(self.depth.shape[1] - 1, int(x * self.depth.shape[1] / pixmap.width()))
                    v = min(self.depth.shape[0] - 1, int(y * self.depth.shape[0] / pixmap.height()))
                    value = self.depth[v, u]
                    text = f'Pixel ({u}, {v}): {value:.3f} m' if np.isfinite(value) else f'Pixel ({u}, {v}): invalid depth'
                    self.hover_callback(text)
            super().mouseMoveEvent(event)

    class CameraWindow(QMainWindow):
        def __init__(self):
            super().__init__()
            self.setWindowTitle('Robot Camera and Maps')
            self.resize(1200, 900)
            self.last_frame = None
            self.viewer_state = {}
            self.current_header = None
            self.state_file = MapImageFile(args.viewer_state) if args.viewer_state else None
            root = QWidget()
            layout = QVBoxLayout(root)
            self.status = QLabel(format_viewer_status(self.viewer_state))
            panels = QGridLayout()
            self.rgb = ImagePanel('Waiting for RGB camera...')
            self.depth = ImagePanel('Waiting for depth camera...', self.set_hover)
            self.top_down = ImagePanel('Waiting for top_down_map.png...')
            self.planned = ImagePanel('Waiting for planned_map.png...')
            for row, col, title, panel in (
                    (0, 0, 'RGB Camera', self.rgb),
                    (0, 1, 'Depth Camera', self.depth),
                    (1, 0, 'Map (Might not be accurate to real world)', self.top_down),
                    (1, 1, "Robot's view", self.planned)):
                column = QVBoxLayout()
                column.addWidget(QLabel(title))
                column.addWidget(panel, 1)
                panels.addLayout(column, row, col)
            layout.addLayout(panels, 1)
            layout.addWidget(self.status)
            self.hover = QLabel('Move the pointer over the depth image to read metres.')
            layout.addWidget(self.hover)
            quit_button = QPushButton('Quit viewer')
            quit_button.clicked.connect(self.close)
            layout.addWidget(quit_button)
            self.setCentralWidget(root)
            self.timer = QTimer(self)
            self.timer.timeout.connect(self.update_camera)
            self.timer.start(33)
            self.map_files = (
                (self.top_down, MapImageFile(args.top_down_map)),
                (self.planned, MapImageFile(args.planned_map)),
            )
            self.map_timer = QTimer(self)
            self.map_timer.timeout.connect(self.update_maps)
            self.map_timer.timeout.connect(self.update_viewer_state)
            self.map_timer.start(250)
            self.update_viewer_state()
            self.update_maps()

        def update_viewer_state(self):
            if self.state_file is None:
                return
            result = self.state_file.read_if_changed()
            if result is None:
                return
            data, stamp = result
            try:
                state = json.loads(data)
                if (not isinstance(state, dict) or not isinstance(state.get('scene'), dict)
                        or not isinstance(state.get('marked_goals'), list)):
                    return
                text = format_viewer_status(state, self.current_header)
            except (ValueError, TypeError, KeyError):
                return
            self.viewer_state = state
            self.status.setText(text)
            self.state_file.mark_loaded(stamp)

        def update_maps(self):
            for panel, file in self.map_files:
                result = file.read_if_changed()
                if result is None:
                    continue
                data, stamp = result
                image = QImage.fromData(data)
                if image.isNull():
                    continue
                panel.image = image
                panel.render()
                file.mark_loaded(stamp)

        def set_hover(self, text):
            self.hover.setText(text)

        def update_camera(self):
            if args.stop_file and Path(args.stop_file).exists():
                self.close()
                return
            packet, error, _ = stream.snapshot()
            if packet is not None:
                self.current_header = packet.header
            self.status.setText(format_viewer_status(self.viewer_state, self.current_header))
            if error:
                self.statusBar().showMessage(f'Camera error: {error}')
                return
            if packet is None:
                return
            header = packet.header
            key = packet.segment, header['seq'], header['t']
            if key == self.last_frame:
                return
            self.last_frame = key
            try:
                rgb, raw_depth, metres = decode_camera_packet(packet, camera_info)
                self.rgb.set_array(rgb)
                self.depth.set_array(depth_preview(raw_depth, camera_info), metres)
                self.statusBar().clearMessage()
            except (ValueError, OSError) as exc:
                self.statusBar().showMessage(f'Could not decode camera frame: {exc}')

        def closeEvent(self, event):
            self.timer.stop()
            self.map_timer.stop()
            stream.stop()
            super().closeEvent(event)

    app = QApplication(sys.argv[:1])
    window = CameraWindow()
    window.show()
    previous_sigint = signal.signal(signal.SIGINT, lambda *_: window.close())
    try:
        return app.exec()
    finally:
        signal.signal(signal.SIGINT, previous_sigint)
        stream.stop()


if __name__ == '__main__':
    raise SystemExit(main())
