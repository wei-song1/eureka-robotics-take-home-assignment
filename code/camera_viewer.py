"""Standalone Qt RGB/depth viewer; closing it does not stop robot navigation."""
import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np

from camera_stream import CameraStream, decode_camera_packet, depth_preview


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
    args = parser.parse_args()
    try:
        from PySide6.QtCore import Qt, QTimer
        from PySide6.QtGui import QImage, QPixmap
        from PySide6.QtWidgets import QApplication, QLabel, QMainWindow, QWidget, QHBoxLayout, QVBoxLayout
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
            self.setWindowTitle('Robot RGBD Camera')
            self.resize(1100, 500)
            self.last_frame = None
            root = QWidget()
            layout = QVBoxLayout(root)
            panels = QHBoxLayout()
            self.rgb = ImagePanel('Waiting for RGB camera...')
            self.depth = ImagePanel('Waiting for depth camera...', self.set_hover)
            for title, panel in (('RGB', self.rgb), ('Depth: near = white, far / invalid = black', self.depth)):
                column = QVBoxLayout()
                column.addWidget(QLabel(title))
                column.addWidget(panel, 1)
                panels.addLayout(column, 1)
            layout.addLayout(panels, 1)
            self.status = QLabel('Waiting for camera messages...')
            self.hover = QLabel('Move the pointer over the depth image to read metres.')
            layout.addWidget(self.status)
            layout.addWidget(self.hover)
            self.setCentralWidget(root)
            self.timer = QTimer(self)
            self.timer.timeout.connect(self.update_camera)
            self.timer.start(33)

        def set_hover(self, text):
            self.hover.setText(text)

        def update_camera(self):
            if args.stop_file and Path(args.stop_file).exists():
                self.close()
                return
            packet, error, gaps = stream.snapshot()
            if error:
                self.status.setText(f'Camera error: {error}')
                return
            if packet is None:
                return
            header = packet.header
            age = (time.monotonic_ns() - packet.received_monotonic_ns) / 1e9
            pose = header['pose']
            self.status.setText(
                f"t={header['t']:.3f}s | seq={header['seq']} | "
                f"x={pose['x']:.2f} m, y={pose['y']:.2f} m | "
                f"frame age={age:.2f}s | sequence gaps={gaps}")
            key = packet.segment, header['seq'], header['t']
            if key == self.last_frame:
                return
            self.last_frame = key
            try:
                rgb, raw_depth, metres = decode_camera_packet(packet, camera_info)
                self.rgb.set_array(rgb)
                self.depth.set_array(depth_preview(raw_depth, camera_info), metres)
            except (ValueError, OSError) as exc:
                self.status.setText(f'Could not decode camera frame: {exc}')

        def closeEvent(self, event):
            self.timer.stop()
            stream.stop()
            super().closeEvent(event)

    app = QApplication(sys.argv[:1])
    window = CameraWindow()
    window.show()
    try:
        return app.exec()
    finally:
        stream.stop()


if __name__ == '__main__':
    raise SystemExit(main())
