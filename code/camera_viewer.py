"""Standalone Qt RGB/depth viewer; closing it does not stop robot navigation."""
import argparse
import json
from pathlib import Path
import signal
import sys

import numpy as np

from camera_stream import CameraStream, decode_camera_packet, depth_preview
from terminal_bridge import NavigatorProcess
from camera_display import format_viewer_status


class MapImageFile:
    """Retry incomplete PNG writes and keep the last successfully shown image."""
    def __init__(self, path):
        self.path = Path(path)
        self.loaded_stamp = None

    def read_if_changed(self):
        try:
            # The size check avoids reading a file while another process replaces it.
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


def main(navigate=False):
    parser = argparse.ArgumentParser(description='Live RGBD camera viewer')
    parser.add_argument('--host', default='localhost')
    parser.add_argument('--port-base', type=int, default=5555)
    parser.add_argument('--endpoint')
    parser.add_argument('--camera-info')
    parser.add_argument('--stop-file')
    parser.add_argument('--planned-map', default='planned_map.png')
    parser.add_argument('--top-down-map', default='top_down_map.png')
    parser.add_argument('--viewer-state')
    parser.add_argument('--terminal', action='store_true', help='Run navigator in the UI terminal')
    args = parser.parse_args()
    try:
        from PySide6.QtCore import Qt, QTimer
        from PySide6.QtGui import QImage, QPixmap, QTextCursor, QFontDatabase, QShortcut, QKeySequence
        from PySide6.QtWidgets import QApplication, QLabel, QMainWindow, QWidget, QGridLayout, QVBoxLayout, QPushButton, QGroupBox, QPlainTextEdit, QLineEdit, QHBoxLayout, QSplitter
    except ImportError as exc:
        raise SystemExit('Install PySide6 to use the camera viewer: pip install PySide6') from exc

    navigate = navigate or args.terminal
    bridge = NavigatorProcess(args.host, args.port_base) if navigate else None
    camera_info = None
    if not navigate and args.camera_info:
        camera_info = json.loads(Path(args.camera_info).read_text(encoding='utf-8'))
    if not navigate and camera_info is None:
        camera_info = load_camera_info(args.host, args.port_base)
    endpoint = args.endpoint or f'tcp://{args.host}:{args.port_base + 3}'
    stream = CameraStream(endpoint, camera_info).start() if camera_info is not None else None

    class ImagePanel(QLabel):
        def __init__(self, title, hover_callback=None):
            super().__init__(title)
            self.setMinimumSize(160, 100)
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
            self.camera_info = camera_info
            self.stream = stream
            self.bridge = bridge
            self._closing = False
            self._reported_exit = False
            self.connection_file = (MapImageFile(bridge.control_directory / 'connection.json')
                                    if bridge else None)
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
            image_area = QWidget()
            image_area.setLayout(panels)
            vertical = QSplitter(Qt.Orientation.Vertical)
            vertical.setChildrenCollapsible(False)
            vertical.setHandleWidth(8)
            vertical.setStyleSheet('QSplitter::handle { background: #7a8591; }')
            vertical.addWidget(image_area)
            bottom = QSplitter(Qt.Orientation.Horizontal)
            bottom.setChildrenCollapsible(False)
            terminal_group = QGroupBox('Terminal')
            terminal_layout = QVBoxLayout(terminal_group)
            self.terminal_output = QPlainTextEdit()
            self.terminal_output.setReadOnly(True)
            self.terminal_output.setMaximumBlockCount(5000)
            self.terminal_output.setFont(QFontDatabase.systemFont(QFontDatabase.SystemFont.FixedFont))
            terminal_layout.addWidget(self.terminal_output, 1)
            input_row = QHBoxLayout()
            self.terminal_input = QLineEdit()
            self.terminal_input.setMaxLength(1024)
            self.terminal_input.setPlaceholderText('Type a selection or command, then press Enter')
            self.terminal_input.returnPressed.connect(self.send_command)
            input_row.addWidget(self.terminal_input, 1)
            send = QPushButton('Send')
            send.clicked.connect(self.send_command)
            input_row.addWidget(send)
            terminal_layout.addLayout(input_row)
            self.interrupt_shortcut = QShortcut(QKeySequence('Ctrl+C'), terminal_group)
            self.interrupt_shortcut.setContext(Qt.ShortcutContext.WidgetWithChildrenShortcut)
            self.interrupt_shortcut.activated.connect(self.terminal_interrupt)
            info_group = QGroupBox('Info')
            info_layout = QVBoxLayout(info_group)
            self.status.setAlignment(Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft)
            info_layout.addWidget(self.status)
            self.hover = QLabel('Move the pointer over the depth image to read metres.')
            self.hover.setWordWrap(True)
            info_layout.addWidget(self.hover)
            info_layout.addStretch(1)
            bottom.addWidget(terminal_group)
            bottom.addWidget(info_group)
            bottom.setStretchFactor(0, 3)
            bottom.setStretchFactor(1, 2)
            bottom.setMinimumHeight(190)
            vertical.addWidget(bottom)
            vertical.setStretchFactor(0, 3)
            vertical.setStretchFactor(1, 1)
            vertical.setSizes([600, 220])
            layout.addWidget(vertical, 1)
            self.setCentralWidget(root)
            self.timer = QTimer(self)
            self.timer.timeout.connect(self.update_camera)
            self.timer.start(33)
            self.map_files = (
                (self.top_down, MapImageFile(args.top_down_map)),
                (self.planned, MapImageFile(args.planned_map)),
            )
            self.map_timer = QTimer(self)
            self.map_timer.timeout.connect(self.update_connection)
            self.map_timer.timeout.connect(self.update_maps)
            self.map_timer.timeout.connect(self.update_viewer_state)
            self.map_timer.start(250)
            self.update_viewer_state()
            self.update_maps()
            self.terminal_timer = QTimer(self)
            self.terminal_timer.timeout.connect(self.update_terminal)
            self.terminal_timer.start(50)
            if self.bridge:
                QTimer.singleShot(0, self.start_navigator)
            else:
                self.append_terminal('Read-only camera viewer. Run python navigator.py for the interactive terminal.\n')
                self.terminal_input.setPlaceholderText('Type quit to close this viewer')

        def append_terminal(self, text):
            if text:
                cursor = self.terminal_output.textCursor()
                cursor.movePosition(QTextCursor.MoveOperation.End)
                self.terminal_output.setTextCursor(cursor)
                self.terminal_output.insertPlainText(text)
                self.terminal_output.ensureCursorVisible()

        def start_navigator(self):
            if self._closing:
                return
            try:
                self.bridge.start()
                self.terminal_input.setFocus()
            except OSError as exc:
                self.append_terminal(f'Could not start navigator: {exc}\n')

        def send_command(self):
            command = self.terminal_input.text()
            self.terminal_input.clear()
            if command.strip().casefold() in ('quit', 'exit'):
                self.close()
                return
            if self.bridge and self.bridge.send(command):
                self.append_terminal(command + '\n')
            else:
                self.append_terminal('[Navigator is not running. Type quit to close.]\n')

        def terminal_interrupt(self):
            if self.terminal_output.hasFocus() and self.terminal_output.textCursor().hasSelection():
                self.terminal_output.copy()
            elif self.terminal_input.hasFocus() and self.terminal_input.hasSelectedText():
                self.terminal_input.copy()
            else:
                self.close()

        def update_terminal(self):
            if self.bridge is None:
                return
            self.append_terminal(self.bridge.read_output())
            if self._closing:
                if self.bridge.poll_shutdown():
                    self.close()
            elif self.bridge.quit_file.exists():
                self.close()
            elif (self.bridge.process is not None and not self.bridge.running
                  and not self._reported_exit):
                self._reported_exit = True
                self.append_terminal(f'\n[Navigator exited ({self.bridge.process.returncode}). '
                                     'The UI stays open; type quit to close.]\n')

        def update_connection(self):
            if self.connection_file is None:
                return
            result = self.connection_file.read_if_changed()
            if result is None:
                return
            data, stamp = result
            try:
                connection = json.loads(data)
                self.camera_info = connection['camera_info']
                self.state_file = MapImageFile(connection['viewer_state'])
                endpoint = connection['endpoint']
            except (ValueError, TypeError, KeyError):
                return
            self.connection_file.mark_loaded(stamp)
            self.last_frame = None
            self.current_header = None
            for panel, file in self.map_files:
                panel.image = None
                panel.setText(f'Waiting for {file.path.name}...')
                file.loaded_stamp = None
            if self.stream is not None:
                self.stream.stop()
            self.stream = CameraStream(endpoint, self.camera_info)
            try:
                self.stream.start()
            except (RuntimeError, TimeoutError) as exc:
                self.append_terminal(f'Camera preview could not start: {exc}\n')
            self.update_viewer_state()

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
            self.update_viewer_state()
            if self.viewer_state.get('phase') == 'complete':
                return
            if self.stream is None:
                return
            packet, error, _ = self.stream.snapshot()
            # A reset can arrive before the completion metadata is polled.
            # Keep the displayed capture until a new run's connection arrives.
            if (self.state_file is not None and self.last_frame is not None
                    and packet is not None and packet.segment != self.last_frame[0]):
                return
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
                rgb, raw_depth, metres = decode_camera_packet(packet, self.camera_info)
                self.rgb.set_array(rgb)
                self.depth.set_array(depth_preview(raw_depth, self.camera_info), metres)
                self.statusBar().clearMessage()
            except (ValueError, OSError) as exc:
                self.statusBar().showMessage(f'Could not decode camera frame: {exc}')

        def closeEvent(self, event):
            if self.bridge and self.bridge.running:
                event.ignore()
                if not self._closing:
                    self._closing = True
                    self.terminal_input.setEnabled(False)
                    self.append_terminal('\n[Stopping navigator and finalising recording...]\n')
                    self.bridge.request_stop()
                return
            self._closing = True
            self.timer.stop()
            self.map_timer.stop()
            self.terminal_timer.stop()
            if self.stream is not None:
                self.stream.stop()
            super().closeEvent(event)

    app = QApplication(sys.argv[:1])
    window = CameraWindow()
    window.show()
    previous_sigint = signal.signal(signal.SIGINT, lambda *_: window.close())
    try:
        return app.exec()
    finally:
        signal.signal(signal.SIGINT, previous_sigint)
        if window.stream is not None:
            window.stream.stop()
        if bridge is not None:
            bridge.cleanup()


if __name__ == '__main__':
    raise SystemExit(main())
