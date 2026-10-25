"""Non-Qt stdin/stdout bridge for the existing terminal navigator."""
import codecs
import os
from pathlib import Path
import queue
import signal
import subprocess
import sys
import tempfile
import threading
import time


class NavigatorProcess:
    def __init__(self, host='localhost', port_base=5555, script=None):
        self.host, self.port_base = host, port_base
        self.script = Path(script or Path(__file__).with_name('navigator_runner.py'))
        self._temporary = tempfile.TemporaryDirectory(prefix='vacuum-ui-')
        self.control_directory = Path(self._temporary.name)
        self.output = queue.Queue(maxsize=1000)
        self.commands = queue.Queue(maxsize=64)
        self.process = None
        self._done = threading.Event()
        self._reader = self._writer = None
        self._stop_thread = None
        self._stopping_at = None

    @property
    def quit_file(self):
        return self.control_directory / 'quit'

    @property
    def running(self):
        return self.process is not None and self.process.poll() is None

    def start(self):
        environment = dict(os.environ)
        environment.update(PYTHONUNBUFFERED='1', PYTHONIOENCODING='utf-8',
                           VACUUM_CAMERA_PREVIEW='0',
                           VACUUM_UI_CONTROL_DIR=str(self.control_directory))
        kwargs = ({'creationflags': subprocess.CREATE_NEW_PROCESS_GROUP}
                  if sys.platform == 'win32' else {'start_new_session': True})
        self.process = subprocess.Popen(
            [sys.executable, '-u', str(self.script), '--host', self.host,
             '--port-base', str(self.port_base)],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, env=environment, **kwargs)
        self._reader = threading.Thread(target=self._read_output, name='terminal-output', daemon=True)
        self._writer = threading.Thread(target=self._write_input, name='terminal-input', daemon=True)
        self._reader.start()
        self._writer.start()
        return self

    def _log(self, text):
        try:
            self.output.put_nowait(text)
        except queue.Full:
            # Bound log memory while the UI is busy; keep the most recent text.
            try:
                self.output.get_nowait()
            except queue.Empty:
                pass
            try:
                self.output.put_nowait('[Older terminal output omitted]\n' + text)
            except queue.Full:
                pass

    def _read_output(self):
        decoder = codecs.getincrementaldecoder('utf-8')(errors='replace')
        try:
            while True:
                chunk = self.process.stdout.read1(4096)
                if not chunk:
                    break
                self._log(decoder.decode(chunk))
            tail = decoder.decode(b'', final=True)
            if tail:
                self._log(tail)
        except (OSError, ValueError) as exc:
            self._log(f'\n[Terminal output ended: {exc}]\n')

    def _write_input(self):
        while not self._done.is_set():
            try:
                command = self.commands.get(timeout=.1)
            except queue.Empty:
                if not self.running:
                    return
                continue
            try:
                self.process.stdin.write((command + '\n').encode('utf-8'))
                self.process.stdin.flush()
            except (BrokenPipeError, OSError, ValueError):
                return

    def send(self, command):
        if not self.running:
            return False
        try:
            self.commands.put_nowait(command)
            return True
        except queue.Full:
            self._log('\n[Input queue is full; wait before sending another command]\n')
            return False

    def read_output(self):
        parts = []
        while True:
            try:
                parts.append(self.output.get_nowait())
            except queue.Empty:
                return ''.join(parts)

    def _stop_robot(self):
        # Use a separate socket owned by this worker, never the navigator's.
        try:
            import zmq
            with zmq.Context() as context:
                with context.socket(zmq.PUSH) as socket:
                    socket.setsockopt(zmq.LINGER, 200)
                    socket.setsockopt(zmq.SNDTIMEO, 500)
                    socket.connect(f'tcp://{self.host}:{self.port_base + 1}')
                    for _ in range(3):
                        socket.send_json({'type': 'wheels', 'left': 0.0, 'right': 0.0})
                        time.sleep(.05)
        except Exception as exc:
            self._log(f'\n[Could not send UI shutdown stop command: {exc}]\n')

    def request_stop(self):
        if self._stopping_at is not None:
            return
        self._stopping_at = time.monotonic()
        already_quitting = self.quit_file.exists()
        self.quit_file.touch()
        if not self.running or already_quitting:
            return
        self._stop_thread = threading.Thread(target=self._stop_robot, name='ui-stop-robot', daemon=True)
        self._stop_thread.start()
        try:
            self.process.send_signal(signal.CTRL_BREAK_EVENT if sys.platform == 'win32' else signal.SIGINT)
        except (OSError, ValueError):
            # The runner can also request KeyboardInterrupt cooperatively.
            (self.control_directory / 'interrupt').touch()
        self.send('quit')

    def poll_shutdown(self):
        if self.running and self._stopping_at is not None and time.monotonic() - self._stopping_at > 35:
            self._log('\n[Navigator did not stop within 35 seconds; terminating]\n')
            self.process.kill()
        return not self.running

    def cleanup(self):
        self.request_stop()
        if self.process is not None:
            try:
                self.process.wait(timeout=36)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        self._done.set()
        for thread in (self._reader, self._writer, self._stop_thread):
            if thread is not None:
                thread.join(timeout=2)
        if self.process is not None:
            self.process.stdin.close()
            self.process.stdout.close()
        self._temporary.cleanup()
