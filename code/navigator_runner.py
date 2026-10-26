"""Run the existing navigator with stdin/stdout connected to the UI terminal."""
import _thread
import os
from pathlib import Path
import signal
import threading


def run():
    def interrupt(*_):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, interrupt)
    if hasattr(signal, 'SIGBREAK'):
        signal.signal(signal.SIGBREAK, interrupt)
    done = threading.Event()
    control = os.environ.get('VACUUM_UI_CONTROL_DIR')

    def watch_interrupt():
        while not done.wait(.1):
            # The UI cannot signal the child reliably on every supported platform.
            if (Path(control) / 'interrupt').exists():
                _thread.interrupt_main()
                return

    if control:
        threading.Thread(target=watch_interrupt, name='ui-interrupt', daemon=True).start()
    try:
        import navigator
        navigator.main()
    except (KeyboardInterrupt, EOFError):
        print('\nNavigator stopped.', flush=True)
        import simulator
        if hasattr(simulator, 'cmd'):
            simulator.stop_robot()
    finally:
        done.set()


if __name__ == '__main__':
    run()
