"""Launch the camera/maps UI with its embedded navigator terminal."""
from camera_viewer import main


if __name__ == '__main__':
    # Keep the application entry point usable from both the shell and a launcher.
    raise SystemExit(main(navigate=True))
