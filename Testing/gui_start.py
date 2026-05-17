from nicegui import app, ui
import argparse
from test_gui import state
from test_gui.constants import GUI_TITLE
from test_gui.hardware_manager import start_daq

# 1. Parse Command Line Arguments
parser = argparse.ArgumentParser(description=GUI_TITLE)
parser.add_argument('--mock', action='store_true')
parser.add_argument('--port', type=str, default=None)
args, unknown = parser.parse_known_args()

# 2. Wire up Startup / Shutdown Hooks
app.on_startup(lambda: start_daq(is_mock=args.mock, target_port=args.port))
app.on_shutdown(lambda: state.daq.stop() if state.daq else None)

# 3. Build the UI
# Just importing the frontend module executes its layout code and builds the page!
from test_gui import frontend

# 4. Start the Server
if __name__ == "__main__":
    print("🚀 Firing up NiceGUI...")
    
if __name__ in {"__main__", "__mp_main__"}:
    ui.run(title=GUI_TITLE)