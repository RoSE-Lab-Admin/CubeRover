# test_gui/state.py
import pandas as pd
from pathlib import Path
from test_engine.test_engine import TestEngine

from test_gui.custom_types import GlobalHistory

# --- Hardware / DAQ Pointers ---
daq = None
is_running = False
start_time = None

# --- Live Data ---
global_history: GlobalHistory = [] 
live_reference_df = pd.DataFrame()

# --- Analysis Data ---
analysis_df_A = pd.DataFrame()
analysis_df_B = pd.DataFrame()
using_live_for_A = False

# --- Test Engine ---
engine_task = None
current_dir = Path(__file__).parent.parent.resolve()
engine = TestEngine(hardware_interface=None, base_path=current_dir)