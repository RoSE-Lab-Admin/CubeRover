import time
import pandas as pd
import numpy as np
import serial.tools.list_ports
from pathlib import Path
from test_engine.test_engine import TestEngine

MOTORS = [
    {"id": "fl", "name": "Front Left", "dash": "Solid"},
    {"id": "fr", "name": "Front Right", "dash": "Dash"},
    {"id": "bl", "name": "Back Left", "dash": "Dot"},
    {"id": "br", "name": "Back Right", "dash": "DashDot"}
]

MOTOR_SENSORS = [
    {"id": "pwm", "name": "Motor PWM (%)", "color": "#ef4444"},   # Red
    {"id": "rpm", "name": "Motor Speed (RPM)", "color": "#10b981"}, # Green
    {"id": "curr", "name": "Motor Current (A)", "color": "#3b82f6"},# Blue
    {"id": "enc", "name": "Encoder Count", "color": "#8b5cf6"}      # Purple (New!)
]

MAX_LIVE_POINTS = 600

# Create separate dictionaries for the Analysis Dropdowns
analysis_motor_options = {m['id']: m['name'] for m in MOTORS}

# Add motor sensors, plus the two global voltages
analysis_sensor_options = {s['id']: s['name'] for s in MOTOR_SENSORS}
analysis_sensor_options['volt1'] = 'Bus Voltage 1 (V)'
analysis_sensor_options['volt2'] = 'Bus Voltage 2 (V)'

# Build a dictionary mapping DataFrame columns to pretty UI labels
analysis_options = {
    'volt1': 'Bus Voltage 1 (V)',
    'volt2': 'Bus Voltage 2 (V)'
}
for m in MOTORS:
    for s in MOTOR_SENSORS:
        # e.g., 'fl_pwm': 'Front Left Motor PWM (%)'
        analysis_options[f"{m['id']}_{s['id']}"] = f"{m['name']} {s['name']}"
all_metric_keys = list(analysis_options.keys())

motor_switches = {}
sensor_switches = {}

daq = None
is_running = False
start_time = None

global_history = [] 

live_reference_df = pd.DataFrame()
analysis_df_A = pd.DataFrame()
analysis_df_B = pd.DataFrame()
using_live_for_A = False

engine_task = None
current_dir = Path(__file__).parent.parent.resolve()
engine = TestEngine(hardware_interface=None, base_path=current_dir)

def calculate_mape(df_A, df_B, metric):
    try:
        t_A = df_A['Seconds'].values
        val_A = df_A[metric].values
        t_B = df_B['Seconds'].values
        val_B = df_B[metric].values
        
        if len(t_A) == 0 or len(t_B) == 0: return "N/A"
        
        interp_B = np.interp(t_A, t_B, val_B)
        safe_A = np.where(np.abs(val_A) < 0.001, 0.001, val_A) 
        
        mape = np.mean(np.abs((val_A - interp_B) / safe_A)) * 100
        return f"{mape:.1f}%"
    except Exception:
        return "N/A"
    
def find_teensy_port():
    """Attempts to auto-detect a connected Teensy microcontroller."""
    ports = serial.tools.list_ports.comports()
    
    # Pass 1: Look for the official Teensy Vendor ID (16C0)
    for p in ports:
        if p.vid == 0x16C0:  # 16C0 is the hex Vendor ID for PJRC / Teensy
            print(f"✅ Auto-detected Teensy on {p.device} (VID: 16C0)")
            return p.device

    # Pass 2: Look for generic USB Serial descriptions or Linux ttyACM
    for p in ports:
        if "USB Serial" in p.description or "ttyACM" in p.device:
            print(f"⚠️ Guessed Teensy on {p.device} based on description ({p.description})")
            return p.device
            
    return None

def start_daq(is_mock=False, target_port=None):
    global daq
    try:
        if is_mock:
            from PySerial.mock_teensy_telemetry import MockTeensyTelemetryReader as TelemetryReader
            print("🚀 STARTED IN MOCK MODE")
            daq = TelemetryReader(port="MOCK", baud=115200)
        else:
            from PySerial.teensy_telemetry import TeensyTelemetryReader as TelemetryReader 
            if not target_port:
                target_port = find_teensy_port()
            
            if not target_port:
                print("❌ ERROR: Could not find Teensy.")
                return

            print(f"🔌 STARTED IN HARDWARE MODE on {target_port}")
            daq = TelemetryReader(port=target_port, baud=115200)

        daq.start()
    except Exception as e:
        print(f"🚨 CRITICAL ERROR IN START_DAQ: {e}")