from typing import TypedDict, Literal

# --- Types ---
NotifyType = Literal['positive', 'negative', 'warning', 'info', 'ongoing']

class ColorDict(TypedDict):
    volt1: str
    volt2: str
    ref_ghost: str
    positive: NotifyType
    negative: NotifyType
    warning: NotifyType

# --- Hardware Scaling Factors ---
BATTERY_SCALE = 0.1
CURRENT_SCALE = 0.01
PWM_SCALE = 1 / 327.67

# --- Global UI Colors ---
# 3. Apply the type hint to your dictionary
COLORS: ColorDict = {
    'volt1': '#f59e0b',        # Amber
    'volt2': '#d97706',        # Dark Amber
    'ref_ghost': '#64748b',    # Slate Gray
    'positive': 'positive',    # NiceGUI standard green
    'negative': 'negative',    # NiceGUI standard red
    'warning': 'warning'       # NiceGUI standard yellow
}

DEFAULT_CSV_PREFIX = "robot_run_"
GUI_TITLE = "Flat Rosey Test UI"

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
ANALYSIS_MOTOR_OPTIONS = {m['id']: m['name'] for m in MOTORS}

# Add motor sensors, plus the two global voltages
ANALYSIS_SENSOR_OPTIONS = {s['id']: s['name'] for s in MOTOR_SENSORS}
ANALYSIS_SENSOR_OPTIONS['volt1'] = 'Bus Voltage 1 (V)'
ANALYSIS_SENSOR_OPTIONS['volt2'] = 'Bus Voltage 2 (V)'

# Build a dictionary mapping DataFrame columns to pretty UI labels
ANALYSIS_OPTIONS = {
    'volt1': 'Bus Voltage 1 (V)',
    'volt2': 'Bus Voltage 2 (V)'
}
for m in MOTORS:
    for s in MOTOR_SENSORS:
        # e.g., 'fl_pwm': 'Front Left Motor PWM (%)'
        ANALYSIS_OPTIONS[f"{m['id']}_{s['id']}"] = f"{m['name']} {s['name']}"
all_metric_keys = list(ANALYSIS_OPTIONS.keys())