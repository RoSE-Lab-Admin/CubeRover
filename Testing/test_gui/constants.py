from typing import TypedDict, Literal, List, Dict

# --- Types ---
NotifyType = Literal['positive', 'negative', 'warning', 'info', 'ongoing']

class ColorDict(TypedDict):
    volt1: str
    volt2: str
    ref_ghost: str
    positive: NotifyType
    negative: NotifyType
    warning: NotifyType

class MotorDef(TypedDict):
    id: str
    name: str
    dash: str

class SensorDef(TypedDict):
    id: str
    name: str
    color: str

# --- Hardware Scaling Factors ---
# Used to convert raw integer telemetry from the DAQ into standard physical units.
BATTERY_SCALE: float = 0.1         # Converts raw to Volts
CURRENT_SCALE: float = 0.01        # Converts raw to Amps
PWM_SCALE: float = 1 / 327.67      # Maps 16-bit int (-32767 to 32767) to a percentage (-100 to 100)

# --- Global UI Configuration ---
COLORS: ColorDict = {
    'volt1': '#f59e0b',        # Amber
    'volt2': '#d97706',        # Dark Amber
    'ref_ghost': '#64748b',    # Slate Gray
    'positive': 'positive',    # NiceGUI standard green
    'negative': 'negative',    # NiceGUI standard red
    'warning': 'warning'       # NiceGUI standard yellow
}

DEFAULT_CSV_PREFIX: str = "robot_run_"
GUI_TITLE: str = "Flat Rosey Test UI"

# Maximum number of data points kept in memory for the live scrolling chart
MAX_LIVE_POINTS: int = 600 

# --- Hardware Definitions ---
MOTORS: List[MotorDef] = [
    {"id": "fl", "name": "Front Left", "dash": "Solid"},
    {"id": "fr", "name": "Front Right", "dash": "Dash"},
    {"id": "bl", "name": "Back Left", "dash": "Dot"},
    {"id": "br", "name": "Back Right", "dash": "DashDot"}
]

MOTOR_SENSORS: List[SensorDef] = [
    {"id": "pwm", "name": "Motor PWM (%)", "color": "#ef4444"},   # Red
    {"id": "rpm", "name": "Motor Speed (RPM)", "color": "#10b981"}, # Green
    {"id": "curr", "name": "Motor Current (A)", "color": "#3b82f6"},# Blue
    {"id": "enc", "name": "Encoder Count", "color": "#8b5cf6"}      # Purple
]

# --- Analysis Dropdown Configurations ---

# Create separate dictionaries for the Analysis Dropdowns
ANALYSIS_MOTOR_OPTIONS: Dict[str, str] = {m['id']: m['name'] for m in MOTORS}

# Add motor sensors, plus the two global voltages
ANALYSIS_SENSOR_OPTIONS: Dict[str, str] = {s['id']: s['name'] for s in MOTOR_SENSORS}
ANALYSIS_SENSOR_OPTIONS['volt1'] = 'Bus Voltage 1 (V)'
ANALYSIS_SENSOR_OPTIONS['volt2'] = 'Bus Voltage 2 (V)'

# Build a master dictionary mapping underlying DataFrame columns to pretty UI labels
ANALYSIS_OPTIONS: Dict[str, str] = {
    'volt1': 'Bus Voltage 1 (V)',
    'volt2': 'Bus Voltage 2 (V)'
}

for m in MOTORS:
    for s in MOTOR_SENSORS:
        # Generates keys like 'fl_pwm' mapped to 'Front Left Motor PWM (%)'
        ANALYSIS_OPTIONS[f"{m['id']}_{s['id']}"] = f"{m['name']} {s['name']}"

all_metric_keys: List[str] = list(ANALYSIS_OPTIONS.keys())