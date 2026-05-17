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