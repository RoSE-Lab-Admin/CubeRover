# test_gui/hardware_manager.py
import serial.tools.list_ports
from test_gui import state

def find_teensy_port():
    """Attempts to auto-detect a connected Teensy microcontroller."""
    ports = serial.tools.list_ports.comports()
    
    for p in ports:
        if p.vid == 0x16C0:
            print(f"✅ Auto-detected Teensy on {p.device} (VID: 16C0)")
            return p.device

    for p in ports:
        if "USB Serial" in p.description or "ttyACM" in p.device:
            print(f"⚠️ Guessed Teensy on {p.device} based on description ({p.description})")
            return p.device
            
    return None

def start_daq(is_mock=False, target_port=None):
    try:
        if is_mock:
            from PySerial.mock_teensy_telemetry import MockTeensyTelemetryReader as TelemetryReader
            print("🚀 STARTED IN MOCK MODE")
            state.daq = TelemetryReader(port="MOCK", baud=115200)
        else:
            from PySerial.teensy_telemetry import TeensyTelemetryReader as TelemetryReader 
            if not target_port:
                target_port = find_teensy_port()
            
            if not target_port:
                print("❌ ERROR: Could not find Teensy.")
                return

            print(f"🔌 STARTED IN HARDWARE MODE on {target_port}")
            state.daq = TelemetryReader(port=target_port, baud=115200)

        state.daq.start()
    except Exception as e:
        print(f"🚨 CRITICAL ERROR IN START_DAQ: {e}")