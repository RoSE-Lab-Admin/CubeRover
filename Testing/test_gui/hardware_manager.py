import serial.tools.list_ports
from typing import Optional
from test_gui import state

def find_teensy_port() -> Optional[str]:
    """
    Attempts to auto-detect a connected Teensy microcontroller by scanning 
    available serial ports.
    
    Returns:
        Optional[str]: The device path (e.g., 'COM3' or '/dev/ttyACM0') if found, 
                       otherwise None.
    """
    ports = serial.tools.list_ports.comports()
    
    # Pass 1: Look for the specific Vendor ID (VID) assigned to PJRC/Teensy devices
    for p in ports:
        if p.vid == 0x16C0:
            print(f"✅ Auto-detected Teensy on {p.device} (VID: 16C0)")
            return p.device

    # Pass 2: Fallback heuristic if the VID is masked or missing but it 
    # appears to be a generic USB serial device
    for p in ports:
        if "USB Serial" in p.description or "ttyACM" in p.device:
            print(f"⚠️ Guessed Teensy on {p.device} based on description ({p.description})")
            return p.device
            
    return None

def start_daq(is_mock: bool = False, target_port: Optional[str] = None) -> None:
    """
    Initializes the Data Acquisition (DAQ) telemetry reader and binds it to the 
    global application state.
    
    Args:
        is_mock (bool): If True, loads a simulated telemetry reader instead of 
                        attempting a physical hardware connection.
        target_port (Optional[str]): A specific serial port to connect to. If None 
                                     and not in mock mode, auto-detection is attempted.
    """
    try:
        if is_mock:
            # Dynamically import the mock reader to avoid hardware serial dependencies
            from PySerial.mock_teensy_telemetry import MockTeensyTelemetryReader as TelemetryReader
            print("🚀 STARTED IN MOCK MODE")
            state.daq = TelemetryReader(port="MOCK", baud=115200)
        else:
            # Dynamically import the actual hardware serial reader
            from PySerial.teensy_telemetry import TeensyTelemetryReader as TelemetryReader 
            
            # Attempt to auto-detect the port if one wasn't explicitly provided
            if not target_port:
                target_port = find_teensy_port()
            
            # Abort if we still don't have a valid port to connect to
            if not target_port:
                print("❌ ERROR: Could not find Teensy.")
                return

            print(f"🔌 STARTED IN HARDWARE MODE on {target_port}")
            state.daq = TelemetryReader(port=target_port, baud=115200)

        # Kick off the underlying DAQ background threads/processes
        state.daq.start()
        
    except Exception as e:
        print(f"🚨 CRITICAL ERROR IN START_DAQ: {e}")