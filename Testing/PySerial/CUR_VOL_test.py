import serial
import serial.tools.list_ports
import time
import os
import re

BAUDRATE = 115200

line_pattern = re.compile(
    r"Motor_V:\s*([-+]?\d*\.?\d+)\s*V\s*\|\s*"
    r"Current_sensor_V:\s*([-+]?\d*\.?\d+)\s*V\s*\|\s*"
    r"Current:\s*([-+]?\d*\.?\d+)\s*A"
)

def clear():
    os.system("cls" if os.name == "nt" else "clear")

def bar(value, max_value, width=20):
    filled = int((abs(value) / max_value) * width) if max_value else 0
    filled = max(0, min(filled, width))
    return "[" + "#" * filled + "-" * (width - filled) + "]"

print("Available serial ports:")
for p in serial.tools.list_ports.comports():
    print(p.device, "-", p.description)

port = input("\nport: ")

ser = serial.Serial(port, BAUDRATE, timeout=1)
time.sleep(2)

print("\033[?25l", end="", flush=True)

try:
    while True:
        line = ser.readline().decode(errors="ignore").strip()

        if not line:
            continue

        match = line_pattern.search(line)

        clear()

        print("=" * 60)
        print("DAQ TEENSY LIVE OUTPUT")
        print("=" * 60)

        if match:
            motor_v = float(match.group(1))
            sensor_v = float(match.group(2))
            current_a = float(match.group(3))

            print(f"\nMotor Voltage:          {motor_v:8.3f} V  {bar(motor_v, 20)}")
            print(f"Current Sensor Voltage: {sensor_v:8.3f} V")
            print(f"Current:                {current_a:8.3f} A  {bar(current_a, 10)}")

        else:
            print("\nRaw line:")
            print(line)

        print("\n" + "=" * 60)
        time.sleep(0.1)

except KeyboardInterrupt:
    pass

finally:
    print("\033[?25h", end="", flush=True)
    ser.close()