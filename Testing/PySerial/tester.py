from teensy_telemetry import TeensyTelemetryReader
import serial.tools.list_ports
import os
import time

BATTERY_SCALE = 0.1   # Roboclaw reports in 10mV units
CURRENT_SCALE = 0.01  # Roboclaw reports in 10mA units

def clear():
    # Move cursor to top-left without clearing
    os.system('cls' if os.name == 'nt' else 'clear')

def bar(value, max_value, width=20, fill='#', empty='-'):
    filled = int((value / max_value) * width) if max_value else 0
    filled = max(0, min(filled, width))
    return f"[{fill * filled}{empty * (width - filled)}]"

def print_motor(label, motor, col_width=30):
    enc   = f"{motor.encoder_count:>10,}"
    vel   = f"{motor.velocity:>8} cnt/s"
    cur   = f"{motor.current * CURRENT_SCALE:>6.2f} A"
    cur_bar = bar(abs(motor.current * CURRENT_SCALE), 10.0, width=15)
    print(f"  {label:<6} enc: {enc}  vel: {vel}  cur: {cur} {cur_bar}")

def print_telemetry(t, error: str = ""):
    clear()
    bv1 = t.battery_voltage_1 * BATTERY_SCALE
    bv2 = t.battery_voltage_2 * BATTERY_SCALE
    age = time.monotonic() - t.timestamp

    print("=" * 72)
    if error:
        print(f"  *** ERROR: {error:<57}")
    else:
        print(f"  ROBOT TELEMETRY                               age: {age:5.2f}s")
    print("=" * 72)

    print("\n  -- Motors (left) --")
    print_motor("FL", t.fl)
    print_motor("BL", t.bl)

    print("\n  -- Motors (right) --")
    print_motor("FR", t.fr)
    print_motor("BR", t.br)

    print("\n  -- Battery --")
    print(f"  Controller 1: {bv1:5.1f} V  {bar(bv1, 17.0, width=30)}")
    print(f"  Controller 2: {bv2:5.1f} V  {bar(bv2, 17.0, width=30)}")

    print("\n" + "=" * 72)

for p in serial.tools.list_ports.comports():
    print(p.device, p.description)

port = input("port: ")
reader = TeensyTelemetryReader(port=port, baud=115200)
reader.start()

# Clear once at startup, then hide cursor for clean updates
os.system('cls' if os.name == 'nt' else 'clear')
print('\033[?25l', end='', flush=True)  # hide cursor

try:
    while True:
        print_telemetry(reader.telemetry, error=reader.last_error)
        time.sleep(0.1)
except KeyboardInterrupt:
    pass
finally:
    print('\033[?25h', end='', flush=True)  # restore cursor on exit
    reader.stop()