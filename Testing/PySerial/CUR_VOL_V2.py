import serial
import serial.tools.list_ports
import time
import os
import re
import sys
import select
import termios
import tty
import csv
from datetime import datetime




BAUDRATE = 115200

# Time calibration runs
CALIBRATION_TIME_SECONDS = 5.0

# Delay between dashboard refreshes
DISPLAY_DELAY_SECONDS = 0.1


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

def key_pressed():
    readable, _, _ = select.select([sys.stdin], [], [], 0)

    if readable:
        return sys.stdin.read(1).lower()
    
    return None

def parse_line(line):
    match = line_pattern.search(line)

    if not match:
        return None
    
    return (
        float(match.group(1)),
        float(match.group(2)),
        float(match.group(3)),
    )

def calibrate_resting_voltage(ser, duration):
    readings = []
    start_time = time.monotonic()

    # Remove any old serial data before calibration
    ser.reset_input_buffer()

    while True:
        elapsed = time.monotonic() - start_time

        if elapsed >= duration:
            break
        
        line = ser.readline().decode(errors="ignore").strip()

        if line:
            parsed = parse_line(line)

            if parsed is not None:
                _, sensor_v, _ = parsed
                readings.append(sensor_v)

        clear()

        remaining = max(0.0, duration - elapsed)

        print("=" * 60)
        print("CURRENT SENSOR RESTING_VOLTAGE CALIBRATION")
        print("+" * 60)
        print()
        print("Keep the moror stopped and ensure no current is flowing")
        print()
        print(f"Calibration duration: {duration: .1f} seconds")
        print(f"Time remaining: {remaining: .1f} seconds")
        print(f"Valid samples: {len(readings)}")

        if readings:
            current_average = sum(readings) / len(readings)
            print(f"Running average:            {current_average:.3f} V")
        else:
            print(f"Running average:            Waiting for data...")

        print()
        print("=" * 60)

    if not readings:
        return None
    
    resting_voltage = sum(readings) / len(readings)

    # Send calibration command to the TEENSY
    command = f"z {resting_voltage:.3f}\n"
    ser.write(command.encode("utf-8"))
    ser.flush()

    return resting_voltage, len(readings), command.strip()

        




print("Available serial ports:")
for p in serial.tools.list_ports.comports():
    print(p.device, "-", p.description)

port = input("\nport: ")

ser = serial.Serial(port, BAUDRATE, timeout=0.1)
time.sleep(2)

original_terminal_settings = termios.tcgetattr(sys.stdin)

tty.setcbreak(sys.stdin.fileno())

print("\033[?25l", end="", flush=True)

calibration_message = None

recording = False
csv_file = None
csv_writer = None
recording_start_time = None
recording_filename = None
samples_recorded = 0



try:
    while True:

        # Check keyboard
        key = key_pressed()

        if key == "r":
            result = calibrate_resting_voltage(
                ser,
                CALIBRATION_TIME_SECONDS
            )

            if result is None:
                calibration_message = "Calibration failed"
            else:
                resting_voltage, sample_count, command = result

                calibration_message = (
                    f"Resting voltage set to {resting_voltage:.3f} V "
                    f"using {sample_count} samples. "
                    f"sent: {command}"
                )
            ser.reset_input_buffer()
        
        elif key == "q":
            break

        elif key == "s":

            if not recording:

                # Filename based on data + time
                timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
                recording_filename = f"motor_data_{timestamp}.csv"

                csv_file = open(
                    recording_filename,
                    "w",
                    newline=""
                )

                csv_writer = csv.writer(csv_file)

                # Columns
                csv_writer.writerow([
                    "Time_s",
                    "Motor_V",
                    "Current_sensor_V",
                    "Current_A"
                ])

                recording_start_time = time.monotonic()
                samples_recorded = 0
                recording = True

            else:

                recording = False

                if csv_file is not None:
                    csv_file.close()
                    csv_file = None
                    csv_writer = None

        # Read TEENSY data
        line = ser.readline().decode(errors="ignore").strip()

        if not line:
            continue

        parsed = parse_line(line)

        clear()

        print("=" * 60)
        print("DAQ TEENSY LIVE OUTPUT")
        print("=" * 60)

        if parsed is not None:
            motor_v, sensor_v, current_a = parsed

            if recording:

                elapsed_time = time.monotonic() - recording_start_time

                csv_writer.writerow([
                    f"{elapsed_time:.4f}",
                    f"{motor_v:.3f}",
                    f"{sensor_v:.3f}",
                    f"{current_a:.3f}"
                ])

                csv_file.flush()

                samples_recorded += 1

            print(
                f"\nMotor Voltage:            "
                f"{motor_v:8.3f}V {bar(motor_v, 20)}"
            )

            print(
                f"Current Sensor Voltage: "
                f"{sensor_v:8.3f} V"
            )

            print(
                f"Current:              "
                f"{current_a:8.3f} A {bar(current_a, 10)}"
            )

        else:
            print("\nRaw line:")
            print(line)

        print()
        print("-" * 60)

        if recording:
            elapsed_recording = time.monotonic() - recording_start_time

            print("RECORDING: ON")
            print(f"File: {recording_filename}")
            print(f"Recording time: {elapsed_recording:.1f} s")
            print(f"Samples saved: {samples_recorded}")
        else:
            print("RECORDING: OFF")

        print()

        print(
            f"Press R to calibrate resting voltage for "
            f"{CALIBRATION_TIME_SECONDS:.1f} seconds."
        )
        print("Press S to start/stop recording.")
        print("Press Q to quit.")


        if calibration_message:
            print()
            print(calibration_message)

        print("=" * 60)


        time.sleep(DISPLAY_DELAY_SECONDS)


except KeyboardInterrupt:
    pass

finally:

    if csv_file is not None:
        csv_file.close()

    termios.tcsetattr(
        sys.stdin,
        termios.TCSADRAIN,
        original_terminal_settings
    )

    print("\033[?25h", end="", flush=True)
    ser.close()