import threading
import time
import math
import random
from dataclasses import dataclass, field

# Reuse the exact same dataclasses from your real module
@dataclass
class MotorTelemetry:
    encoder_count: int = 0
    velocity: int = 0      # encoder counts/sec
    current: int = 0       # 10mA units
    pwm: int = 0           # pwm ratio

@dataclass
class RoverTelemetry:
    fl: MotorTelemetry = field(default_factory=MotorTelemetry)
    bl: MotorTelemetry = field(default_factory=MotorTelemetry)
    fr: MotorTelemetry = field(default_factory=MotorTelemetry)
    br: MotorTelemetry = field(default_factory=MotorTelemetry)
    battery_voltage_1: int = 0  # 10mV units
    battery_voltage_2: int = 0
    timestamp: float = field(default_factory=time.monotonic)

class MockTeensyTelemetryReader:
    """
    A drop-in replacement for TeensyTelemetryReader that generates fake
    data on a background thread to simulate hardware behavior.
    """
    def __init__(self, port: str, baud: int = 115200, timeout: float = 1.0):
        self._port = port
        self._baud = baud
        self._timeout = timeout
        self._last_error = ""

        self._thread = None
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._telemetry = RoverTelemetry()
        
        self._start_time = 0.0

    def start(self) -> None:
        self._stop_event.clear()
        self._start_time = time.time()
        self._thread = threading.Thread(target=self._fake_read_loop, daemon=True)
        self._thread.start()
        print(f"Mock DAQ started on simulated port {self._port}")

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        print("Mock DAQ stopped")

    # Mock method for simulating the ramp up
    def reset_clock(self):
        """Resets the internal timer so the data ramps from 0 again."""
        self._start_time = time.time()

    @property
    def telemetry(self) -> RoverTelemetry:
        """Thread-safe snapshot matching the real API."""
        with self._lock:
            snap = RoverTelemetry(
                fl=MotorTelemetry(**vars(self._telemetry.fl)),
                bl=MotorTelemetry(**vars(self._telemetry.bl)),
                fr=MotorTelemetry(**vars(self._telemetry.fr)),
                br=MotorTelemetry(**vars(self._telemetry.br)),
                battery_voltage_1=self._telemetry.battery_voltage_1,
                battery_voltage_2=self._telemetry.battery_voltage_2,
                timestamp=self._telemetry.timestamp,
            )
        return snap
    
    @property
    def last_error(self) -> str:
        return self._last_error

    def _fake_read_loop(self) -> None:
        """Continuously updates the telemetry object with simulated math."""
        while not self._stop_event.is_set():
            elapsed = time.time() - self._start_time
            ramp_factor = min(elapsed / 5.0, 1.0)
            
            with self._lock:
                # Simulating the Front Left (FL) motor
                self._telemetry.fl.pwm = int(max(0, ramp_factor * 80.0 + random.uniform(-1, 1)) * 327.67)
                self._telemetry.fl.velocity = int(max(0, ramp_factor * 3000.0 + random.uniform(-50, 50)))
                
                # Real hardware uses 10mA units (e.g. 15A = 1500)
                amps = max(0.1, ramp_factor * 15.0 + random.uniform(-0.5, 0.5))
                self._telemetry.fl.current = int(amps * 100) 
                
                # Real hardware uses 10mV units (e.g. 24V = 2400)
                volts = max(0.1, 24.0 - (ramp_factor * 1.2) + random.uniform(-0.1, 0.1))
                self._telemetry.battery_voltage_1 = int(volts * 100)
                
                self._telemetry.timestamp = time.monotonic()
                
            # Simulate a 100Hz hardware update rate
            time.sleep(0.01)