import threading
import time
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

    def reset_clock(self):
        """Resets the internal timer so the data ramps from 0 again."""
        self._start_time = time.time()
        # Reset the telemetry object so encoder counts drop back to 0
        with self._lock:
            self._telemetry = RoverTelemetry()

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
        last_time = time.time()
        
        while not self._stop_event.is_set():
            current_time = time.time()
            elapsed = current_time - self._start_time
            dt = current_time - last_time  # Calculate time delta for encoder integration
            last_time = current_time
            
            ramp_factor = min(elapsed / 5.0, 1.0)
            
            with self._lock:
                # 1. Define slightly different performance modifiers for each motor
                # so the traces don't perfectly overlap on the graph
                motor_configs = [
                    (self._telemetry.fl, 1.00), # 100% performance
                    (self._telemetry.bl, 0.98), # 98% performance
                    (self._telemetry.fr, 0.95), # 95% performance
                    (self._telemetry.br, 0.93), # 93% performance
                ]
                
                # 2. Update all 4 motors
                for motor, perf_mod in motor_configs:
                    # PWM
                    base_pwm = max(0, ramp_factor * 80.0 * perf_mod + random.uniform(-1, 1))
                    motor.pwm = int(base_pwm * 327.67) 
                    
                    # Velocity
                    base_vel = max(0, ramp_factor * 3000.0 * perf_mod + random.uniform(-50, 50))
                    motor.velocity = int(base_vel)
                    
                    # Current (10mA units)
                    amps = max(0.1, ramp_factor * 15.0 * perf_mod + random.uniform(-0.5, 0.5))
                    motor.current = int(amps * 100) 
                    
                    # Encoder (Integral of velocity over time: counts = velocity * dt)
                    motor.encoder_count += int(base_vel * dt)
                
                # 3. Update both battery voltages (10mV units)
                volts_1 = max(0.1, 24.0 - (ramp_factor * 1.2) + random.uniform(-0.1, 0.1))
                self._telemetry.battery_voltage_1 = int(volts_1 * 100)
                
                # Make battery 2 sag slightly more just for visual difference
                volts_2 = max(0.1, 24.0 - (ramp_factor * 1.5) + random.uniform(-0.1, 0.1))
                self._telemetry.battery_voltage_2 = int(volts_2 * 100)
                
                self._telemetry.timestamp = time.monotonic()
                
            # Simulate a 100Hz hardware update rate
            time.sleep(0.01)