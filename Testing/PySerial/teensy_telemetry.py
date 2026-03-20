import serial
import threading
from dataclasses import dataclass, field
from typing import Optional
import time
import logging

logger = logging.getLogger(__name__)


@dataclass
class MotorTelemetry:
    """Telemetry for a single motor."""
    encoder_count: int = 0
    velocity: int = 0 # encoder counts/sec
    current: int = 0 # 10mA units


@dataclass
class RoverTelemetry:
    """Full rover telemetry parsed from a single frame."""
    fl: MotorTelemetry = field(default_factory=MotorTelemetry)
    bl: MotorTelemetry = field(default_factory=MotorTelemetry)
    fr: MotorTelemetry = field(default_factory=MotorTelemetry)
    br: MotorTelemetry = field(default_factory=MotorTelemetry)
    battery_voltage_1: int = 0  # 10mV units
    battery_voltage_2: int = 0
    timestamp: float = field(default_factory=time.monotonic)


class TeensyTelemetryReader:
    """
    Reads telemetry frames from a Teensy over serial and exposes the
    latest values as a RoverTelemetry instance.

    Frame format:
        e <enc1> <enc2> <enc3> <enc4>
          <spd1> <spd2> <spd3> <spd4>
          <cur1> <cur2> <cur3> <cur4>
          <vbat1> <vbat2>

    Motor index mapping matches the RoSEy Teensy get_telemetry() order:
        [0] FL enc   [1] BL enc   [2] FR enc   [3] BR enc
        [4] FL spd   [5] BL spd   [6] FR spd   [7] BR spd
        [8] FL cur   [9] BL cur   [10] FR cur  [11] BR cur
        [12] vbat1   [13] vbat2
    """

    FRAME_TOKEN = 'e'
    EXPECTED_FIELDS = 14

    def __init__(self, port: str, baud: int = 115200, timeout: float = 1.0):
        self._port = port
        self._baud = baud
        self._timeout = timeout
        self._last_error: str = ""

        self._serial: Optional[serial.Serial] = None
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._stop_event = threading.Event()

        # Latest parsed telemetry
        self._telemetry = RoverTelemetry()

    def start(self) -> None:
        """Open the serial port and begin the background reader thread."""
        self._serial = serial.Serial(
            port=self._port,
            baudrate=self._baud,
            timeout=self._timeout,
        )
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._read_loop,
            name="TeensyTelemetryReader",
            daemon=True,
        )
        self._thread.start()
        logger.info("Telemetry reader started on %s @ %d baud", self._port, self._baud)

    def stop(self) -> None:
        """Signal the reader thread to stop and close the serial port."""
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._serial and self._serial.is_open:
            self._serial.close()
        logger.info("Telemetry reader stopped")

    @property
    def telemetry(self) -> RoverTelemetry:
        """
        Return a snapshot of the most recently parsed telemetry frame.
        Thread-safe -- callers receive a shallow copy.
        """
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
        with self._lock:
            return self._last_error

    def _read_loop(self) -> None:
        """Background thread: read lines and parse valid telemetry frames."""
        while not self._stop_event.is_set():
            try:
                raw = self._serial.readline()
                if not raw:
                    continue
                self._parse_frame(raw.decode('ascii', errors='replace').strip())
            except serial.SerialException as exc:
                logger.error("Serial error: %s", exc)
                break
            except Exception as exc:
                logger.warning("Unexpected error in read loop: %s", exc)

    def _parse_frame(self, line: str) -> None:
        """
        Parse one telemetry line and update self._telemetry.
        Discards malformed frames.
        """
        parts = line.split()

        if not parts or parts[0] != self.FRAME_TOKEN:
            if parts:
                with self._lock:
                    self._last_error = f"Bad token: '{parts[0]}'"
            return

        values = parts[1:]
        if len(values) != self.EXPECTED_FIELDS:
            with self._lock:
                self._last_error = f"Wrong field count: got {len(values)}, expected {self.EXPECTED_FIELDS}"
            return

        try:
            d = [int(v) for v in values]
        except ValueError:
            with self._lock:
                self._last_error = f"Non-integer field in frame: {line}"
            return

        with self._lock:
            self._last_error = ""   # clear error on successful parse
            self._telemetry.fl = MotorTelemetry(
                encoder_count=d[0], velocity=d[4], current=d[8]
            )
            self._telemetry.bl = MotorTelemetry(
                encoder_count=d[1], velocity=d[5], current=d[9]
            )
            self._telemetry.fr = MotorTelemetry(
                encoder_count=d[2], velocity=d[6], current=d[10]
            )
            self._telemetry.br = MotorTelemetry(
                encoder_count=d[3], velocity=d[7], current=d[11]
            )
            self._telemetry.battery_voltage_1 = d[12]
            self._telemetry.battery_voltage_2 = d[13]
            self._telemetry.timestamp = time.monotonic()