"""Mock hardware backend so control logic can be developed on a PC with no GPIO.

The mock enforces the pin-type tags from pinout.py: asking for PWM on a "Digital Input"
pin raises PinTypeError, exactly the kind of wiring bug that otherwise burns a driver.

To migrate to real hardware, implement the same HardwareBackend interface:
  * Raspberry Pi 5: gpiozero (lgpio pin factory) - NOT RPi.GPIO
  * ESP32: this class stays on the Pi side; the ESP32 firmware owns the real pins and
    you talk to it over USB serial / micro-ROS.
"""
import math
import random
from abc import ABC, abstractmethod
from typing import Dict

from .perf_monitor import FixedRateLoop, PerfTracker
from .pinout import PINOUT, PinType, get_pin


class PinTypeError(RuntimeError):
    """Raised when a pin is used for a function its tag does not allow."""


class HardwareBackend(ABC):
    """Interface every backend (mock, gpiozero, serial-to-ESP32...) must provide."""

    @abstractmethod
    def write_pwm(self, name: str, duty: float) -> None: ...
    @abstractmethod
    def write_digital(self, name: str, value: bool) -> None: ...
    @abstractmethod
    def read_digital(self, name: str) -> bool: ...
    @abstractmethod
    def read_analog(self, name: str) -> float: ...


class MockHardware(HardwareBackend):
    """In-memory backend with type checking and simulated sensor values."""

    def __init__(self, device: str):
        if device not in PINOUT:
            raise ValueError(f"Unknown device '{device}'. Known: {list(PINOUT)}")
        self.device = device
        self.state: Dict[str, float] = {name: 0.0 for name in PINOUT[device]}

    def _require(self, name: str, *allowed: PinType) -> dict:
        cfg = get_pin(self.device, name)
        if cfg["type"] not in {t.value for t in allowed}:
            raise PinTypeError(
                f"{self.device}.{name} (GPIO{cfg['pin']}) is '{cfg['type']}', "
                f"but this call needs {[t.value for t in allowed]}")
        return cfg

    def write_pwm(self, name: str, duty: float) -> None:
        """duty in [0.0, 1.0]."""
        self._require(name, PinType.PWM_OUTPUT)
        self.state[name] = max(0.0, min(1.0, duty))

    def write_digital(self, name: str, value: bool) -> None:
        self._require(name, PinType.DIGITAL_OUTPUT)
        self.state[name] = 1.0 if value else 0.0

    def read_digital(self, name: str) -> bool:
        self._require(name, PinType.DIGITAL_INPUT)
        if name.startswith("ESTOP"):
            return True                  # released (active-low switch with pull-up)
        return random.random() > 0.5     # encoder-ish noise

    def read_analog(self, name: str) -> float:
        """Returns volts. BATTERY_SENSE simulates a slowly draining 3S pack."""
        self._require(name, PinType.ANALOG_INPUT)
        return 11.1 + random.uniform(-0.05, 0.05)


def main() -> None:
    """Demo: exercise every pin through the type-checked mock at 50 Hz and print timing."""
    hw = MockHardware("esp32")
    tracker = PerfTracker("mock_motor_loop", target_hz=50)
    t = {"i": 0}

    def step() -> None:
        t["i"] += 1
        duty = 0.5 + 0.5 * math.sin(t["i"] / 10.0)
        for wheel in ("FL", "FR", "RL", "RR"):
            hw.write_pwm(f"MOTOR_{wheel}_PWM", duty)
            hw.write_digital(f"MOTOR_{wheel}_DIR", duty > 0.5)
            hw.read_digital(f"ENC_{wheel}_A")
            hw.read_digital(f"ENC_{wheel}_B")
        hw.read_analog("BATTERY_SENSE")

    FixedRateLoop(50, tracker).run(step, duration_s=2.0)
    print(tracker.report())

    # Demonstrate the type guard
    try:
        hw.write_pwm("ESTOP_SWITCH", 0.5)
    except PinTypeError as err:
        print("Type guard OK ->", err)


if __name__ == "__main__":
    main()
