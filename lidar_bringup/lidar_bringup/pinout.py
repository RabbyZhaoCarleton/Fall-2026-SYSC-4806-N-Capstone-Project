"""Centralised hardware pinout for the maze robot.

SINGLE SOURCE OF TRUTH: every pin used anywhere in the codebase is looked up here by a
logical name (e.g. "MOTOR_FL_PWM"), never by a raw number. When the final layout is
decided (Raspberry Pi 5 header, ESP32, or another microcontroller) edit ONLY this file.

Each pin is tagged with its required type, e.g. "PWM Output", "Digital Input",
"Analog Input", "Digital Output". The mock backend (mock_hardware.py) refuses to use a
pin for a function that does not match its tag, catching wiring/config bugs early.

!!! ALL PIN NUMBERS BELOW ARE PLACEHOLDERS / SUGGESTIONS. Modify them to match your final
!!! Raspberry Pi 5 or external-microcontroller wiring. Architecture follows the proposal:
!!!   ESP32  -> motors (Cytron MDD10A), encoders, IMU, battery sense, E-stop
!!!   RPi 5  -> PCA9685 servo driver (I2C), status LED, spare E-stop mirror
"""
import sys
from enum import Enum
from typing import Dict, List


class PinType(str, Enum):
    """Allowed pin function tags (string-valued so they read nicely in dumps/logs)."""
    PWM_OUTPUT = "PWM Output"
    DIGITAL_OUTPUT = "Digital Output"
    DIGITAL_INPUT = "Digital Input"
    ANALOG_INPUT = "Analog Input"
    I2C_SDA = "I2C SDA"
    I2C_SCL = "I2C SCL"
    UART_TX = "UART TX"
    UART_RX = "UART RX"


OUTPUT_TYPES = {PinType.PWM_OUTPUT, PinType.DIGITAL_OUTPUT}


def _pin(number: int, ptype: PinType, note: str = "") -> dict:
    return {"pin": number, "type": ptype.value, "note": note}


# --------------------------------------------------------------------------------------
# ESP32 DevKit (GPIO numbers). Constraints are enforced by validate_pinout() below.
#   * GPIO 34,35,36,39 are INPUT-ONLY (no output driver, no pull-ups)
#   * GPIO 6-11 are wired to flash - never use
#   * GPIO 0,2,5,12,15 are strapping pins - avoid or use with care
#   * ADC2 pins (0,2,4,12-15,25-27) cannot be read while Wi-Fi is active; use ADC1
#     (32-39) for analog sensing
#   * GPIO 16/17 are used by PSRAM on WROVER modules - change if you use one
# --------------------------------------------------------------------------------------
ESP32_PINOUT: Dict[str, dict] = {
    # --- Mecanum drive: Cytron MDD10A takes one PWM + one DIR per motor --------------
    "MOTOR_FL_PWM": _pin(25, PinType.PWM_OUTPUT, "Front-left speed"),
    "MOTOR_FR_PWM": _pin(26, PinType.PWM_OUTPUT, "Front-right speed"),
    "MOTOR_RL_PWM": _pin(27, PinType.PWM_OUTPUT, "Rear-left speed"),
    "MOTOR_RR_PWM": _pin(14, PinType.PWM_OUTPUT, "Rear-right speed"),
    "MOTOR_FL_DIR": _pin(33, PinType.DIGITAL_OUTPUT, "Front-left direction"),
    "MOTOR_FR_DIR": _pin(19, PinType.DIGITAL_OUTPUT, "Front-right direction"),
    "MOTOR_RL_DIR": _pin(18, PinType.DIGITAL_OUTPUT, "Rear-left direction"),
    "MOTOR_RR_DIR": _pin(5,  PinType.DIGITAL_OUTPUT, "Rear-right direction (strapping pin)"),

    # --- Quadrature encoders (2 channels x 4 motors). 34/35/36/39 are input-only. ----
    "ENC_FL_A": _pin(34, PinType.DIGITAL_INPUT, "Front-left encoder A"),
    "ENC_FL_B": _pin(35, PinType.DIGITAL_INPUT, "Front-left encoder B"),
    "ENC_FR_A": _pin(36, PinType.DIGITAL_INPUT, "Front-right encoder A"),
    "ENC_FR_B": _pin(39, PinType.DIGITAL_INPUT, "Front-right encoder B"),
    "ENC_RL_A": _pin(16, PinType.DIGITAL_INPUT, "Rear-left encoder A (PSRAM conflict on WROVER)"),
    "ENC_RL_B": _pin(17, PinType.DIGITAL_INPUT, "Rear-left encoder B (PSRAM conflict on WROVER)"),
    "ENC_RR_A": _pin(4,  PinType.DIGITAL_INPUT, "Rear-right encoder A"),
    "ENC_RR_B": _pin(13, PinType.DIGITAL_INPUT, "Rear-right encoder B"),

    # --- IMU (GY-BNO055) on I2C0 -----------------------------------------------------
    "IMU_SDA": _pin(21, PinType.I2C_SDA, "BNO055 SDA"),
    "IMU_SCL": _pin(22, PinType.I2C_SCL, "BNO055 SCL"),

    # --- Power / safety --------------------------------------------------------------
    "BATTERY_SENSE": _pin(32, PinType.ANALOG_INPUT, "Battery voltage divider (ADC1)"),
    "ESTOP_SWITCH": _pin(23, PinType.DIGITAL_INPUT, "Emergency stop (active-low, pull-up)"),
    "STATUS_LED": _pin(2, PinType.DIGITAL_OUTPUT, "On-board LED (strapping pin)"),
}

# --------------------------------------------------------------------------------------
# Raspberry Pi 5 40-pin header (BCM / GPIO numbers). The LiDAR is USB, so it uses no pin.
# NOTE: RPi.GPIO does NOT work on Pi 5 (RP1 I/O chip). Use gpiozero + lgpio / libgpiod.
# Hardware PWM on Pi 5: add `dtoverlay=pwm-2chan` to /boot/firmware/config.txt
#   (default pins GPIO12/13; alt GPIO18/19).
# --------------------------------------------------------------------------------------
RPI5_PINOUT: Dict[str, dict] = {
    "PCA9685_SDA": _pin(2, PinType.I2C_SDA, "I2C1 SDA -> PCA9685 servo driver (physical pin 3)"),
    "PCA9685_SCL": _pin(3, PinType.I2C_SCL, "I2C1 SCL -> PCA9685 servo driver (physical pin 5)"),
    "PCA9685_OE": _pin(22, PinType.DIGITAL_OUTPUT, "PCA9685 output-enable (cut servo drive)"),
    "ESTOP_MIRROR": _pin(17, PinType.DIGITAL_INPUT, "Mirror of E-stop for software stop"),
    "STATUS_LED": _pin(27, PinType.DIGITAL_OUTPUT, "Heartbeat / fault LED"),
    "SPARE_PWM": _pin(12, PinType.PWM_OUTPUT, "Hardware PWM0 (needs pwm-2chan overlay)"),
    "ESP32_UART_TX": _pin(14, PinType.UART_TX, "Optional UART link to ESP32 (instead of USB)"),
    "ESP32_UART_RX": _pin(15, PinType.UART_RX, "Optional UART link to ESP32 (instead of USB)"),
}

# Master dictionary. Add another entry (e.g. "teensy41") to support a different board.
PINOUT: Dict[str, Dict[str, dict]] = {
    "esp32": ESP32_PINOUT,
    "rpi5": RPI5_PINOUT,
}

# --------------------------------------------------------------------------------------
# Per-board electrical constraints used by validate_pinout()
# --------------------------------------------------------------------------------------
_ESP32_INPUT_ONLY = {34, 35, 36, 39}
_ESP32_FLASH = set(range(6, 12))
_ESP32_STRAPPING = {0, 2, 5, 12, 15}
_ESP32_ADC2 = {0, 2, 4, 12, 13, 14, 15, 25, 26, 27}


def get_pin(device: str, name: str) -> dict:
    """Look up a pin by logical name; raises KeyError with a helpful message."""
    try:
        return PINOUT[device][name]
    except KeyError as exc:
        raise KeyError(f"Unknown pin '{name}' on device '{device}'. "
                       f"Known: {sorted(PINOUT.get(device, {}))}") from exc


def validate_pinout() -> List[str]:
    """Static sanity check. Returns a list of 'ERROR:'/'WARN:' strings (empty = clean)."""
    problems: List[str] = []
    valid_types = {t.value for t in PinType}

    for device, pins in PINOUT.items():
        seen: Dict[tuple, str] = {}
        for name, cfg in pins.items():
            num, ptype = cfg["pin"], cfg["type"]
            if ptype not in valid_types:
                problems.append(f"ERROR: {device}.{name}: unknown pin type '{ptype}'")
            # Pins may be shared only on I2C buses (SDA/SCL appear once per bus here).
            key = (num,)
            if key in seen:
                problems.append(f"ERROR: {device}.{name}: pin {num} already used by {seen[key]}")
            seen[key] = name

            if device == "esp32":
                is_output = ptype in {t.value for t in OUTPUT_TYPES}
                if num in _ESP32_FLASH:
                    problems.append(f"ERROR: esp32.{name}: GPIO{num} is connected to flash")
                if num in _ESP32_INPUT_ONLY and is_output:
                    problems.append(f"ERROR: esp32.{name}: GPIO{num} is input-only")
                if num in _ESP32_STRAPPING:
                    problems.append(f"WARN: esp32.{name}: GPIO{num} is a strapping pin")
                if ptype == PinType.ANALOG_INPUT.value and num in _ESP32_ADC2:
                    problems.append(f"WARN: esp32.{name}: ADC2 pin unusable while Wi-Fi is on")
            if device == "rpi5" and not 0 <= num <= 27:
                problems.append(f"ERROR: rpi5.{name}: BCM GPIO{num} not on the 40-pin header")
    return problems


def dump_pinout() -> str:
    """Human-readable table of the whole pinout."""
    lines = []
    for device, pins in PINOUT.items():
        lines.append(f"== {device} ==")
        for name, cfg in pins.items():
            lines.append(f"  {name:<16} GPIO{cfg['pin']:<3} {cfg['type']:<15} {cfg['note']}")
    return "\n".join(lines)


def main() -> int:
    """CLI: `ros2 run lidar_bringup validate_pinout` (or python3 -m lidar_bringup.pinout)."""
    print(dump_pinout(), "\n")
    problems = validate_pinout()
    for p in problems:
        print(p)
    errors = [p for p in problems if p.startswith("ERROR")]
    print(f"\n{len(errors)} error(s), {len(problems) - len(errors)} warning(s)")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
