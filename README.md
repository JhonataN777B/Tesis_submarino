# ROV / Submarine thesis

Repository for the low-cost remotely operated vehicle (ROV): ESP32-S3 firmware, surface-control interface, hardware exports, telemetry examples, and validation evidence.

## Repository layout

- firmware/integrated: current ESP32-S3 integrated firmware.
- firmware/tests: independent Arduino sketches for sensor and actuator checks.
- interface/solo_control.py: current joystick, serial, camera HUD, telemetry, logging, and recording application.
- interface/prueba_control_mando.py: independent PG-9076/Pygame axis and button diagnostic.
- hardware: PCB files and STL models used for the printed assembly.
- data: a lightweight, illustrative telemetry sample. Runtime logs are not committed.
- docs/evidencia: PCB, design, and selected operational evidence videos.

## Current integration

The firmware streams telemetry at 10 Hz over 115200-baud serial and accepts text commands on the same port. It reads the ACS712, battery divider, MS5837 pressure sensor, and BNO055/SEN0374 orientation sensor when available. Missing sensor values are printed as --. The Python interface accepts current 24-field rows and legacy 20-field rows.

The BNO055 is configured for NDOF fusion. The Python HUD shows heading (yaw), pitch, and roll, and logs telemetry; the current software does not implement automatic attitude or depth stabilization.

The Python controller defaults to COM3 and camera index 1. Adjust those settings in interface/solo_control.py for the computer in use. Close Arduino Serial Monitor before starting the Python app because both need the same COM port.

## Setup

1. Open firmware/integrated/Control_Submarino_Unificado.ino in Arduino IDE.
2. Install ESP32Servo, Adafruit NeoPixel, a compatible MS5837 library, and DFRobot_BNO055.
3. Install Python dependencies from interface/requirements.txt.
4. Set the correct serial port and camera index in interface/solo_control.py.
5. Review docs/protocolo_serial.md and docs/pruebas.md before an integrated test.

Keep the vehicle restrained and verify stop/neutral before testing propulsion.
