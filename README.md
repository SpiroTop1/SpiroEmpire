# ARES-X6: 6-Wheel Mars Rover (Arduino Uno)

ARES-X6 is a Bluetooth-controlled 6-wheel rover project with:

- **Arduino Uno** controller
- **HC-05 Bluetooth** for manual + voice command input
- **OV7670 camera support** integrated in the same sketch (**with FIFO camera module required**)
- **Ultrasonic obstacle detection** (HC-SR04)
- **Soil sensor readings** streaming
- **Voice commands** (`START READINGS`, `STOP READINGS`, movement commands)
- **Manual controls** (`F`, `B`, `L`, `R`, `X`, speed changes)

## Important Camera Note

If your OV7670 is the **bare module (no FIFO)**, Arduino Uno cannot reliably do full frame capture and stream while driving motors.
This sketch's camera capture logic is designed for **OV7670 + AL422 FIFO camera modules**.

## Files

- `ARES_X6.ino` → Main combined Arduino sketch (drive + sensors + camera commands).

## Commands

### Drive / Manual / Voice
- `F` or `MOVE FORWARD`
- `B` or `MOVE BACKWARD`
- `L` or `TURN LEFT`
- `R` or `TURN RIGHT`
- `X` or `STOP`
- `SPEED 0..255`
- `AUTO ON` / `AUTO OFF`
- `START READINGS` / `STOP READINGS`
- `STATUS`

### Camera
- `CAM SNAP` → capture one frame into FIFO
- `CAM DUMP` → dump a sample of frame bytes as HEX over Bluetooth

## What “camera integrated” means here

- Camera register init through `Wire`.
- Snapshot trigger (`CAM SNAP`) to store pixels in FIFO.
- Readout sample (`CAM DUMP`) from FIFO as hex bytes.

This is fully integrated in one `.ino` file with rover logic, but it is still constrained by Uno RAM/CPU and Bluetooth bandwidth.

## Wiring Guidance

The sketch includes camera pin constants that must match your specific FIFO camera board wiring. Adjust pin assignments in `ARES_X6.ino` before upload.

## Upload Steps

1. Open `ARES_X6.ino` in Arduino IDE.
2. Select **Board: Arduino Uno** and the correct COM port.
3. Adjust camera pin mapping for your hardware.
4. Upload.
5. Pair HC-05 and send newline-terminated commands from a terminal app.
