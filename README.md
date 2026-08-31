# ESP32-S3 Camera Hexagon Auto-Clicker

This repository contains a PlatformIO-based MVP for an ESP32-S3 camera-driven auto-clicker that watches a monitor for white hexagon outlines and sends a native USB mouse left click when the shrinking outer hexagon matches the fixed inner hexagon.

The project is intentionally designed around the Seeed Studio XIAO ESP32S3 Sense with OV2640 camera and uses a simple threshold-based image pipeline rather than machine learning.

## Features

- OV2640 camera initialization and QVGA capture
- configurable ROI and brightness threshold
- simple white-pixel / bounding-box hexagon detector
- match-confirmation logic to avoid noisy single-frame triggers
- USB HID mouse click output with a software safety gate
- serial command interface for enabling/disabling clicking
- performance metrics and debug output
- modular project structure for later calibration work

## Project structure

- `src/main.cpp` – main application loop and command handling
- `src/camera_manager.*` – camera init, capture, frame timing
- `src/hexagon_detector.*` – lightweight bright-region detector
- `src/vision.*` – match detection and confirmation logic
- `src/mouse_hid.*` – native USB mouse HID output
- `src/debug.*` – serial diagnostics and debug config
- `src/calibration.*` – calibration state and simple command parsing
- `src/configuration.h` – central project constants
- `platformio.ini` – PlatformIO build configuration

## Hardware target

- Seeed Studio XIAO ESP32S3 Sense
- OV2640 camera installed on the board
- USB-C to Windows PC
- optional camera stand or bracket

## Required software

- PlatformIO Core (recommended) or VS Code + PlatformIO extension
- Python 3
- USB serial drivers for the ESP32-S3, if your OS requires them

## PlatformIO setup

1. Install PlatformIO Core:

   ```bash
   python -m pip install platformio
   ```

2. In the repository root, run:

   ```bash
   pio run
   ```

3. Upload the firmware:

   ```bash
   pio run --target upload
   ```

4. Open the serial monitor:

   ```bash
   pio device monitor -b 115200
   ```

## USB HID mouse behaviour

The board presents itself as a standard mouse when connected to a computer. However, the app defaults to click-disabled mode for development safety.

Serial commands:

- `CLICK ON` – enable left click output
- `CLICK OFF` – disable left click output
- `STATUS` – print current click state and settings
- `DEBUG 0|1|2|3` – set debug level
- `HELP` – list available commands

The default state after boot is intentionally `CLICKING DISABLED`.

## Detection strategy

This MVP uses a deliberately simple approach for a fast embedded implementation:

1. Capture a grayscale QVGA frame.
2. Threshold bright pixels inside a fixed ROI.
3. Calculate the largest bright bounding-box region and a second candidate region.
4. Treat the larger region as the outer hexagon and the smaller region as the inner target.
5. Compare the difference in radius between the two candidates.
6. Require the match condition to remain valid for a configurable number of frames.
7. Only emit a USB left-click when the match is confirmed and clicking is enabled.

This is intentionally simple and deterministic, which keeps the implementation feasible for the ESP32-S3 while still providing a solid MVP.

## Configuration reference

All tuning values are centralized in `src/configuration.h`:

- `CAMERA_WIDTH`, `CAMERA_HEIGHT`
- `ROI_X`, `ROI_Y`, `ROI_WIDTH`, `ROI_HEIGHT`
- `BRIGHTNESS_THRESHOLD`
- `MATCH_TOLERANCE_PIXELS`
- `MATCH_CONFIRMATION_FRAMES`
- `CLICK_DURATION_MS`
- `MIN_HEXAGON_SIZE`, `MAX_HEXAGON_SIZE`
- `DEBUG_LEVEL`

## Calibration notes

The project includes initial calibration hooks in `src/calibration.*`, but the first prototype relies on fixed constants. To tune the detection:

1. Point the camera at the monitor.
2. Set the ROI so it encloses the hexagons.
3. Adjust the brightness threshold.
4. Tune the match tolerance and confirmation frames.
5. Keep clicks disabled while you validate the detector.

## Flashing and testing

1. Connect the XIAO ESP32S3 Sense to the PC via USB-C.
2. Build and upload the project with PlatformIO.
3. Open the serial monitor.
4. Confirm camera startup messages and frame metrics.
5. Use `CLICK OFF` to keep the device safe while checking detection.
6. Once the detector is stable, send `CLICK ON` to allow left-click output.

## Troubleshooting

- Camera not detected: verify the OV2640 ribbon is seated correctly and camera power is available.
- No bright blobs found: raise the brightness threshold or enlarge the ROI.
- False clicks: increase `MATCH_TOLERANCE_PIXELS` or `MATCH_CONFIRMATION_FRAMES`.
- Unstable lighting: reduce glare, adjust exposure, or improve monitor contrast.
- Missing USB mouse: verify the board is set to native USB and that the Windows host enumerates the device correctly.

## Important safety note

This project intentionally starts with `CLICKING DISABLED` after boot. Always validate the vision pipeline against the live monitor while the click feature remains off before enabling it.

## Future extensions

The current code is structured for easy extension toward:

- automatic ROI calibration
- threshold tuning
- predictive click timing
- telemetry and logging
- more advanced contour detection
- calibration web interface

## Disclaimer

This is a working MVP framework intended for prototyping and experimentation. The exact camera pin mapping and threshold settings may require board-specific calibration and monitor-specific tuning.
