# Tarkov_Gym (ESP-IDF)

This repository contains a minimal executable ESP-IDF application that starts the firmware and blinks a GPIO output on the target board.

## Requirements

- ESP-IDF v5.x installed and exported in your shell
- A supported ESP32 target board

## Build and flash

```bash
idf.py set-target esp32
idf.py build
idf.py -p <PORT> flash monitor
```

The app currently toggles GPIO2 at 500 ms intervals to provide a simple runtime heartbeat. If your board uses a different LED pin, update `BLINK_GPIO` in `main/main.c`.