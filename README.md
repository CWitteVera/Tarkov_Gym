# Tarkov_Gym (ESP-IDF)

This repository now contains a minimal ESP-IDF application scaffold.

## Requirements

- ESP-IDF v5.x installed and exported in your shell
- A supported ESP32 target board

## Build and flash

```bash
idf.py set-target esp32
idf.py build
idf.py -p <PORT> flash monitor
```