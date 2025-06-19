# SensorShield-Testing

This repository contains a small suite of firmware and Python tools for an ESP32-based "Sensor Shield" prototype. The board logs environmental data and exposes it over USB, Wi‑Fi, or BLE.

## Repository Layout

- `Nates Docs.md` – hardware notes and usage tips. Includes wiring tables, storage recommendations, and a typical workflow for retrieving data.
- `SmartShield_ESP32_DAS_Test.ino` – Arduino sketch that logs sensor data to SPIFFS and broadcasts it via BLE.
- `test_ble_live_data.py` – Python command-line utility for pulling data from the device over various interfaces (USB serial, Wi‑Fi access point, or BLE).
- `test_sensor_device.py` – automated script that validates DAS capability, storage capacity, USB access, BLE functions, and the overall workflow.
- `requirements.txt` – Python dependencies used by the scripts.

## Arduino Firmware

The example sketch configures an ESP32 to:

1. Initialize SPIFFS storage and create a JSON log file.
2. Collect readings from temperature, humidity, and battery fuel-gauge sensors.
3. Periodically store readings and broadcast them over BLE using a custom service.
4. Host a Wi-Fi access point at `192.168.4.1` with `/data` and `/status` endpoints.

See the comments inside `SmartShield_ESP32_DAS_Test.ino` for configuration constants such as `LOG_INTERVAL` and BLE UUIDs.

## Python Utilities

Install the dependencies with:

```bash
pip install -r requirements.txt
```

### `test_ble_live_data.py`

Retrieves sensor data through multiple methods. Example usage:

```bash
python3 test_ble_live_data.py --flash          # dump JSON log via LittleFS
python3 test_ble_live_data.py --serial         # monitor real-time serial output
python3 test_ble_live_data.py --access-point   # download via Wi-Fi AP
python3 test_ble_live_data.py --ble-live       # live BLE streaming
python3 test_ble_live_data.py --ble-dump       # full history over BLE
```

### `test_sensor_device.py`

Runs a comprehensive checklist to confirm the board’s features. It reports on DAS capability, USB and BLE access, and demonstrates the full data logging workflow.

Run it with:

```bash
python3 test_sensor_device.py
```

### `main.py`

Run without arguments to get an interactive menu powered by `InquirerPy`:

```bash
python3 main.py
```

All command-line options still work for scripting purposes.

## Hardware Notes

`Nates Docs.md` provides detailed information on supported storage (MicroSD, SPI flash, I²C EEPROM), wiring guidelines, BLE usage, and troubleshooting tips. It also describes a typical user workflow:

1. Insert a MicroSD card to start logging automatically.
2. Connect via USB for a serial console or use BLE / Wi‑Fi to download data.

Consult that document when wiring sensors or configuring the board.

---

This repo serves as a starting point for exploring ESP32 data logging with multiple retrieval methods. Read through the firmware and scripts to adapt them to your own hardware.
