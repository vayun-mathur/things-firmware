# Things Firmware — Seeed XIAO ESP32

BLE GATT server that sends a water intake notification to the Things Android app when the button is pressed.

## Setup

1. Install [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/) (v5.x+)
2. Set your target chip:
   ```bash
   # For XIAO ESP32-C3:
   idf.py set-target esp32c3
   # For XIAO ESP32-S3:
   idf.py set-target esp32s3
   ```
   If using the S3 variant, change `BUTTON_GPIO` in `main/main.c` from `9` to `0`.

3. Build and flash:
   ```bash
   idf.py build
   idf.py -p /dev/cu.usbmodem* flash monitor
   ```

## How it works

- The ESP32 advertises a BLE service with UUID `12345678-1234-5678-1234-56789abcdef0`
- It exposes one characteristic (`...def1`) with read + notify
- Press the BOOT button → sends "You drank 300 mL of water" as a BLE notification
- The Things Android app scans for this service, connects, and subscribes to notifications

## Wiring

No extra wiring needed — uses the built-in BOOT button on the XIAO board. To use an external button, connect it between your chosen GPIO pin and GND, and update `BUTTON_GPIO` in `main/main.c`.
