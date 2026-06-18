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
- Press the BOOT button → reads the sensors and sends a BLE notification like
  `You drank 300 mL of water | accel x=0.01 y=-0.02 z=1.00 g | weight 12 g`
- The Things Android app scans for this service, connects, and subscribes to notifications

If a sensor is not connected it reports `accel n/a` / `weight n/a` instead of blocking.

## Wiring (Seeed XIAO ESP32-C3)

The button uses the built-in BOOT button (GPIO9). Power **both sensors from 3V3**
— the ESP32-C3 GPIOs are not 5V tolerant.

| Sensor | Signal | XIAO pin | GPIO |
|--------|--------|----------|------|
| ADXL345 (I²C, addr 0x53) | SDA | D4 | GPIO6 |
| | SCL | D5 | GPIO7 |
| HX711 | DOUT | D2 | GPIO4 |
| | SCK | D3 | GPIO5 |

Pins GPIO4–7 are used because they avoid the C3 strapping pins (GPIO2/8/9).
To use other pins, edit the `*_GPIO` defines in `main/main.c`.

### Load cell calibration

The HX711 is tared (zeroed) at boot, so wire and load nothing on the cell when
powering on. Weight is reported in grams using `HX711_SCALE` (counts per gram) in
`main/main.c`. To calibrate: read the raw value with a known weight, then set
`HX711_SCALE = (raw - tare_offset) / known_grams` and re-flash.
