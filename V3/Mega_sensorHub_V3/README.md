# Aerostat Mega Sensor Hub — V2.1 (Arduino Mega)

## Overview
This firmware runs on an **Arduino Mega 2560** and acts as the primary field sensor node for an aerostat/weather-balloon monitoring system. It reads pressure, temperature, wind speed, tether load, GPS position, and compass heading, then packages all readings into a single CSV line and transmits it to an ESP32 Telemetry Hub over UART.

## Hardware Used
- Arduino Mega 2560
- Dual BMP390 pressure/temperature sensors (one I2C, one SPI)
- HX711 load cell amplifier (tether tension)
- Analog anemometer (wind speed)
- DS3231 RTC module (timestamping)
- NEO-M8N GPS module (latitude/longitude)
- QMC5883L or HMC5883L compass (auto-detected, heading)
- 2x relay modules (RTC-scheduled switching)
- ESP32 (receiving end, connected via Serial1)

## Wiring

| Component | Interface | Mega Pins | Notes |
|---|---|---|---|
| BMP390 #1 | I2C | Default SDA/SCL (Pins 20/21), address `0x77` | Ground-relative sensor |
| BMP390 #2 | SPI | CS = Pin 2 | Used for AGL calculation |
| HX711 Load Cell | Digital | DOUT = Pin 5, SCK = Pin 6 | Tether tension in kg |
| Anemometer | Analog | A5 | Wind speed via voltage divider |
| Tare Button | Digital | A6 (INPUT_PULLUP) | Zeroes load cell, debounced |
| DS3231 RTC | I2C | Pins 20/21, address `0x68` | Shared I2C bus |
| Compass (QMC5883L/HMC5883L) | I2C | SDA = Pin 20, SCL = Pin 21 | Auto-detects chip type |
| Relay 1 | Digital | A0 | RTC-scheduled ON/OFF |
| Relay 2 | Digital | A1 | RTC-scheduled ON/OFF |
| Status LED | Digital | A3 | Blinks on successful TX, solid on error |
| NEO-M8N GPS | UART (Serial3) | RX3 = Pin 15, TX3 = Pin 14 | 9600 baud |
| ESP32 Link | UART (Serial1) | TX = Pin 18, RX = Pin 19 | 115200 baud, to ESP32 GPIO27/26 |

## Working Principle
1. On boot, the watchdog timer is disabled temporarily, all peripherals initialize (BMP390 x2, HX711, RTC, GPS, compass), and EEPROM-stored load cell tare is restored if present.
2. The 8-second AVR hardware watchdog is enabled once setup completes, so the board self-resets if the main loop ever hangs.
3. Every loop cycle resets the watchdog, checks for I2C bus timeouts, and updates the status LED to reflect system health.
4. Sensor reads are scheduled independently using `millis()` intervals rather than blocking `delay()` calls:
   - BMP390 pair every 100 ms (with exponential smoothing/filtering)
   - Anemometer every 500 ms
   - Load cell every 500 ms
   - RTC + relay schedule check every 1000 ms
   - Compass every 250 ms
   - Full CSV construction and transmission every 2000 ms
5. GPS data is parsed continuously in the background using TinyGPSPlus; if no GPS fix is available within 10 seconds, latitude/longitude default to `0.000000`.
6. A ground-level pressure baseline is captured 5 seconds after boot to compute Above-Ground-Level (AGL) altitude from the second BMP390.
7. Relay 1 and Relay 2 switch ON/OFF automatically based on programmable RTC time windows (default: 10:00–17:30 and 10:00–17:00).
8. The tare button (A6) can be pressed at any time to zero the load cell; the new tare offset is saved to EEPROM so it persists across power cycles.
9. Every 2 seconds, all sensor values are formatted into a 13-field CSV string and sent to the ESP32 over Serial1, and also printed to the USB Serial monitor for debugging.

## CSV Output Format (13 fields, no headers)
