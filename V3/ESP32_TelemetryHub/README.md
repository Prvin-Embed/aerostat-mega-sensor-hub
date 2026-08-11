# Aerostat ESP32 Telemetry Hub — V2.1 (Production Grade)

## Overview
This firmware runs on an **ESP32** and acts as the central telemetry aggregator for the aerostat ground control system. It receives the 13-field CSV stream from the Arduino Mega sensor node, fuses it with its own local sensors (BNO055 orientation, DHT22 temperature/humidity), and outputs a combined 20-field CSV to an SD card and over Ethernet (W5500) via TCP.

## Hardware Used
- ESP32 development board
- Arduino Mega (sensor source, connected via UART)
- BNO055 orientation sensor (I2C) — Yaw/Pitch/Roll
- DHT22 temperature/humidity sensor
- W5500 Ethernet module (SPI) — TCP server
- MicroSD card module (SPI) — CSV logging

## Wiring

| Component | Interface | ESP32 Pin | Notes |
|---|---|---|---|
| Mega UART Link | UART1 | RX = GPIO27, TX = GPIO26 | 115200 baud, receives 13-field CSV |
| DHT22 | Digital | GPIO25 | Local temperature/humidity |
| BNO055 | I2C | Default SDA/SCL | Local Yaw/Pitch/Roll |
| SPI Bus (shared) | SPI | SCK = GPIO18, MISO = GPIO19, MOSI = GPIO23 | Shared by SD and W5500 |
| W5500 Ethernet | SPI | CS = GPIO5 | Static IP: 192.168.1.177, TCP port 5000 |
| SD Card | SPI | CS = GPIO13 | Logs to `/TelemetryLog.csv` |

## Working Principle
1. On boot, UART1 is mapped to GPIO26 (TX)/GPIO27 (RX) at 115200 baud to receive the Mega's CSV stream.
2. The DHT22 and BNO055 sensors initialize locally on the ESP32 for on-board environmental and orientation sensing.
3. The SD card mounts and either creates a new `TelemetryLog.csv` with a header row or appends to an existing log — using a `FILE_APPEND` + `size()==0` check rather than `SD.exists()`, which is unreliable on some ESP32 cores.
4. The W5500 Ethernet module initializes with a static IP (192.168.1.177) and starts a TCP server on port 5000 so a ground station PC can connect and stream live telemetry.
5. Every loop cycle:
   - Manages the TCP client connection, cleaning up dead sockets automatically.
   - Non-blocking UART1 read builds up the incoming Mega CSV line character by character until a newline is received.
   - Once a full line arrives, it is parsed into 13 numeric/string fields (timestamp, dual pressure/temp/altitude, wind, load, pressure differential, GPS lat/lon, heading).
   - If the SD card previously failed to mount, the code retries remounting every 30 seconds and re-writes the header if needed.
6. Every 2 seconds, the hub:
   - Checks if Mega data is stale (no update for 6+ seconds); if so, it marks the link as lost and floods the record with `-999.0` sentinel values instead of using outdated data.
   - Reads the local DHT22 (falls back to `-99.0` on sensor error) and BNO055 orientation values.
   - Assembles all data into a final 20-field CSV string.
   - Writes the record to the SD card.
   - Streams the same record to any connected TCP client over Ethernet.
   - Prints the record to the USB Serial monitor for live debugging.

## CSV Output Format (20 fields)
