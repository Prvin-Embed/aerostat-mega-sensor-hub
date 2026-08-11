# Aerostat ESP32 Telemetry Hub — V2.2 (Production Grade, RS485 Wind Added)

## Overview
This firmware runs on an **ESP32** and acts as the central telemetry aggregator for the aerostat ground control system. It receives the 13-field CSV stream from the Arduino Mega sensor node, fuses it with its own local sensors (BNO055 orientation, DHT22 temperature/humidity), polls an **RS485 ultrasonic anemometer via Modbus RTU** for wind speed and direction, then logs the combined 22-field record to SD card and streams it over Ethernet (W5500) via TCP.

## Hardware Used
- ESP32 development board
- Arduino Mega (sensor source, connected via UART)
- BNO055 orientation sensor (I2C) — Yaw/Pitch/Roll
- DHT22 temperature/humidity sensor
- **RS485 ultrasonic anemometer** (Modbus RTU) — wind speed and wind direction
- W5500 Ethernet module (SPI) — TCP server
- MicroSD card module (SPI) — CSV logging

## Wiring

| Component | Interface | ESP32 Pin | Notes |
|---|---|---|---|
| Mega UART Link | UART1 | RX = GPIO27, TX = GPIO26 | 115200 baud, receives 13-field CSV |
| **RS485 Ultrasonic Anemometer** | UART2 (Modbus RTU) | RX2 = GPIO17, TX2 = GPIO16 | 4800 baud, Modbus slave ID 1, via RS485-to-TTL transceiver |
| DHT22 | Digital | GPIO25 | Local temperature/humidity |
| BNO055 | I2C | Default SDA/SCL | Local Yaw/Pitch/Roll |
| SPI Bus (shared) | SPI | SCK = GPIO18, MISO = GPIO19, MOSI = GPIO23 | Shared by SD and W5500 |
| W5500 Ethernet | SPI | CS = GPIO5 | Static IP: 192.168.1.177, TCP port 5000 |
| SD Card | SPI | CS = GPIO13 | Logs to `/TelemetryLog.csv` |

> Note: An RS485-to-TTL converter module is required between the ultrasonic anemometer's A/B differential lines and the ESP32's UART2 RX/TX pins (GPIO17/GPIO16), since the ESP32 UART pins are single-ended TTL, not RS485.

## Working Principle
1. On boot, UART1 is mapped to GPIO26 (TX)/GPIO27 (RX) at 115200 baud to receive the Mega's CSV stream.
2. UART2 initializes at 4800 baud on GPIO16/GPIO17, and a Modbus RTU master (`ModbusMaster` library) is bound to slave ID 1 — this is the RS485 ultrasonic anemometer.
3. The DHT22 and BNO055 sensors initialize locally on the ESP32 for on-board environmental and orientation sensing.
4. The SD card mounts and either creates a new `TelemetryLog.csv` with a header row or appends to an existing log, using a `FILE_APPEND` + `size()==0` check rather than the unreliable `SD.exists()`.
5. The W5500 Ethernet module initializes with a static IP (192.168.1.177) and starts a TCP server on port 5000 for a ground station to connect and stream live telemetry.
6. Every loop cycle:
   - Manages the TCP client connection, cleaning up dead sockets automatically.
   - Non-blocking UART1 read builds up the incoming Mega CSV line character by character until a newline is received, then parses it into 13 fields.
   - If the SD card previously failed to mount, retries remounting every 30 seconds.
7. Every 2 seconds, the hub:
   - Checks if Mega data is stale (no update for 6+ seconds); if so, marks the link as lost and floods that portion of the record with `-999.0` sentinel values.
   - Reads the local DHT22 (falls back to `-99.0` on sensor error) and BNO055 orientation values.
   - **Polls the RS485 anemometer over Modbus RTU** by reading 2 holding registers starting at address `0x0000`:
     - Register 0 → raw wind speed (scaled ÷100 to get m/s, then ×3.6 to convert to km/h)
     - Register 1 → raw wind direction in degrees
     - If the Modbus read fails, both wind speed and direction default to `-99.0` and a warning with the Modbus error code is printed to Serial.
   - Assembles all data into a final **22-field** CSV string.
   - Writes the record to the SD card.
   - Streams the same record to any connected TCP client over Ethernet.
   - Prints the record to the USB Serial monitor for live debugging.

## CSV Output Format (22 fields)
