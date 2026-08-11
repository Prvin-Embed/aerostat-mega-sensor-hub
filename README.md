# Aerostat Mega Sensor Hub

Firmware for a **tethered aerostat telemetry system** split across two microcontrollers:
- **Arduino Mega** — collects environmental & mechanical sensor data, streams CSV via Serial1
- **ESP32** — receives Mega data via UART, merges IMU + humidity, logs to SD, transmits over W5500 LAN (OFC cable to ground station)

---

## Repository Structure

```
aerostat-mega-sensor-hub/
│
├── new_Mega_2bmp_i2cSpi_gyro_AGL_wind_load_1string_sdlog_rtos_3.ino   ← V1 (root, original)
│
├── V1/
│   └── Mega_V1_Gyro_SD_Onboard/
│       └── Mega_V1_Gyro_SD_Onboard.ino    ← V1 placeholder (see root file)
│
├── V2/
│   ├── Mega_SensorHub_V2/
│   │   └── Mega_SensorHub_V2.ino          ← Arduino Mega V2 (Serial1 output, no Gyro/SD)
│   └── ESP32_TelemetryHub_V2/
│       └── ESP32_TelemetryHub_V2.ino      ← ESP32 V2 (BNO055 + DHT22 + SD + W5500)
│
└── README.md
```

---

## System Architecture

```
Arduino Mega
  BMP390 (I2C)  ─┐
  BMP390 (SPI)  ─┤
  HX711 LoadCell─┤──► Serial1 TX (Pin 18) ──────► ESP32 GPIO27 (RX)
  Anemometer    ─┤                                      │
  DS3231 RTC    ─┤                              ┌───────┴────────┐
  2x Relay      ─┘                              │               │
                                           BNO055 (I2C)   DHT22 (GPIO25)
                                                └───────┬────────┘
                                                        │
                                                 Final 17-field CSV
                                                   ┌────┴────┐
                                               SD Card     W5500 LAN
                                              (GPIO13)     (GPIO5)
                                                              │
                                                       TCP Port 5000
                                                              │
                                                    Ground Station
                                                     (via OFC Cable)
```

---

## Version History

| Version | MCU | Key Features | Folder |
|---------|-----|-------------|--------|
| **V1** | Arduino Mega | All-in-one: BMP390×2, HX711, Anemometer, MPU9250 Gyro, DS3231 RTC, SD Card logging | `/` (root) |
| **V2** | Arduino Mega + ESP32 | **Mega:** BMP390×2, HX711, Anemometer, RTC, Relay → streams via Serial1. **ESP32:** Receives Mega data + adds BNO055 + DHT22, logs SD, sends W5500 TCP | `V2/` |

---

## V2 Hardware & Wiring

### Arduino Mega V2

| Pin | Function | Sensor/Module |
|-----|----------|---------------|
| SDA / SCL | I2C Bus | BMP390 #1 (addr 0x77), DS3231 RTC |
| Pin 2 | SPI CS | BMP390 #2 |
| Pin 5 | HX711 DOUT | Load Cell |
| Pin 6 | HX711 SCK | Load Cell |
| Pin 18 (TX1) | UART1 TX | → ESP32 GPIO27 |
| Pin 19 (RX1) | UART1 RX | ← ESP32 GPIO26 |
| A0 | Relay 1 | Control |
| A1 | Relay 2 | Control |
| A3 | Status LED | Blink=OK, Solid=Error |
| A5 | Analog In | Anemometer (0.32–5V) |
| A6 | Button Input | Load Cell Tare (→ GND) |

### ESP32 V2

| Pin | Function | Module |
|-----|----------|--------|
| GPIO27 (RX1) | UART1 RX | ← Mega Pin 18 (TX1) |
| GPIO26 (TX1) | UART1 TX | → Mega Pin 19 (RX1) |
| GPIO25 | DHT22 Data | Temperature + Humidity |
| SDA / SCL | I2C | BNO055 IMU |
| GPIO18 (SCK) | SPI Clock | W5500, SD Card |
| GPIO19 (MISO) | SPI MISO | W5500, SD Card |
| GPIO23 (MOSI) | SPI MOSI | W5500, SD Card |
| GPIO5 | SPI CS | W5500 Ethernet |
| GPIO13 | SPI CS | SD Card |

> **⚠️ Important:** ESP32 default UART1 uses GPIO9/10 which conflict with the integrated SPI flash. UART1 is remapped to GPIO27(RX) / GPIO26(TX) in software using `Serial1.begin(115200, SERIAL_8N1, 27, 26)`.

---

## V2 Data Format

### Mega → ESP32 (10 fields via Serial1)
```
Timestamp,Temp1_C,Pressure1_hPa,Alt1_ASL_m,Temp2_C,Pressure2_hPa,Alt2_AGL_m,Wind_kmh,Load_kg,PressureDiff_hPa
```

Example:
```
2026/06/18-10:45:00,28.43,1011.23,85.60,27.91,1010.88,12.30,14.52,42.350,0.35
```

### ESP32 Final Output — SD Card & W5500 TCP (17 fields)
```
Count,ESP_Millis,RTC_Timestamp,Temp1_C,Pressure1_hPa,Alt1_ASL_m,Temp2_C,Pressure2_hPa,Alt2_AGL_m,Wind_kmh,Load_kg,PressureDiff_hPa,Yaw_deg,Pitch_deg,Roll_deg,DHT_Temp_C,DHT_Humidity_pct
```

Example:
```
42,180032,2026/06/18-10:45:00,28.43,1011.23,85.60,27.91,1010.88,12.30,14.52,42.350,0.35,215.40,1.20,-0.80,29.10,62.40
```

| # | Field | Source | Unit |
|---|-------|--------|------|
| 1 | Count | ESP32 | — |
| 2 | ESP_Millis | ESP32 | ms |
| 3 | RTC_Timestamp | Mega DS3231 | YYYY/MM/DD-HH:MM:SS |
| 4 | Temp1_C | BMP390 #1 (ground, I2C) | °C |
| 5 | Pressure1_hPa | BMP390 #1 | hPa |
| 6 | Alt1_ASL_m | BMP390 #1 | m (above sea level) |
| 7 | Temp2_C | BMP390 #2 (air, SPI) | °C |
| 8 | Pressure2_hPa | BMP390 #2 | hPa |
| 9 | Alt2_AGL_m | BMP390 #2 | m (above ground) |
| 10 | Wind_kmh | Anemometer (Mega A5) | km/h |
| 11 | Load_kg | HX711 Load Cell | kg |
| 12 | PressureDiff_hPa | P1−P2 | hPa |
| 13 | Yaw_deg | BNO055 (ESP32) | ° |
| 14 | Pitch_deg | BNO055 (ESP32) | ° |
| 15 | Roll_deg | BNO055 (ESP32) | ° |
| 16 | DHT_Temp_C | DHT22 (ESP32) | °C |
| 17 | DHT_Humidity_pct | DHT22 (ESP32) | % |

---

## Required Libraries

### Arduino Mega (V2)
| Library | Install via Arduino Library Manager |
|---------|-------------------------------------|
| Adafruit BMP3XX | `Adafruit BMP3XX Library` |
| uRTCLib | `uRTCLib` |
| HX711_ADC | `HX711_ADC` by Olav Kallhovd |

### ESP32 (V2)
| Library | Install via Arduino Library Manager |
|---------|-------------------------------------|
| EasyBNO055_ESP | `EasyBNO055_ESP` |
| DHT sensor library | `DHT sensor library` by Adafruit |
| Ethernet | `Ethernet` (for W5500) |
| SD | Built-in Arduino SD library |

---

## Cloning & Local Development

```bash
git clone https://github.com/Prvin-Embed/aerostat-mega-sensor-hub.git
cd aerostat-mega-sensor-hub
```

Open sketch files directly in Arduino IDE:
- **Mega V2:** `V2/Mega_SensorHub_V2/Mega_SensorHub_V2.ino`
- **ESP32 V2:** `V2/ESP32_TelemetryHub_V2/ESP32_TelemetryHub_V2.ino`

To pull latest changes:
```bash
git pull origin main
```

---

## Network Configuration

| Parameter | Default Value |
|-----------|---------------|
| IP Address | `192.168.1.177` |
| TCP Port | `5000` |
| MAC Address | `DE:AD:BE:EF:FE:ED` |
| SD Log File | `/TelemetryLog.csv` |

Change IP/MAC in `ESP32_TelemetryHub_V2.ino` at the top of the file.

---

Note there are now **two independent wind readings** in the same record:
- `Wind_kmh` — from the analog anemometer on the Mega (voltage-based estimate)
- `RS485_Wind_kmh` / `RS485_WindDir_deg` — from the RS485 ultrasonic anemometer polled directly by the ESP32 (higher-precision speed + direction)

## Reliability Features
- Non-blocking UART parser for the Mega link (no blocking `delay()` calls)
- Stale-data watchdog: marks Mega link as lost after 6 seconds of silence and sentinel-fills with `-999.0`
- Modbus RTU read failure handling: defaults RS485 wind speed/direction to `-99.0` and logs the specific Modbus error code instead of crashing
- Automatic SD card remount every 30 seconds if the initial mount fails
- TCP dead-socket recycling to prevent orphaned client connections
- DHT22 read failures default to `-99.0` instead of logging `NaN`

## Network Access
- Connect any TCP client (e.g., a ground station laptop) to `192.168.1.177:5000` to receive the live 22-field CSV stream in real time.

## Required Libraries
- SPI
- SD
- Ethernet (W5500-compatible)
- DHT sensor library
- EasyBNO055_ESP (or equivalent BNO055 wrapper)
- **ModbusMaster** (for RS485 ultrasonic anemometer communication)

## How to Upload
1. Open in Arduino IDE and select your **ESP32** board variant.
2. Install all required libraries via Library Manager, including **ModbusMaster**.
3. Wire all peripherals per the table above, ensuring the RS485-to-TTL converter is correctly connected to GPIO16/GPIO17 and the anemometer's Modbus slave ID matches `1`.
4. Connect Mega's Serial1 TX/RX to ESP32 GPIO26/27 as documented.
5. Upload and open Serial Monitor at 115200 baud to confirm SD mount, Ethernet IP, Modbus RTU status, and incoming Mega/anemometer data.

## Notes
- This board is the central data-fusion and distribution point of the aerostat telemetry system, combining Mega ground-truth sensor data, local orientation/environmental sensors, and RS485 ultrasonic wind data before persisting and broadcasting.
- The dual wind-speed sources (analog anemometer vs. RS485 ultrasonic anemometer) allow cross-validation of wind readings in the field.
## Safety Features

- **Hardware Watchdog (8s)** on Mega — auto-resets on firmware freeze
- **I2C Bus Timeout** (3ms) with auto-recovery on Mega
- **EEPROM Tare Persistence** — load cell zero survives power cycles
- **Stale Data Guard** on ESP32 — logs last known values + serial warning if Mega stops sending
- **SD Header Once** — header written only on first run, not duplicated on power cycle
