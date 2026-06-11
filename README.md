# Aerostat / Weather Balloon — Arduino Mega Sensor Hub

> **"Final Robust Arduino Mega Code — 24/7 Stability Edition"**  
> A production-grade, continuously running sensor data collector for tethered aerostat and weather balloon ground stations.

---

## Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Hardware Requirements](#hardware-requirements)
- [Pin Mapping](#pin-mapping)
- [Software Dependencies](#software-dependencies)
- [Safety & Reliability Features](#safety--reliability-features)
- [Sensor Details & Signal Processing](#sensor-details--signal-processing)
- [Data Output Format](#data-output-format)
- [Relay Scheduler](#relay-scheduler)
- [EEPROM Memory Map](#eeprom-memory-map)
- [Timing & Task Schedule](#timing--task-schedule)
- [Setup & First Boot](#setup--first-boot)
- [Calibration Guide](#calibration-guide)
- [Serial Monitor Output](#serial-monitor-output)
- [Troubleshooting](#troubleshooting)
- [Future Improvements](#future-improvements)

---

## Overview

This firmware runs on an **Arduino Mega 2560** and serves as a multi-sensor data acquisition and transmission hub. It is designed for **unattended, continuous 24/7 operation** in outdoor environments — specifically ground stations for tethered aerostats, weather balloons, and aerial monitoring platforms.

The system collects data from 6 sensor subsystems, timestamps every reading using a real-time clock, logs all data locally to an SD card, and transmits a compact CSV data packet over UART (`Serial TX0`) every 2 seconds. This serial output is typically consumed by a downstream wireless node (e.g., ESP32) for MQTT cloud uplink.

---

## System Architecture

```
┌─────────────────────────────────────────────────┐
│              Arduino Mega 2560                  │
│                                                 │
│  Sensors ──────────────────────────────────┐   │
│  BMP390 (I2C)     Ground Pressure/Temp      │   │
│  BMP390 (SPI)     Airborne Pressure/AGL     │   │
│  MPU9250 (I2C)    Roll / Pitch / Yaw        │   │
│  HX711 + LC       Tether Tension (kg)       │   │
│  Anemometer       Wind Speed (km/h)         │   │
│  DS3231 RTC       Timestamp                 │   │
│                                             │   │
│  ──────────────────────────────────────────┘   │
│  SD Card          Local LOG.txt logging         │
│  2× Relays        RTC-scheduled automation      │
│  Status LED       System health indicator       │
│                                                 │
│  Serial TX0  ──►  ESP32 (MQTT uplink to cloud)  │
└─────────────────────────────────────────────────┘
```

---

## Hardware Requirements

| Component | Part | Interface |
|---|---|---|
| Microcontroller | Arduino Mega 2560 | — |
| Barometric Sensor 1 (Ground) | Bosch BMP390 / BMP388 | I2C (0x77) |
| Barometric Sensor 2 (Airborne) | Bosch BMP390 / BMP388 | SPI (CS pin 2) |
| IMU | MPU9250 / MPU6050 | I2C (0x69) |
| Load Cell ADC | HX711 module | Digital pins 5, 6 |
| Load Cell | 500 kg capacity | — |
| Anemometer | Analog voltage output type (0.32–5V) | Analog pin A5 |
| Real-Time Clock | DS3231 | I2C (0x68) |
| SD Card Module | SPI SD breakout | SPI (CS pin 53) |
| Relay Module | 2-channel relay | Pins A0, A1 |
| Status LED | Any LED + resistor | Pin A3 |
| Tare Button | Momentary push button | Pin A6 (to GND) |

> ⚠️ **Power Note:** The BMP390 on SPI and the SD card both use the SPI bus. Ensure SD card CS (pin 53) and BMP2 CS (pin 2) are wired correctly. Both are managed by the `SD.h` and `Adafruit_BMP3XX` libraries independently.

---

## Pin Mapping

```
┌─────────────┬────────────┬─────────────────────────────────────┐
│ Pin         │ Direction  │ Function                            │
├─────────────┼────────────┼─────────────────────────────────────┤
│ A0          │ OUTPUT     │ Relay 1 Control                     │
│ A1          │ OUTPUT     │ Relay 2 Control                     │
│ A3          │ OUTPUT     │ Status LED (Error=Solid, TX=Blink)  │
│ A5          │ INPUT      │ Anemometer Analog Voltage           │
│ A6          │ INPUT_PU   │ Tare Button (LOW when pressed)      │
│ 2           │ OUTPUT     │ BMP390 SPI Chip Select              │
│ 5           │ I/O        │ HX711 Data (DOUT)                   │
│ 6           │ OUTPUT     │ HX711 Clock (SCK)                   │
│ 20 (SDA)    │ I2C        │ MPU9250 + BMP390 #1 + DS3231 SDA    │
│ 21 (SCL)    │ I2C        │ MPU9250 + BMP390 #1 + DS3231 SCL    │
│ 50 (MISO)   │ SPI        │ SD Card + BMP390 #2                 │
│ 51 (MOSI)   │ SPI        │ SD Card + BMP390 #2                 │
│ 52 (SCK)    │ SPI        │ SD Card + BMP390 #2                 │
│ 53          │ OUTPUT     │ SD Card Chip Select                 │
│ TX0 (pin 1) │ UART TX    │ Serial Data Out → ESP32             │
└─────────────┴────────────┴─────────────────────────────────────┘
```

---

## Software Dependencies

Install these libraries via the **Arduino Library Manager** or from their GitHub repositories:

| Library | Purpose | Install Name |
|---|---|---|
| `Adafruit BMP3XX` | BMP390/BMP388 driver | `Adafruit BMP3XX Library` |
| `uRTCLib` | DS3231 RTC driver | `uRTCLib` |
| `HX711_ADC` | HX711 load cell ADC (non-blocking) | `HX711_ADC` |
| `Wire` | I2C bus (built-in) | Built-in |
| `SPI` | SPI bus (built-in) | Built-in |
| `SD` | SD card filesystem (built-in) | Built-in |
| `EEPROM` | EEPROM read/write (built-in) | Built-in |
| `avr/wdt.h` | Hardware watchdog timer | Built-in (AVR) |

---

## Safety & Reliability Features

### 1. Hardware Watchdog Timer (8 Seconds)
The AVR hardware watchdog is enabled at the end of `setup()` with an 8-second timeout. `wdt_reset()` is called at the **top of every `loop()` cycle**. If the MCU hangs (I2C freeze, sensor stall, stack overflow), the watchdog reboots the system automatically without human intervention.

```cpp
wdt_enable(WDTO_8S);  // Setup
// ...
void loop() {
  wdt_reset();  // Must be called within 8s or board reboots
```

### 2. I2C Bus Timeout & Auto-Recovery
The `Wire` library is initialized with a **3-second bus timeout**:
```cpp
Wire.setWireTimeout(3000, true); // 3000 µs timeout, auto-reset bus on hang
```
Every loop checks `Wire.getWireTimeoutFlag()`. A timeout sets `system_error = true` and lights the status LED solid, allowing the watchdog to eventually recover.

### 3. EEPROM-Backed Gyro Auto-Calibration
On the first boot (or when `CALIBRATION_MAGIC_BYTE` is changed in code), the system automatically runs a **2000-sample gyro calibration** over ~4 seconds while blinking the status LED rapidly. Calculated offsets are saved to EEPROM addresses 0–12. On all subsequent boots, calibration loads instantly from EEPROM — no drift on every reset.

To **force a re-calibration**, change `CALIBRATION_MAGIC_BYTE` from `0x12` to any other value (e.g., `0x13`) and re-upload.

### 4. SD Card Hot-Swap / Auto-Recovery
If an SD write fails (`SD.open()` returns null), `sd_online` is set to `false`. The system retries `SD.begin()` every **5 seconds**. Upon successful recovery, it writes a `--- SD RECOVERED ---` marker to the log.

### 5. Persistent Load Cell Zero (EEPROM Tare)
After pressing the tare button (pin A6), the tare offset is saved to EEPROM addresses 20–28 with a signature `0x4154` ("AT"). This offset is reloaded on every boot — your calibrated zero reference **survives power cuts**, which is critical for unattended field deployments.

### 6. Status LED Logic
| LED State | Meaning |
|---|---|
| **Solid ON** | System error (SD offline or I2C bus timeout) |
| **Short blink (500ms)** | Data packet successfully transmitted |
| **Rapid blink (25ms×20)** | IMU auto-calibration in progress |
| **1-second ON** | IMU calibration complete |

---

## Sensor Details & Signal Processing

### Dual BMP390 — Pressure & AGL Altitude

**BMP1 (I2C, address 0x77)** — Mounted at ground level. Reads absolute pressure and temperature. Altitude is calculated relative to standard sea level (1013.25 hPa) using `readAltitude()`.

**BMP2 (SPI, CS pin 2)** — Mounted on the aerostat/payload. Reads airborne pressure. AGL (Above Ground Level) altitude is computed 5 seconds after boot using the barometric formula with the captured ground baseline:

```
h_AGL = 44330 × (1 − (P_airborne / P_ground)^0.1903)   [meters]
```

Both sensors apply an **Exponential Moving Average (EMA)** filter with α = 0.2 to smooth noisy readings:
```
filtered = (filtered × 0.8) + (raw × 0.2)
```

Sensor configuration:
- Temperature Oversampling: 4× (`BMP3_OVERSAMPLING_4X`)
- Pressure Oversampling: 4× (`BMP3_OVERSAMPLING_4X`)
- Output Data Rate: 50 Hz
- IIR Filter Coefficient: 3

### MPU9250 — Roll, Pitch, Yaw (IMU)

The firmware implements a **Complementary Filter** (no external library needed) fusing gyroscope integration with accelerometer angle correction:

```
Roll  = 0.96 × (Roll  + GyroRateX × dt) + 0.04 × AccAngleX
Pitch = 0.96 × (Pitch + GyroRateY × dt) + 0.04 × AccAngleY
Yaw   = Yaw + GyroRateZ × dt  (gyro only, with deadzone)
```

**Yaw Drift Fix:** A deadzone of `±0.5 deg/s` prevents gyro noise from accumulating as yaw drift during stationary periods.

Gyro raw values are divided by `131.0` (sensitivity scale factor for ±250°/s range) and corrected with EEPROM-stored offsets.

### HX711 — Load Cell (Tether Tension)

Uses the `HX711_ADC` library in non-blocking mode (`LoadCell.update()` called every loop). Data is read every 500 ms:
```cpp
weight_kg = LoadCell.getData() / 1000.0;
```
Calibration factor for **500 kg load cell**: `-8.513999`  
Calibration factor for **100 kg load cell**: `-44.013999`

### Anemometer — Wind Speed

The anemometer outputs an analog voltage (0.32 V = 0 m/s, 5.0 V = 32.4 m/s). The ADC value is converted:
```
V = (ADC / 1023.0) × 5.0
Speed_mps = ((V - 0.32) / (5.0 - 0.32)) × 32.4
Speed_kmh = Speed_mps × 3.6
```

### DS3231 RTC — Timestamping

The DS3231 is polled every 1 second. Time is formatted as:
```
YYYY/MM/DD-HH:MM:SS
```

---

## Data Output Format

Every **2 seconds**, a single CSV line is printed on `Serial` (115200 baud) and appended to `LOG.txt` on the SD card:

```
YYYY/MM/DD-HH:MM:SS,Yaw,Pitch,Roll,Temp1,Pressure1,Alt1_ASL,Temp2,Pressure2,Alt2_AGL,WindSpeed_kmh,Load_kg,PressureDiff_hPa
```

### Field Descriptions

| Field # | Name | Unit | Source |
|---|---|---|---|
| 1 | Timestamp | `YYYY/MM/DD-HH:MM:SS` | DS3231 RTC |
| 2 | Yaw | degrees | MPU9250 gyro |
| 3 | Pitch | degrees | MPU9250 complementary filter |
| 4 | Roll | degrees | MPU9250 complementary filter |
| 5 | Temperature 1 | °C | BMP390 #1 (I2C, ground) |
| 6 | Pressure 1 | hPa | BMP390 #1 (I2C, ground) |
| 7 | Altitude ASL | m | BMP390 #1 (Above Sea Level) |
| 8 | Temperature 2 | °C | BMP390 #2 (SPI, airborne) |
| 9 | Pressure 2 | hPa | BMP390 #2 (SPI, airborne) |
| 10 | Altitude AGL | m | BMP390 #2 (Above Ground Level) |
| 11 | Wind Speed | km/h | Analog anemometer |
| 12 | Load / Tension | kg | HX711 + load cell |
| 13 | Pressure Diff | hPa | BMP1 pressure − BMP2 pressure |

### Example Output Line
```
2026/06/11-14:22:05,  2.34,  -1.12,   0.88, 32.45, 1008.23,  242.10, 31.98, 1002.45,   58.34, 12.60,  45.320,   5.78
```

---

## Relay Scheduler

Two relays (A0 and A1) are independently controlled by RTC time schedules. Default schedule:

| Relay | ON Time | OFF Time | Typical Use |
|---|---|---|---|
| Relay 1 | 10:00 | 17:30 | Payload power / cameras |
| Relay 2 | 10:00 | 17:00 | Ground lights / heating |

Schedules are set via constants at the top of the code:
```cpp
int R1_Start_Hour = 10, R1_Start_Min = 00;
int R1_Stop_Hour  = 17, R1_Stop_Min  = 30;
```

The scheduler supports **overnight schedules** (e.g., 22:00–06:00). If `start > stop`, the logic correctly wraps around midnight.

> **Note:** Many relay modules are Active LOW. If the relay logic is inverted, swap `RELAY_ON = HIGH` to `RELAY_ON = LOW` at the top of the code.

---

## EEPROM Memory Map

| Address | Size | Content |
|---|---|---|
| 0 | 4 bytes | `gyroErrorX` (float) |
| 4 | 4 bytes | `gyroErrorY` (float) |
| 8 | 4 bytes | `gyroErrorZ` (float) |
| 12 | 1 byte | Calibration magic byte (`0x12`) |
| 20 | 2 bytes (int) | Load cell tare signature (`0x4154`) |
| 24 | 4 bytes (long) | Load cell tare offset |

> **Caution:** The Arduino Mega EEPROM has 4096 bytes and is rated for ~100,000 write cycles per address. The load cell tare and IMU calibration writes are one-time or rare events, so this is well within the safe lifetime.

---

## Timing & Task Schedule

The firmware uses a **cooperative, non-blocking, millis()-based scheduler** — similar in spirit to an RTOS but running on a single thread. Each task has its own timer and interval:

| Task | Interval | Notes |
|---|---|---|
| `update_custom_mpu()` | Every loop (~1–2 ms) | Must run continuously for accurate integration |
| `LoadCell.update()` | Every loop | Non-blocking HX711 polling |
| `update_bmp_logic()` | 100 ms | EMA-filtered pressure/temperature |
| Wind speed read | 500 ms | ADC read + voltage-to-speed conversion |
| Load cell data read | 500 ms | Retrieve latest weight from HX711 buffer |
| RTC refresh + relay | 1000 ms | Timestamp update + relay schedule check |
| `construct_and_send()` | 2000 ms | Serial TX + SD write |

---

## Setup & First Boot

1. **Install all required libraries** (see [Software Dependencies](#software-dependencies)).
2. **Wire all sensors** per the [Pin Mapping](#pin-mapping) table.
3. **Set the RTC time** using a separate DS3231 time-set sketch before deploying.
4. **Upload the firmware** to Arduino Mega 2560 at 115200 baud.
5. **Open Serial Monitor** at 115200 baud to observe boot messages.
6. On **first boot**, the system will auto-calibrate the IMU (LED blinks rapidly for ~4 seconds). **Do not move the board during this time.**
7. After calibration, the system prints `--- System Live (WDT Active) ---` and starts transmitting data.
8. **Press the tare button (A6)** to zero the load cell. The offset is saved to EEPROM automatically.

---

## Calibration Guide

### IMU (MPU9250) Gyro Calibration
- **Automatic on first boot** or when `CALIBRATION_MAGIC_BYTE` changes.
- Place the board flat and still before powering on.
- LED blinks rapidly during calibration (~4 seconds).
- To force re-calibration: change `#define CALIBRATION_MAGIC_BYTE 0x12` to `0x13` and re-upload.

### Load Cell Calibration Factor
- Set `savedCalibrationFactor` based on your load cell capacity:
  - 500 kg load cell: `-8.513999`
  - 100 kg load cell: `-44.013999`
- To find your specific calibration factor, use the `HX711_ADC` calibration example sketch.

### AGL Baseline (Automatic)
- Captured automatically **5 seconds after boot** from BMP2's first valid pressure reading.
- Ensure the airborne BMP390 is at ground level during startup for an accurate AGL baseline.

---

## Serial Monitor Output

Example boot sequence:
```
--- Mega Sensor Hub Starting (Robust) ---
[OK] Valid IMU Cal loaded from EEPROM
[OK] SD Ready
[OK] Load Cell Tare Offset Loaded from EEPROM
--- System Live (WDT Active) ---
AGL Baseline set: 1008.23
2026/06/11-14:22:05,  2.34,  -1.12,   0.88, 32.45, 1008.23,  242.10, 31.98, 1002.45,   58.34, 12.60,  45.320,   5.78
2026/06/11-14:22:07,  2.35,  -1.11,   0.87, 32.46, 1008.22,  242.12, 31.98, 1002.44,   58.41, 12.58,  45.318,   5.78
```

---

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---|---|---|
| `[ERR] BMP1 (I2C) Fail` | Wrong I2C address or wiring | Check `0x77` vs `0x76` SDO pin; check SDA/SCL wiring |
| `[ERR] BMP2 (SPI) Fail` | Wrong CS pin or SPI conflict | Verify CS is pin 2; check MOSI/MISO/SCK wiring |
| `[ERROR] SD Init Failed` | SD not inserted, wrong CS, or 3.3V logic issue | Verify CS = pin 53; use level-shifter if needed |
| `[ERROR] Load Cell Timeout` | HX711 wiring issue or powered off | Check DOUT (pin 5) and SCK (pin 6) connections |
| LED stays solid ON | SD offline or I2C bus timeout | Check SD card; verify all I2C devices respond |
| All zeros for weight | Load cell not tared or calibration factor wrong | Press tare button (A6); verify calibration factor |
| AGL altitude never updates | BMP2 not responding at boot | Check SPI wiring; verify BMP2 CS pin |
| Board randomly reboots | Watchdog triggered (hang somewhere) | Monitor Serial for last output before reboot |

---

## Future Improvements

- [ ] Add GPS module (NEO-6M/M8N) for position logging alongside sensor data
- [ ] Integrate ESP32 MQTT uplink directly on the Mega via UART bridge
- [ ] Add a magnetometer correction for absolute yaw (using MPU9250's built-in compass)
- [ ] Implement a simple moving average (SMA) buffer for wind gusts (max speed over rolling window)
- [ ] Add low-battery alert via analog voltage divider on a free analog pin
- [ ] OTA firmware update support (via ESP32 co-processor)
- [ ] Web dashboard for real-time data visualization

---

## Author

**Parveen Kumar**  
Embedded Firmware Engineer — Empyreal Galaxy Pvt. Ltd.  
GitHub: [@Prvin-Embed](https://github.com/Prvin-Embed)

---

## License

MIT License — Free to use, modify, and distribute with attribution.
