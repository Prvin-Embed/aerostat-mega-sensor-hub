/*
 * ================================================================
 * ARDUINO MEGA SENSOR HUB — VERSION 2
 * Serial Output: Serial1 (TX1 = Pin 18, RX1 = Pin 19)
 *                Connected to ESP32 GPIO27(RX) & GPIO26(TX)
 * Baud Rate    : 115200
 *
 * Active Sensors:
 *   1. Dual BMP390 (I2C + SPI) — Pressure, Temp, AGL Altitude
 *   2. HX711 Load Cell         — Tether Tension (kg)
 *   3. Analog Anemometer       — Wind Speed (km/h)
 *   4. DS3231 RTC              — Timestamp
 *   5. 2x Relay                — RTC-Scheduled Automation
 *
 * Safety:
 *   - Hardware Watchdog Timer (8s)
 *   - I2C Bus Timeout & Auto-Recovery
 *   - EEPROM Persistent Load Cell Tare (survives power loss)
 *
 * Removed from V1 (now handled by ESP32):
 *   - MPU9250 / Gyro
 *   - SD Card
 *
 * CSV Output Format (10 fields):
 *   Timestamp, Temp1, Pressure1, Alt1_ASL, Temp2, Pressure2,
 *   Alt2_AGL, Wind_kmh, Load_kg, PressureDiff_hPa
 * ================================================================
 */

#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include "Adafruit_BMP3XX.h"
#include <uRTCLib.h>
#include <HX711_ADC.h>
#include <avr/wdt.h>
#include <math.h>

// ================================================================
// PIN DEFINITIONS
// ================================================================

const int statusLedPin  = A3;   // Blink on TX, Solid on I2C Error
const int relay1Pin     = A0;
const int relay2Pin     = A1;

// Relay logic: change to LOW/HIGH if your module is Active-LOW
const int RELAY_ON  = HIGH;
const int RELAY_OFF = LOW;

// Relay 1 Schedule (HH:MM)
int R1_Start_Hour = 10, R1_Start_Min = 00;
int R1_Stop_Hour  = 17, R1_Stop_Min  = 30;

// Relay 2 Schedule (HH:MM)
int R2_Start_Hour = 10, R2_Start_Min = 00;
int R2_Stop_Hour  = 17, R2_Stop_Min  = 00;

// BMP390: BMP1 = I2C, BMP2 = SPI
#define BMP1_I2C_ADDRESS 0x77
#define BMP2_CS          2

// HX711 Load Cell
const int HX711_dout    = 5;
const int HX711_sck     = 6;
const int tareButtonPin = A6;   // Momentary button → GND

// Analog Anemometer
const int anemometerPin = A5;

// ================================================================
// GLOBAL VARIABLES
// ================================================================

bool system_error = false;
unsigned long led_blink_start    = 0;
const unsigned long LED_BLINK_DUR = 500;

// BMP390
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BMP3XX bmp1, bmp2;
bool   agl_calibrated      = false;
double ground_pressure_hpa = 0.0;
unsigned long cal_start_time = 0;
const unsigned long AGL_CAL_DELAY = 5000;  // 5s after boot

double temp1_bmp = 0, pressure1_bmp_hpa = 0, altitude1_bmp = 0;
double temp2_bmp = 0, pressure2_bmp_hpa = 0, altitude2_bmp_agl = 0;
double pressure_difference_hpa = 0;

// DS3231 RTC
uRTCLib rtc(0x68);
char rtcBuffer[25];

// HX711
HX711_ADC LoadCell(HX711_dout, HX711_sck);
const float savedCalibrationFactor = -8.513999;
// 500 kg cell: -8.513999  |  100 kg cell: -44.013999
float weight_kg = 0.0;

// EEPROM addresses for load cell tare
const int  EEPROM_SIG_ADDR_LC  = 0;
const int  EEPROM_TARE_ADDR_LC = 4;
const int  EEPROM_SIGNATURE_LC = 0x4154;  // "AT"

// Anemometer
const float minVoltage   = 0.32;
const float maxVoltage   = 5.0;
const float maxWindSpeed = 32.4;  // m/s at max voltage
const float mps_to_kmh   = 3.6;
float windSpeed_kmh = 0.0;

// Task timers
unsigned long prev_ms_bmp   = 0, interval_bmp   = 100;
unsigned long prev_ms_wind  = 0, interval_wind  = 500;
unsigned long prev_ms_load  = 0, interval_load  = 500;
unsigned long prev_ms_rtc   = 0, interval_rtc   = 1000;
unsigned long prev_ms_print = 0, interval_print = 2000;

// Serial TX buffer
char txBuffer[200];

// ================================================================
// FUNCTION PROTOTYPES
// ================================================================
void update_bmp_logic();
void construct_and_send();
void check_relay_schedule();

// ================================================================
// SETUP
// ================================================================
void setup() {
  wdt_disable();
  delay(2000);

  pinMode(statusLedPin,  OUTPUT);
  pinMode(relay1Pin,     OUTPUT);
  pinMode(relay2Pin,     OUTPUT);
  pinMode(tareButtonPin, INPUT_PULLUP);
  digitalWrite(relay1Pin,    RELAY_OFF);
  digitalWrite(relay2Pin,    RELAY_OFF);
  digitalWrite(statusLedPin, LOW);

  Serial.begin(115200);
  Serial.println(F("=== Mega Sensor Hub V2 ==="));
  Serial.println(F("Serial1 (Pin18/19) -> ESP32 GPIO27/26"));

  // Serial1 -> ESP32 UART (Pin 18=TX1, Pin 19=RX1)
  Serial1.begin(115200);

  Wire.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(3000, true);

  if (!bmp1.begin_I2C(BMP1_I2C_ADDRESS)) Serial.println(F("[ERR] BMP1 I2C Fail"));
  else                                    Serial.println(F("[OK]  BMP1 I2C Ready"));

  if (!bmp2.begin_SPI(BMP2_CS))          Serial.println(F("[ERR] BMP2 SPI Fail"));
  else                                    Serial.println(F("[OK]  BMP2 SPI Ready"));

  bmp1.setTemperatureOversampling(BMP3_OVERSAMPLING_4X);
  bmp1.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp1.setOutputDataRate(BMP3_ODR_50_HZ);
  bmp1.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);

  bmp2.setTemperatureOversampling(BMP3_OVERSAMPLING_4X);
  bmp2.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp2.setOutputDataRate(BMP3_ODR_50_HZ);
  bmp2.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);

  cal_start_time = millis();

  LoadCell.begin();
  LoadCell.start(2000, false);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println(F("[ERR] Load Cell Timeout! Check HX711 wiring."));
  } else {
    LoadCell.setCalFactor(savedCalibrationFactor);
    Serial.println(F("[OK]  Load Cell Ready"));
  }

  int sig;
  EEPROM.get(EEPROM_SIG_ADDR_LC, sig);
  if (sig == EEPROM_SIGNATURE_LC) {
    long savedTare;
    EEPROM.get(EEPROM_TARE_ADDR_LC, savedTare);
    LoadCell.setTareOffset(savedTare);
    Serial.println(F("[OK]  Load Cell Tare Loaded from EEPROM"));
  } else {
    Serial.println(F("[INFO] No Tare in EEPROM. Press A6 button to zero."));
  }

  wdt_enable(WDTO_8S);
  Serial.println(F("--- System Live (WDT=8s Active) ---"));
}

// ================================================================
// LOOP
// ================================================================
void loop() {
  wdt_reset();

  if (Wire.getWireTimeoutFlag()) {
    Wire.clearWireTimeoutFlag();
    system_error = true;
  } else {
    system_error = false;
  }

  if (system_error) {
    digitalWrite(statusLedPin, HIGH);
  } else {
    if (digitalRead(statusLedPin) == HIGH &&
        (millis() - led_blink_start > LED_BLINK_DUR)) {
      digitalWrite(statusLedPin, LOW);
    }
  }

  LoadCell.update();

  static bool btn = HIGH, lastBtn = HIGH;
  static unsigned long debounceT = 0;
  bool reading = digitalRead(tareButtonPin);
  if (reading != lastBtn) debounceT = millis();
  if ((millis() - debounceT) > 50) {
    if (reading != btn) {
      btn = reading;
      if (btn == LOW) {
        Serial.println(F("Tare pressed — zeroing load cell..."));
        LoadCell.tareNoDelay();
      }
    }
  }
  lastBtn = reading;

  if (LoadCell.getTareStatus()) {
    long newTare = LoadCell.getTareOffset();
    EEPROM.put(EEPROM_SIG_ADDR_LC,  EEPROM_SIGNATURE_LC);
    EEPROM.put(EEPROM_TARE_ADDR_LC, newTare);
    Serial.println(F("[OK] Tare saved to EEPROM"));
  }

  unsigned long now = millis();

  if (!agl_calibrated && (now - cal_start_time >= AGL_CAL_DELAY)) {
    if (bmp2.performReading()) {
      ground_pressure_hpa = bmp2.pressure / 100.0;
      agl_calibrated = true;
      Serial.print(F("[OK] AGL Baseline: "));
      Serial.print(ground_pressure_hpa); Serial.println(F(" hPa"));
    }
  }

  if (now - prev_ms_bmp >= interval_bmp) {
    update_bmp_logic();
    prev_ms_bmp = now;
  }

  if (now - prev_ms_wind >= interval_wind) {
    float v = (analogRead(anemometerPin) / 1023.0) * 5.0;
    v = constrain(v, minVoltage, maxVoltage);
    windSpeed_kmh = ((v - minVoltage) / (maxVoltage - minVoltage)) * maxWindSpeed * mps_to_kmh;
    prev_ms_wind = now;
  }

  if (now - prev_ms_load >= interval_load) {
    weight_kg = LoadCell.getData() / 1000.0;
    prev_ms_load = now;
  }

  if (now - prev_ms_rtc >= interval_rtc) {
    rtc.refresh();
    snprintf(rtcBuffer, sizeof(rtcBuffer), "%04d/%02d/%02d-%02d:%02d:%02d",
             rtc.year(), rtc.month(), rtc.day(),
             rtc.hour(), rtc.minute(), rtc.second());
    check_relay_schedule();
    prev_ms_rtc = now;
  }

  if (now - prev_ms_print >= interval_print) {
    construct_and_send();
    prev_ms_print = now;
  }
}

// ================================================================
// HELPER FUNCTIONS
// ================================================================

void update_bmp_logic() {
  const float alpha = 0.2;

  if (bmp1.performReading()) {
    double rT = bmp1.temperature, rP = bmp1.pressure / 100.0;
    temp1_bmp         = (temp1_bmp == 0)         ? rT : temp1_bmp         * (1.0 - alpha) + rT * alpha;
    pressure1_bmp_hpa = (pressure1_bmp_hpa == 0) ? rP : pressure1_bmp_hpa * (1.0 - alpha) + rP * alpha;
    altitude1_bmp     = bmp1.readAltitude(SEALEVELPRESSURE_HPA);
  }

  if (bmp2.performReading()) {
    double rT = bmp2.temperature, rP = bmp2.pressure / 100.0;
    temp2_bmp         = (temp2_bmp == 0)         ? rT : temp2_bmp         * (1.0 - alpha) + rT * alpha;
    pressure2_bmp_hpa = (pressure2_bmp_hpa == 0) ? rP : pressure2_bmp_hpa * (1.0 - alpha) + rP * alpha;
    if (agl_calibrated)
      altitude2_bmp_agl = 44330.0 * (1.0 - pow(pressure2_bmp_hpa / ground_pressure_hpa, 0.1903));
  }

  pressure_difference_hpa = pressure1_bmp_hpa - pressure2_bmp_hpa;
}

void construct_and_send() {
  memset(txBuffer, 0, sizeof(txBuffer));

  char s_t1[8], s_p1[10], s_a1[8];
  char s_t2[8], s_p2[10], s_a2[8];
  char s_w[8],  s_l[8],   s_d[8];

  dtostrf(temp1_bmp,              5, 2, s_t1);
  dtostrf(pressure1_bmp_hpa,      7, 2, s_p1);
  dtostrf(altitude1_bmp,          6, 2, s_a1);
  dtostrf(temp2_bmp,              5, 2, s_t2);
  dtostrf(pressure2_bmp_hpa,      7, 2, s_p2);
  dtostrf(altitude2_bmp_agl,      6, 2, s_a2);
  dtostrf(windSpeed_kmh,          5, 2, s_w);
  dtostrf(weight_kg,              6, 3, s_l);
  dtostrf(pressure_difference_hpa,6, 2, s_d);

  snprintf(txBuffer, sizeof(txBuffer),
    "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s",
    rtcBuffer,
    s_t1, s_p1, s_a1,
    s_t2, s_p2, s_a2,
    s_w, s_l, s_d
  );

  // Send to ESP32 via Serial1 (Pin 18=TX1, Pin 19=RX1)
  Serial1.println(txBuffer);

  // Mirror to USB for debug
  Serial.print(F("[TX->ESP32] "));
  Serial.println(txBuffer);

  if (!system_error) {
    digitalWrite(statusLedPin, HIGH);
    led_blink_start = millis();
  }
}

void check_relay_schedule() {
  int now_m = (rtc.hour() * 60) + rtc.minute();

  int r1s = (R1_Start_Hour * 60) + R1_Start_Min;
  int r1e = (R1_Stop_Hour  * 60) + R1_Stop_Min;
  bool r1 = (r1s < r1e) ? (now_m >= r1s && now_m < r1e)
                         : (now_m >= r1s || now_m < r1e);
  digitalWrite(relay1Pin, r1 ? RELAY_ON : RELAY_OFF);

  int r2s = (R2_Start_Hour * 60) + R2_Start_Min;
  int r2e = (R2_Stop_Hour  * 60) + R2_Stop_Min;
  bool r2 = (r2s < r2e) ? (now_m >= r2s && now_m < r2e)
                         : (now_m >= r2s || now_m < r2e);
  digitalWrite(relay2Pin, r2 ? RELAY_ON : RELAY_OFF);
}
