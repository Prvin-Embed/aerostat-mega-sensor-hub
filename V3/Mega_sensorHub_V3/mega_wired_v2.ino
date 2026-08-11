/*
 * ================================================================
 * ARDUINO MEGA SENSOR HUB — VERSION 2.1
 * ================================================================
 * Serial1 (Pin 18=TX, 19=RX) → ESP32 GPIO27/26 @ 115200
 * Serial3 (Pin 14=TX, 15=RX) ← NEO-M8N GPS    @   9600
 *
 * Active Sensors:
 *  1. Dual BMP390  (I2C addr 0x77 + SPI CS=2) — Pressure/Temp/Alt
 *  2. HX711 Load Cell (pins 5,6)              — Tether Tension kg
 *  3. Analog Anemometer (A5)                  — Wind Speed km/h
 *  4. DS3231 RTC (I2C 0x68)                   — Timestamp
 *  5. 2x Relay (A0, A1)                       — RTC-Scheduled
 *  6. NEO-M8N GPS (Serial3, Pin 15=RX3)       — Lat, Lon
 *  7. QMC5883L / HMC5883L (I2C auto-detect)   — Compass Heading
 *
 * Safety:
 *  - AVR Hardware Watchdog (8 s)
 *  - I2C Wire timeout + auto-clear
 *  - EEPROM persistent load cell tare
 *  - GPS timeout guard (sends 0.000000 until fix)
 *
 * GPS Wiring:
 *   GPS TX → Mega Pin 15 (RX3)
 *   GPS RX → Mega Pin 14 (TX3)
 *   GPS VCC → 5V  |  GPS GND → GND
 *
 * Compass Wiring (shared I2C bus):
 *   SDA → Mega Pin 20  |  SCL → Mega Pin 21
 *
 * CSV Output — 13 fields, NO labels:
 *   Timestamp, Temp1, Pressure1, Alt1_ASL,
 *   Temp2, Pressure2, Alt2_AGL,
 *   Wind_kmh, Load_kg, PressureDiff_hPa,
 *   Latitude, Longitude, Heading_deg
 * ================================================================
 */

#include <avr/wdt.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_BMP3XX.h"
#include <HX711_ADC.h>
#include <uRTCLib.h>
#include <EEPROM.h>
#include <TinyGPSPlus.h>

// ================================================================
// PIN DEFINITIONS
// ================================================================
const int statusLedPin  = A3;
const int relay1Pin     = A0;
const int relay2Pin     = A1;
const int RELAY_ON      = HIGH;
const int RELAY_OFF     = LOW;

int R1_Start_Hour = 10, R1_Start_Min = 00;
int R1_Stop_Hour  = 17, R1_Stop_Min  = 30;
int R2_Start_Hour = 10, R2_Start_Min = 00;
int R2_Stop_Hour  = 17, R2_Stop_Min  = 00;

#define BMP1_I2C_ADDRESS  0x77
#define BMP2_CS           2

const int HX711_dout    = 5;
const int HX711_sck     = 6;
const int tareButtonPin = A6;
const int anemometerPin = A5;

#define GPS_BAUD        9600
#define GPS_TIMEOUT_MS  10000UL

#define QMC5883L_ADDR   0x0D
#define HMC5883L_ADDR   0x1E

// ================================================================
// GLOBALS
// ================================================================
bool          system_error    = false;
unsigned long led_blink_start = 0;
const unsigned long LED_BLINK_DUR = 500;

#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BMP3XX bmp1, bmp2;
bool   agl_calibrated      = false;
double ground_pressure_hpa = 0.0;
unsigned long cal_start_time = 0;
const unsigned long AGL_CAL_DELAY = 5000;

double temp1_bmp = 0, pressure1_bmp_hpa = 0, altitude1_bmp     = 0;
double temp2_bmp = 0, pressure2_bmp_hpa = 0, altitude2_bmp_agl = 0;
double pressure_difference_hpa = 0;

uRTCLib rtc(0x68);
char rtcBuffer[25];

HX711_ADC LoadCell(HX711_dout, HX711_sck);
const float savedCalibrationFactor = -8.513999;
float weight_kg = 0.0;
const int EEPROM_SIG_ADDR_LC  = 0;
const int EEPROM_TARE_ADDR_LC = 4;
const int EEPROM_SIGNATURE_LC = 0x4154;

const float minVoltage   = 0.32;
const float maxVoltage   = 5.0;
const float maxWindSpeed = 32.4;
const float mps_to_kmh   = 3.6;
float windSpeed_kmh = 0.0;

TinyGPSPlus   gps;
double        gps_lat       = 0.0;
double        gps_lon       = 0.0;
unsigned long lastGpsByteMs = 0;

enum CompassType { COMPASS_NONE, COMPASS_QMC5883L, COMPASS_HMC5883L };
CompassType compassType    = COMPASS_NONE;
float       compass_heading = 0.0;

unsigned long prev_ms_bmp     = 0, interval_bmp     = 100;
unsigned long prev_ms_wind    = 0, interval_wind    = 500;
unsigned long prev_ms_load    = 0, interval_load    = 500;
unsigned long prev_ms_rtc     = 0, interval_rtc     = 1000;
unsigned long prev_ms_compass = 0, interval_compass = 250;
unsigned long prev_ms_print   = 0, interval_print   = 2000;

char txBuffer[240];

// ================================================================
// COMPASS INIT
// ================================================================
void init_compass() {
  Wire.beginTransmission(QMC5883L_ADDR);
  if (Wire.endTransmission() == 0) {
    Wire.beginTransmission(QMC5883L_ADDR);
    Wire.write(0x0B); Wire.write(0x01); Wire.endTransmission();
    Wire.beginTransmission(QMC5883L_ADDR);
    Wire.write(0x0A); Wire.write(0x80); Wire.endTransmission();
    delay(15);
    Wire.beginTransmission(QMC5883L_ADDR);
    Wire.write(0x09); Wire.write(0b00011101); Wire.endTransmission();
    compassType = COMPASS_QMC5883L;
    Serial.println(F("[OK] Compass: QMC5883L (0x0D)"));
    return;
  }
  Wire.beginTransmission(HMC5883L_ADDR);
  if (Wire.endTransmission() == 0) {
    Wire.beginTransmission(HMC5883L_ADDR);
    Wire.write(0x00); Wire.write(0x70); Wire.endTransmission();
    Wire.beginTransmission(HMC5883L_ADDR);
    Wire.write(0x01); Wire.write(0x20); Wire.endTransmission();
    Wire.beginTransmission(HMC5883L_ADDR);
    Wire.write(0x02); Wire.write(0x00); Wire.endTransmission();
    compassType = COMPASS_HMC5883L;
    Serial.println(F("[OK] Compass: HMC5883L (0x1E)"));
    return;
  }
  Serial.println(F("[WARN] Compass: not found on I2C"));
}

// ================================================================
// COMPASS READ
// ================================================================
void update_compass() {
  int16_t cx = 0, cy = 0;
  if (compassType == COMPASS_QMC5883L) {
    Wire.beginTransmission(QMC5883L_ADDR);
    Wire.write(0x06); Wire.endTransmission(false);
    Wire.requestFrom(QMC5883L_ADDR, (uint8_t)1);
    if (!Wire.available()) return;
    if (!(Wire.read() & 0x01)) return;
    Wire.beginTransmission(QMC5883L_ADDR);
    Wire.write(0x00); Wire.endTransmission(false);
    Wire.requestFrom(QMC5883L_ADDR, (uint8_t)6);
    if (Wire.available() < 6) return;
    cx = (int16_t)(Wire.read() | (Wire.read() << 8));
    cy = (int16_t)(Wire.read() | (Wire.read() << 8));
    Wire.read(); Wire.read();
  } else if (compassType == COMPASS_HMC5883L) {
    Wire.beginTransmission(HMC5883L_ADDR);
    Wire.write(0x03); Wire.endTransmission(false);
    Wire.requestFrom(HMC5883L_ADDR, (uint8_t)6);
    if (Wire.available() < 6) return;
    cx = (int16_t)((Wire.read() << 8) | Wire.read());
    Wire.read(); Wire.read();
    cy = (int16_t)((Wire.read() << 8) | Wire.read());
    if (cx == -4096 || cy == -4096) return;
  } else { return; }
  float h = atan2((float)cy, (float)cx) * (180.0 / M_PI);
  if (h < 0) h += 360.0;
  compass_heading = h;
}

// ================================================================
// GPS READ
// ================================================================
void update_gps() {
  while (Serial3.available()) {
    char c = (char)Serial3.read();
    if (gps.encode(c)) lastGpsByteMs = millis();
  }
  if (gps.location.isValid()) {
    gps_lat = gps.location.lat();
    gps_lon = gps.location.lng();
  }
}

// ================================================================
// BMP LOGIC
// ================================================================
void update_bmp_logic() {
  const float alpha = 0.2f;
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

// ================================================================
// BUILD & SEND
// ================================================================
void construct_and_send() {
  memset(txBuffer, 0, sizeof(txBuffer));
  char s_t1[8], s_p1[10], s_a1[8];
  char s_t2[8], s_p2[10], s_a2[8];
  char s_w[8],  s_l[9],   s_d[9];
  char s_lat[14], s_lon[14], s_hdg[8];

  dtostrf(temp1_bmp,               5, 2, s_t1);
  dtostrf(pressure1_bmp_hpa,       7, 2, s_p1);
  dtostrf(altitude1_bmp,           6, 2, s_a1);
  dtostrf(temp2_bmp,               5, 2, s_t2);
  dtostrf(pressure2_bmp_hpa,       7, 2, s_p2);
  dtostrf(altitude2_bmp_agl,       6, 2, s_a2);
  dtostrf(windSpeed_kmh,           5, 2, s_w);
  dtostrf(weight_kg,               6, 3, s_l);
  dtostrf(pressure_difference_hpa, 6, 2, s_d);
  dtostrf(gps_lat,                10, 6, s_lat);
  dtostrf(gps_lon,                10, 6, s_lon);
  dtostrf(compass_heading,         6, 1, s_hdg);

  snprintf(txBuffer, sizeof(txBuffer),
           "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s",
           rtcBuffer,
           s_t1, s_p1, s_a1,
           s_t2, s_p2, s_a2,
           s_w,  s_l,  s_d,
           s_lat, s_lon, s_hdg);

  Serial1.println(txBuffer);

  if ((millis() - lastGpsByteMs) > GPS_TIMEOUT_MS)
    Serial.println(F("[GPS] No fix — Lat/Lon = 0.000000"));

  Serial.print(F("[TX->ESP32] "));
  Serial.println(txBuffer);

  if (!system_error) {
    digitalWrite(statusLedPin, HIGH);
    led_blink_start = millis();
  }
}

// ================================================================
// RELAY SCHEDULE
// ================================================================
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
  Serial.println(F("=== Mega Sensor Hub V2.1 ==="));
  Serial.println(F("Serial1(Pin18/19)->ESP32 | Serial3(Pin14/15)<-GPS"));

  Serial1.begin(115200);
  Serial3.begin(GPS_BAUD);
  lastGpsByteMs = millis();
  Serial.println(F("[OK] GPS on Serial3 (RX3=Pin15, TX3=Pin14) @ 9600"));

  Wire.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(3000, true);

  if (!bmp1.begin_I2C(BMP1_I2C_ADDRESS)) Serial.println(F("[ERR] BMP1 I2C Fail"));
  else Serial.println(F("[OK] BMP1 I2C Ready"));

  if (!bmp2.begin_SPI(BMP2_CS)) Serial.println(F("[ERR] BMP2 SPI Fail"));
  else Serial.println(F("[OK] BMP2 SPI Ready"));

  bmp1.setTemperatureOversampling(BMP3_OVERSAMPLING_4X);
  bmp1.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp1.setOutputDataRate(BMP3_ODR_50_HZ);
  bmp1.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);

  bmp2.setTemperatureOversampling(BMP3_OVERSAMPLING_4X);
  bmp2.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp2.setOutputDataRate(BMP3_ODR_50_HZ);
  bmp2.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);

  cal_start_time = millis();
  init_compass();

  LoadCell.begin();
  LoadCell.start(2000, false);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println(F("[ERR] Load Cell Timeout! Check HX711 wiring."));
  } else {
    LoadCell.setCalFactor(savedCalibrationFactor);
    Serial.println(F("[OK] Load Cell Ready"));
    int sig;
    EEPROM.get(EEPROM_SIG_ADDR_LC, sig);
    if (sig == EEPROM_SIGNATURE_LC) {
      long savedTare;
      EEPROM.get(EEPROM_TARE_ADDR_LC, savedTare);
      LoadCell.setTareOffset(savedTare);
      Serial.println(F("[OK] Load Cell Tare Loaded from EEPROM"));
    } else {
      Serial.println(F("[INFO] No Tare in EEPROM. Press A6 to zero."));
    }
  }

  wdt_enable(WDTO_8S);
  Serial.println(F("--- System Live (WDT=8s) ---"));
  Serial.println(F("CSV: Timestamp,T1,P1,A1,T2,P2,A2,Wind,Load,dP,Lat,Lon,Heading"));
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
        (millis() - led_blink_start > LED_BLINK_DUR))
      digitalWrite(statusLedPin, LOW);
  }

  update_gps();
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

  if (now - prev_ms_bmp     >= interval_bmp)     { update_bmp_logic();  prev_ms_bmp     = now; }
  if (now - prev_ms_wind    >= interval_wind)    {
    float v = (analogRead(anemometerPin) / 1023.0) * 5.0;
    v = constrain(v, minVoltage, maxVoltage);
    windSpeed_kmh = ((v - minVoltage) / (maxVoltage - minVoltage)) * maxWindSpeed * mps_to_kmh;
    prev_ms_wind = now;
  }
  if (now - prev_ms_load    >= interval_load)    { weight_kg = LoadCell.getData() / 1000.0; prev_ms_load = now; }
  if (now - prev_ms_rtc     >= interval_rtc)     {
    rtc.refresh();
    snprintf(rtcBuffer, sizeof(rtcBuffer), "%04d/%02d/%02d-%02d:%02d:%02d",
             rtc.year(), rtc.month(), rtc.day(),
             rtc.hour(), rtc.minute(), rtc.second());
    check_relay_schedule();
    prev_ms_rtc = now;
  }
  if (now - prev_ms_compass >= interval_compass) { update_compass();     prev_ms_compass = now; }
  if (now - prev_ms_print   >= interval_print)   { construct_and_send(); prev_ms_print   = now; }
}