/*
 * ----------------------------------------------------------------
 * FINAL ROBUST ARDUINO MEGA CODE (24/7 STABILITY EDITION)
 * ----------------------------------------------------------------
 * Architecture: Sensor Collector
 * Output: Serial (TX0/RX0)
 * Safety:
 *   1. Hardware Watchdog Timer (8s)
 *   2. I2C Bus Timeout & Recovery (Prevents Wire freeze)
 *   3. EEPROM Gyro Calibration (Auto-Calibrate on First Run)
 *   4. SD Card Hot-Swap/Recovery support
 *   5. Status LED & Relay Automation
 *   6. Load Cell EEPROM Permanent Zero (Survives Power Loss)
 * ----------------------------------------------------------------
 */

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <EEPROM.h>
#include "Adafruit_BMP3XX.h"
#include <uRTCLib.h>
#include <HX711_ADC.h>
#include <avr/wdt.h>
#include <math.h>

// ================================================================
// PIN DEFINITIONS
// ================================================================

// Status Indicators & Controls
const int statusLedPin = A3;  // Blink on TX, Solid on Error
const int relay1Pin    = A0;  // Relay 1 GPIO
const int relay2Pin    = A1;  // Relay 2 GPIO

// --- RELAY CONFIGURATION ---
// Many relay modules are "Active LOW" (LOW turns them ON).
// If your relays work backwards (ON when they should be OFF), swap these values.
const int RELAY_ON  = HIGH;
const int RELAY_OFF = LOW;

// Relay 1 Schedule
int R1_Start_Hour = 10;
int R1_Start_Min  = 00;
int R1_Stop_Hour  = 17;
int R1_Stop_Min   = 30;

// Relay 2 Schedule
int R2_Start_Hour = 10;
int R2_Start_Min  = 00;
int R2_Stop_Hour  = 17;
int R2_Stop_Min   = 00;

// SD Card
const int chipSelect = 53;

// BMP390
#define BMP1_I2C_ADDRESS 0x77
#define BMP2_CS 2

// Load Cell (HX711)
const int HX711_dout  = 5;
const int HX711_sck   = 6;
const int tareButtonPin = A6; // Button to tare load cell (Connect between A6 and GND)

// Anemometer
const int anemometerPin = A5;

// MPU9250 (I2C)
const int MPU_ADDR    = 0x69;
const int PWR_MGMT_1  = 0x6B;
const int ACCEL_XOUT_H = 0x3B;
const int GYRO_XOUT_H  = 0x43;

// ================================================================
// GLOBAL VARIABLES
// ================================================================

// --- System Status ---
bool system_error = false;
unsigned long led_blink_start = 0;
const unsigned long LED_BLINK_DURATION = 500; // ms

// --- MPU9250 ---
int16_t accelX, accelY, accelZ;
int16_t gyroX, gyroY, gyroZ;
float angleX = 0, angleY = 0, angleZ = 0;
float gyroErrorX = 0, gyroErrorY = 0, gyroErrorZ = 0;
unsigned long currTimeMPU, prevTimeMPU;
float elapsedTimeMPU;
float currentYaw, currentPitch, currentRoll;

// DRIFT FIX: Deadzone Threshold
// Any rotation slower than 0.5 deg/sec is ignored as noise
#define GYRO_DEADZONE 0.5

// --- CALIBRATION CONFIG (CODE ONLY) ---
// Change this value (e.g. to 0x13, 0x14) to FORCE a new calibration upon upload.
#define CALIBRATION_MAGIC_BYTE 0x12

// --- BMP390 ---
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BMP3XX bmp1, bmp2;
bool agl_calibrated = false;
double ground_pressure_hpa = 0.0;
unsigned long calibration_start_time = 0;
const unsigned long CALIBRATION_DELAY = 5000;

// Filtered Data Holders (Exponential Moving Average)
double temp1_bmp = 0, pressure1_bmp_hpa = 0, altitude1_bmp = 0;
double temp2_bmp = 0, pressure2_bmp_hpa = 0, altitude2_bmp_agl = 0;
double pressure_difference_hpa = 0;

// --- RTC DS3231 ---
uRTCLib rtc(0x68);
char rtcBuffer[25];

// --- Load Cell ---
HX711_ADC LoadCell(HX711_dout, HX711_sck);
const float savedCalibrationFactor = -8.513999;
// For 500kg load cell: -8.513999
// For 100kg load cell: -44.013999
float weight_kg = 0.0;

// LOAD CELL EEPROM Addresses (Starting at 20 to avoid overwriting MPU9250 calibration at 0-12)
const int EEPROM_SIG_ADDR_LC  = 20;
const int EEPROM_TARE_ADDR_LC = 24;
const int EEPROM_SIGNATURE_LC = 0x4154; // "AT"

// --- Anemometer ---
const float minVoltage   = 0.32;
const float maxVoltage   = 5.0;
const float maxWindSpeed = 32.4; // m/s
const float mps_to_kmh   = 3.6;
float windSpeed_kmh = 0.0;

// --- Timing Intervals ---
unsigned long prev_ms_rtc   = 0, interval_rtc   = 1000;
unsigned long prev_ms_bmp   = 0, interval_bmp   = 100;
unsigned long prev_ms_wind  = 0, interval_wind  = 500;
unsigned long prev_ms_load  = 0, interval_load  = 500;
unsigned long prev_ms_print = 0, interval_print = 2000;

// SD Recovery
unsigned long prev_ms_sd_retry = 0;
bool sd_online = false;

// --- Data Buffer ---
char txBuffer[300];

// ================================================================
// EEPROM HELPERS
// ================================================================
struct MPUCalib {
  float x;
  float y;
  float z;
  byte valid;
};

bool load_imu_calibration() {
  MPUCalib cal;
  EEPROM.get(0, cal);
  if (cal.valid == CALIBRATION_MAGIC_BYTE) {
    gyroErrorX = cal.x;
    gyroErrorY = cal.y;
    gyroErrorZ = cal.z;
    Serial.println(F("[OK] Valid IMU Cal loaded from EEPROM"));
    return true;
  } else {
    Serial.println(F("[INFO] No valid calibration found (New Chip or Code Update)."));
    return false;
  }
}

void run_and_save_calibration() {
  Serial.println(F("!!! AUTO-CALIBRATION STARTED !!!"));
  Serial.println(F("DO NOT MOVE THE SENSOR..."));

  for (int i = 0; i < 20; i++) {
    digitalWrite(statusLedPin, HIGH); delay(25);
    digitalWrite(statusLedPin, LOW);  delay(25);
    wdt_reset();
  }

  float gX = 0, gY = 0, gZ = 0;
  int c = 0;
  int samples = 2000;

  while (c < samples) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(GYRO_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);
    gX += (int16_t)(Wire.read() << 8 | Wire.read());
    gY += (int16_t)(Wire.read() << 8 | Wire.read());
    gZ += (int16_t)(Wire.read() << 8 | Wire.read());
    c++;
    wdt_reset();
    delay(2);
  }

  MPUCalib cal;
  cal.x = gX / (float)samples;
  cal.y = gY / (float)samples;
  cal.z = gZ / (float)samples;
  cal.valid = CALIBRATION_MAGIC_BYTE;

  EEPROM.put(0, cal);
  gyroErrorX = cal.x;
  gyroErrorY = cal.y;
  gyroErrorZ = cal.z;

  Serial.print(F("Calibration Complete. Offset Z: ")); Serial.println(gyroErrorZ);
  digitalWrite(statusLedPin, HIGH); delay(1000); digitalWrite(statusLedPin, LOW);
}

// ================================================================
// FUNCTION PROTOTYPES
// ================================================================
void wakeUpMPU();
void readSensorData();
void update_custom_mpu();
void update_bmp_logic();
void construct_and_send();
void check_relay_schedule();

// ================================================================
// SETUP
// ================================================================
void setup() {
  // 1. Watchdog Init
  wdt_disable();
  delay(2000);

  // 2. IO Init
  pinMode(statusLedPin, OUTPUT);
  pinMode(relay1Pin,    OUTPUT);
  pinMode(relay2Pin,    OUTPUT);
  pinMode(tareButtonPin, INPUT_PULLUP);

  digitalWrite(relay1Pin,    RELAY_OFF);
  digitalWrite(relay2Pin,    RELAY_OFF);
  digitalWrite(statusLedPin, LOW);

  Serial.begin(115200);
  Serial.println(F("--- Mega Sensor Hub Starting (Robust) ---"));

  // 3. I2C Init with SAFETY TIMEOUT
  Wire.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(3000, true);

  // 4. MPU9250 Init
  wakeUpMPU();
  if (!load_imu_calibration()) {
    run_and_save_calibration();
  }
  prevTimeMPU = micros();

  // 5. SD Card Init
  pinMode(chipSelect, OUTPUT);
  if (SD.begin(chipSelect)) {
    sd_online = true;
    Serial.println(F("[OK] SD Ready"));
  } else {
    sd_online = false;
    Serial.println(F("[ERROR] SD Init Failed (Will retry in loop)"));
  }

  // 6. BMP390 Init
  if (!bmp1.begin_I2C(BMP1_I2C_ADDRESS)) Serial.println(F("[ERR] BMP1 (I2C) Fail"));
  if (!bmp2.begin_SPI(BMP2_CS))          Serial.println(F("[ERR] BMP2 (SPI) Fail"));

  bmp1.setTemperatureOversampling(BMP3_OVERSAMPLING_4X);
  bmp1.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp1.setOutputDataRate(BMP3_ODR_50_HZ);
  bmp1.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);

  bmp2.setTemperatureOversampling(BMP3_OVERSAMPLING_4X);
  bmp2.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp2.setOutputDataRate(BMP3_ODR_50_HZ);
  bmp2.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);

  calibration_start_time = millis();

  // 7. Load Cell Init
  LoadCell.begin();
  LoadCell.start(2000, false); // false = do NOT auto-tare on startup
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println(F("[ERROR] Load Cell Timeout!"));
  } else {
    LoadCell.setCalFactor(savedCalibrationFactor);
  }

  int signatureLC;
  EEPROM.get(EEPROM_SIG_ADDR_LC, signatureLC);
  if (signatureLC == EEPROM_SIGNATURE_LC) {
    long savedTareOffset;
    EEPROM.get(EEPROM_TARE_ADDR_LC, savedTareOffset);
    LoadCell.setTareOffset(savedTareOffset);
    Serial.println(F("[OK] Load Cell Tare Offset Loaded from EEPROM"));
  } else {
    Serial.println(F("[INFO] No Load Cell Tare found. Press Tare button (A6) to set initial Zero."));
  }

  // 8. Enable Watchdog (8 Seconds)
  wdt_enable(WDTO_8S);
  Serial.println(F("--- System Live (WDT Active) ---"));
}

// ================================================================
// LOOP
// ================================================================
void loop() {
  wdt_reset();

  // I2C Freeze & Error Check
  bool i2c_error = false;
  if (Wire.getWireTimeoutFlag()) {
    Wire.clearWireTimeoutFlag();
    i2c_error = true;
  }

  system_error = (!sd_online || i2c_error);

  // LED CONTROL (Non-blocking)
  if (system_error) {
    digitalWrite(statusLedPin, HIGH);
  } else {
    if (digitalRead(statusLedPin) == HIGH && (millis() - led_blink_start > LED_BLINK_DURATION)) {
      digitalWrite(statusLedPin, LOW);
    }
  }

  // --- CONTINUOUS SENSOR POLLING ---
  LoadCell.update();

  // Tare Button with debounce
  static bool buttonState     = HIGH;
  static bool lastButtonState = HIGH;
  static unsigned long lastDebounceTime = 0;
  const unsigned long debounceDelay = 50;

  bool reading = digitalRead(tareButtonPin);
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == LOW) {
        Serial.println(F("Tare Button Pressed! Taring Load Cell..."));
        LoadCell.tareNoDelay();
      }
    }
  }
  lastButtonState = reading;

  if (LoadCell.getTareStatus() == true) {
    long newTareOffset = LoadCell.getTareOffset();
    EEPROM.put(EEPROM_SIG_ADDR_LC,  EEPROM_SIGNATURE_LC);
    EEPROM.put(EEPROM_TARE_ADDR_LC, newTareOffset);
    Serial.println(F("[OK] New Zero Point saved permanently to EEPROM"));
  }

  unsigned long now = millis();

  // 1. MPU Logic
  update_custom_mpu();

  // 2. AGL Baseline Calibration (once, 5s after boot)
  if (!agl_calibrated && now - calibration_start_time >= CALIBRATION_DELAY) {
    if (bmp2.performReading()) {
      ground_pressure_hpa = bmp2.pressure / 100.0;
      agl_calibrated = true;
      Serial.print(F("AGL Baseline set: "));
      Serial.println(ground_pressure_hpa);
    }
  }

  // 3. Wind Speed
  if (now - prev_ms_wind >= interval_wind) {
    int adc = analogRead(anemometerPin);
    float v = (adc / 1023.0) * 5.0;
    v = constrain(v, minVoltage, maxVoltage);
    float mps = ((v - minVoltage) / (maxVoltage - minVoltage)) * maxWindSpeed;
    windSpeed_kmh = mps * mps_to_kmh;
    prev_ms_wind = now;
  }

  // 4. Load Cell
  if (now - prev_ms_load >= interval_load) {
    weight_kg = LoadCell.getData() / 1000.0;
    prev_ms_load = now;
  }

  // 5. RTC + Relay Logic
  if (now - prev_ms_rtc >= interval_rtc) {
    rtc.refresh();
    snprintf(rtcBuffer, sizeof(rtcBuffer), "%04d/%02d/%02d-%02d:%02d:%02d",
             rtc.year(), rtc.month(), rtc.day(),
             rtc.hour(), rtc.minute(), rtc.second());
    check_relay_schedule();
    prev_ms_rtc = now;
  }

  // 6. BMP Data
  if (now - prev_ms_bmp >= interval_bmp) {
    update_bmp_logic();
    prev_ms_bmp = now;
  }

  // 7. Construct & Send
  if (now - prev_ms_print >= interval_print) {
    construct_and_send();
    prev_ms_print = now;
  }
}

// ================================================================
// HELPER FUNCTIONS
// ================================================================

void check_relay_schedule() {
  int currentMinutesOfDay = (rtc.hour() * 60) + rtc.minute();

  int r1Start = (R1_Start_Hour * 60) + R1_Start_Min;
  int r1Stop  = (R1_Stop_Hour  * 60) + R1_Stop_Min;
  bool shouldRelay1BeOn = false;
  if (r1Start < r1Stop) {
    if (currentMinutesOfDay >= r1Start && currentMinutesOfDay < r1Stop) shouldRelay1BeOn = true;
  } else {
    if (currentMinutesOfDay >= r1Start || currentMinutesOfDay < r1Stop) shouldRelay1BeOn = true;
  }

  int r2Start = (R2_Start_Hour * 60) + R2_Start_Min;
  int r2Stop  = (R2_Stop_Hour  * 60) + R2_Stop_Min;
  bool shouldRelay2BeOn = false;
  if (r2Start < r2Stop) {
    if (currentMinutesOfDay >= r2Start && currentMinutesOfDay < r2Stop) shouldRelay2BeOn = true;
  } else {
    if (currentMinutesOfDay >= r2Start || currentMinutesOfDay < r2Stop) shouldRelay2BeOn = true;
  }

  digitalWrite(relay1Pin, shouldRelay1BeOn ? RELAY_ON : RELAY_OFF);
  digitalWrite(relay2Pin, shouldRelay2BeOn ? RELAY_ON : RELAY_OFF);
}

void update_bmp_logic() {
  float alpha = 0.2;
  if (bmp1.performReading()) {
    double rawT = bmp1.temperature;
    double rawP = bmp1.pressure / 100.0;
    temp1_bmp        = (temp1_bmp == 0)        ? rawT : (temp1_bmp * (1.0 - alpha))        + (rawT * alpha);
    pressure1_bmp_hpa = (pressure1_bmp_hpa == 0) ? rawP : (pressure1_bmp_hpa * (1.0 - alpha)) + (rawP * alpha);
    altitude1_bmp    = bmp1.readAltitude(SEALEVELPRESSURE_HPA);
  }
  if (bmp2.performReading()) {
    double rawT = bmp2.temperature;
    double rawP = bmp2.pressure / 100.0;
    temp2_bmp        = (temp2_bmp == 0)        ? rawT : (temp2_bmp * (1.0 - alpha))        + (rawT * alpha);
    pressure2_bmp_hpa = (pressure2_bmp_hpa == 0) ? rawP : (pressure2_bmp_hpa * (1.0 - alpha)) + (rawP * alpha);
    if (agl_calibrated) {
      altitude2_bmp_agl = 44330.0 * (1.0 - pow(pressure2_bmp_hpa / ground_pressure_hpa, 0.1903));
    }
  }
  pressure_difference_hpa = pressure1_bmp_hpa - pressure2_bmp_hpa;
}

void construct_and_send() {
  memset(txBuffer, 0, sizeof(txBuffer));

  char s_yaw[8], s_pitch[8], s_roll[8];
  char s_t1[8],  s_p1[10],  s_a1[8];
  char s_t2[8],  s_p2[10],  s_a2[8];
  char s_wind[8], s_load[8], s_diff[8];

  dtostrf(currentYaw,   6, 2, s_yaw);
  dtostrf(currentPitch, 6, 2, s_pitch);
  dtostrf(currentRoll,  6, 2, s_roll);

  dtostrf(temp1_bmp,        5, 2, s_t1);
  dtostrf(pressure1_bmp_hpa, 7, 2, s_p1);
  dtostrf(altitude1_bmp,    6, 2, s_a1);

  dtostrf(temp2_bmp,         5, 2, s_t2);
  dtostrf(pressure2_bmp_hpa,  7, 2, s_p2);
  dtostrf(altitude2_bmp_agl, 6, 2, s_a2);

  dtostrf(windSpeed_kmh,       5, 2, s_wind);
  dtostrf(weight_kg,           6, 3, s_load);
  dtostrf(pressure_difference_hpa, 6, 2, s_diff);

  snprintf(txBuffer, sizeof(txBuffer),
    "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s",
    rtcBuffer,
    s_yaw, s_pitch, s_roll,
    s_t1, s_p1, s_a1,
    s_t2, s_p2, s_a2,
    s_wind, s_load, s_diff
  );

  Serial.println(txBuffer);

  if (!system_error) {
    digitalWrite(statusLedPin, HIGH);
    led_blink_start = millis();
  }

  if (sd_online) {
    File f = SD.open("LOG.txt", FILE_WRITE);
    if (f) {
      f.println(txBuffer);
      f.close();
    } else {
      sd_online = false;
    }
  } else {
    if (millis() - prev_ms_sd_retry > 5000) {
      if (SD.begin(chipSelect)) {
        sd_online = true;
        File f = SD.open("LOG.txt", FILE_WRITE);
        if (f) { f.println("--- SD RECOVERED ---"); f.close(); }
      }
      prev_ms_sd_retry = millis();
    }
  }
}

void update_custom_mpu() {
  currTimeMPU = micros();
  elapsedTimeMPU = (currTimeMPU - prevTimeMPU) / 1000000.0;
  if (elapsedTimeMPU > 1.0) elapsedTimeMPU = 0;
  prevTimeMPU = currTimeMPU;

  readSensorData();

  float gyroRateX = (gyroX - gyroErrorX) / 131.0;
  float gyroRateY = (gyroY - gyroErrorY) / 131.0;
  float gyroRateZ = (gyroZ - gyroErrorZ) / 131.0;

  float accAngleX = (atan2(accelY, accelZ) * 180.0) / PI;
  float accAngleY = (atan2(-accelX, sqrt(pow(accelY, 2) + pow(accelZ, 2))) * 180.0) / PI;

  angleX = 0.96 * (angleX + gyroRateX * elapsedTimeMPU) + 0.04 * accAngleX;
  angleY = 0.96 * (angleY + gyroRateY * elapsedTimeMPU) + 0.04 * accAngleY;

  if (abs(gyroRateZ) > GYRO_DEADZONE) {
    angleZ = angleZ + gyroRateZ * elapsedTimeMPU;
  }

  currentRoll  = angleX;
  currentPitch = angleY;
  currentYaw   = angleZ;
}

void wakeUpMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0);
  Wire.endTransmission(true);
}

void readSensorData() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);
  if (Wire.available() >= 6) {
    accelX = (Wire.read() << 8 | Wire.read());
    accelY = (Wire.read() << 8 | Wire.read());
    accelZ = (Wire.read() << 8 | Wire.read());
  }

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);
  if (Wire.available() >= 6) {
    gyroX = (Wire.read() << 8 | Wire.read());
    gyroY = (Wire.read() << 8 | Wire.read());
    gyroZ = (Wire.read() << 8 | Wire.read());
  }
}
