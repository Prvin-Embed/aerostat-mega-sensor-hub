/*
 * ================================================================
 * ESP32 — TELEMETRY HUB V2
 * ================================================================
 * Receives  : Mega CSV string via UART1 (GPIO26=TX, GPIO27=RX)
 * Sensors   : BNO055 (I2C, Yaw/Pitch/Roll)
 *             DHT22  (GPIO25, Temp + Humidity)
 * Logs to   : SD Card (SPI, CS=GPIO13) — full CSV with header
 * Sends via : W5500 Ethernet (SPI, CS=GPIO5) — TCP server port 5000
 *
 * Final CSV (17 fields):
 *   Count, Timestamp, RTC_Time,
 *   Temp1(BMP-Gnd), Pressure1, Alt1_ASL,
 *   Temp2(BMP-Air), Pressure2, Alt2_AGL,
 *   Wind_kmh, Load_kg, PressureDiff_hPa,
 *   Yaw, Pitch, Roll,
 *   DHT_Temp, DHT_Humidity
 *
 * UART Wiring:
 *   ESP32 GPIO27 (RX) <- Arduino Mega Pin 18 (TX1)
 *   ESP32 GPIO26 (TX) -> Arduino Mega Pin 19 (RX1)
 *   Common GND required!
 *
 * SPI Pins (shared W5500 + SD):
 *   SCK=18, MISO=19, MOSI=23
 *   W5500 CS=5,  SD CS=13
 * ================================================================
 */

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Ethernet.h>
#include <DHT.h>
#include <EasyBNO055_ESP.h>

// ================================================================
// PIN DEFINITIONS
// ================================================================

// UART1 remapped — default GPIO9/10 conflicts with SPI flash
#define MEGA_RX_PIN  27   // ESP32 receives from Mega TX1 (Pin 18)
#define MEGA_TX_PIN  26   // ESP32 transmits to  Mega RX1 (Pin 19)
#define MEGA_BAUD    115200

// DHT22
#define DHTPIN   25
#define DHTTYPE  DHT22

// SPI
#define SCK_PIN   18
#define MISO_PIN  19
#define MOSI_PIN  23
#define W5500_CS   5
#define SD_CS     13

// ================================================================
// OBJECTS
// ================================================================

DHT dht(DHTPIN, DHTTYPE);
EasyBNO055_ESP bno;

byte      mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 1, 177);
EthernetServer server(5000);
EthernetClient client;

// ================================================================
// CONSTANTS & GLOBALS
// ================================================================

const char* LOG_FILE = "/TelemetryLog.csv";

const char* CSV_HEADER =
  "Count,ESP_Millis,RTC_Timestamp,"
  "Temp1_C,Pressure1_hPa,Alt1_ASL_m,"
  "Temp2_C,Pressure2_hPa,Alt2_AGL_m,"
  "Wind_kmh,Load_kg,PressureDiff_hPa,"
  "Yaw_deg,Pitch_deg,Roll_deg,"
  "DHT_Temp_C,DHT_Humidity_pct";

unsigned long counter        = 0;
unsigned long previousMillis = 0;
const long    SEND_INTERVAL  = 2000;

bool sd_ok = false;

#define MEGA_BUF_SIZE 220
char   megaBuf[MEGA_BUF_SIZE];
int    megaBufIdx = 0;
bool   megaLineReady = false;

char   mega_timestamp[25]  = "0000/00/00-00:00:00";
float  mega_temp1     = 0, mega_pressure1 = 0, mega_alt1    = 0;
float  mega_temp2     = 0, mega_pressure2 = 0, mega_alt2    = 0;
float  mega_wind      = 0, mega_load      = 0, mega_pdiff   = 0;
bool   mega_data_fresh = false;

// ================================================================
// BNO055 I2C callback (required by EasyBNO055_ESP)
// ================================================================
void otherI2CUpdate() { }

// ================================================================
// NON-BLOCKING MEGA SERIAL READER
// ================================================================
void readMegaSerial() {
  while (Serial1.available()) {
    char c = (char)Serial1.read();
    if (c == '\n') {
      megaBuf[megaBufIdx] = '\0';
      megaBufIdx    = 0;
      megaLineReady = true;
      return;
    }
    if (c != '\r' && megaBufIdx < MEGA_BUF_SIZE - 1) {
      megaBuf[megaBufIdx++] = c;
    }
  }
}

float parseNextFloat(char** ptr) {
  char* start = *ptr;
  char* end   = strchr(start, ',');
  if (end) { *end = '\0'; *ptr = end + 1; }
  else      { *ptr = start + strlen(start); }
  return atof(start);
}

void parseMegaLine() {
  char tmp[MEGA_BUF_SIZE];
  strncpy(tmp, megaBuf, MEGA_BUF_SIZE);
  tmp[MEGA_BUF_SIZE - 1] = '\0';

  char* ptr = tmp;

  // Field 0: Timestamp string
  char* tsEnd = strchr(ptr, ',');
  if (!tsEnd) return;
  *tsEnd = '\0';
  strncpy(mega_timestamp, ptr, sizeof(mega_timestamp) - 1);
  ptr = tsEnd + 1;

  mega_temp1     = parseNextFloat(&ptr);
  mega_pressure1 = parseNextFloat(&ptr);
  mega_alt1      = parseNextFloat(&ptr);
  mega_temp2     = parseNextFloat(&ptr);
  mega_pressure2 = parseNextFloat(&ptr);
  mega_alt2      = parseNextFloat(&ptr);
  mega_wind      = parseNextFloat(&ptr);
  mega_load      = parseNextFloat(&ptr);
  mega_pdiff     = parseNextFloat(&ptr);

  mega_data_fresh = true;
}

// ================================================================
// SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  Serial.println(F("=== ESP32 Telemetry Hub V2 ==="));

  // UART1 remapped to GPIO27(RX) / GPIO26(TX)
  Serial1.begin(MEGA_BAUD, SERIAL_8N1, MEGA_RX_PIN, MEGA_TX_PIN);
  Serial.println(F("[OK] UART1 ready on GPIO27(RX)/26(TX) <- Mega Serial1"));

  dht.begin();
  Serial.println(F("[OK] DHT22 ready on GPIO25"));

  bno.start(&otherI2CUpdate);
  Serial.println(F("[OK] BNO055 ready (I2C)"));

  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN);

  if (!SD.begin(SD_CS)) {
    Serial.println(F("[ERR] SD Card failed! Check GPIO13."));
    sd_ok = false;
  } else {
    Serial.println(F("[OK] SD Card ready on GPIO13"));
    sd_ok = true;
    if (!SD.exists(LOG_FILE)) {
      File f = SD.open(LOG_FILE, FILE_WRITE);
      if (f) {
        f.println(CSV_HEADER);
        f.close();
        Serial.println(F("[OK] CSV header written"));
      }
    } else {
      Serial.println(F("[OK] Existing log file found — appending"));
    }
  }

  Ethernet.init(W5500_CS);
  Ethernet.begin(mac, ip);
  delay(1000);
  Serial.print(F("[OK] W5500 IP: "));
  Serial.println(Ethernet.localIP());

  server.begin();
  Serial.println(F("[OK] TCP Server started on port 5000"));
  Serial.println(F("--- System Live. Waiting for Mega data... ---"));
}

// ================================================================
// LOOP
// ================================================================
void loop() {
  // 1. Maintain TCP client connection
  if (!client || !client.connected()) {
    client = server.available();
    if (client) Serial.println(F("[NET] TCP Client connected"));
  }

  // 2. Non-blocking read from Mega UART1
  readMegaSerial();

  // 3. Parse line when ready
  if (megaLineReady) {
    megaLineReady = false;
    parseMegaLine();
  }

  // 4. Main publish loop every SEND_INTERVAL ms
  unsigned long now = millis();
  if (now - previousMillis >= SEND_INTERVAL) {
    previousMillis = now;

    float dht_temp = dht.readTemperature();
    float dht_hum  = dht.readHumidity();
    if (isnan(dht_temp)) dht_temp = -99.0;
    if (isnan(dht_hum))  dht_hum  = -99.0;

    float yaw   = bno.orientationX;  // BNO055: X=Yaw(Heading)
    float pitch = bno.orientationY;
    float roll  = bno.orientationZ;

    char finalCSV[380];
    snprintf(finalCSV, sizeof(finalCSV),
      "%lu,%lu,%s,"
      "%.2f,%.2f,%.2f,"
      "%.2f,%.2f,%.2f,"
      "%.2f,%.3f,%.2f,"
      "%.2f,%.2f,%.2f,"
      "%.2f,%.2f",
      counter, now, mega_timestamp,
      mega_temp1, mega_pressure1, mega_alt1,
      mega_temp2, mega_pressure2, mega_alt2,
      mega_wind,  mega_load,      mega_pdiff,
      yaw, pitch, roll,
      dht_temp, dht_hum
    );

    if (sd_ok) {
      File logFile = SD.open(LOG_FILE, FILE_APPEND);
      if (logFile) {
        logFile.println(finalCSV);
        logFile.close();
      } else {
        Serial.println(F("[ERR] SD open failed"));
      }
    }

    if (client && client.connected()) {
      client.println(finalCSV);
    }

    Serial.print(F("[LOG] "));
    Serial.println(finalCSV);

    if (!mega_data_fresh) {
      Serial.println(F("[WARN] No fresh Mega data — using last known values"));
    }
    mega_data_fresh = false;

    counter++;
  }
}
