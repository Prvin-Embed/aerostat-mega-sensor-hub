/*
 * ================================================================
 * ESP32 — TELEMETRY HUB V2.1 (PRODUCTION GRADE)
 * ================================================================
 * Receives  : Mega CSV string via UART1 (GPIO26=TX, GPIO27=RX)
 * Sensors   : BNO055 (I2C, Yaw/Pitch/Roll) | DHT22 (GPIO25)
 * Logs to   : SD Card (SPI, CS=GPIO13) — 20-field CSV
 * Streams   : W5500 Ethernet (SPI, CS=GPIO5) — TCP Port 5000
 *
 * Final CSV (20 fields):
 * Count, ESP_Millis, RTC_Timestamp,
 * Temp1_C, Pressure1_hPa, Alt1_ASL_m,
 * Temp2_C, Pressure2_hPa, Alt2_AGL_m,
 * Wind_kmh, Load_kg, PressureDiff_hPa,
 * Latitude, Longitude, Heading_deg,
 * Yaw_deg, Pitch_deg, Roll_deg,
 * DHT_Temp_C, DHT_Humidity_pct
 * ================================================================
 */

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Ethernet.h>
#include <DHT.h>
#include <EasyBNO055_ESP.h>

// ================================================================
// PIN DEFINITIONS & CONFIG
// ================================================================
#define MEGA_RX_PIN  26   
#define MEGA_TX_PIN  27   
#define MEGA_BAUD    115200

#define DHTPIN       25
#define DHTTYPE      DHT22

#define SCK_PIN      18
#define MISO_PIN     19
#define MOSI_PIN     23
#define W5500_CS      5
#define SD_CS        13

// ================================================================
// OBJECTS
// ================================================================
DHT dht(DHTPIN, DHTTYPE);
EasyBNO055_ESP bno;

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 1, 177);
EthernetServer server(5000);
EthernetClient client;

// ================================================================
// GLOBALS & PRODUCTION SENTINELS
// ================================================================
const char* LOG_FILE = "/TelemetryLog.csv";
const char* CSV_HEADER =
  "Count,ESP_Millis,RTC_Timestamp,"
  "Temp1_C,Pressure1_hPa,Alt1_ASL_m,"
  "Temp2_C,Pressure2_hPa,Alt2_AGL_m,"
  "Wind_kmh,Load_kg,PressureDiff_hPa,"
  "Latitude,Longitude,Heading_deg,"
  "Yaw_deg,Pitch_deg,Roll_deg,"
  "DHT_Temp_C,DHT_Humidity_pct";

unsigned long counter          = 0;
unsigned long previousMillis   = 0;
const long    SEND_INTERVAL    = 2000;  

bool sd_ok                     = false;
unsigned long lastSDCheck      = 0;
const long    SD_RETRY_MS      = 30000; // Auto-remount interval

#define MEGA_BUF_SIZE 256
char megaBuf[MEGA_BUF_SIZE];
int  megaBufIdx                = 0;
bool megaLineReady             = false;

// Parsed Mega Storage (13 fields)
char   mega_timestamp[25]      = "NO_MEGA_DATA";
double mega_temp1 = 0, mega_pressure1 = 0, mega_alt1 = 0;
double mega_temp2 = 0, mega_pressure2 = 0, mega_alt2 = 0;
double mega_wind  = 0, mega_load      = 0, mega_pdiff = 0;
double mega_lat   = 0, mega_lon       = 0;
double mega_heading            = 0;

bool          mega_data_fresh  = false;
unsigned long lastMegaRxMillis = 0;
const long    MEGA_TIMEOUT_MS  = 6000; // Missed 3 cycles = Link lost

void otherI2CUpdate() { } // BNO bus hook

// ================================================================
// NON-BLOCKING UART1 READER
// ================================================================
void readMegaSerial() {
  while (Serial1.available()) {
    char c = (char)Serial1.read();
    if (c == '\n') {
      megaBuf[megaBufIdx] = '\0';
      megaBufIdx = 0;
      megaLineReady = true;
      return;
    }
    if (c != '\r' && megaBufIdx < MEGA_BUF_SIZE - 1) {
      megaBuf[megaBufIdx++] = c;
    }
  }
}

// Double-precision token parser 
double parseNextDouble(char** ptr) {
  char* start = *ptr;
  if (!start || *start == '\0') return 0.0;
  
  char* end = strchr(start, ',');
  if (end) { 
    *end = '\0'; 
    *ptr = end + 1; 
  } else { 
    *ptr = start + strlen(start); 
  }
  return strtod(start, NULL);
}

// ================================================================
// MEGA PAYLOAD DECODER (13 Fields)
// ================================================================
void parseMegaLine() {
  char tmp[MEGA_BUF_SIZE];
  strncpy(tmp, megaBuf, MEGA_BUF_SIZE);
  tmp[MEGA_BUF_SIZE - 1] = '\0';

  char* ptr = tmp;

  // 0: RTC Timestamp String
  char* tsEnd = strchr(ptr, ',');
  if (!tsEnd) return; 
  *tsEnd = '\0';
  strncpy(mega_timestamp, ptr, sizeof(mega_timestamp) - 1);
  mega_timestamp[sizeof(mega_timestamp) - 1] = '\0';
  ptr = tsEnd + 1;

  // 1-9: Atmospheric & Mechanics
  mega_temp1     = parseNextDouble(&ptr);
  mega_pressure1 = parseNextDouble(&ptr);
  mega_alt1      = parseNextDouble(&ptr);
  mega_temp2     = parseNextDouble(&ptr);
  mega_pressure2 = parseNextDouble(&ptr);
  mega_alt2      = parseNextDouble(&ptr);
  mega_wind      = parseNextDouble(&ptr);
  mega_load      = parseNextDouble(&ptr);
  mega_pdiff     = parseNextDouble(&ptr);

  // 10-12: Navigation
  mega_lat       = parseNextDouble(&ptr);
  mega_lon       = parseNextDouble(&ptr);
  mega_heading   = parseNextDouble(&ptr);

  lastMegaRxMillis = millis();
  mega_data_fresh  = true;
}

// ================================================================
// SYSTEM SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  Serial.println(F("=== ESP32 Telemetry Hub V2.1 (PROD) ==="));

  Serial1.begin(MEGA_BAUD, SERIAL_8N1, MEGA_RX_PIN, MEGA_TX_PIN);
  Serial.println(F("[OK] UART1 mapped: GPIO27(RX) / GPIO26(TX)"));

  dht.begin();
  Serial.println(F("[OK] Local DHT22 Active"));

  bno.start(&otherI2CUpdate);
  Serial.println(F("[OK] Local BNO055 Active"));

  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN);

  if (!SD.begin(SD_CS)) {
    Serial.println(F("[WARN] SD init failed. Auto-retries enabled."));
    sd_ok = false;
  } else {
    Serial.println(F("[OK] SD Card Mounted"));
    sd_ok = true;
    // Use FILE_APPEND + size()==0 — reliable on ESP32 core 3.x.
    // SD.exists() is broken (returns false for existing files on some cores).
    // FILE_WRITE would truncate an existing file, wiping logged data.
    File f = SD.open(LOG_FILE, FILE_APPEND);
    if (f) {
      if (f.size() == 0) {
        f.println(CSV_HEADER);
        Serial.println(F("[OK] SD CSV header written (new file)"));
      } else {
        Serial.println(F("[OK] SD existing log found — appending data"));
      }
      f.close();
    } else {
      Serial.println(F("[ERR] SD: cannot open log file for write"));
      sd_ok = false;
    }
  }

  Ethernet.init(W5500_CS);
  Ethernet.begin(mac, ip);
  delay(1000);
  Serial.print(F("[OK] W5500 TCP Server Online @ IP: "));
  Serial.println(Ethernet.localIP());

  server.begin();
  Serial.println(F("--- Hub Live. Monitoring stream... ---"));
}

// ================================================================
// MAIN PRODUCTION LOOP
// ================================================================
void loop() {
  unsigned long now = millis();

  // 1. TCP Socket Manager (Orphan scrubbing)
  if (client) {
    if (!client.connected()) {
      client.stop(); 
      Serial.println(F("[NET] Dead TCP socket recycled"));
    }
  } else {
    client = server.available();
  }

  // 2. Non-blocking Stream Ingestion
  readMegaSerial();
  if (megaLineReady) {
    megaLineReady = false;
    parseMegaLine();
  }

  // 3. Dynamic SD Card Healer
  if (!sd_ok && (now - lastSDCheck >= SD_RETRY_MS)) {
    lastSDCheck = now;
    SD.end(); // Clear hanging SPI pointers
    if (SD.begin(SD_CS)) {
      Serial.println(F("[RECOVERY] SD Card successfully re-mounted!"));
      sd_ok = true;
      // Same fix as setup(): FILE_APPEND + size()==0 for reliable header check
      File f = SD.open(LOG_FILE, FILE_APPEND);
      if (f) {
        if (f.size() == 0) {
          f.println(CSV_HEADER);
          Serial.println(F("[RECOVERY] SD CSV header written (new file)"));
        }
        f.close();
      } else {
        Serial.println(F("[RECOVERY] SD: log file open failed after remount"));
        sd_ok = false;
      }
    }
  }

  // 4. Master Publish Schedule
  if (now - previousMillis >= SEND_INTERVAL) {
    previousMillis = now;

    // --- Stale Data Watchdog ---
    if (now - lastMegaRxMillis > MEGA_TIMEOUT_MS) {
      mega_data_fresh = false;
      strncpy(mega_timestamp, "LINK_LOST", sizeof(mega_timestamp));
      mega_temp1 = -999.0; mega_pressure1 = -999.0; mega_alt1 = -999.0;
      mega_temp2 = -999.0; mega_pressure2 = -999.0; mega_alt2 = -999.0;
      mega_wind  = -999.0; mega_load      = -999.0; mega_pdiff = -999.0;
      mega_lat   = 0.0;    mega_lon       = 0.0;    mega_heading = -999.0;
    }

    // --- Poll Local ESP32 Sensors ---
    float dht_temp = dht.readTemperature();
    float dht_hum  = dht.readHumidity();
    if (isnan(dht_temp)) dht_temp = -99.0;
    if (isnan(dht_hum))  dht_hum  = -99.0;

    float yaw   = bno.orientationX;
    float pitch = bno.orientationY;
    float roll  = bno.orientationZ;

    // --- Assemble Master 20-Field Payload ---
    char finalCSV[512];
    snprintf(finalCSV, sizeof(finalCSV),
      "%lu,%lu,%s,"
      "%.2f,%.2f,%.2f,"
      "%.2f,%.2f,%.2f,"
      "%.2f,%.3f,%.2f,"
      "%.6f,%.6f,%.1f,"
      "%.2f,%.2f,%.2f,"
      "%.2f,%.2f",
      counter, now, mega_timestamp,
      mega_temp1, mega_pressure1, mega_alt1,
      mega_temp2, mega_pressure2, mega_alt2,
      mega_wind,  mega_load,      mega_pdiff,
      mega_lat,   mega_lon,       mega_heading,
      yaw,        pitch,          roll,
      dht_temp,   dht_hum
    );

    // --- Dispatch SD ---
    if (sd_ok) {
      File logFile = SD.open(LOG_FILE, FILE_APPEND);
      if (logFile) {
        logFile.println(finalCSV);
        logFile.close();
      } else {
        sd_ok = false; // Trigger auto-recovery on next pass
      }
    }

    // --- Dispatch W5500 LAN ---
    if (client && client.connected()) {
      client.println(finalCSV);
    }

    // --- Dispatch Ground Monitor ---
    Serial.print(F("[TELEMETRY] "));
    Serial.println(finalCSV);

    if (!mega_data_fresh) {
      Serial.println(F("[ALARM] Mega link lost! Zero-padding broadcast."));
    }

    mega_data_fresh = false;
    counter++;
  }
}