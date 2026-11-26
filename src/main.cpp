#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <Wire.h>
#include <LittleFS.h>
#include "iot_display.h"
#include "geofence.h"

#define SERVICE_UUID "71b12873-ad3c-4b5a-ad8a-f7139a8ef342"
#define LED_CHAR_UUID "499306f5-0f14-4d5d-9d31-bdd52c597914"
#define GPS_LAT_UUID "83f10001-8ca4-4c09-bb11-7357ac492601"
#define GPS_LON_UUID "83f10002-8ca4-4c09-bb11-7357ac492601"
#define GPS_STATUS_UUID "83f10003-8ca4-4c09-bb11-7357ac492601"
#define FENCE_CONFIG_UUID "83f10004-8ca4-4c09-bb11-7357ac492601"
#define FENCE_STATE_UUID "83f10005-8ca4-4c09-bb11-7357ac492601"
#define FENCE_ALERT_UUID "83f10006-8ca4-4c09-bb11-7357ac492601"
#define PHONE_POS_UUID "83f10007-8ca4-4c09-bb11-7357ac492601"
#define FIND_UUID "83f10008-8ca4-4c09-bb11-7357ac492601"
#define LOG_UUID "83f10009-8ca4-4c09-bb11-7357ac492601"

#define GPS_RX_PIN 35  // GPS TX -> ESP32 RX35
#define GPS_TX_PIN 33  // GPS RX <- ESP32 TX33

// In PHONE mode the phone must keep writing its position; older than this counts as missing.
#define PHONE_POS_MAX_AGE_MS 30000

HardwareSerial gpsSerial(2);
BLECharacteristic *ledChar, *latChar, *lonChar, *statusChar;
volatile bool bleConnected = false;
bool oledOk = false, gotByte = false, haveFix = false, havePosition = false;
String gpsLine;
double latitude = 0, longitude = 0;
int satellites = -1;
uint32_t lastByte = 0, lastFix = 0, lastUpdate = 0;
String gpsTime = "--:--:--";  // UTC time from the last valid GPS sentence

// ---- Event log: kept in flash (LittleFS), so it survives power-offs ----
// Every event goes to the serial monitor, to /log.txt, and to the Recent log characteristic.
// Serial monitor commands: "log" prints the whole log, "clearlog" erases it.
#define LOG_PATH "/log.txt"
#define LOG_OLD_PATH "/log_old.txt"  // the log rolls over to this file at LOG_MAX_BYTES
#define LOG_MAX_BYTES 65536
#define LOG_RECENT_LINES 6           // newest lines readable over BLE
BLECharacteristic* logChar = nullptr;
bool logOk = false;

// ---- Machine-readable JSON lines on the serial port (for Node-RED etc.) ----
// Off by default so the serial monitor stays readable; type "json on" / "json off" to switch.
bool jsonOut = false;

String jsonString(const String& text) {
  String out = "\"";
  for (unsigned int i = 0; i < text.length(); i++) {
    char c = text[i];
    if (c == '"' || c == '\\') out += '\\';
    if (c >= 32) out += c;
  }
  return out + "\"";
}

String jsonNumberOrNull(bool valid, double value, int decimals) {
  return valid ? String(value, decimals) : String("null");
}
String recentLog[LOG_RECENT_LINES];
int recentNext = 0, recentCount = 0;

// Call only from loop()/setup(), never from a BLE callback.
void logEvent(const String& message) {
  String line = "[" + String(millis() / 1000) + "s " + gpsTime + "] " + message;
  Serial.println(line);
  if (jsonOut)
    Serial.println("{\"type\":\"log\",\"msg\":" + jsonString(message) + ",\"uptime\":" +
                   String(millis() / 1000) + ",\"t\":" + jsonString(gpsTime) + "}");
  if (logOk) {
    File f = LittleFS.open(LOG_PATH, FILE_APPEND);
    if (f && f.size() > LOG_MAX_BYTES) {
      f.close();
      LittleFS.remove(LOG_OLD_PATH);
      LittleFS.rename(LOG_PATH, LOG_OLD_PATH);
      f = LittleFS.open(LOG_PATH, FILE_APPEND);
    }
    if (f) {
      f.println(line);
      f.close();
    }
  }
  recentLog[recentNext] = line;
  recentNext = (recentNext + 1) % LOG_RECENT_LINES;
  if (recentCount < LOG_RECENT_LINES) recentCount++;
  if (logChar) {
    String all = "";
    for (int i = 0; i < recentCount; i++) {
      int index = (recentNext - recentCount + i + LOG_RECENT_LINES) % LOG_RECENT_LINES;
      all += recentLog[index] + "\n";
    }
    if (all.length() > 500) all = all.substring(all.length() - 500);  // BLE value limit is 512
    logChar->setValue(all.c_str());
    if (bleConnected) logChar->notify();
  }
}

void printLogFile(const char* path) {
  File f = LittleFS.open(path, FILE_READ);
  if (!f) return;
  while (f.available()) Serial.write(f.read());
  f.close();
}

// Reads commands typed into the serial monitor.
void handleSerialCommands() {
  static String command;
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c != '\n' && c != '\r') {
      if (command.length() < 20) command += c;
      continue;
    }
    if (command == "log") {
      Serial.println("----- LOG START -----");
      printLogFile(LOG_OLD_PATH);
      printLogFile(LOG_PATH);
      Serial.println("----- LOG END -----");
    } else if (command == "json on") {
      jsonOut = true;
      Serial.println("JSON output on");
    } else if (command == "json off") {
      jsonOut = false;
      Serial.println("JSON output off");
    } else if (command == "clearlog") {
      LittleFS.remove(LOG_OLD_PATH);
      LittleFS.remove(LOG_PATH);
      Serial.println("Log cleared");
    }
    command = "";
  }
}

// ---- Geofence ----
// CUSTOM: a fixed point + radius written by the user.
// PHONE:  the centre follows the phone's own position (written to PHONE_POS), + radius.
enum FenceMode { FENCE_NONE, FENCE_CUSTOM, FENCE_PHONE };
BLECharacteristic *fenceChar, *fenceStateChar, *fenceAlertChar, *phonePosChar;
portMUX_TYPE fenceMux = portMUX_INITIALIZER_UNLOCKED;  // BLE task writes, loop() reads
FenceMode fenceMode = FENCE_NONE;
double fenceLat = 0, fenceLon = 0, fenceRadius = 0;    // degrees, degrees, metres
double phoneLat = 0, phoneLon = 0;                     // latest phone position
bool phonePosSet = false;
uint32_t phonePosTime = 0;                             // millis() of the last phone write
uint32_t fenceVersion = 0;                             // bumped on every fence change
uint32_t fenceSeenVersion = 0;                         // only used by loop
int fenceLastState = 0;  // 0 = unknown, 1 = inside, 2 = outside (only used by loop)
int fencePending = 0;    // readings in a row that disagree with fenceLastState (only used by loop)

// Edge-case settings
#define MIN_FENCE_RADIUS_M 20      // GPS wanders 3-10 m; smaller fences give false alerts
#define FENCE_HYSTERESIS_M 10      // must be this far back inside before "BACK INSIDE"
#define FENCE_CONFIRM_READINGS 3   // readings in a row (1 per second) before the state changes
#define PHONE_LOST_GRACE_MS 10000  // phone mode: disconnected this long = phone out of range
#define GPS_OFFLINE_ALERT_MS 10000 // GPS silent this long (while it should be on) = alert
#define GPS_RESTART_MS 30000       // while offline, restart the GPS serial port this often

// When the device leaves the fence, the outside alert is sent this many times, 1 s apart.
#define OUTSIDE_ALERT_REPEATS 3
int alertRepeatsLeft = 0;  // only used by loop

// ---- Find-by-buzz: the phone writes ON/OFF; while on, the device buzzes and keeps sending location ----
#define FIND_TIMEOUT_MS 120000  // find mode switches itself off after 2 minutes
#define FIND_ON_MS 700          // buzz pattern while finding: 0.7 s on, 0.3 s off
#define FIND_OFF_MS 300
BLECharacteristic* findChar;
portMUX_TYPE findMux = portMUX_INITIALIZER_UNLOCKED;  // BLE task writes, loop() reads
bool findMode = false;
uint32_t findStart = 0;
bool findShown = false;  // last state reported on the characteristic (only used by loop)

// True while find mode is on; also ends it after FIND_TIMEOUT_MS.
bool findActive(uint32_t& started) {
  portENTER_CRITICAL(&findMux);
  bool on = findMode;
  started = findStart;
  if (on && uint32_t(millis() - started) > FIND_TIMEOUT_MS) {
    findMode = false;
    on = false;
  }
  portEXIT_CRITICAL(&findMux);
  return on;
}

// ---- Buzzer ----
#define BUZZER_PIN 27
#define BUZZER_ACTIVE false  // true: active buzzer (sounds on HIGH). false: passive buzzer (needs a tone)
#define BUZZER_LEDC_CHANNEL 4  // PWM channel for the passive buzzer (only used on ESP32 core 2.x)
#define BUZZER_TONE_HZ 400   // passive buzzer only: low = harsh buzz, ~2700 = clean beep
// Each outside alert adds 1 s of sound with no gap, so 3 alerts = one continuous ~3 s buzz.
// For separate beeps instead, use e.g. ON 300 / OFF 200.
#define BEEP_ON_MS 1000
#define BEEP_OFF_MS 0
int beepsLeft = 0;
bool beepOn = false;
uint32_t beepChange = 0;

// The passive buzzer is driven straight from the LEDC (PWM) hardware, set up once here.
// (tone()/noTone() switched on and off quickly logs "LEDC is not initialized" errors.)
void buzzerBegin() {
  if (BUZZER_ACTIVE) {
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    return;
  }
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(BUZZER_PIN, BUZZER_TONE_HZ, 10);
  ledcWrite(BUZZER_PIN, 0);
#else
  ledcSetup(BUZZER_LEDC_CHANNEL, BUZZER_TONE_HZ, 10);
  ledcAttachPin(BUZZER_PIN, BUZZER_LEDC_CHANNEL);
  ledcWrite(BUZZER_LEDC_CHANNEL, 0);
#endif
}

void buzzerSet(bool on) {
  if (BUZZER_ACTIVE) {
    digitalWrite(BUZZER_PIN, on ? HIGH : LOW);
    return;
  }
  // A frequency of 0 silences the output.
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWriteTone(BUZZER_PIN, on ? BUZZER_TONE_HZ : 0);
#else
  ledcWriteTone(BUZZER_LEDC_CHANNEL, on ? BUZZER_TONE_HZ : 0);
#endif
}

// Queues beeps; updateBuzzer() plays them without blocking loop().
void startBeeps(int count) {
  beepsLeft += count;
}

bool buzzerOn = false;
bool wasFinding = false;

void buzzerSetIfChanged(bool on) {
  if (on == buzzerOn) return;
  buzzerSet(on);
  buzzerOn = on;
}

void updateBuzzer() {
  uint32_t now = millis();

  // Find mode takes over the buzzer with a repeating pattern.
  uint32_t started;
  if (findActive(started)) {
    buzzerSetIfChanged(uint32_t(now - started) % (FIND_ON_MS + FIND_OFF_MS) < FIND_ON_MS);
    beepsLeft = 0;  // drop geofence beeps while finding; the alerts themselves still go out
    beepOn = false;
    wasFinding = true;
    return;
  }
  if (wasFinding) {
    wasFinding = false;
    buzzerSetIfChanged(false);
    beepChange = now;
  }

  if (beepOn) {
    if (uint32_t(now - beepChange) < BEEP_ON_MS) return;
    beepsLeft--;
    beepChange = now;
    if (beepsLeft > 0 && BEEP_OFF_MS == 0) return;  // next one follows with no gap: stay on
    buzzerSetIfChanged(false);
    beepOn = false;
  } else if (beepsLeft > 0 && uint32_t(now - beepChange) >= BEEP_OFF_MS) {
    buzzerSetIfChanged(true);
    beepOn = true;
    beepChange = now;
  }
}

// Sends an alert on the Alerts characteristic and logs it.
// event is a short code for machines: exit, enter, phone_lost, phone_back, gps_offline, gps_online.
void sendFenceAlert(const String& alert, const char* event) {
  logEvent(alert);
  fenceAlertChar->setValue(alert.c_str());
  if (bleConnected) fenceAlertChar->notify();
  if (jsonOut)
    Serial.println("{\"type\":\"alert\",\"event\":" + jsonString(event) + ",\"msg\":" + jsonString(alert) +
                   ",\"lat\":" + jsonNumberOrNull(havePosition, latitude, 6) +
                   ",\"lon\":" + jsonNumberOrNull(havePosition, longitude, 6) +
                   ",\"t\":" + jsonString(gpsTime) + "}");
}

// Field 0 is the sentence name; fields after it are separated by commas.
String gpsField(String text, int wanted) {
  int field = 0, start = 0;
  for (unsigned int i = 0; i <= text.length(); i++) {
    if (i == text.length() || text[i] == ',' || text[i] == '*') {
      if (field == wanted) return text.substring(start, i);
      if (i == text.length() || text[i] == '*') break;
      field++;
      start = i + 1;
    }
  }
  return "";
}

bool gpsNumber(String text) {
  bool digit = false, dot = false;
  for (unsigned int i = 0; i < text.length(); i++) {
    if (text[i] >= '0' && text[i] <= '9') digit = true;
    else if (text[i] == '.' && !dot) dot = true;
    else return false;
  }
  return digit;
}
// Adds a human-readable label (descriptor 0x2901) that BLE apps can display.
void addLabel(BLECharacteristic* c, const char* label) {
  BLEDescriptor* d = new BLEDescriptor(BLEUUID((uint16_t)0x2901));
  d->setValue(label);
  c->addDescriptor(d);
}


int hexDigit(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

bool goodChecksum(String text) {
  int star = text.indexOf('*');
  if (text.length() == 0 || text[0] != '$' || star < 1 || star + 3 != (int)text.length()) return false;
  int high = hexDigit(text[star + 1]), low = hexDigit(text[star + 2]);
  if (high < 0 || low < 0) return false;
  int checksum = 0;
  for (int i = 1; i < star; i++) checksum ^= text[i];
  return checksum == high * 16 + low;
}

// NMEA ddmm.mmmm -> decimal degrees. -999 means an invalid coordinate.
double gpsDegrees(String text, String direction, bool isLatitude) {
  if (!gpsNumber(text)) return -999;
  double raw = atof(text.c_str());
  if (!isfinite(raw) || raw > 18060) return -999;
  int degrees = raw / 100;
  double minutes = raw - degrees * 100;
  double result = degrees + minutes / 60;
  if (minutes >= 60 || result > (isLatitude ? 90 : 180)) return -999;
  if (isLatitude && direction != "N" && direction != "S") return -999;
  if (!isLatitude && direction != "E" && direction != "W") return -999;
  if (direction == "S" || direction == "W") result = -result;
  return result;
}

void parseGps(String text, uint32_t now) {
  if (!goodChecksum(text)) return;
  String type = gpsField(text, 0);
  if (type.length() != 6) return;
  bool isGga = type.substring(3) == "GGA";
  if (!isGga && type.substring(3) != "RMC") return;
  int commas = 0;
  for (unsigned int i = 0; i < text.length(); i++) {
    if (text[i] == ',') commas++;
  }
  if (commas < 9) return;
  int firstCoordinate = isGga ? 2 : 3;
  String utc = gpsField(text, 1);  // hhmmss.ss in both GGA and RMC
  if (utc.length() >= 6 && gpsNumber(utc))
    gpsTime = utc.substring(0, 2) + ":" + utc.substring(2, 4) + ":" + utc.substring(4, 6) + "Z";
  int newSatellites = satellites;
  bool validFix;
  if (isGga) {
    String quality = gpsField(text, 6), count = gpsField(text, 7);
    if (quality.length() != 1 || quality[0] < '0' || quality[0] > '8' ||
        !gpsNumber(count) || count.indexOf('.') >= 0 || count.length() > 2) return;
    validFix = quality != "0";
    newSatellites = count.toInt();
  } else {
    String flag = gpsField(text, 2);
    if (flag != "A" && flag != "V") return;
    validFix = flag == "A";
  }
  if (!validFix) {
    satellites = newSatellites;
    haveFix = false;
    return;
  }
  double lat = gpsDegrees(gpsField(text, firstCoordinate), gpsField(text, firstCoordinate + 1), true);
  double lon = gpsDegrees(gpsField(text, firstCoordinate + 2), gpsField(text, firstCoordinate + 3), false);
  if (lat == -999 || lon == -999) return;
  latitude = lat;
  longitude = lon;
  satellites = newSatellites;
  haveFix = true;
  havePosition = true;
  lastFix = now;
}

// Copies a BLE write into a NUL-terminated buffer. False if it does not fit.
bool bleText(BLECharacteristic* c, char* out, size_t size) {
  size_t len = c->getLength();
  if (len >= size) return false;
  memcpy(out, c->getData(), len);
  out[len] = '\0';
  return true;
}

// "lat,lon" in decimal degrees, e.g. "51.5007,-0.1246".
bool parsePoint(const char* text, double& lat, double& lon) {
  char* end;
  lat = strtod(text, &end);
  if (end == text || *end != ',') return false;
  const char* rest = end + 1;
  lon = strtod(rest, &end);
  if (end == rest || *end != '\0') return false;
  return isfinite(lat) && isfinite(lon) && fabs(lat) <= 90 && fabs(lon) <= 180;
}

// ---- Motion detection: MPU6050 on I2C, polled (no interrupt wire) ----
#define MPU_ADDR 0x68           // 0x69 if the board's AD0 pin is pulled high
#define MPU_SDA 21
#define MPU_SCL 22
#define MOTION_SAMPLE_MS 100    // read the accelerometer 10 times a second
#define MOTION_THRESHOLD_G 0.08 // change from the resting reading that counts as motion
#define STILL_TIMEOUT_MS 30000  // GPS turns off after this long without motion
#define MPU_FAILS_BEFORE_RESET 5  // this many failed reads in a row = the sensor is lost
#define MPU_RETRY_MS 2000         // while lost, try to recover it this often
bool mpuOk = false, mpuBaseSet = false;
float mpuBaseX = 0, mpuBaseY = 0, mpuBaseZ = 0;
uint32_t lastMotionSample = 0, lastMotion = 0, lastMpuRetry = 0;
int mpuFailures = 0;

// Frees an I2C bus that a sensor is holding stuck (data line low) after a glitch:
// clock SCL until the sensor lets go of SDA, then send a STOP condition.
void i2cBusRecover() {
  Wire.end();
  pinMode(MPU_SDA, INPUT_PULLUP);
  pinMode(MPU_SCL, OUTPUT_OPEN_DRAIN);
  digitalWrite(MPU_SCL, HIGH);
  for (int i = 0; i < 9 && digitalRead(MPU_SDA) == LOW; i++) {
    digitalWrite(MPU_SCL, LOW);
    delayMicroseconds(5);
    digitalWrite(MPU_SCL, HIGH);
    delayMicroseconds(5);
  }
  pinMode(MPU_SDA, OUTPUT_OPEN_DRAIN);
  digitalWrite(MPU_SDA, LOW);
  delayMicroseconds(5);
  digitalWrite(MPU_SCL, HIGH);
  delayMicroseconds(5);
  digitalWrite(MPU_SDA, HIGH);
  delayMicroseconds(5);
}

bool mpuWrite(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool mpuBegin() {
  Wire.begin(MPU_SDA, MPU_SCL);
  if (!mpuWrite(0x6B, 0x00)) return false;  // PWR_MGMT_1: wake up (it starts asleep)
  mpuWrite(0x1C, 0x00);                     // ACCEL_CONFIG: +-2 g
  mpuWrite(0x1A, 0x03);                     // CONFIG: ~44 Hz low-pass to smooth vibration
  return true;
}

bool mpuRead(float& x, float& y, float& z) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, 6) != 6) return false;
  int16_t raw[3];
  for (int i = 0; i < 3; i++) raw[i] = (int16_t)((Wire.read() << 8) | Wire.read());
  x = raw[0] / 16384.0f;  // 16384 counts per g at +-2 g
  y = raw[1] / 16384.0f;
  z = raw[2] / 16384.0f;
  return true;
}

// Motion = any axis moves away from a slowly-updated resting reading.
// With no working accelerometer the device acts as if always moving, so the GPS stays on.
void updateMotion(uint32_t now) {
  if (!mpuOk) {
    lastMotion = now;  // keep the GPS on while the sensor is missing
    if (uint32_t(now - lastMpuRetry) < MPU_RETRY_MS) return;
    lastMpuRetry = now;
    i2cBusRecover();
    if (mpuBegin()) {
      mpuOk = true;
      mpuFailures = 0;
      mpuBaseSet = false;  // take a fresh resting reading
      logEvent("MPU6050 recovered");
    }
    return;
  }
  if (uint32_t(now - lastMotionSample) < MOTION_SAMPLE_MS) return;
  lastMotionSample = now;
  float x, y, z;
  if (!mpuRead(x, y, z)) {
    lastMotion = now;
    if (++mpuFailures >= MPU_FAILS_BEFORE_RESET) {
      mpuOk = false;
      lastMpuRetry = now;
      logEvent("MPU6050 lost - resetting I2C bus (GPS stays on meanwhile)");
    }
    return;
  }
  mpuFailures = 0;
  if (!mpuBaseSet) {
    mpuBaseX = x; mpuBaseY = y; mpuBaseZ = z;
    mpuBaseSet = true;
    return;
  }
  float change = max(fabs(x - mpuBaseX), max(fabs(y - mpuBaseY), fabs(z - mpuBaseZ)));
  if (change > MOTION_THRESHOLD_G) lastMotion = now;
  mpuBaseX += (x - mpuBaseX) * 0.1f;
  mpuBaseY += (y - mpuBaseY) * 0.1f;
  mpuBaseZ += (z - mpuBaseZ) * 0.1f;
}

// ---- GPS power: NEO-6M sleep/wake over the serial link (UBX commands, no extra wiring) ----
bool gpsPowered = true;
uint32_t lastWakeSend = 0;

void ubxSend(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len) {
  uint8_t head[4] = {cls, id, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8)};
  uint8_t ckA = 0, ckB = 0;
  gpsSerial.write(0xB5);
  gpsSerial.write(0x62);
  for (int i = 0; i < 4; i++) { gpsSerial.write(head[i]); ckA += head[i]; ckB += ckA; }
  for (uint16_t i = 0; i < len; i++) { gpsSerial.write(payload[i]); ckA += payload[i]; ckB += ckA; }
  gpsSerial.write(ckA);
  gpsSerial.write(ckB);
}

// UBX-RXM-PMREQ: backup mode with no time limit. The NEO-6M wakes on activity on its RX line.
void gpsSleep() {
  const uint8_t payload[8] = {0, 0, 0, 0, 0x02, 0, 0, 0};  // duration 0 = until woken; flags = backup
  ubxSend(0x02, 0x41, payload, sizeof(payload));
}

void gpsWakeBytes() {
  for (int i = 0; i < 8; i++) gpsSerial.write(0xFF);
}

// GPS off after STILL_TIMEOUT_MS without motion; back on at the first motion or in find mode.
void updateGpsPower(uint32_t now) {
  uint32_t findStarted;
  bool finding = findActive(findStarted);
  bool still = uint32_t(now - lastMotion) > STILL_TIMEOUT_MS;
  if (gpsPowered && still && !finding) {
    gpsSleep();
    gpsPowered = false;
    haveFix = false;  // the next live coordinates must come from a fresh fix
    gpsLine = "";
    logEvent("GPS OFF (no motion)");
  } else if (!gpsPowered && (!still || finding)) {
    gpsWakeBytes();
    gpsPowered = true;
    haveFix = false;
    lastByte = now;   // give it time to start talking before calling it OFFLINE
    lastWakeSend = now;
    logEvent(finding ? "GPS ON (find mode)" : "GPS ON (motion)");
  } else if (gpsPowered && uint32_t(now - lastByte) > 2000 && uint32_t(now - lastWakeSend) > 1000) {
    gpsWakeBytes();   // silent while it should be on: nudge it again in case it is still asleep
    lastWakeSend = now;
  }
}

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    bleConnected = true;
  }
  void onDisconnect(BLEServer*) override {
    bleConnected = false;
    BLEDevice::startAdvertising();
  }
};

class LedCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    std::string value = c->getValue();
    bool on;
    if (value == "1" || value == "ON" || (value.size() == 1 && value[0] == 1)) on = true;
    else if (value == "0" || value == "OFF" || (value.size() == 1 && value[0] == 0)) on = false;
    else return;
    digitalWrite(IoTPins::LED_1, on ? HIGH : LOW);
    c->setValue(on ? "ON" : "OFF");
  }
};

// The user writes one of:
//   "lat,lon,radius"  custom fixed centre, e.g. "51.5007,-0.1246,150" (degrees, degrees, metres)
//   "PHONE,radius"    centre follows the phone's position, e.g. "PHONE,30"
//   "OFF"             no fence
class FenceCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    char text[64];
    if (bleText(c, text, sizeof(text))) {
      double lat, lon, radius;
      FenceMode mode = FENCE_NONE;
      bool ok = false;
      if (strcmp(text, "OFF") == 0) {
        ok = true;
      } else if (strncmp(text, "PHONE,", 6) == 0) {
        // Reuse parseFence's radius checks so both modes accept the same radii.
        char check[80];
        snprintf(check, sizeof(check), "0,0,%s", text + 6);
        ok = parseFence(check, lat, lon, radius);
        mode = FENCE_PHONE;
      } else {
        ok = parseFence(text, lat, lon, radius);
        mode = FENCE_CUSTOM;
      }
      if (ok && mode != FENCE_NONE && radius < MIN_FENCE_RADIUS_M) ok = false;  // too small for GPS accuracy
      if (ok) {
        portENTER_CRITICAL(&fenceMux);
        fenceMode = mode;
        if (mode == FENCE_CUSTOM) {
          fenceLat = lat;
          fenceLon = lon;
        }
        fenceRadius = mode == FENCE_NONE ? 0 : radius;
        fenceVersion++;
        portEXIT_CRITICAL(&fenceMux);
      }
    }
    // Reading the characteristic always shows the fence in use.
    // (Only this task writes the fence settings, so reading them here needs no lock.)
    String shown = "NOT SET";
    if (fenceMode == FENCE_CUSTOM)
      shown = String(fenceLat, 6) + "," + String(fenceLon, 6) + "," + String(fenceRadius, 0);
    else if (fenceMode == FENCE_PHONE)
      shown = "PHONE," + String(fenceRadius, 0);
    c->setValue(shown.c_str());
  }
};

// The phone writes its own position "lat,lon" here every few seconds while in PHONE mode.
class PhonePosCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    char text[48];
    double lat, lon;
    if (!bleText(c, text, sizeof(text)) || !parsePoint(text, lat, lon)) return;
    uint32_t t = millis();
    portENTER_CRITICAL(&fenceMux);
    phoneLat = lat;
    phoneLon = lon;
    phonePosSet = true;
    phonePosTime = t;
    portEXIT_CRITICAL(&fenceMux);
  }
};

// The phone writes "1"/"ON" to start find mode (or restart its 2-minute timer), "0"/"OFF" to stop.
class FindCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    std::string value = c->getValue();
    bool on;
    if (value == "1" || value == "ON" || (value.size() == 1 && value[0] == 1)) on = true;
    else if (value == "0" || value == "OFF" || (value.size() == 1 && value[0] == 0)) on = false;
    else return;
    uint32_t t = millis();
    portENTER_CRITICAL(&findMux);
    findMode = on;
    if (on) findStart = t;
    portEXIT_CRITICAL(&findMux);
    c->setValue(on ? "ON" : "OFF");
  }
};

void setup() {
  setCpuFrequencyMhz(80);  // lowest speed that still runs BLE; cuts the ESP32's own current draw
  Serial.begin(115200);
  // GPS TX -> ESP32 RX35; GPS RX <- ESP32 TX33; common ground.
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  gpsLine.reserve(121);
  SPI.begin(IoTPins::SPI_SCK, -1, IoTPins::SPI_MOSI, IoTPins::OLED_CS);
  oledOk = iotInitDisplay();
  pinMode(IoTPins::LED_1, OUTPUT);
  digitalWrite(IoTPins::LED_1, LOW);
  buzzerBegin();
  logOk = LittleFS.begin(true);  // true = format the flash area the first time
  if (!logOk) Serial.println("LittleFS failed - events go to the serial monitor only");
  logEvent("BOOT");
  mpuOk = mpuBegin();
  logEvent(mpuOk ? "MPU6050 ok" : "MPU6050 NOT FOUND - GPS will stay on");
  lastMotion = millis();

  BLEDevice::init("IoT-T09-GPS");
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());
  // 38 handles are used; each characteristic takes 2, plus 1 per descriptor.
  BLEService* service = server->createService(BLEUUID(SERVICE_UUID), 48);
  ledChar = service->createCharacteristic(LED_CHAR_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  ledChar->setCallbacks(new LedCallbacks());
  ledChar->setValue("OFF");
  latChar = service->createCharacteristic(GPS_LAT_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  lonChar = service->createCharacteristic(GPS_LON_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  statusChar = service->createCharacteristic(GPS_STATUS_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  latChar->addDescriptor(new BLE2902());
  lonChar->addDescriptor(new BLE2902());
  statusChar->addDescriptor(new BLE2902());
  latChar->setValue("--");
  lonChar->setValue("--");
  statusChar->setValue("OFFLINE SATS:?");
  fenceChar = service->createCharacteristic(FENCE_CONFIG_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  fenceChar->setCallbacks(new FenceCallbacks());
  fenceChar->setValue("NOT SET");
  fenceStateChar = service->createCharacteristic(FENCE_STATE_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  fenceStateChar->addDescriptor(new BLE2902());
  fenceStateChar->setValue("NOT SET");
  fenceAlertChar = service->createCharacteristic(FENCE_ALERT_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  fenceAlertChar->addDescriptor(new BLE2902());
  fenceAlertChar->setValue("NONE");
  phonePosChar = service->createCharacteristic(PHONE_POS_UUID, BLECharacteristic::PROPERTY_WRITE);
  phonePosChar->setCallbacks(new PhonePosCallbacks());
  findChar = service->createCharacteristic(FIND_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
  findChar->addDescriptor(new BLE2902());
  findChar->setCallbacks(new FindCallbacks());
  findChar->setValue("OFF");
  logChar = service->createCharacteristic(LOG_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  logChar->addDescriptor(new BLE2902());
  logChar->setValue("");

  addLabel(ledChar,        "LED (write 1/0)");
  addLabel(latChar,        "Latitude");
  addLabel(lonChar,        "Longitude");
  addLabel(statusChar,     "GPS status");
  addLabel(fenceChar,      "Fence: lat,lon,radius | PHONE,radius | OFF");
  addLabel(fenceStateChar, "Fence state");
  addLabel(fenceAlertChar, "Alerts");
  addLabel(phonePosChar,   "Phone position: write lat,lon");
  addLabel(findChar,       "Find-by-buzz (write 1/0)");
  addLabel(logChar,        "Recent log");

  service->start();
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06);
  adv->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
}

void loop() {
  updateBuzzer();  // runs every pass so beeps keep their timing
  updateMotion(millis());
  updateGpsPower(millis());
  handleSerialCommands();
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    if (!gpsPowered) continue;  // ignore leftovers sent just before the GPS went to sleep
    gotByte = true;
    lastByte = millis();
    if (c == '$') gpsLine = "$";
    else if (c == '\n') {
      if (gpsLine != "") parseGps(gpsLine, lastByte);
      gpsLine = "";
    } else if (c != '\r' && gpsLine != "") {
      if (gpsLine.length() >= 120) gpsLine = "";
      else gpsLine += c;
    }
  }

  uint32_t now = millis();
  if (uint32_t(now - lastUpdate) < 1000) return;
  lastUpdate = now;
  String status = "FIX";
  if (!gpsPowered) status = "SLEEP";  // switched off on purpose: device is not moving
  else if (!gotByte || uint32_t(now - lastByte) > 5000) status = "OFFLINE";
  else if (!haveFix) status = havePosition ? "LAST" : "NO FIX";
  else if (uint32_t(now - lastFix) > 3000) status = "STALE";

  // Print the status on the serial monitor whenever it changes (SLEEP, LAST, FIX, ...).
  static String lastStatusPrinted = "";
  if (status != lastStatusPrinted) {
    lastStatusPrinted = status;
    Serial.println("STATUS: " + status);
  }

  // ---- Find-by-buzz: report when it starts, stops or times out ----
  uint32_t findStarted;
  bool finding = findActive(findStarted);
  if (finding != findShown) {
    findShown = finding;
    logEvent(finding ? "FIND MODE ON" : "FIND MODE OFF");
    findChar->setValue(finding ? "ON" : "OFF");
    if (bleConnected) findChar->notify();
  }

  // ---- Geofence: compare the current position with the fence centre + radius ----
  portENTER_CRITICAL(&fenceMux);
  FenceMode mode = fenceMode;
  double fLat = fenceLat, fLon = fenceLon, fRadius = fenceRadius;
  double customLat = fenceLat, customLon = fenceLon;
  bool phoneSet = phonePosSet;
  uint32_t phoneTime = phonePosTime;
  if (mode == FENCE_PHONE) {
    fLat = phoneLat;
    fLon = phoneLon;
  }
  uint32_t version = fenceVersion;
  portEXIT_CRITICAL(&fenceMux);

  // A new or changed fence starts fresh, so the first reading against it is judged again.
  if (version != fenceSeenVersion) {
    fenceSeenVersion = version;
    fenceLastState = 0;
    fencePending = 0;
    alertRepeatsLeft = 0;
    if (mode == FENCE_CUSTOM)
      logEvent("FENCE SET " + String(customLat, 6) + "," + String(customLon, 6) + " r=" + String(fRadius, 0) + "m");
    else if (mode == FENCE_PHONE)
      logEvent("FENCE SET PHONE r=" + String(fRadius, 0) + "m");
    else
      logEvent("FENCE OFF");
  }
  bool phoneFresh = phoneSet && uint32_t(millis() - phoneTime) <= PHONE_POS_MAX_AGE_MS;

  String fence = "NOT SET";
  if (mode == FENCE_PHONE && !phoneFresh) fence = "NO PHONE POS";
  // While the GPS sleeps the device hasn't moved, so its last position is still valid.
  else if (mode != FENCE_NONE && status != "FIX" && !(status == "SLEEP" && havePosition)) fence = "NO FIX";
  else if (mode != FENCE_NONE) {
    double distance = distanceMeters(fLat, fLon, latitude, longitude);
    // Buffer zone: once outside, the device must come FENCE_HYSTERESIS_M back inside the radius
    // to count as inside, so GPS wander at the boundary doesn't flip the state back and forth.
    bool inside = fenceLastState == 2 ? distance < max(0.0, fRadius - FENCE_HYSTERESIS_M)
                                      : distance <= fRadius;
    int reading = inside ? 1 : 2;

    // A change only counts after FENCE_CONFIRM_READINGS readings in a row agree, so a single
    // GPS jump doesn't trigger an alert. The first reading against a new fence counts at once.
    int state = fenceLastState;
    if (state == 0) state = reading;
    else if (reading != state) {
      if (++fencePending >= FENCE_CONFIRM_READINGS) state = reading;
    } else {
      fencePending = 0;
    }
    fence = String(state == 1 ? "INSIDE " : "OUTSIDE ") + String(distance, 0) + "m";

    // On crossing the fence: "outside" is alerted 3 times (one per second), "back inside" once.
    if (state != fenceLastState) {
      fencePending = 0;
      if (state == 1) {
        alertRepeatsLeft = 0;  // cancel any outside alerts still pending
        if (fenceLastState != 0)  // no alert for "inside" on the very first fix
          sendFenceAlert("BACK INSIDE " + String(distance, 0) + "m", "enter");
      } else {
        alertRepeatsLeft = OUTSIDE_ALERT_REPEATS;
      }
      fenceLastState = state;
    }
    if (alertRepeatsLeft > 0) {
      int number = OUTSIDE_ALERT_REPEATS - alertRepeatsLeft + 1;
      alertRepeatsLeft--;
      startBeeps(1);  // works even with no phone connected
      sendFenceAlert("ALERT: OUTSIDE FENCE " + String(distance, 0) + "m (" +
                     String(number) + "/" + String(OUTSIDE_ALERT_REPEATS) + ")", "exit");
    }
  }

  // ---- BLE connection changes; in phone mode a lost connection means the phone is out of range ----
  static bool wasConnected = false, phoneLostAlerted = false;
  static uint32_t disconnectedAt = 0;
  bool connected = bleConnected;
  if (connected != wasConnected) {
    wasConnected = connected;
    logEvent(connected ? "BLE CONNECTED" : "BLE DISCONNECTED");
    if (!connected) disconnectedAt = now;
    else if (phoneLostAlerted) {
      phoneLostAlerted = false;
      sendFenceAlert("PHONE BACK IN RANGE", "phone_back");
    }
  }
  if (!connected && mode == FENCE_PHONE && !phoneLostAlerted &&
      uint32_t(now - disconnectedAt) > PHONE_LOST_GRACE_MS) {
    phoneLostAlerted = true;
    sendFenceAlert("ALERT: PHONE OUT OF RANGE", "phone_lost");
    startBeeps(3);  // the phone can't hear this alert, so the buzzer has to
  }

  // ---- GPS offline fallback: alert, buzz once, and keep trying to restart the GPS link ----
  static bool gpsWasOffline = false, gpsOfflineAlerted = false;
  static uint32_t offlineSince = 0, lastGpsRestart = 0;
  bool offline = status == "OFFLINE";
  if (offline && !gpsWasOffline) {
    gpsWasOffline = true;
    offlineSince = now;
    lastGpsRestart = now;
  }
  if (offline && !gpsOfflineAlerted && uint32_t(now - offlineSince) > GPS_OFFLINE_ALERT_MS) {
    gpsOfflineAlerted = true;
    sendFenceAlert("ALERT: GPS OFFLINE (sending last known position)", "gps_offline");
    startBeeps(1);
  }
  if (offline && uint32_t(now - lastGpsRestart) > GPS_RESTART_MS) {
    lastGpsRestart = now;
    logEvent("GPS restart attempt");
    gpsSerial.end();
    gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    gpsWakeBytes();
  }
  if (!offline && gpsWasOffline) {
    gpsWasOffline = false;
    if (gpsOfflineAlerted) {
      gpsOfflineAlerted = false;
      sendFenceAlert("GPS BACK ONLINE", "gps_online");
    }
  }

  // Keep sending the last valid coordinates after the fix drops.
  String lat = havePosition ? String(latitude, 6) : String("--");
  String lon = havePosition ? String(longitude, 6) : String("--");

  // One JSON location line per second when "json on" is set. The fence centre is included
  // so a map can draw the circle (in phone mode only while the phone's position is fresh).
  if (jsonOut) {
    bool centreKnown = mode == FENCE_CUSTOM || (mode == FENCE_PHONE && phoneFresh);
    String modeName = mode == FENCE_CUSTOM ? "custom" : mode == FENCE_PHONE ? "phone" : "off";
    Serial.println("{\"type\":\"loc\",\"lat\":" + jsonNumberOrNull(havePosition, latitude, 6) +
                   ",\"lon\":" + jsonNumberOrNull(havePosition, longitude, 6) +
                   ",\"sats\":" + (satellites < 0 ? String("null") : String(satellites)) +
                   ",\"status\":" + jsonString(status) + ",\"fence\":" + jsonString(fence) +
                   ",\"fence_mode\":" + jsonString(modeName) +
                   ",\"fence_lat\":" + jsonNumberOrNull(centreKnown, fLat, 6) +
                   ",\"fence_lon\":" + jsonNumberOrNull(centreKnown, fLon, 6) +
                   ",\"fence_r\":" + jsonNumberOrNull(centreKnown, fRadius, 0) +
                   ",\"find\":" + (finding ? "true" : "false") +
                   ",\"ble\":" + (bleConnected ? "true" : "false") +
                   ",\"t\":" + jsonString(gpsTime) + "}");
  }

  status += " SATS:" + (satellites < 0 ? String("?") : String(satellites));
  if (oledOk) {
    iotDisplayLines(String(bleConnected ? "BLE ok " : "BLE wait ") + fence,
                    status + (finding ? " FIND" : ""), "Lat: " + lat, "Lon: " + lon);
  }

  latChar->setValue(lat.c_str());
  lonChar->setValue(lon.c_str());
  statusChar->setValue(status.c_str());
  fenceStateChar->setValue(fence.c_str());
  if (bleConnected) {
    latChar->notify();
    lonChar->notify();
    statusChar->notify();
    fenceStateChar->notify();
  }

  // In find mode the find characteristic also carries the coordinates, once a second:
  // "ON 33.640512,-117.844301 FIX SATS:7"
  if (finding) {
    String findValue = "ON " + lat + "," + lon + " " + status;
    findChar->setValue(findValue.c_str());
    if (bleConnected) findChar->notify();
  }
}