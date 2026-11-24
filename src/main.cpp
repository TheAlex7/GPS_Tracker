#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>
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

// When the device leaves the fence, the outside alert is sent this many times, 1 s apart.
#define OUTSIDE_ALERT_REPEATS 3
int alertRepeatsLeft = 0;  // only used by loop

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

void updateBuzzer() {
  uint32_t now = millis();
  if (beepOn) {
    if (uint32_t(now - beepChange) < BEEP_ON_MS) return;
    beepsLeft--;
    beepChange = now;
    if (beepsLeft > 0 && BEEP_OFF_MS == 0) return;  // next one follows with no gap: stay on
    buzzerSet(false);
    beepOn = false;
  } else if (beepsLeft > 0 && uint32_t(now - beepChange) >= BEEP_OFF_MS) {
    buzzerSet(true);
    beepOn = true;
    beepChange = now;
  }
}

void sendFenceAlert(const String& alert) {
  Serial.println(alert);
  fenceAlertChar->setValue(alert.c_str());
  if (bleConnected) fenceAlertChar->notify();
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

void setup() {
  Serial.begin(115200);
  // GPS TX -> ESP32 RX35; GPS RX <- ESP32 TX33; common ground.
  gpsSerial.begin(9600, SERIAL_8N1, 35, 33);
  gpsLine.reserve(121);
  SPI.begin(IoTPins::SPI_SCK, -1, IoTPins::SPI_MOSI, IoTPins::OLED_CS);
  oledOk = iotInitDisplay();
  pinMode(IoTPins::LED_1, OUTPUT);
  digitalWrite(IoTPins::LED_1, LOW);
  buzzerBegin();

  BLEDevice::init("IoT-T09-GPS");
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());
  // 30 handles are used now; 40 leaves room for e.g. a find-by-buzz characteristic.
  BLEService* service = server->createService(BLEUUID(SERVICE_UUID), 40);
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

  addLabel(ledChar,        "LED (write 1/0)");
  addLabel(latChar,        "Latitude");
  addLabel(lonChar,        "Longitude");
  addLabel(statusChar,     "GPS status");
  addLabel(fenceChar,      "Fence: lat,lon,radius | PHONE,radius | OFF");
  addLabel(fenceStateChar, "Fence state");
  addLabel(fenceAlertChar, "Fence alert");
  addLabel(phonePosChar,   "Phone position: write lat,lon");

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
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
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
  if (!gotByte || uint32_t(now - lastByte) > 5000) status = "OFFLINE";
  else if (!haveFix) status = havePosition ? "LAST" : "NO FIX";
  else if (uint32_t(now - lastFix) > 3000) status = "STALE";

  // ---- Geofence: compare the current position with the fence centre + radius ----
  portENTER_CRITICAL(&fenceMux);
  FenceMode mode = fenceMode;
  double fLat = fenceLat, fLon = fenceLon, fRadius = fenceRadius;
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
    alertRepeatsLeft = 0;
  }
  bool phoneFresh = phoneSet && uint32_t(millis() - phoneTime) <= PHONE_POS_MAX_AGE_MS;

  String fence = "NOT SET";
  if (mode == FENCE_PHONE && !phoneFresh) fence = "NO PHONE POS";
  else if (mode != FENCE_NONE && status != "FIX") fence = "NO FIX";
  else if (mode != FENCE_NONE) {
    double distance = distanceMeters(fLat, fLon, latitude, longitude);
    bool inside = distance <= fRadius;
    fence = String(inside ? "INSIDE " : "OUTSIDE ") + String(distance, 0) + "m";

    // On crossing the fence: "outside" is alerted 3 times (one per second), "back inside" once.
    int state = inside ? 1 : 2;
    if (state != fenceLastState) {
      if (inside) {
        alertRepeatsLeft = 0;  // cancel any outside alerts still pending
        if (fenceLastState != 0)  // no alert for "inside" on the very first fix
          sendFenceAlert("BACK INSIDE " + String(distance, 0) + "m");
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
                     String(number) + "/" + String(OUTSIDE_ALERT_REPEATS) + ")");
    }
  }

  // Keep sending the last valid coordinates after the fix drops.
  String lat = havePosition ? String(latitude, 6) : String("--");
  String lon = havePosition ? String(longitude, 6) : String("--");
  status += " SATS:" + (satellites < 0 ? String("?") : String(satellites));
  if (oledOk) {
    iotDisplayLines(String(bleConnected ? "BLE ok " : "BLE wait ") + fence,
                    status, "Lat: " + lat, "Lon: " + lon);
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
}
