#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include "iot_display.h"

#define SERVICE_UUID "71b12873-ad3c-4b5a-ad8a-f7139a8ef342"
#define LED_CHAR_UUID "499306f5-0f14-4d5d-9d31-bdd52c597914"
#define GPS_LAT_UUID "83f10001-8ca4-4c09-bb11-7357ac492601"
#define GPS_LON_UUID "83f10002-8ca4-4c09-bb11-7357ac492601"
#define GPS_STATUS_UUID "83f10003-8ca4-4c09-bb11-7357ac492601"
#define FENCE_CONFIG_UUID "83f10004-8ca4-4c09-bb11-7357ac492601"
#define FENCE_STATE_UUID "83f10005-8ca4-4c09-bb11-7357ac492601"

HardwareSerial gpsSerial(2);
BLECharacteristic *ledChar, *latChar, *lonChar, *statusChar;
volatile bool bleConnected = false;
bool oledOk = false, gotByte = false, haveFix = false, havePosition = false;
String gpsLine;
double latitude = 0, longitude = 0;
int satellites = -1;
uint32_t lastByte = 0, lastFix = 0, lastUpdate = 0;

// ---- Geofence: a point and a radius, both written by the user over BLE ----
BLECharacteristic *fenceChar, *fenceStateChar;
portMUX_TYPE fenceMux = portMUX_INITIALIZER_UNLOCKED;  // BLE task writes, loop() reads
bool fenceSet = false;
double fenceLat = 0, fenceLon = 0, fenceRadius = 0;    // degrees, degrees, metres

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

// ---- Geofence logic ----

// Distance in metres between two points (haversine formula).
double distanceMeters(double lat1, double lon1, double lat2, double lon2) {
  double dLat = (lat2 - lat1) * DEG_TO_RAD, dLon = (lon2 - lon1) * DEG_TO_RAD;
  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(lat1 * DEG_TO_RAD) * cos(lat2 * DEG_TO_RAD) * sin(dLon / 2) * sin(dLon / 2);
  return 2 * 6371000.0 * asin(sqrt(fmin(1.0, a)));
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

// The user writes "lat,lon,radius", e.g. "51.5007,-0.1246,150" (degrees, degrees, metres).
class FenceCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    char text[64] = "";
    size_t len = c->getLength();
    if (len < sizeof(text)) memcpy(text, c->getData(), len);
    double lat, lon, radius;
    char extra;
    bool valid = sscanf(text, "%lf,%lf,%lf %c", &lat, &lon, &radius, &extra) == 3 &&
                 fabs(lat) <= 90 && fabs(lon) <= 180 && radius > 0;
    if (valid) {
      portENTER_CRITICAL(&fenceMux);
      fenceLat = lat;
      fenceLon = lon;
      fenceRadius = radius;
      fenceSet = true;
      portEXIT_CRITICAL(&fenceMux);
    }
    // Reading the characteristic always shows the fence in use.
    c->setValue(fenceSet ? (String(fenceLat, 6) + "," + String(fenceLon, 6) + "," + String(fenceRadius, 0)).c_str()
                         : "NOT SET");
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

  BLEDevice::init("IoT-T09-GPS");
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());
  BLEService* service = server->createService(BLEUUID(SERVICE_UUID), 32);
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
  service->start();
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06);
  adv->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
}

void loop() {
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

  // ---- Geofence: compare the current position with the user's point + radius ----
  portENTER_CRITICAL(&fenceMux);
  bool fenceReady = fenceSet;
  double fLat = fenceLat, fLon = fenceLon, fRadius = fenceRadius;
  portEXIT_CRITICAL(&fenceMux);
  String fence = "NOT SET";
  if (fenceReady && status != "FIX") fence = "NO FIX";
  else if (fenceReady) {
    double distance = distanceMeters(fLat, fLon, latitude, longitude);
    fence = String(distance <= fRadius ? "INSIDE " : "OUTSIDE ") + String(distance, 0) + "m";
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
