/*
 * Smart Traffic Light - ESP32
 * Arsitektur modular & scalable: tambah persimpangan cukup di array intersections[].
 */

#include <Arduino.h>
#include <WiFi.h>
#include <time.h>

// Set 0 untuk build produksi (tanpa log sensor di hot path)
#ifndef DEBUG_SERIAL
#define DEBUG_SERIAL 1
#endif

#if DEBUG_SERIAL
#define DBG_PRINT(x)    Serial.print(x)
#define DBG_PRINTLN(x)  Serial.println(x)
#else
#define DBG_PRINT(x)
#define DBG_PRINTLN(x)
#endif

// --- Parameter sistem ---
constexpr size_t  MAX_INTERSECTIONS          = 8;
const unsigned long GREEN_MAX_MS             = 30000UL;
const unsigned long MIN_GREEN_MS             = 30000UL;
const unsigned long YELLOW_DURATION_MS       = 3000UL;
const int           VEHICLE_THRESHOLD_CM     = 7;
const unsigned long SENSOR_INTERVAL_MS       = 200UL;
const unsigned long SENSOR_SAMPLE_GAP_MS     = 10UL;
const unsigned long SENSOR_CROSSTALK_GAP_MS  = 25UL;
const unsigned long IDLE_GREEN_MS            = 15000UL;  // hijau tetap saat traffic kosong
const int           SENSOR_SAMPLES             = 3;
const unsigned long SETUP_STABILIZE_MS       = 500UL;
const unsigned long ULTRASONIC_PULSE_LOW_US  = 2UL;
const unsigned long ULTRASONIC_PULSE_HIGH_US = 10UL;
const unsigned long PULSE_TIMEOUT_US         = 12000UL;
const uint8_t       IDLE_CONFIRM_SCANS       = 3;  // siklus kosong berturut sebelum idle
const uint8_t       NORMAL_CONFIRM_SCANS     = 2;  // siklus ada kendaraan sebelum keluar idle

// --- Jadwal malam: kuning kedip (hati-hati), seperti lampu jalan ---
#ifndef ENABLE_NTP_TIME
#define ENABLE_NTP_TIME 1
#endif
const char* const   WIFI_SSID                = "Gaga";
const char* const   WIFI_PASSWORD            = "12345678";
const long          NTP_GMT_OFFSET_SEC       = 7 * 3600;   // WIB UTC+7
const int           NTP_DAYLIGHT_OFFSET_SEC  = 0;
const uint8_t       NIGHT_START_HOUR         = 19;
const uint8_t       NIGHT_START_MINUTE       = 58;
const uint8_t       NIGHT_END_HOUR           = 20;
const uint8_t       NIGHT_END_MINUTE         = 00;
const unsigned long YELLOW_FLASH_HALF_MS     = 500UL;      // periode kedip (on/off)
const unsigned long TIME_CHECK_INTERVAL_MS   = 30000UL;    // cek jadwal malam
const unsigned long NTP_SYNC_TIMEOUT_MS      = 15000UL;

// --- State machine per lampu ---
enum LightPhase { PHASE_RED, PHASE_YELLOW, PHASE_GREEN };

// --- State machine pengontrol ---
enum ControllerPhase { CTRL_GREEN, CTRL_YELLOW };

struct TrafficIntersection {
  uint8_t    pinRed;
  uint8_t    pinYellow;
  uint8_t    pinGreen;
  uint8_t    pinTrig;
  uint8_t    pinEcho;
  LightPhase phase;
};

TrafficIntersection intersections[] = {
  { 23, 22, 21, 13, 12, PHASE_RED },  // Persimpangan 0
  { 19, 18,  5, 14, 27, PHASE_RED },  // Persimpangan 1
  // { 25, 26, 32, 33, 35, PHASE_RED },  // Persimpangan 2
};

const size_t INTERSECTION_COUNT =
    sizeof(intersections) / sizeof(intersections[0]);

size_t          activeIndex          = 0;
ControllerPhase controllerPhase      = CTRL_GREEN;
unsigned long   phaseStartMs         = 0;
unsigned long   lastSensorDecisionMs = 0;
bool            vehicleOnActiveGreen = false;
bool            idleMode             = false;
bool            nightFlashMode       = false;
bool            timeSynced           = false;
bool            yellowFlashOn        = false;
unsigned long   lastFlashToggleMs    = 0;
unsigned long   lastTimeCheckMs      = 0;

bool vehicleDetected[MAX_INTERSECTIONS];

// Debounce transisi idle ↔ normal
uint8_t emptyConfirmCount    = 0;
uint8_t occupiedConfirmCount = 0;

// Scanner ultrasonik (non-blocking, interrupt echo)
size_t          scanIntersection = 0;
uint8_t         scanSampleNum    = 0;
long            scanAccumCm      = 0;
int             scanValidCount   = 0;
unsigned long   scanNextActionMs = 0;
bool            sensorCacheReady = false;
bool            scanReadPending  = false;

// Ultrasonik async (satu sensor aktif per waktu)
volatile uint8_t  usActiveEchoPin    = 0;
volatile bool     usEchoReceived     = false;
volatile uint32_t usEchoStartUs      = 0;
volatile uint32_t usEchoDurationUs   = 0;
volatile uint32_t usWaitStartUs      = 0;
bool              usInterruptAttached = false;

void IRAM_ATTR ultrasonicEchoIsr() {
  const uint32_t now = micros();
  if (digitalRead(usActiveEchoPin)) {
    usEchoStartUs = now;
    usEchoReceived = false;
  } else if (usEchoStartUs > 0) {
    usEchoDurationUs = now - usEchoStartUs;
    usEchoReceived = true;
  }
}

// --- Deklarasi ---
void initHardware();
void writeLightPins(size_t index, bool red, bool yellow, bool green);
void setRed(size_t index);
void setYellow(size_t index);
void setGreen(size_t index);
void setAllRedExcept(size_t greenIndex);
void detachUltrasonicInterrupt();
void startAsyncDistanceRead(uint8_t trigPin, uint8_t echoPin);
long pollAsyncDistanceRead();
void updateSensorScan();
void waitForSensorCycle();
bool getVehiclePresent(size_t index);
bool isAnyVehiclePresent();
void ensureValidActiveIndex();
size_t findFirstOccupiedIndex(size_t startFrom);
void logSensorReading(size_t index, long avgCm, bool present);
bool syncTimeFromNtp();
bool isNightHours();
void updateNightFlashBlink();
void applyIdleScheduleByTime();
void setAllYellowFlash(bool on);
void enterNightFlashMode();
void exitNightFlashToIdleRotate();
void enterIdleMode(bool rotateToNext = true);
void exitIdleToNormal();
void beginYellowTransition();
void finishYellowAndActivateNext();
void applySensorDecisions();

void setup() {
  Serial.begin(115200);
  delay(SETUP_STABILIZE_MS);
  Serial.println(F("\n=== Smart Traffic Light ESP32 (Modular) ==="));

  if (INTERSECTION_COUNT > MAX_INTERSECTIONS) {
    Serial.println(F("ERROR: INTERSECTION_COUNT > MAX_INTERSECTIONS"));
    while (true) {
      delay(1000);
    }
  }

  Serial.print(F("Jumlah persimpangan: "));
  Serial.println(INTERSECTION_COUNT);

  initHardware();

#if ENABLE_NTP_TIME
  timeSynced = syncTimeFromNtp();
  if (!timeSynced) {
    Serial.println(F("PERINGATAN: NTP gagal → idle malam (kuning kedip) nonaktif"));
  }
#else
  Serial.println(F("NTP nonaktif (ENABLE_NTP_TIME=0) → hanya idle rotasi hijau"));
#endif

  for (size_t i = 0; i < INTERSECTION_COUNT; i++) {
    vehicleDetected[i] = false;
  }

  activeIndex            = 0;
  phaseStartMs           = millis();
  lastSensorDecisionMs   = 0;
  scanNextActionMs       = millis();
  emptyConfirmCount      = 0;
  occupiedConfirmCount   = 0;

  waitForSensorCycle();

  if (isAnyVehiclePresent()) {
    activeIndex = findFirstOccupiedIndex(0);
    controllerPhase = CTRL_GREEN;
    setAllRedExcept(activeIndex);
    setGreen(activeIndex);
    vehicleOnActiveGreen = getVehiclePresent(activeIndex);
    Serial.print(F("Start: persimpangan "));
    Serial.print(activeIndex);
    Serial.println(F(" HIJAU"));
  } else {
    enterIdleMode(false);
    Serial.println(F("Start: traffic kosong → siklus waktu tetap"));
  }
}

void loop() {
  ensureValidActiveIndex();
  updateSensorScan();

  const unsigned long now = millis();
  if (sensorCacheReady && (now - lastSensorDecisionMs >= SENSOR_INTERVAL_MS)) {
    lastSensorDecisionMs = now;
    applySensorDecisions();
  }

  if (idleMode && now - lastTimeCheckMs >= TIME_CHECK_INTERVAL_MS) {
    lastTimeCheckMs = now;
    applyIdleScheduleByTime();
  }

  if (nightFlashMode) {
    updateNightFlashBlink();
    return;
  }

  if (controllerPhase == CTRL_GREEN) {
    bool shouldSwitch = false;

    if (idleMode) {
      shouldSwitch = (now - phaseStartMs >= IDLE_GREEN_MS);
      if (shouldSwitch) {
        Serial.print(F("[Idle] Persimpangan "));
        Serial.print(activeIndex);
        Serial.print(F(" hijau "));
        Serial.print(IDLE_GREEN_MS / 1000UL);
        Serial.println(F(" detik → ganti"));
      }
    } else {
      const unsigned long greenElapsed = now - phaseStartMs;
      const bool maxTimeReached = greenElapsed >= GREEN_MAX_MS;
      const bool minGreenMet    = greenElapsed >= MIN_GREEN_MS;
      const bool laneEmpty      = !vehicleOnActiveGreen;
      shouldSwitch = maxTimeReached || (minGreenMet && laneEmpty);

      if (shouldSwitch) {
        if (maxTimeReached) {
          Serial.print(F("[Timer] Persimpangan "));
          Serial.print(activeIndex);
          Serial.println(F(" 60 detik → ganti"));
        } else {
          Serial.print(F("[Sensor] Persimpangan "));
          Serial.print(activeIndex);
          Serial.println(F(" kosong → ganti"));
        }
      }
    }

    if (shouldSwitch) {
      beginYellowTransition();
    }
  } else if (controllerPhase == CTRL_YELLOW) {
    if (now - phaseStartMs >= YELLOW_DURATION_MS) {
      finishYellowAndActivateNext();
    }
  }
}

void applySensorDecisions() {
  const bool anyVehicle = isAnyVehiclePresent();

  if (idleMode) {
    if (anyVehicle) {
      occupiedConfirmCount++;
      emptyConfirmCount = 0;
      if (occupiedConfirmCount >= NORMAL_CONFIRM_SCANS) {
        occupiedConfirmCount = 0;
        exitIdleToNormal();
      }
    } else {
      occupiedConfirmCount = 0;
    }
    return;
  }

  if (!anyVehicle) {
    emptyConfirmCount++;
    occupiedConfirmCount = 0;
    if (emptyConfirmCount >= IDLE_CONFIRM_SCANS) {
      emptyConfirmCount = 0;
      enterIdleMode();
    }
  } else {
    emptyConfirmCount = 0;
    vehicleOnActiveGreen = getVehiclePresent(activeIndex);
  }
}

/*
 * Hanya untuk setup(): menunggu satu siklus scan penuh.
 * yield() memberi waktu ke task FreeRTOS di ESP32, bukan menjalankan loop() aplikasi.
 * Jangan panggil dari loop() — gunakan flag sensorCacheReady di sana.
 */
void waitForSensorCycle() {
  sensorCacheReady = false;
  while (!sensorCacheReady) {
    updateSensorScan();
    yield();
  }
}

void detachUltrasonicInterrupt() {
  if (!usInterruptAttached) {
    return;
  }
  detachInterrupt(digitalPinToInterrupt(usActiveEchoPin));
  usInterruptAttached = false;
}

void startAsyncDistanceRead(uint8_t trigPin, uint8_t echoPin) {
  detachUltrasonicInterrupt();

  usActiveEchoPin   = echoPin;
  usEchoStartUs     = 0;
  usEchoDurationUs  = 0;
  usEchoReceived    = false;
  usWaitStartUs     = micros();

  attachInterrupt(digitalPinToInterrupt(echoPin), ultrasonicEchoIsr, CHANGE);
  usInterruptAttached = true;

  digitalWrite(trigPin, LOW);
  delayMicroseconds(ULTRASONIC_PULSE_LOW_US);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(ULTRASONIC_PULSE_HIGH_US);
  digitalWrite(trigPin, LOW);
}

// -2 = masih menunggu; -1 = gagal/timeout; >=0 = jarak cm
long pollAsyncDistanceRead() {
  if (usEchoReceived) {
    detachUltrasonicInterrupt();
    if (usEchoDurationUs == 0) {
      return -1;
    }
    return (long)(usEchoDurationUs / 58);
  }

  if (micros() - usWaitStartUs > PULSE_TIMEOUT_US) {
    detachUltrasonicInterrupt();
    return -1;
  }

  return -2;
}

void finalizeScanIntersection(size_t idx) {
  if (scanValidCount == 0) {
    vehicleDetected[idx] = true;
    DBG_PRINT(F("[Sensor] #"));
    DBG_PRINT(idx);
    DBG_PRINTLN(F(" gagal baca → anggap ada kendaraan"));
  } else {
    const long avgCm = scanAccumCm / scanValidCount;
    vehicleDetected[idx] = avgCm < VEHICLE_THRESHOLD_CM;
    logSensorReading(idx, avgCm, vehicleDetected[idx]);
  }

  scanSampleNum = 0;
  scanIntersection++;
}

void updateSensorScan() {
  const unsigned long now = millis();

  if (scanReadPending) {
    const long cm = pollAsyncDistanceRead();
    if (cm == -2) {
      return;
    }

    scanReadPending = false;

    if (cm >= 0) {
      scanAccumCm += cm;
      scanValidCount++;
    }

    scanSampleNum++;

    if (scanSampleNum < SENSOR_SAMPLES) {
      scanNextActionMs = now + SENSOR_SAMPLE_GAP_MS;
      return;
    }

    finalizeScanIntersection(scanIntersection);

    if (scanIntersection >= INTERSECTION_COUNT) {
      scanIntersection = 0;
      sensorCacheReady = true;
    }
    scanNextActionMs = now + SENSOR_CROSSTALK_GAP_MS;
    return;
  }

  if (now < scanNextActionMs) {
    return;
  }

  if (scanIntersection >= INTERSECTION_COUNT) {
    scanIntersection = 0;
  }

  if (scanSampleNum == 0) {
    scanAccumCm    = 0;
    scanValidCount = 0;
  }

  const uint8_t trigPin = intersections[scanIntersection].pinTrig;
  const uint8_t echoPin = intersections[scanIntersection].pinEcho;

  startAsyncDistanceRead(trigPin, echoPin);
  scanReadPending = true;
}

void logSensorReading(size_t index, long avgCm, bool present) {
#if DEBUG_SERIAL
  if (index != activeIndex && !idleMode) {
    return;
  }
  DBG_PRINT(F("[Sensor] #"));
  DBG_PRINT(index);
  DBG_PRINT(F(" jarak="));
  DBG_PRINT(avgCm);
  DBG_PRINT(F(" cm → "));
  DBG_PRINTLN(present ? F("ADA kendaraan") : F("KOSONG"));
#endif
}

bool getVehiclePresent(size_t index) {
  if (index >= INTERSECTION_COUNT) {
    return true;
  }
  return vehicleDetected[index];
}

bool isAnyVehiclePresent() {
  for (size_t i = 0; i < INTERSECTION_COUNT; i++) {
    if (vehicleDetected[i]) {
      return true;
    }
  }
  return false;
}

size_t findFirstOccupiedIndex(size_t startFrom) {
  if (INTERSECTION_COUNT == 0) {
    return 0;
  }
  const size_t start = startFrom % INTERSECTION_COUNT;
  for (size_t k = 0; k < INTERSECTION_COUNT; k++) {
    const size_t i = (start + k) % INTERSECTION_COUNT;
    if (getVehiclePresent(i)) {
      return i;
    }
  }
  return start;
}

void ensureValidActiveIndex() {
  if (activeIndex < INTERSECTION_COUNT) {
    return;
  }
  Serial.println(F("[Watchdog] activeIndex tidak valid → reset ke 0"));
  activeIndex = 0;
  if (!idleMode) {
    enterIdleMode();
  }
}

void initHardware() {
  for (size_t i = 0; i < INTERSECTION_COUNT; i++) {
    pinMode(intersections[i].pinRed, OUTPUT);
    pinMode(intersections[i].pinYellow, OUTPUT);
    pinMode(intersections[i].pinGreen, OUTPUT);
    pinMode(intersections[i].pinTrig, OUTPUT);
    pinMode(intersections[i].pinEcho, INPUT);
  }
}

void writeLightPins(size_t index, bool red, bool yellow, bool green) {
  digitalWrite(intersections[index].pinRed, red ? HIGH : LOW);
  digitalWrite(intersections[index].pinYellow, yellow ? HIGH : LOW);
  digitalWrite(intersections[index].pinGreen, green ? HIGH : LOW);
}

void setRed(size_t index) {
  if (index >= INTERSECTION_COUNT) return;
  writeLightPins(index, true, false, false);
  intersections[index].phase = PHASE_RED;
}

void setYellow(size_t index) {
  if (index >= INTERSECTION_COUNT) return;
  writeLightPins(index, false, true, false);
  intersections[index].phase = PHASE_YELLOW;
}

void setGreen(size_t index) {
  if (index >= INTERSECTION_COUNT) return;
  writeLightPins(index, false, false, true);
  intersections[index].phase = PHASE_GREEN;
}

void setAllRedExcept(size_t greenIndex) {
  for (size_t i = 0; i < INTERSECTION_COUNT; i++) {
    if (i != greenIndex) {
      setRed(i);
    }
  }
}

bool syncTimeFromNtp() {
#if !ENABLE_NTP_TIME
  return false;
#else
  Serial.print(F("WiFi: "));
  Serial.println(WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  const unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < NTP_SYNC_TIMEOUT_MS) {
    delay(250);
    Serial.print('.');
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("WiFi gagal terhubung"));
    return false;
  }

  Serial.print(F("IP: "));
  Serial.println(WiFi.localIP());

  configTime(NTP_GMT_OFFSET_SEC, NTP_DAYLIGHT_OFFSET_SEC, "pool.ntp.org", "time.nist.gov");

  struct tm timeInfo;
  for (uint8_t attempt = 0; attempt < 20; attempt++) {
    if (getLocalTime(&timeInfo, 500)) {
      Serial.printf("Waktu sinkron: %02d:%02d:%02d\n",
                    timeInfo.tm_hour, timeInfo.tm_min, timeInfo.tm_sec);
      return true;
    }
    delay(500);
  }

  Serial.println(F("Sinkronisasi NTP gagal"));
  return false;
#endif
}

bool isNightHours() {
#if !ENABLE_NTP_TIME
  (void)timeSynced;
  return false;
#else
  if (!timeSynced) {
    return false;
  }

  struct tm timeInfo;
  if (!getLocalTime(&timeInfo)) {
    return false;
  }

  const int nowMinutes    = timeInfo.tm_hour * 60 + timeInfo.tm_min;
  const int startMinutes  = NIGHT_START_HOUR * 60 + NIGHT_START_MINUTE;
  const int endMinutes    = NIGHT_END_HOUR * 60 + NIGHT_END_MINUTE;

  if (startMinutes > endMinutes) {
    return nowMinutes >= startMinutes || nowMinutes < endMinutes;
  }
  return nowMinutes >= startMinutes && nowMinutes < endMinutes;
#endif
}

void setAllYellowFlash(bool on) {
  for (size_t i = 0; i < INTERSECTION_COUNT; i++) {
    if (on) {
      writeLightPins(i, false, true, false);
      intersections[i].phase = PHASE_YELLOW;
    } else {
      writeLightPins(i, false, false, false);
    }
  }
  yellowFlashOn = on;
}

void enterNightFlashMode() {
  if (nightFlashMode) {
    return;
  }

  detachUltrasonicInterrupt();
  scanReadPending = false;

  nightFlashMode    = true;
  idleMode          = true;
  yellowFlashOn     = true;
  lastFlashToggleMs = millis();
  lastTimeCheckMs   = millis();
  setAllYellowFlash(true);

  emptyConfirmCount    = 0;
  occupiedConfirmCount = 0;

  Serial.print(F("[Malam "));
  Serial.print(NIGHT_START_HOUR);
  Serial.print(':');
  if (NIGHT_START_MINUTE < 10) Serial.print('0');
  Serial.print(NIGHT_START_MINUTE);
  Serial.print(F("–"));
  Serial.print(NIGHT_END_HOUR);
  Serial.print(':');
  if (NIGHT_END_MINUTE < 10) Serial.print('0');
  Serial.println(NIGHT_END_MINUTE);
  Serial.println(F("] Kuning kedip (hati-hati) — semua persimpangan"));
}

void exitNightFlashToIdleRotate() {
  if (!nightFlashMode) {
    return;
  }

  nightFlashMode = false;
  setAllRedExcept(activeIndex);
  setGreen(activeIndex);

  controllerPhase = CTRL_GREEN;
  phaseStartMs    = millis();

  Serial.print(F("[Siang] Kembali idle rotasi → persimpangan "));
  Serial.print(activeIndex);
  Serial.println(F(" HIJAU"));
}

void updateNightFlashBlink() {
  const unsigned long now = millis();
  if (now - lastFlashToggleMs < YELLOW_FLASH_HALF_MS) {
    return;
  }
  lastFlashToggleMs = now;
  setAllYellowFlash(!yellowFlashOn);
}

void applyIdleScheduleByTime() {
  if (!idleMode || !timeSynced) {
    return;
  }

  if (isNightHours() && !nightFlashMode) {
    enterNightFlashMode();
  } else if (!isNightHours() && nightFlashMode) {
    exitNightFlashToIdleRotate();
  }
}

void enterIdleMode(bool rotateToNext) {
  if (idleMode && !nightFlashMode) {
    return;
  }
  if (nightFlashMode && isNightHours()) {
    return;
  }

  detachUltrasonicInterrupt();
  scanReadPending = false;

  idleMode = true;
  nightFlashMode = false;

#if ENABLE_NTP_TIME
  if (timeSynced && isNightHours()) {
    enterNightFlashMode();
    return;
  }
#endif

  if (rotateToNext) {
    activeIndex = (activeIndex + 1) % INTERSECTION_COUNT;
  }
  setAllRedExcept(activeIndex);
  setGreen(activeIndex);

  controllerPhase      = CTRL_GREEN;
  phaseStartMs         = millis();
  lastSensorDecisionMs = millis();
  lastTimeCheckMs      = millis();
  emptyConfirmCount    = 0;
  occupiedConfirmCount = 0;
  Serial.print(F("[Idle] Traffic kosong → siklus tetap, persimpangan "));
  Serial.print(activeIndex);
  Serial.println(F(" HIJAU"));
}

void exitIdleToNormal() {
  const size_t start = (activeIndex + 1) % INTERSECTION_COUNT;
  activeIndex = findFirstOccupiedIndex(start);

  idleMode       = false;
  nightFlashMode = false;
  setAllRedExcept(activeIndex);
  setGreen(activeIndex);

  controllerPhase      = CTRL_GREEN;
  phaseStartMs         = millis();
  lastSensorDecisionMs = millis();
  vehicleOnActiveGreen = getVehiclePresent(activeIndex);
  emptyConfirmCount    = 0;
  occupiedConfirmCount = 0;

  Serial.print(F("[Normal] Kendaraan terdeteksi → persimpangan "));
  Serial.print(activeIndex);
  Serial.println(F(" HIJAU"));
}

void beginYellowTransition() {
  setYellow(activeIndex);
  for (size_t i = 0; i < INTERSECTION_COUNT; i++) {
    if (i != activeIndex) {
      setRed(i);
    }
  }

  controllerPhase = CTRL_YELLOW;
  phaseStartMs    = millis();

  Serial.print(F("Transisi: persimpangan "));
  Serial.print(activeIndex);
  Serial.println(F(" KUNING (3 detik)"));
}

void finishYellowAndActivateNext() {
  setRed(activeIndex);

  if (idleMode) {
    activeIndex = (activeIndex + 1) % INTERSECTION_COUNT;
    setAllRedExcept(activeIndex);
    setGreen(activeIndex);
    controllerPhase = CTRL_GREEN;
    phaseStartMs      = millis();
    Serial.print(F("[Idle] Persimpangan "));
    Serial.print(activeIndex);
    Serial.println(F(" HIJAU"));
    return;
  }

  if (!isAnyVehiclePresent()) {
    enterIdleMode();
    return;
  }

  const size_t nextStart = (activeIndex + 1) % INTERSECTION_COUNT;
  activeIndex = findFirstOccupiedIndex(nextStart);

  setAllRedExcept(activeIndex);
  setGreen(activeIndex);

  controllerPhase      = CTRL_GREEN;
  phaseStartMs         = millis();
  vehicleOnActiveGreen = getVehiclePresent(activeIndex);

  Serial.print(F("Persimpangan "));
  Serial.print(activeIndex);
  Serial.println(F(" HIJAU"));
}
