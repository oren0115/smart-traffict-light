/*
 * Smart Traffic Light - ESP32
 * Arsitektur modular & scalable: tambah persimpangan cukup di array intersections[].
 */

#include <Arduino.h>

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
const unsigned long GREEN_MAX_MS             = 60000UL;
const unsigned long MIN_GREEN_MS             = 5000UL;
const unsigned long YELLOW_DURATION_MS       = 3000UL;
const int           VEHICLE_THRESHOLD_CM     = 15;
const unsigned long SENSOR_INTERVAL_MS       = 200UL;
const unsigned long SENSOR_SAMPLE_GAP_MS     = 10UL;
const unsigned long SENSOR_CROSSTALK_GAP_MS  = 25UL;
const unsigned long IDLE_BLINK_MS            = 500UL;
const int           SENSOR_SAMPLES             = 3;
const unsigned long SETUP_STABILIZE_MS       = 500UL;
const unsigned long ULTRASONIC_PULSE_LOW_US  = 2UL;
const unsigned long ULTRASONIC_PULSE_HIGH_US = 10UL;
const unsigned long PULSE_TIMEOUT_US         = 12000UL;
const uint8_t       IDLE_CONFIRM_SCANS       = 3;  // siklus kosong berturut sebelum idle
const uint8_t       NORMAL_CONFIRM_SCANS     = 2;  // siklus ada kendaraan sebelum keluar idle

// --- State machine per lampu ---
enum LightPhase { PHASE_RED, PHASE_YELLOW, PHASE_GREEN };

// --- State machine pengontrol ---
enum ControllerPhase { CTRL_IDLE, CTRL_GREEN, CTRL_YELLOW };

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
  { 25, 26, 32, 33, 35, PHASE_RED },  // Persimpangan 2
};

const size_t INTERSECTION_COUNT =
    sizeof(intersections) / sizeof(intersections[0]);

size_t          activeIndex          = 0;
ControllerPhase controllerPhase      = CTRL_GREEN;
unsigned long   phaseStartMs         = 0;
unsigned long   lastSensorDecisionMs = 0;
unsigned long   lastIdleBlinkMs      = 0;
bool            vehicleOnActiveGreen = false;
bool            idleYellowOn         = false;

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
void setIdleYellowBlink(bool on);
void updateIdleBlink();
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
void enterIdleMode();
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
    enterIdleMode();
    Serial.println(F("Start: traffic kosong → kuning KEDIP"));
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

  if (controllerPhase == CTRL_IDLE) {
    updateIdleBlink();
    return;
  }

  if (controllerPhase == CTRL_GREEN) {
    const unsigned long greenElapsed = now - phaseStartMs;
    const bool maxTimeReached = greenElapsed >= GREEN_MAX_MS;
    const bool minGreenMet    = greenElapsed >= MIN_GREEN_MS;
    const bool laneEmpty      = !vehicleOnActiveGreen;
    const bool shouldSwitch   = maxTimeReached || (minGreenMet && laneEmpty);

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

  if (controllerPhase == CTRL_IDLE) {
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
  if (index != activeIndex && controllerPhase != CTRL_IDLE) {
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
  if (controllerPhase != CTRL_IDLE) {
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

void setIdleYellowBlink(bool on) {
  for (size_t i = 0; i < INTERSECTION_COUNT; i++) {
    writeLightPins(i, false, on, false);
    intersections[i].phase = PHASE_YELLOW;
  }
}

void updateIdleBlink() {
  const unsigned long now = millis();
  if (now - lastIdleBlinkMs < IDLE_BLINK_MS) {
    return;
  }
  lastIdleBlinkMs = now;
  idleYellowOn    = !idleYellowOn;
  setIdleYellowBlink(idleYellowOn);
}

void enterIdleMode() {
  if (controllerPhase == CTRL_IDLE) {
    return;
  }
  detachUltrasonicInterrupt();
  scanReadPending = false;

  controllerPhase = CTRL_IDLE;
  lastIdleBlinkMs = millis();
  idleYellowOn    = true;
  setIdleYellowBlink(true);
  emptyConfirmCount    = 0;
  occupiedConfirmCount = 0;
  Serial.println(F("[Idle] Traffic kosong → kuning KEDIP"));
}

void exitIdleToNormal() {
  const size_t start = (activeIndex + 1) % INTERSECTION_COUNT;
  activeIndex = findFirstOccupiedIndex(start);

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
