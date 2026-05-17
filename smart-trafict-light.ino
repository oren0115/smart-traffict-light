/*
 * Smart Traffic Light - ESP32
 * Arsitektur modular & scalable: tambah persimpangan cukup di array intersections[].
 */

#include <Arduino.h>

// --- Parameter sistem ---
const unsigned long GREEN_MAX_MS        = 60000UL;
const unsigned long YELLOW_DURATION_MS  = 3000UL;
const int           VEHICLE_THRESHOLD_CM = 15;
const unsigned long SENSOR_INTERVAL_MS  = 200UL;
const int           SENSOR_SAMPLES      = 3;

// --- State machine per lampu ---
enum LightPhase { PHASE_RED, PHASE_YELLOW, PHASE_GREEN };

// --- State machine pengontrol (satu hijau aktif) ---
enum ControllerPhase { CTRL_GREEN, CTRL_YELLOW };

struct TrafficIntersection {
  uint8_t    pinRed;
  uint8_t    pinYellow;
  uint8_t    pinGreen;
  uint8_t    pinTrig;
  uint8_t    pinEcho;
  LightPhase phase;
};

/*
 * Konfigurasi pin — tambah baris baru untuk persimpangan tambahan.
 * Jumlah persimpangan dihitung otomatis dari ukuran array.
 */
TrafficIntersection intersections[] = {
  { 23, 22, 21, 13, 12, PHASE_RED },  // Persimpangan 0
  { 19, 18,  5, 14, 27, PHASE_RED },  // Persimpangan 1
  // { pinR, pinY, pinG, trig, echo, PHASE_RED },  // Contoh persimpangan 2
};

const size_t INTERSECTION_COUNT =
    sizeof(intersections) / sizeof(intersections[0]);

size_t          activeIndex          = 0;
ControllerPhase controllerPhase      = CTRL_GREEN;
unsigned long   phaseStartMs         = 0;
unsigned long   lastSensorReadMs     = 0;
bool            vehicleOnActiveGreen   = false;

// --- Deklarasi ---
void initHardware();
void setRed(size_t index);
void setYellow(size_t index);
void setGreen(size_t index);
void setAllRedExcept(size_t greenIndex);
long readDistanceCm(uint8_t trigPin, uint8_t echoPin);
bool isVehiclePresent(size_t index);
void beginYellowTransition();
void finishYellowAndActivateNext();

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(F("\n=== Smart Traffic Light ESP32 (Modular) ==="));
  Serial.print(F("Jumlah persimpangan: "));
  Serial.println(INTERSECTION_COUNT);

  initHardware();

  activeIndex     = 0;
  controllerPhase = CTRL_GREEN;
  phaseStartMs    = millis();
  lastSensorReadMs = 0;

  setAllRedExcept(activeIndex);
  setGreen(activeIndex);
  vehicleOnActiveGreen = isVehiclePresent(activeIndex);

  Serial.print(F("Start: persimpangan "));
  Serial.print(activeIndex);
  Serial.println(F(" HIJAU"));
}

void loop() {
  const unsigned long now = millis();

  if (controllerPhase == CTRL_GREEN) {
    if (now - lastSensorReadMs >= SENSOR_INTERVAL_MS) {
      lastSensorReadMs   = now;
      vehicleOnActiveGreen = isVehiclePresent(activeIndex);
    }

    const unsigned long greenElapsed = now - phaseStartMs;
    const bool maxTimeReached = greenElapsed >= GREEN_MAX_MS;
    const bool shouldSwitch   = !vehicleOnActiveGreen || maxTimeReached;

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

  activeIndex = (activeIndex + 1) % INTERSECTION_COUNT;

  setAllRedExcept(activeIndex);
  setGreen(activeIndex);

  controllerPhase      = CTRL_GREEN;
  phaseStartMs         = millis();
  lastSensorReadMs     = 0;
  vehicleOnActiveGreen = isVehiclePresent(activeIndex);

  Serial.print(F("Persimpangan "));
  Serial.print(activeIndex);
  Serial.println(F(" HIJAU"));
}

bool isVehiclePresent(size_t index) {
  if (index >= INTERSECTION_COUNT) return true;

  const uint8_t trigPin = intersections[index].pinTrig;
  const uint8_t echoPin = intersections[index].pinEcho;

  long totalCm      = 0;
  int  validSamples = 0;

  for (int s = 0; s < SENSOR_SAMPLES; s++) {
    const long cm = readDistanceCm(trigPin, echoPin);
    if (cm >= 0) {
      totalCm += cm;
      validSamples++;
    }
    delay(10);
  }

  if (validSamples == 0) {
    Serial.print(F("[Sensor] #"));
    Serial.print(index);
    Serial.println(F(" gagal baca → anggap ada kendaraan"));
    return true;
  }

  const long avgCm   = totalCm / validSamples;
  const bool present = avgCm < VEHICLE_THRESHOLD_CM;

  Serial.print(F("[Sensor] #"));
  Serial.print(index);
  Serial.print(F(" jarak="));
  Serial.print(avgCm);
  Serial.print(F(" cm → "));
  Serial.println(present ? F("ADA kendaraan") : F("KOSONG"));

  return present;
}

long readDistanceCm(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  const unsigned long duration = pulseIn(echoPin, HIGH, 30000UL);
  if (duration == 0) {
    return -1;
  }
  return (long)(duration / 58.0);
}
