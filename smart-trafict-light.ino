/*
 * Smart Traffic Light Otomatis - ESP32
 * 2 jalur (A & B), sensor HC-SR04 per jalur
 *
 * Hijau maksimal 60 detik; jika jalur hijau kosong sebelum itu → kuning 3 detik → merah → jalur lain hijau
 */

// --- Lampu Jalur A ---
const int PIN_RED_A    = 23;
const int PIN_YELLOW_A = 22;
const int PIN_GREEN_A  = 21;

// --- Lampu Jalur B ---
const int PIN_RED_B    = 19;
const int PIN_YELLOW_B = 18;
const int PIN_GREEN_B  = 5;

// --- HC-SR04 Jalur A ---
const int PIN_TRIG_A = 13;
const int PIN_ECHO_A = 12;

// --- HC-SR04 Jalur B ---
const int PIN_TRIG_B = 14;
const int PIN_ECHO_B = 27;

// --- Parameter ---
const unsigned long GREEN_MAX_MS      = 60000UL;  // 60 detik
const unsigned long YELLOW_DURATION_MS = 3000UL;  // 3 detik
const int VEHICLE_THRESHOLD_CM        = 15;       // < 15 cm = ada kendaraan
const unsigned long SENSOR_INTERVAL_MS = 200UL;   // interval baca sensor
const int SENSOR_SAMPLES              = 3;        // rata-rata untuk stabilitas

enum ActiveLane { LANE_A, LANE_B };
enum Phase { PHASE_GREEN, PHASE_YELLOW };

ActiveLane activeLane = LANE_A;
Phase phase = PHASE_GREEN;

unsigned long phaseStartMs = 0;
unsigned long lastSensorReadMs = 0;
bool vehicleOnGreenLane = false;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(F("\n=== Smart Traffic Light ESP32 ==="));

  pinMode(PIN_RED_A, OUTPUT);
  pinMode(PIN_YELLOW_A, OUTPUT);
  pinMode(PIN_GREEN_A, OUTPUT);
  pinMode(PIN_RED_B, OUTPUT);
  pinMode(PIN_YELLOW_B, OUTPUT);
  pinMode(PIN_GREEN_B, OUTPUT);

  pinMode(PIN_TRIG_A, OUTPUT);
  pinMode(PIN_ECHO_A, INPUT);
  pinMode(PIN_TRIG_B, OUTPUT);
  pinMode(PIN_ECHO_B, INPUT);

  // Kondisi awal: Jalur A hijau, Jalur B merah
  setLaneLights(LANE_A, false, false, true);   // A: hijau
  setLaneLights(LANE_B, true, false, false);    // B: merah

  phase = PHASE_GREEN;
  activeLane = LANE_A;
  phaseStartMs = millis();
  lastSensorReadMs = 0;

  Serial.println(F("Start: Jalur A HIJAU, Jalur B MERAH"));
}

void loop() {
  unsigned long now = millis();

  if (phase == PHASE_GREEN) {
    if (now - lastSensorReadMs >= SENSOR_INTERVAL_MS) {
      lastSensorReadMs = now;
      vehicleOnGreenLane = isVehiclePresent(activeLane);
    }

    unsigned long greenElapsed = now - phaseStartMs;
    bool maxTimeReached = greenElapsed >= GREEN_MAX_MS;
    bool shouldSwitch = !vehicleOnGreenLane || maxTimeReached;

    if (shouldSwitch) {
      if (maxTimeReached) {
        Serial.println(F("[Timer] 60 detik habis → ganti jalur"));
      } else {
        Serial.println(F("[Sensor] Jalur hijau kosong → ganti jalur"));
      }
      beginYellowTransition();
    }
  } else if (phase == PHASE_YELLOW) {
    if (now - phaseStartMs >= YELLOW_DURATION_MS) {
      finishYellowAndSwitchGreen();
    }
  }
}

// Hijau OFF → Kuning ON (jalur yang selesai hijau)
void beginYellowTransition() {
  ActiveLane finishing = activeLane;

  if (finishing == LANE_A) {
    setLaneLights(LANE_A, false, true, false);   // kuning A
    setLaneLights(LANE_B, true, false, false);   // B tetap merah
    Serial.println(F("Transisi: A KUNING (3 detik)"));
  } else {
    setLaneLights(LANE_B, false, true, false);
    setLaneLights(LANE_A, true, false, false);
    Serial.println(F("Transisi: B KUNING (3 detik)"));
  }

  phase = PHASE_YELLOW;
  phaseStartMs = millis();
}

// Kuning OFF → Merah (jalur lama) → Jalur lain HIJAU
void finishYellowAndSwitchGreen() {
  ActiveLane oldLane = activeLane;
  ActiveLane newLane = (oldLane == LANE_A) ? LANE_B : LANE_A;

  setLaneLights(oldLane, true, false, false);   // merah
  setLaneLights(newLane, false, false, true);   // hijau

  activeLane = newLane;
  phase = PHASE_GREEN;
  phaseStartMs = millis();
  lastSensorReadMs = 0;
  vehicleOnGreenLane = isVehiclePresent(activeLane);

  if (newLane == LANE_A) {
    Serial.println(F("Jalur B MERAH → Jalur A HIJAU"));
  } else {
    Serial.println(F("Jalur A MERAH → Jalur B HIJAU"));
  }
}

void setLaneLights(ActiveLane lane, bool red, bool yellow, bool green) {
  int pinR, pinY, pinG;
  if (lane == LANE_A) {
    pinR = PIN_RED_A;
    pinY = PIN_YELLOW_A;
    pinG = PIN_GREEN_A;
  } else {
    pinR = PIN_RED_B;
    pinY = PIN_YELLOW_B;
    pinG = PIN_GREEN_B;
  }

  // Modul traffic light umumnya active-LOW; ubah ke LOW jika lampu tidak menyala
  digitalWrite(pinR, red ? HIGH : LOW);
  digitalWrite(pinY, yellow ? HIGH : LOW);
  digitalWrite(pinG, green ? HIGH : LOW);
}

bool isVehiclePresent(ActiveLane lane) {
  int trigPin = (lane == LANE_A) ? PIN_TRIG_A : PIN_TRIG_B;
  int echoPin = (lane == LANE_A) ? PIN_ECHO_A : PIN_ECHO_B;

  long totalCm = 0;
  int validSamples = 0;

  for (int i = 0; i < SENSOR_SAMPLES; i++) {
    long cm = readDistanceCm(trigPin, echoPin);
    if (cm >= 0) {
      totalCm += cm;
      validSamples++;
    }
    delay(10);
  }

  if (validSamples == 0) {
    // Sensor gagal baca → anggap masih ada kendaraan (lebih aman)
    Serial.println(F("[Sensor] Gagal baca → anggap ada kendaraan"));
    return true;
  }

  long avgCm = totalCm / validSamples;
  bool present = avgCm < VEHICLE_THRESHOLD_CM;

  Serial.print(F("[Sensor] Jalur "));
  Serial.print(lane == LANE_A ? 'A' : 'B');
  Serial.print(F(" jarak="));
  Serial.print(avgCm);
  Serial.print(F(" cm → "));
  Serial.println(present ? F("ADA kendaraan") : F("KOSONG"));

  return present;
}

long readDistanceCm(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, 30000UL);  // timeout ~30 ms
  if (duration == 0) {
    return -1;
  }

  // Kecepatan suara ~343 m/s → cm = durasi_us / 58
  return (long)(duration / 58.0);
}
