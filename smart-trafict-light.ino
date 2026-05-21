/*
 * Smart Traffic Light - ESP32
 * Proyek 2 gang (persimpangan): Gang 1 hijau → Gang 2 merah, dan sebaliknya.
 * Konfigurasi pin hanya di intersections[] (tepat 2 entri).
 */

 #include <Arduino.h>
 #include <WiFi.h>
 #include <time.h>
 #include <ctype.h>
 #include <string.h>
 
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
 constexpr size_t  MAX_INTERSECTIONS          = 2;   // proyek ini: tetap 2 gang
 constexpr uint8_t MAX_SENSORS_PER_LANE       = 4;
const unsigned long DEFAULT_BASE_GREEN_MS      = 15000UL;
const unsigned long DEFAULT_GREEN_PER_LEVEL_MS = 7000UL;
const unsigned long DEFAULT_GREEN_MAX_MS       = 60000UL;
const unsigned long DEFAULT_MIN_GREEN_MS       = 10000UL;
const unsigned long YELLOW_DURATION_MS         = 3000UL;
const unsigned long RED_HOLD_MS                = 2000UL;  // semua merah sebelum hijau berikutnya
const unsigned long TUNING_MIN_ALLOWED_MS      = 10000UL;  // batas bawah set base/per/min/max via Serial
 const int           VEHICLE_THRESHOLD_CM     = 7;
 const unsigned long SENSOR_INTERVAL_MS       = 200UL;
 const unsigned long SENSOR_SAMPLE_GAP_MS     = 10UL;
 const unsigned long SENSOR_CROSSTALK_GAP_MS  = 25UL;
 const unsigned long IDLE_GREEN_MS            = 5000UL;  // hijau tetap saat traffic kosong
 const int           SENSOR_SAMPLES             = 3;
 const unsigned long SETUP_STABILIZE_MS       = 500UL;
 const unsigned long ULTRASONIC_PULSE_LOW_US  = 2UL;
 const unsigned long ULTRASONIC_PULSE_HIGH_US = 10UL;
 const unsigned long PULSE_TIMEOUT_US         = 7000UL;
const uint8_t       IDLE_CONFIRM_SCANS         = 3;
const uint8_t       NORMAL_CONFIRM_SCANS       = 2;
const uint8_t       VEHICLE_PRESENT_CONFIRM    = 2;  // debounce deteksi kendaraan per jalur
const uint8_t       VEHICLE_ABSENT_CONFIRM      = 2;
const uint8_t       ACTIVE_LANE_EMPTY_CONFIRM  = 3;  // debounce "jalur hijau kosong"

// Set 1 jika modul lampu/relay menyala saat pin LOW (active-low)
#ifndef LIGHT_ACTIVE_LOW
#define LIGHT_ACTIVE_LOW 0
#endif
 
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
 const uint8_t       NIGHT_END_HOUR           = 23;
 const uint8_t       NIGHT_END_MINUTE         = 00;
 const unsigned long YELLOW_FLASH_HALF_MS     = 200UL;      // periode kedip (on/off)
 const unsigned long TIME_CHECK_INTERVAL_MS   = 10000UL;    // cek jadwal malam
 const unsigned long NTP_SYNC_TIMEOUT_MS      = 7000UL;
 
 // --- State machine per lampu ---
 enum LightPhase { PHASE_RED, PHASE_YELLOW, PHASE_GREEN };
 
 // --- State machine pengontrol ---
 enum ControllerPhase { CTRL_GREEN, CTRL_YELLOW, CTRL_ALL_RED };
 
 struct TrafficIntersection {
   uint8_t    pinRed;
   uint8_t    pinYellow;
   uint8_t    pinGreen;
   uint8_t    sensorCount;
   uint8_t    pinTrig[MAX_SENSORS_PER_LANE];
   uint8_t    pinEcho[MAX_SENSORS_PER_LANE];
   LightPhase phase;
 };
 
TrafficIntersection intersections[] = {
  // Format: {red, yellow, green, jumlahSensor, {trig...}, {echo...}, phase}
  // Gang 1 (index 0)
  { 12, 14, 27, 2, { 13, 25, 255, 255 }, { 26, 33, 255, 255 }, PHASE_RED },
  // Gang 2 (index 1) — TRIG sensor pakai GPIO 4 & 16 (bukan 14, bentrok kuning Gang 1)
  { 19, 18,  5, 2, {  4, 16, 255, 255 }, { 32, 34, 255, 255 }, PHASE_RED },
};
 
 const size_t INTERSECTION_COUNT =
     sizeof(intersections) / sizeof(intersections[0]);
 
 size_t          activeIndex          = 0;
 ControllerPhase controllerPhase      = CTRL_GREEN;
 unsigned long   phaseStartMs         = 0;
 unsigned long   lastSensorDecisionMs = 0;
 bool            vehicleOnActiveGreen = false;
 unsigned long   activeGreenDurationMs = DEFAULT_BASE_GREEN_MS;
 unsigned long   tuningBaseGreenMs     = DEFAULT_BASE_GREEN_MS;
 unsigned long   tuningPerLevelMs      = DEFAULT_GREEN_PER_LEVEL_MS;
 unsigned long   tuningGreenMaxMs      = DEFAULT_GREEN_MAX_MS;
 unsigned long   tuningGreenMinMs      = DEFAULT_MIN_GREEN_MS;
 bool            idleMode             = false;
 bool            nightFlashMode       = false;
 bool            timeSynced           = false;
 bool            yellowFlashOn        = false;
 unsigned long   lastFlashToggleMs    = 0;
 unsigned long   lastTimeCheckMs      = 0;
 
 bool vehicleDetected[MAX_INTERSECTIONS];
 uint8_t queueLevel[MAX_INTERSECTIONS];
 uint16_t waitCycles[MAX_INTERSECTIONS];
 bool anyVehicleCached = false;
 
// Debounce transisi idle ↔ normal
uint8_t emptyConfirmCount    = 0;
uint8_t occupiedConfirmCount = 0;
uint8_t vehiclePresentStreak[MAX_INTERSECTIONS];
uint8_t vehicleAbsentStreak[MAX_INTERSECTIONS];
uint8_t activeLaneEmptyStreak = 0;
 
 // Scanner ultrasonik (non-blocking, interrupt echo)
 size_t          scanIntersection = 0;
 uint8_t         scanSensorIndex  = 0;
 uint8_t         scanSampleNum    = 0;
 long            scanAccumCm      = 0;
 int             scanValidCount   = 0;
 unsigned long   scanNextActionMs = 0;
 bool            sensorCacheReady = false;
 bool            scanReadPending  = false;
 bool            scanHadValidRead = false;  // true jika minimal 1 sensor berhasil dibaca dalam siklus ini
 
 // Ultrasonik async (satu sensor aktif per waktu)
 volatile uint8_t  usActiveEchoPin    = 0;
 volatile bool     usEchoReceived     = false;
 volatile uint32_t usEchoStartUs      = 0;
 volatile uint32_t usEchoDurationUs   = 0;
 volatile uint32_t usWaitStartUs      = 0;
 bool              usInterruptAttached = false;
 bool              sensorLogEnabled = DEBUG_SERIAL;
 
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
 void refreshAnyVehicleCache();
 void ensureValidActiveIndex();
 size_t findFirstOccupiedIndex(size_t startFrom);
 size_t selectPriorityIndex(size_t startFrom);
 unsigned long computeDynamicGreenDuration(size_t index);
 void updateWaitCycles(size_t servedIndex);
 void logSensorReading(size_t index, long avgCm, bool present);
 bool syncTimeFromNtp();
 bool isNightHours();
 void updateNightFlashBlink();
 void applyIdleScheduleByTime();
 void setAllYellowFlash(bool on);
 void enterNightFlashMode();
 void exitNightFlashToIdleRotate();
 void disableWifiAfterTimeSync();
 void enterIdleMode(bool rotateToNext = true);
 void exitIdleToNormal();
 void beginYellowTransition();
 void finishYellowAndActivateNext();
 void applySensorDecisions();
 void handleSerialCommands();
 void printControllerStatus();
 
 void setup() {
   Serial.begin(115200);
   delay(SETUP_STABILIZE_MS);
   Serial.println(F("\n=== Smart Traffic Light ESP32 (2 Gang) ==="));
 
  if (INTERSECTION_COUNT != MAX_INTERSECTIONS) {
    Serial.print(F("ERROR: intersections[] harus tepat "));
    Serial.print(MAX_INTERSECTIONS);
    Serial.print(F(" gang, saat ini: "));
    Serial.println(INTERSECTION_COUNT);
    while (true) {
      delay(1000);
    }
  }

  Serial.print(F("Jumlah gang: "));
  Serial.println(INTERSECTION_COUNT);
 
  initHardware();
  for (size_t i = 0; i < INTERSECTION_COUNT; i++) {
    setRed(i);
  }

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
    queueLevel[i] = 0;
    waitCycles[i] = 0;
    vehiclePresentStreak[i] = 0;
    vehicleAbsentStreak[i] = 0;
  }
  activeLaneEmptyStreak = 0;
 
   activeIndex            = 0;
   phaseStartMs           = millis();
   lastSensorDecisionMs   = 0;
   scanNextActionMs       = millis();
   emptyConfirmCount      = 0;
   occupiedConfirmCount   = 0;
 
   waitForSensorCycle();
   refreshAnyVehicleCache();
 
   if (isAnyVehiclePresent()) {
     activeIndex = selectPriorityIndex(0);
     controllerPhase = CTRL_GREEN;
     setAllRedExcept(activeIndex);
     setGreen(activeIndex);
     vehicleOnActiveGreen = getVehiclePresent(activeIndex);
     activeGreenDurationMs = computeDynamicGreenDuration(activeIndex);
     updateWaitCycles(activeIndex);
    Serial.print(F("Start: Gang "));
    Serial.print(activeIndex + 1);
     Serial.print(F(" HIJAU | antrean="));
     Serial.print(queueLevel[activeIndex]);
     Serial.print(F(" | hijau="));
     Serial.print(activeGreenDurationMs / 1000UL);
     Serial.println(F(" detik"));
   } else {
     enterIdleMode(false);
     Serial.println(F("Start: traffic kosong → siklus waktu tetap"));
   }
 
   Serial.println(F("Serial command siap. ketik 'help' untuk daftar perintah."));
 }
 
 void loop() {
   handleSerialCommands();
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
    setAllRedExcept(activeIndex);
     bool shouldSwitch = false;
 
     if (idleMode) {
       shouldSwitch = (now - phaseStartMs >= IDLE_GREEN_MS);
       if (shouldSwitch) {
         Serial.print(F("[Idle] Gang "));
         Serial.print(activeIndex + 1);
         Serial.print(F(" hijau "));
         Serial.print(IDLE_GREEN_MS / 1000UL);
         Serial.println(F(" detik → ganti"));
       }
     } else {
       const unsigned long greenElapsed = now - phaseStartMs;
       const bool maxTimeReached = greenElapsed >= activeGreenDurationMs;
       const bool minGreenMet    = greenElapsed >= tuningGreenMinMs;
       const bool laneEmpty      = !vehicleOnActiveGreen;
       shouldSwitch = maxTimeReached || (minGreenMet && laneEmpty);
 
       if (shouldSwitch) {
         if (maxTimeReached) {
          Serial.print(F("[Timer] Gang "));
          Serial.print(activeIndex + 1);
           Serial.println(F(" selesai durasi dinamis → ganti"));
         } else {
          Serial.print(F("[Sensor] Gang "));
          Serial.print(activeIndex + 1);
           Serial.println(F(" kosong → ganti"));
         }
       }
     }
 
     if (shouldSwitch) {
       beginYellowTransition();
     }
  } else if (controllerPhase == CTRL_YELLOW) {
    if (now - phaseStartMs >= YELLOW_DURATION_MS) {
      for (size_t i = 0; i < INTERSECTION_COUNT; i++) {
        setRed(i);
      }
      controllerPhase = CTRL_ALL_RED;
      phaseStartMs      = millis();
      activeLaneEmptyStreak = 0;
      Serial.println(F("Transisi: semua MERAH (jeda aman)"));
    }
  } else if (controllerPhase == CTRL_ALL_RED) {
    if (now - phaseStartMs >= RED_HOLD_MS) {
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
    if (!getVehiclePresent(activeIndex)) {
      if (activeLaneEmptyStreak < 255) {
        activeLaneEmptyStreak++;
      }
    } else {
      activeLaneEmptyStreak = 0;
    }
    vehicleOnActiveGreen = activeLaneEmptyStreak < ACTIVE_LANE_EMPTY_CONFIRM;
  }
}
 
 void printControllerStatus() {
   Serial.println(F("\n=== STATUS SMART TRAFFIC ==="));
   Serial.print(F("Mode: "));
   if (nightFlashMode) {
     Serial.println(F("NIGHT_FLASH"));
   } else if (idleMode) {
     Serial.println(F("IDLE"));
   } else {
     Serial.println(F("NORMAL"));
   }
 
  Serial.print(F("Gang aktif: "));
  Serial.println(activeIndex + 1);
  Serial.print(F("Fase kontrol: "));
  if (controllerPhase == CTRL_GREEN) {
    Serial.println(F("HIJAU"));
  } else if (controllerPhase == CTRL_YELLOW) {
    Serial.println(F("KUNING"));
  } else {
    Serial.println(F("SEMUA_MERAH"));
  }
  Serial.print(F("Tuning aktif min/max (detik): "));
  Serial.print(tuningGreenMinMs / 1000UL);
  Serial.print(F("/"));
  Serial.println(tuningGreenMaxMs / 1000UL);
  Serial.print(F("Durasi hijau aktif (detik): "));
   Serial.println(activeGreenDurationMs / 1000UL);
   Serial.print(F("Tuning (base/per/min/max detik): "));
   Serial.print(tuningBaseGreenMs / 1000UL);
   Serial.print(F("/"));
   Serial.print(tuningPerLevelMs / 1000UL);
   Serial.print(F("/"));
   Serial.print(tuningGreenMinMs / 1000UL);
   Serial.print(F("/"));
   Serial.println(tuningGreenMaxMs / 1000UL);
   Serial.print(F("Sensor log: "));
   Serial.println(sensorLogEnabled ? F("ON") : F("OFF"));
 
   for (size_t i = 0; i < INTERSECTION_COUNT; i++) {
     Serial.print(F("Gang "));
     Serial.print(i + 1);
     Serial.print(F(" -> antrean="));
     Serial.print(queueLevel[i]);
     Serial.print(F(", ada kendaraan="));
     Serial.print(vehicleDetected[i] ? F("YA") : F("TIDAK"));
     Serial.print(F(", waitCycles="));
     Serial.println(waitCycles[i]);
   }
 }
 
 void handleSerialCommands() {
   if (!Serial.available()) {
     return;
   }
 
   static char cmdBuf[64];
   const size_t len = Serial.readBytesUntil('\n', cmdBuf, sizeof(cmdBuf) - 1);
   cmdBuf[len] = '\0';
   if (len == 0) {
     return;
   }
 
   // Trim leading spaces and newline/CR sisa terminal.
   char* cmd = cmdBuf;
   while (*cmd == ' ' || *cmd == '\t' || *cmd == '\r') {
     cmd++;
   }
   if (*cmd == '\0') {
     return;
   }
 
   size_t end = strlen(cmd);
   while (end > 0 && (cmd[end - 1] == '\r' || cmd[end - 1] == ' ' || cmd[end - 1] == '\t')) {
     cmd[end - 1] = '\0';
     end--;
   }
 
   if (strcasecmp(cmd, "help") == 0) {
     Serial.println(F("Perintah:"));
     Serial.println(F("- status"));
     Serial.println(F("- debug <0|1>"));
     Serial.println(F("- set base <detik>"));
     Serial.println(F("- set per <detik>"));
     Serial.println(F("- set min <detik>"));
     Serial.println(F("- set max <detik>"));
     Serial.println(F("- set default"));
     return;
   }
 
   if (strcasecmp(cmd, "status") == 0) {
     printControllerStatus();
     return;
   }
 
   int debugValue = -1;
   if (sscanf(cmd, "debug %d", &debugValue) == 1) {
     if (debugValue == 0 || debugValue == 1) {
       sensorLogEnabled = debugValue == 1;
       Serial.print(F("Sensor log "));
       Serial.println(sensorLogEnabled ? F("AKTIF") : F("NONAKTIF"));
     } else {
       Serial.println(F("Nilai debug harus 0 atau 1."));
     }
     return;
   }
 
   if (strcasecmp(cmd, "set default") == 0) {
     tuningBaseGreenMs = DEFAULT_BASE_GREEN_MS;
     tuningPerLevelMs  = DEFAULT_GREEN_PER_LEVEL_MS;
     tuningGreenMinMs  = DEFAULT_MIN_GREEN_MS;
     tuningGreenMaxMs  = DEFAULT_GREEN_MAX_MS;
     activeGreenDurationMs = computeDynamicGreenDuration(activeIndex);
     Serial.println(F("Tuning dikembalikan ke default."));
     printControllerStatus();
     return;
   }
 
   char key[12];
   long valueSec = 0;
   if (sscanf(cmd, "set %11s %ld", key, &valueSec) != 2) {
     Serial.println(F("Perintah tidak dikenal. ketik 'help'."));
     return;
   }
 
  if (valueSec <= 0) {
    Serial.println(F("Nilai harus > 0 detik."));
    return;
  }
  if ((unsigned long)valueSec * 1000UL < TUNING_MIN_ALLOWED_MS) {
    Serial.print(F("Nilai minimal "));
    Serial.print(TUNING_MIN_ALLOWED_MS / 1000UL);
    Serial.println(F(" detik (hindari hijau < 1 detik)."));
    return;
  }
   for (size_t i = 0; key[i] != '\0'; i++) {
     key[i] = (char)tolower((unsigned char)key[i]);
   }
 
   const unsigned long valueMs = (unsigned long)valueSec * 1000UL;
   if (strcmp(key, "base") == 0) {
     tuningBaseGreenMs = valueMs;
   } else if (strcmp(key, "per") == 0) {
     tuningPerLevelMs = valueMs;
   } else if (strcmp(key, "min") == 0) {
     tuningGreenMinMs = valueMs;
   } else if (strcmp(key, "max") == 0) {
     tuningGreenMaxMs = valueMs;
   } else {
     Serial.println(F("Key tidak valid. Gunakan: base/per/min/max"));
     return;
   }
 
   activeGreenDurationMs = computeDynamicGreenDuration(activeIndex);
   Serial.println(F("Tuning diperbarui."));
   printControllerStatus();
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
 
void updateVehicleDetectedDebounced(size_t idx, bool rawPresent) {
  if (rawPresent) {
    vehicleAbsentStreak[idx] = 0;
    if (vehiclePresentStreak[idx] < 255) {
      vehiclePresentStreak[idx]++;
    }
  } else {
    vehiclePresentStreak[idx] = 0;
    if (vehicleAbsentStreak[idx] < 255) {
      vehicleAbsentStreak[idx]++;
    }
  }

  if (vehiclePresentStreak[idx] >= VEHICLE_PRESENT_CONFIRM) {
    vehicleDetected[idx] = true;
  } else if (vehicleAbsentStreak[idx] >= VEHICLE_ABSENT_CONFIRM) {
    vehicleDetected[idx] = false;
  }
}

void finalizeScanIntersection(size_t idx) {
  const uint8_t sensorCount = intersections[idx].sensorCount;
  if (scanValidCount == 0) {
    DBG_PRINT(F("[Sensor] #"));
    DBG_PRINT(idx);
    DBG_PRINT(F("["));
    DBG_PRINT(scanSensorIndex);
    DBG_PRINTLN(F("] gagal baca → pertahankan status terakhir"));
    // queueLevel tidak diubah; scanHadValidRead tetap false untuk sensor ini
  } else {
    scanHadValidRead = true;
    const long avgCm = scanAccumCm / scanValidCount;
    const bool present = avgCm < VEHICLE_THRESHOLD_CM;
    if (present && queueLevel[idx] < sensorCount) {
      queueLevel[idx]++;
    }
    logSensorReading(idx, avgCm, present);
  }

  scanSampleNum = 0;
  scanSensorIndex++;
  if (scanSensorIndex >= sensorCount) {
    if (scanHadValidRead) {
      // Minimal 1 sensor berhasil → update debounce dengan hasil nyata
      const bool rawPresent = queueLevel[idx] > 0;
      updateVehicleDetectedDebounced(idx, rawPresent);
    }
    // Semua sensor gagal → vehicleDetected[idx] tidak disentuh sama sekali
    scanSensorIndex = 0;
    scanIntersection++;
  }
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
       refreshAnyVehicleCache();
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
     scanSensorIndex = 0;
   }
 
   if (scanSampleNum == 0) {
     scanAccumCm    = 0;
     scanValidCount = 0;
   }
 
  if (scanSensorIndex == 0 && scanSampleNum == 0) {
    queueLevel[scanIntersection] = 0;
    scanHadValidRead = false;
  }
 
   const uint8_t trigPin = intersections[scanIntersection].pinTrig[scanSensorIndex];
   const uint8_t echoPin = intersections[scanIntersection].pinEcho[scanSensorIndex];
 
   startAsyncDistanceRead(trigPin, echoPin);
   scanReadPending = true;
 }
 
 void logSensorReading(size_t index, long avgCm, bool present) {
 #if DEBUG_SERIAL
   if (!sensorLogEnabled) {
     return;
   }
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
   return anyVehicleCached;
 }
 
 void refreshAnyVehicleCache() {
   anyVehicleCached = false;
   for (size_t i = 0; i < INTERSECTION_COUNT; i++) {
     if (vehicleDetected[i]) {
       anyVehicleCached = true;
       return;
     }
   }
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
 
 size_t selectPriorityIndex(size_t startFrom) {
   if (INTERSECTION_COUNT == 0) {
     return 0;
   }
 
   const size_t start = startFrom % INTERSECTION_COUNT;
   size_t bestIndex = start;
   int bestScore = -1;
 
   for (size_t k = 0; k < INTERSECTION_COUNT; k++) {
     const size_t i = (start + k) % INTERSECTION_COUNT;
     if (!getVehiclePresent(i)) {
       continue;
     }
 
     // Bobot antrean lebih tinggi, waitCycles mencegah starvation.
     const int score = (int)queueLevel[i] * 100 + (int)waitCycles[i] * 10;
     if (score > bestScore) {
       bestScore = score;
       bestIndex = i;
     }
   }
 
   if (bestScore < 0) {
     return start;
   }
   return bestIndex;
 }
 
 unsigned long computeDynamicGreenDuration(size_t index) {
   if (index >= INTERSECTION_COUNT) {
     return tuningBaseGreenMs;
   }
   unsigned long minGreen = tuningGreenMinMs;
   unsigned long maxGreen = tuningGreenMaxMs;
   if (minGreen > maxGreen) {
     const unsigned long tmp = minGreen;
     minGreen = maxGreen;
     maxGreen = tmp;
   }
 
   const unsigned long dynamicMs = tuningBaseGreenMs + (unsigned long)queueLevel[index] * tuningPerLevelMs;
   return constrain(dynamicMs, minGreen, maxGreen);
 }
 
 void updateWaitCycles(size_t servedIndex) {
   for (size_t i = 0; i < INTERSECTION_COUNT; i++) {
     if (i == servedIndex) {
       waitCycles[i] = 0;
     } else if (getVehiclePresent(i) && waitCycles[i] < 1000) {
       waitCycles[i]++;
     } else if (!getVehiclePresent(i)) {
       waitCycles[i] = 0;
     }
   }
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
     if (intersections[i].sensorCount == 0 || intersections[i].sensorCount > MAX_SENSORS_PER_LANE) {
       Serial.print(F("ERROR: sensorCount tidak valid di jalur "));
       Serial.println(i);
       while (true) {
         delay(1000);
       }
     }
 
     pinMode(intersections[i].pinRed, OUTPUT);
     pinMode(intersections[i].pinYellow, OUTPUT);
     pinMode(intersections[i].pinGreen, OUTPUT);
     for (uint8_t s = 0; s < intersections[i].sensorCount; s++) {
       pinMode(intersections[i].pinTrig[s], OUTPUT);
       pinMode(intersections[i].pinEcho[s], INPUT);
     }
   }
 }
 
void writeLightPins(size_t index, bool red, bool yellow, bool green) {
#if LIGHT_ACTIVE_LOW
  digitalWrite(intersections[index].pinRed, red ? LOW : HIGH);
  digitalWrite(intersections[index].pinYellow, yellow ? LOW : HIGH);
  digitalWrite(intersections[index].pinGreen, green ? LOW : HIGH);
#else
  digitalWrite(intersections[index].pinRed, red ? HIGH : LOW);
  digitalWrite(intersections[index].pinYellow, yellow ? HIGH : LOW);
  digitalWrite(intersections[index].pinGreen, green ? HIGH : LOW);
#endif
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
       disableWifiAfterTimeSync();
       return true;
     }
     delay(500);
   }
 
   Serial.println(F("Sinkronisasi NTP gagal"));
   disableWifiAfterTimeSync();
   return false;
 #endif
 }
 
 void disableWifiAfterTimeSync() {
 #if ENABLE_NTP_TIME
   // WiFi tidak dibutuhkan untuk kontrol lampu harian, matikan agar lebih hemat resource.
   if (WiFi.getMode() != WIFI_OFF) {
     WiFi.disconnect(true, true);
     WiFi.mode(WIFI_OFF);
   }
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
   Serial.println(F("] Kuning kedip (hati-hati) — kedua gang"));
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
 
  Serial.print(F("[Siang] Kembali idle rotasi → Gang "));
  Serial.print(activeIndex + 1);
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
 
   const bool nightNow = isNightHours();
   if (nightNow && !nightFlashMode) {
     enterNightFlashMode();
   } else if (!nightNow && nightFlashMode) {
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
   activeGreenDurationMs = IDLE_GREEN_MS;
   lastSensorDecisionMs = millis();
   lastTimeCheckMs      = millis();
   emptyConfirmCount    = 0;
   occupiedConfirmCount = 0;
  Serial.print(F("[Idle] Traffic kosong → siklus tetap, Gang "));
  Serial.print(activeIndex + 1);
   Serial.println(F(" HIJAU"));
 }
 
 void exitIdleToNormal() {
   const size_t start = (activeIndex + 1) % INTERSECTION_COUNT;
   activeIndex = selectPriorityIndex(start);
 
   idleMode       = false;
   nightFlashMode = false;
   setAllRedExcept(activeIndex);
   setGreen(activeIndex);
 
  controllerPhase       = CTRL_GREEN;
  phaseStartMs          = millis();
  activeLaneEmptyStreak = 0;
  activeGreenDurationMs = computeDynamicGreenDuration(activeIndex);
  lastSensorDecisionMs  = millis();
  vehicleOnActiveGreen  = getVehiclePresent(activeIndex);
  updateWaitCycles(activeIndex);
  emptyConfirmCount     = 0;
  occupiedConfirmCount  = 0;

  Serial.print(F("[Normal] Kendaraan terdeteksi → Gang "));
  Serial.print(activeIndex + 1);
   Serial.print(F(" HIJAU | antrean="));
   Serial.print(queueLevel[activeIndex]);
   Serial.print(F(" | hijau="));
   Serial.print(activeGreenDurationMs / 1000UL);
   Serial.println(F(" detik"));
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
 
  Serial.print(F("Transisi: Gang "));
  Serial.print(activeIndex + 1);
  Serial.print(F(" KUNING ("));
  Serial.print(YELLOW_DURATION_MS / 1000UL);
  Serial.println(F(" detik)"));
}

void finishYellowAndActivateNext() {
  if (idleMode) {
     activeIndex = (activeIndex + 1) % INTERSECTION_COUNT;
     setAllRedExcept(activeIndex);
     setGreen(activeIndex);
     controllerPhase = CTRL_GREEN;
     phaseStartMs      = millis();
     Serial.print(F("[Idle] Gang "));
     Serial.print(activeIndex + 1);
     Serial.println(F(" HIJAU"));
     return;
   }
 
   if (!isAnyVehiclePresent()) {
     enterIdleMode();
     return;
   }
 
   const size_t nextStart = (activeIndex + 1) % INTERSECTION_COUNT;
   activeIndex = selectPriorityIndex(nextStart);
 
  setAllRedExcept(activeIndex);
  setGreen(activeIndex);

  controllerPhase       = CTRL_GREEN;
  phaseStartMs          = millis();
  activeLaneEmptyStreak = 0;
  activeGreenDurationMs = computeDynamicGreenDuration(activeIndex);
  vehicleOnActiveGreen  = getVehiclePresent(activeIndex);
  if (vehicleOnActiveGreen) {
    activeLaneEmptyStreak = 0;
  }
  updateWaitCycles(activeIndex);

  Serial.print(F("Gang "));
  Serial.print(activeIndex + 1);
   Serial.print(F(" HIJAU | antrean="));
   Serial.print(queueLevel[activeIndex]);
   Serial.print(F(" | hijau="));
   Serial.print(activeGreenDurationMs / 1000UL);
   Serial.println(F(" detik"));
 }
 