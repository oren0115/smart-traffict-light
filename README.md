# Smart Traffic Light (ESP32)

Sistem lampu lalu lintas pintar berbasis ESP32 dengan multi-sensor ultrasonik pada tiap jalur untuk mendeteksi panjang antrean kendaraan, memilih jalur paling padat, lalu memberi durasi hijau lebih lama secara dinamis.

## Fitur Utama

- Prioritas jalur terpadat memakai skor antrean + waktu tunggu (anti-starvation).
- Multi-sensor HC-SR04 per jalur (`sensorCount` per entri `intersections[]`).
- Scanner ultrasonik non-blocking dan interrupt-driven.
- Durasi hijau dinamis (`base + per_level * queueLevel`) dengan batas min/max.
- Mode idle saat semua jalur kosong.
- Mode malam kuning kedip berdasarkan jadwal NTP.
- Tuning parameter langsung via Serial Monitor tanpa upload ulang.

## Arsitektur Sistem

1. Setiap jalur memiliki 1..4 sensor ultrasonik.
2. Sistem membaca sensor per jalur, menghitung `queueLevel[]` (berapa sensor yang tertutup kendaraan).
3. Jalur berikutnya dipilih dari skor prioritas:
   - bobot antrean (`queueLevel`) lebih tinggi,
   - ditambah `waitCycles` agar jalur lain tidak kelaparan.
4. Durasi hijau aktif dihitung dinamis berdasarkan antrean jalur terpilih.
5. Transisi aman tetap: hijau -> kuning -> merah -> jalur berikutnya.

## Konfigurasi Jalur dan Sensor

Konfigurasi ada di `intersections[]` pada `smart-trafict-light.ino`:

```cpp
// Format:
// {pinRed, pinYellow, pinGreen, sensorCount, {trig...}, {echo...}, PHASE_RED}
TrafficIntersection intersections[] = {
  { 23, 22, 21, 2, { 13, 25, 255, 255 }, { 12, 26, 255, 255 }, PHASE_RED },
  { 19, 18,  5, 2, { 14, 33, 255, 255 }, { 27, 32, 255, 255 }, PHASE_RED },
};
```

Keterangan:
- `sensorCount` maksimal `MAX_SENSORS_PER_LANE` (default 4).
- Isi slot sensor yang tidak dipakai dengan placeholder (contoh `255`).
- Pastikan pin echo aman untuk ESP32 (gunakan level shifter/pembagi tegangan bila perlu).

## Parameter Algoritma

Parameter default di sketch:
- `DEFAULT_BASE_GREEN_MS = 15000`
- `DEFAULT_GREEN_PER_LEVEL_MS = 7000`
- `DEFAULT_MIN_GREEN_MS = 10000`
- `DEFAULT_GREEN_MAX_MS = 60000`

Rumus durasi hijau:

```text
green_time = clamp(base + queueLevel * per_level, min, max)
```

## Tuning Runtime Via Serial

Baud rate: `115200`

Perintah yang tersedia:
- `help`
- `status`
- `set base <detik>`
- `set per <detik>`
- `set min <detik>`
- `set max <detik>`
- `set default`

Contoh:
- `set base 20`
- `set per 8`
- `set min 12`
- `set max 55`
- `status`

`status` menampilkan mode sistem, jalur aktif, durasi hijau aktif, parameter tuning, serta antrean tiap jalur.

## Mode Operasi

- **Normal**: jalur dipilih berdasar skor prioritas dan antrean.
- **Idle**: jika semua jalur kosong, sistem rotasi fixed-time.
- **Night flash**: pada rentang jadwal malam (`NIGHT_START_*` hingga `NIGHT_END_*`), semua jalur kuning kedip.

Jika sinkronisasi WiFi/NTP gagal, mode malam tidak aktif dan sistem tetap berjalan normal/idle.

## Instalasi dan Upload

1. Install Arduino IDE / PlatformIO.
2. Install board package ESP32 by Espressif.
3. Pilih board ESP32 sesuai device.
4. Buka `smart-trafict-light.ino`.
5. Atur `WIFI_SSID` dan `WIFI_PASSWORD` bila memakai NTP.
6. Upload ke board, buka Serial Monitor `115200`.

## Catatan Wiring

- Lampu lalu lintas: sesuaikan apakah modul active-high atau active-low.
- HC-SR04:
  - VCC sesuai modul (umumnya 5V),
  - GND common dengan ESP32,
  - ECHO ke ESP32 wajib level aman 3.3V.
- Jaga jarak dan arah sensor agar crosstalk minimal.

## Troubleshooting Singkat

- Sensor selalu terdeteksi: cek threshold `VEHICLE_THRESHOLD_CM`, orientasi sensor, dan noise.
- Pembacaan sering timeout: cek kabel echo/trig, suplai daya, dan ground common.
- Prioritas tidak terasa: cek `queueLevel` dan tuning `base/per/min/max` via `status`.
- Mode malam tidak aktif: cek WiFi, NTP, dan offset zona waktu.

## Lisensi

Proyek akademik IoT, bebas dimodifikasi untuk pembelajaran.
