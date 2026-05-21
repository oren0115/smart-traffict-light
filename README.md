# Smart Traffic Light (ESP32) — 2 Gang

Sistem lampu lalu lintas pintar berbasis ESP32 untuk **tepat 2 gang** (persimpangan 2 arah). Saat Gang 1 hijau, Gang 2 merah — dan sebaliknya.

## Fitur Utama

- **2 gang** dengan lampu M/K/H dan 2 sensor HC-SR04 per gang.
- Prioritas gang terpadat (skor antrean + waktu tunggu).
- Scanner ultrasonik non-blocking (interrupt echo).
- Durasi hijau dinamis dengan batas min/max.
- Mode idle saat semua gang kosong (rotasi hijau tetap).
- Mode malam: kuning kedip (NTP).
- Tuning via Serial Monitor (`115200`).

## Logika Lampu (2 Gang)

| Fase | Gang aktif (hijau) | Gang lain |
|------|-------------------|-----------|
| Hijau | Hijau | **Merah** |
| Kuning | Kuning | Merah |
| Jeda aman | Merah | Merah |
| Ganti | — | Gang lain hijau |

Transisi: **Hijau (min 10 s) → Kuning (3 s) → Semua merah (2 s) → Hijau gang berikutnya.**

## Konfigurasi Pin (`intersections[]`)

Proyek ini **wajib 2 entri** di array. Jangan tambah entri ketiga tanpa mengubah `MAX_INTERSECTIONS`.

```cpp
TrafficIntersection intersections[] = {
  // Gang 1 (index 0): M12 K14 H27 | TRIG 13,25 | ECHO 26,33
  { 12, 14, 27, 2, { 13, 25, 255, 255 }, { 26, 33, 255, 255 }, PHASE_RED },
  // Gang 2 (index 1): M19 K18 H5 | TRIG 4,16 | ECHO 32,34
  { 19, 18,  5, 2, {  4, 16, 255, 255 }, { 32, 34, 255, 255 }, PHASE_RED },
};
```

| Gang | Merah | Kuning | Hijau | Trig | Echo |
|------|-------|--------|-------|------|------|
| 1 | 12 | 14 | 27 | 13, 25 | 26, 33 |
| 2 | 19 | 18 | 5 | 4, 16 | 32, 34 |

> GPIO 14 hanya untuk **kuning Gang 1**. Sensor Gang 2 memakai **GPIO 4** (bukan 14).

## Parameter Default

- `DEFAULT_BASE_GREEN_MS = 15000`
- `DEFAULT_GREEN_PER_LEVEL_MS = 7000`
- `DEFAULT_MIN_GREEN_MS = 10000`
- `DEFAULT_GREEN_MAX_MS = 60000`
- `YELLOW_DURATION_MS = 3000`
- `RED_HOLD_MS = 2000`

## Serial Monitor

| Perintah | Fungsi |
|----------|--------|
| `help` | Daftar perintah |
| `status` | Mode, gang aktif, tuning |
| `debug 0/1` | Log sensor on/off |
| `set base/per/min/max <detik>` | Tuning |
| `set default` | Reset tuning |

## Simulasi Wokwi

Buka `diagram.json` + `smart-trafict-light.ino` di [wokwi.com](https://wokwi.com). Diagram sudah diset untuk 2 gang sesuai pin di atas.

## Wiring

- Lampu: active-high default (`LIGHT_ACTIVE_LOW 0`). Jika relay active-low, set `#define LIGHT_ACTIVE_LOW 1`.
- HC-SR04: ECHO ke ESP32 harus level aman 3.3V.
- GPIO 5 (hijau Gang 2): pin strapping — hindari level tinggi saat boot.

## Lisensi

Proyek akademik IoT, bebas dimodifikasi untuk pembelajaran.
