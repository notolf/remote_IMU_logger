# M5Stack CoreS3 — Static Level Logger

Firmware for an **M5Stack CoreS3** that measures **static tilt (pitch & roll) to
~0.1°** while the robot is at rest, shows it on the device screen, serves a
**live web view** to a PC browser over WiFi, lets the operator **mark events
remotely**, and **logs every reading to a CSV file on the microSD card**.

The device only measures **at rest** — there is no motion/vibration fusion. The
gyro is used *only* to detect movement; the tilt comes purely from a heavily
averaged, calibration-corrected accelerometer reading.

- **Board:** M5Stack CoreS3 (ESP32-S3, 16 MB flash, 8 MB PSRAM, 2.0" touch IPS, microSD, BM8563 RTC, AXP2101 PMU)
- **IMU:** Bosch **BMI270** via **M5Unified** `M5.Imu` (no raw chip driver, BMM150 unused)
- **Toolchain:** PlatformIO + Arduino, board `m5stack-cores3`

---

## Project layout

| File | Purpose |
|------|---------|
| `platformio.ini` | Build configuration & library dependencies |
| `src/config.h`   | **All tunable parameters** (WiFi, thresholds, axes, cadences) |
| `src/web_page.h` | The self-contained live web page (inline HTML/CSS/JS, no CDN) |
| `src/main.cpp`   | Firmware: IMU, stationary detection, calibration, SD logging, WiFi, web |

---

## 1. Set your WiFi credentials

Open **`src/config.h`** and edit the section at the top:

```c
#define WIFI_SSID   "YOUR_WIFI_SSID"
#define WIFI_PASS   "YOUR_WIFI_PASSWORD"
```

WiFi mode is selectable with `WIFI_MODE_SELECT`:

- `WIFI_MODE_AUTO` *(default)* — try to join your router; if that fails within
  `WIFI_CONNECT_TIMEOUT_MS`, fall back to a **soft-AP** so the PC can still
  connect when the robot roams out of range.
- `WIFI_MODE_STA` — station only.
- `WIFI_MODE_AP` — soft-AP only.

The soft-AP credentials are also in `config.h`:

```c
#define AP_SSID   "LevelLogger"
#define AP_PASS   "level1234"     // >= 8 chars
```

Timezone defaults to **Asia/Singapore (UTC+8)** via `TZ_INFO "<+08>-8"` /
`TZ_OFFSET_SECONDS`. Change both to match your locale.

---

## 2. Build & flash with PlatformIO

```bash
# from the project root
pio run                 # build
pio run -t upload       # build + flash over USB-C
pio device monitor      # serial console @ 115200
```

(or use the PlatformIO VS Code extension: **Build** / **Upload** / **Monitor**.)

The serial console prints the WiFi IP, SD status, calibration load/save, and
NTP result on boot.

### If the board or async stack won't resolve

The ESP32 Arduino/PlatformIO ecosystem changes often. If `pio` cannot find the
`m5stack-cores3` board or fails to compile the async server against Arduino
core 3.x, switch to the community **pioarduino** platform — replace the
`platform = espressif32` line in `platformio.ini` with:

```ini
platform = https://github.com/pioarduino/platform-espressif32/releases/download/53.03.13/platform-espressif32.zip
```

The async libraries (`ESP32Async/AsyncTCP`, `ESP32Async/ESPAsyncWebServer`) are
the maintained forks and support both ESP32 Arduino core 2.x and 3.x.

---

## 3. Connect & open the web page

1. **Find the device address** (shown on the device screen, the `WiFi:` line,
   and on the serial console at boot):
   - **Station mode:** `WiFi: STA 192.168.x.y` → browse to `http://192.168.x.y/`
   - **Soft-AP mode:** join WiFi network **`LevelLogger`** (password
     `level1234`), then browse to **`http://192.168.4.1/`**
2. The page opens a WebSocket and shows **live pitch, roll, settled state,
   calibration status, battery, IMU temperature, events, SD status** and the log
   filename. It **reconnects automatically** if the socket drops.
3. Controls on the page: **Mark Event** (with optional label), **Start/Stop
   Logging**, **Calibrate** (runs the guided wizard), and **Clear Calibration**.

The same three actions are available on the device via the on-screen touch
buttons **CAL / MARK / LOG**.

---

## 4. Granite-plate flip/reversal calibration (step by step)

You need a surface plate that is flat/level to better than the 0.1° target
(e.g. a granite plate good to ~0.011°). Calibration cancels both the BMI270's
zero-g offset **and** any residual plate tilt.

> Start the wizard from the web **Calibrate** button or the on-screen **CAL**
> button. Advance steps with **Next ▶** (web) or **NEXT** (device). The web page
> shows the current prompt and a progress bar; the physical flip happens at the
> device.

1. **Step 1/2 — Orientation A.** Place the device **flat** on the plate in some
   orientation. Hold it perfectly still and press **Next**. The device collects
   `AVG_SAMPLES` (default 500 ≈ 5 s) of *stationary* samples — if it is disturbed,
   the capture restarts automatically.
2. **Step 2/2 — Orientation B.** **Rotate the device 180° about the vertical
   axis** (turn it around in place, same spot on the plate). Hold still and press
   **Next**. It captures again.
3. The firmware computes, per horizontal axis,
   `offset = (reading_A + reading_B) / 2`, stores it in **NVS**, and shows
   **Calibration complete**. Press **Finish**.

Why it works: the 180° vertical rotation flips the *sign* of the real horizontal
gravity component (the plate's residual tilt) while the sensor's own offset stays
put, so averaging A and B isolates the pure sensor offset:

```
reading_A = +g·sin(tilt) + offset
reading_B = −g·sin(tilt) + offset      →      (A + B) / 2 = offset
```

- Offsets are **persisted in NVS** and reloaded on boot. With no stored
  calibration the device runs at zero offset and clearly shows **UNCALIBRATED**
  on the screen and in the web view.
- **Re-run** any time (CAL / Calibrate). **Clear** stored calibration with the
  web **Clear Calibration** button.
- The BMI270's zero-g offset **drifts with temperature**, so recalibrate if the
  operating temperature changes a lot. Every CSV row logs `imu_temp_c` so you can
  correlate drift after the fact.

### Accelerometer range (±2 g)

The spec calls for the **±2 g** range for the finest resolution near level
(~1.7 mg ≈ 0.1°). We use **M5Unified's `M5.Imu`** exclusively — its high-level
API does *not* expose a runtime accel full-scale setter, and `getAccel()` returns
values already scaled to *g* for whatever range the driver configured. We
deliberately do **not** poke the BMI270 `ACC_RANGE` register (`0x41`) directly,
because that would be a raw chip-driver write **and** would desync M5Unified's
internal LSB→g scale factor (giving 4×-wrong readings). The 0.1° target is reached
through heavy averaging (`AVG_SAMPLES`) plus flip-calibrated offsets regardless of
the exact range. If you specifically need ±2 g enforced in hardware, set
`ACC_RANGE = 0x00` inside the M5Unified BMI270 driver (`IMU_BMI270.cpp`) so its
scaling stays consistent, then rebuild.

---

## 5. Where the CSV lands & how to read it

- A new file is created on the microSD at boot:
  - With a valid clock: **`/level_log_YYYYMMDD_HHMMSS.csv`**
  - Without NTP/RTC: an incrementing index, **`/level_log_0001.csv`**, etc.
- The file is **flushed after every write** (`LOG_FLUSH_EVERY`), so data survives
  a power loss on the robot. A missing/failed card is handled gracefully — the
  device keeps running, shows the SD state on screen/web, and re-probes for a card
  every few seconds.
- **Local SD is the source of truth**; the WebSocket stream is only a live mirror.

### Columns

```
timestamp_iso, millis, pitch_deg, roll_deg, ax_g, ay_g, az_g,
settled, calibrated, imu_temp_c, battery_pct, event_flag, event_label
```

| Column | Meaning |
|--------|---------|
| `timestamp_iso` | Wall-clock time `YYYY-MM-DDTHH:MM:SS+08:00`, or `NO_NTP` if the clock was never set (use `millis` then) |
| `millis` | Monotonic device uptime in ms (always valid) |
| `pitch_deg`, `roll_deg` | Tilt in degrees, `ANGLE_DECIMALS` places. For settled rows these are the **N-sample averaged** values; for event rows while moving they are the instantaneous reading |
| `ax_g`, `ay_g`, `az_g` | **Offset-corrected** accelerometer used for the angle (averaged when settled, instantaneous otherwise) |
| `settled` | `1` = device was at rest ≥ dwell time (high-confidence); `0` = moving |
| `calibrated` | `1` = stored flip-calibration applied; `0` = UNCALIBRATED |
| `imu_temp_c` | BMI270 die temperature (for drift correlation) |
| `battery_pct` | Battery %, `-1` if unavailable |
| `event_flag` | `1` if this row was produced by a **Mark Event**, else `0` |
| `event_label` | Optional operator label for the event (CSV-escaped) |

Row cadence:
- One **settled** row every `SETTLED_LOG_INTERVAL_MS` (default 1 s) while at rest.
- **Event** rows are written immediately when **Mark Event** is pressed (web or
  device), even while moving, so the moment is captured.
- Moving rows are *off* by default; enable with `LOG_WHILE_MOVING` in `config.h`.

---

## 6. How it works (brief)

- **Sampling:** the accelerometer + gyro are read at `IMU_SAMPLE_RATE_HZ`
  (100 Hz) on a non-blocking `millis()` schedule.
- **Stationary detection:** requires a quiet gyro (de-biased magnitude below
  `STATIONARY_GYRO_THRESH_DPS`) **and** a quiet accelerometer (low `std(|a|)` and
  `|a|` near 1 g), sustained for `STATIONARY_DWELL_MS`.
- **Tilt:** while stationary, a moving average of up to `AVG_SAMPLES` *consecutive
  rest* samples is taken (the window resets the instant motion is seen, so it
  never mixes in moving data); the row is flagged `settled` once the device has
  been still for the dwell time, and the average keeps sharpening toward the full
  `AVG_SAMPLES` as it stays still. The calibration offsets are subtracted and
  pitch/roll are computed with
  `atan2`. Axis signs are flippable via `PITCH_SIGN`, `ROLL_SIGN` and the two
  axes can be exchanged with `SWAP_PITCH_ROLL` (mounting orientation is unknown).
- **Gyro bias:** measured at boot (keep the device still for ~2 s) and slowly
  re-tracked while confidently at rest to follow temperature drift.

All thresholds, the averaging size, the dwell, cadences, axis conventions, and
credentials are grouped at the top of **`src/config.h`**.

---

## Acceptance checklist

- ✅ Builds for `m5stack-cores3` under PlatformIO/Arduino with the listed deps.
- ✅ Uses M5Unified `M5.Imu` (BMI270), no raw chip driver, no BMM150. See
  **§4 “Accelerometer range”** for the ±2 g note (documented per spec, since
  M5Unified exposes no range setter).
- ✅ Flip/reversal calibration, offsets persisted in NVS, survive reboot,
  **UNCALIBRATED** clearly indicated when absent.
- ✅ Static tilt output to 2–3 decimals; only settled readings flagged
  high-confidence.
- ✅ Self-contained web page (no external CDN), live values, auto-reconnect; the
  Mark Event button writes an event row to the CSV.
- ✅ CSV with the specified header/columns, flushed per write, tolerates a
  missing card without crashing.
- ✅ Both WiFi station and soft-AP fallback implemented and selectable at the top
  of `config.h`.
