# M5Stack CoreS3 — Static Level Logger (standalone)

Firmware for an **M5Stack CoreS3** that measures **static tilt (pitch & roll) to
~0.1°** while the robot is at rest, shows it live on the device screen as a
**bubble (bullseye) level**, lets the operator **mark events** with the touch
buttons, and **logs every reading to a CSV file on the microSD card**.

This is a **standalone, offline** build — there is **no WiFi and no web UI**. The
device is operated entirely from the touchscreen and the data lives on the SD card.

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
| `platformio.ini` | Build configuration & the single library dependency (M5Unified) |
| `src/config.h`   | **All tunable parameters** (timezone, thresholds, axes, bubble, cadences) |
| `src/main.cpp`   | Firmware: IMU, stationary detection, calibration, bubble display, SD logging |

---

## 1. Configure (optional)

Everything tunable is at the top of **`src/config.h`**. There are **no WiFi
credentials** to set. The only thing you might change before first flash is the
timezone used for timestamps:

```c
#define TZ_INFO            "<+08>-8"     // Asia/Singapore, UTC+8 (POSIX TZ string)
#define TZ_OFFSET_SECONDS  (8 * 3600)    // keep in sync with TZ_INFO
```

Timestamps come from the battery-backed **BM8563 RTC**. Since there is no NTP,
they are real wall-clock only if the RTC was set beforehand (e.g. with
M5Burner's *Set time*). If the RTC is unset, log files use an incrementing index
and `timestamp_iso` is logged as `NO_TIME` — the `millis` column is always valid.

---

## 2. Build & flash with PlatformIO

```bash
# from the project root
pio run                 # build
pio run -t upload       # build + flash over USB-C
pio device monitor      # serial console @ 115200
```

(or use the PlatformIO VS Code extension: **Build** / **Upload** / **Monitor**.)

The serial console prints SD status, calibration load/save, and gyro-bias on boot.

### If the board won't resolve

If `pio` cannot find the `m5stack-cores3` board, switch to the community
**pioarduino** platform — replace the `platform = espressif32` line in
`platformio.ini` with:

```ini
platform = https://github.com/pioarduino/platform-espressif32/releases/download/53.03.13/platform-espressif32.zip
```

---

## 3. On-device controls (touchscreen)

The main screen shows the live bubble level plus three touch buttons along the
bottom:

| Button | Action |
|--------|--------|
| **CAL**  | **Tap** = start the calibration wizard. **Hold ~1.5 s** = clear stored calibration (back to UNCALIBRATED). |
| **MARK** | Write an event row to the CSV immediately (even while moving). |
| **LOG**  | Start / stop logging (shows **STOP** while logging). |

During the calibration wizard the bottom buttons become **NEXT** (advance /
finish) and **CANCEL**.

---

## 4. Granite-plate flip/reversal calibration (step by step)

You need a surface plate that is flat/level to better than the 0.1° target
(e.g. a granite plate good to ~0.011°). Calibration cancels both the BMI270's
zero-g offset **and** any residual plate tilt.

> Start the wizard by tapping **CAL**. Advance each step with **NEXT**. The
> screen shows the current prompt and a progress bar.

1. **Step 1/2 — Orientation A.** Place the device **flat** on the plate in some
   orientation. Hold it perfectly still and tap **NEXT**. The device collects
   `AVG_SAMPLES` (default 500 ≈ 5 s) of *stationary* samples — if it is disturbed,
   the capture restarts automatically.
2. **Step 2/2 — Orientation B.** **Rotate the device 180° about the vertical
   axis** (turn it around in place, same spot on the plate). Hold still and tap
   **NEXT**. It captures again.
3. The firmware validates the flip, computes `offset = (reading_A + reading_B) / 2`
   per horizontal axis, stores it in **NVS**, and shows **Calibration complete**.
   Tap **FINISH**.

Why it works: the 180° vertical rotation flips the *sign* of the real horizontal
gravity component (the plate's residual tilt) while the sensor's own offset stays
put, so averaging A and B isolates the pure sensor offset:

```
reading_A = +g·sin(tilt) + offset
reading_B = −g·sin(tilt) + offset      →      (A + B) / 2 = offset
```

- **Validation:** before saving, the firmware checks the device was roughly flat
  in both captures (Z vertical, `az ≈ 1 g`) and that the turn was about the
  **vertical** axis (`az` unchanged A↔B). If a check fails the offsets are **not**
  saved and the screen shows a red `FAILED: …` so you can retry. Tolerances are
  `CAL_FLAT_TOL_G` / `CAL_VERT_TOL_G`.
- Offsets are **persisted in NVS** and reloaded on boot. With no stored
  calibration the device runs at zero offset and clearly shows **UNCALIBRATED**.
- **Re-run** any time (tap CAL). **Clear** stored calibration by **holding** CAL
  ~1.5 s.
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
  - With a valid clock (RTC set): **`/level_log_YYYYMMDD_HHMMSS.csv`**
  - Without a clock: an incrementing index, **`/level_log_0001.csv`**, etc.
- The file is **flushed after every write** (`LOG_FLUSH_EVERY`), so data survives
  a power loss on the robot. A missing/failed card is handled gracefully — the
  device keeps running, shows the SD state on screen, and re-probes for a card
  every few seconds.

### Columns

```
timestamp_iso, millis, pitch_deg, roll_deg,
ax_g, ay_g, az_g, ax_raw_g, ay_raw_g, az_raw_g, gx_dps, gy_dps, gz_dps,
settled, calibrated, imu_temp_c, battery_pct, event_flag, event_label
```

| Column | Meaning |
|--------|---------|
| `timestamp_iso` | Wall-clock time `YYYY-MM-DDTHH:MM:SS+08:00`, or `NO_TIME` if the RTC was never set (use `millis` then) |
| `millis` | Monotonic device uptime in ms (always valid) |
| `pitch_deg`, `roll_deg` | Tilt in degrees, `ANGLE_DECIMALS` places. For settled rows these are the **N-sample averaged** values; for event rows while moving they are the instantaneous reading |
| `ax_g`, `ay_g`, `az_g` | **Offset-corrected** accelerometer used for the angle (averaged when settled, instantaneous otherwise) |
| `ax_raw_g`, `ay_raw_g`, `az_raw_g` | **Raw** accelerometer (no calibration offset) — `ax_g = ax_raw_g − offset_x`, etc. Lets you re-derive angles or re-calibrate offline |
| `gx_dps`, `gy_dps`, `gz_dps` | **Raw** gyroscope (deg/s, latest sample) — useful for spotting vibration/disturbance in post-analysis |
| `settled` | `1` = device was at rest ≥ dwell time (high-confidence); `0` = moving |
| `calibrated` | `1` = stored flip-calibration applied; `0` = UNCALIBRATED |
| `imu_temp_c` | BMI270 die temperature (for drift correlation) |
| `battery_pct` | Battery %, `-1` if unavailable |
| `event_flag` | `1` if this row was produced by a **MARK** press, else `0` |
| `event_label` | Event label (`device-button` for the MARK button), CSV-escaped |

Row cadence:
- One **settled** row every `SETTLED_LOG_INTERVAL_MS` (default 1 s) while at rest.
- **Event** rows are written immediately when **MARK** is pressed, even while
  moving, so the moment is captured.
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
  pitch/roll are computed with `atan2`. Axis signs are flippable via `PITCH_SIGN`,
  `ROLL_SIGN` and the two axes can be exchanged with `SWAP_PITCH_ROLL` (mounting
  orientation is unknown).
- **Gyro bias:** measured at boot (keep the device still for ~2 s) and slowly
  re-tracked while confidently at rest to follow temperature drift.

### On-device bubble level

The main screen is a live **bullseye spirit level**: a bubble drifts toward the
raised side as you tilt the device, and the numeric **pitch and roll** for both
axes are shown beside it (green when settled/high-confidence, amber while moving).

- **Auto-ranging scale** — the vial's full-scale (degrees at the outer ring)
  adjusts automatically with hysteresis: it zooms *in* for sub-degree work (max
  sensitivity) and zooms *out* so the bubble never leaves the vial. The current
  full-scale (e.g. `+/-2.0 deg`) is printed under the bubble. Steps and behaviour
  are set by `BUBBLE_SCALE_STEPS` / `BUBBLE_FILL_FRACTION` in `config.h`.
- The bubble turns **green** when both axes are within `LEVEL_TOLERANCE_DEG` of
  level. Its motion is lightly smoothed (`BUBBLE_SMOOTH_ALPHA`) so it is
  responsive but steady. If the bubble drifts the "wrong" way for your mounting,
  flip `PITCH_SIGN` / `ROLL_SIGN`.

All thresholds, the averaging size, the dwell, cadences, axis conventions, and
the bubble behaviour are grouped at the top of **`src/config.h`**.

---

## Notes

- Uses M5Unified `M5.Imu` (BMI270) only — no raw chip driver, no BMM150. See
  **§4 “Accelerometer range”** for the ±2 g note.
- Flip/reversal calibration with validation, offsets persisted in NVS, survive
  reboot, **UNCALIBRATED** clearly indicated when absent.
- Static tilt output to 2–3 decimals; only settled readings flagged
  high-confidence.
- CSV with raw + corrected data, flushed per write, tolerates a missing card
  without crashing.
- **No networking:** WiFi, NTP and the web server have been removed; the device
  is fully self-contained.
