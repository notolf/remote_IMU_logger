# M5Stack CoreS3 — Static Level Logger

Firmware for an **M5Stack CoreS3** that measures **static tilt (pitch & roll) to
~0.1°** while the robot is at rest, shows it live on the device screen as a
**bubble (bullseye) level**, logs every reading to a **CSV file on the microSD
card**, and serves a **live dashboard to your phone** over its own WiFi —
scan the on-screen QR code and the dashboard pops up. No app, no infrastructure,
no company network involvement.

The device only measures **at rest** — there is no motion/vibration fusion. The
gyro is used *only* to detect movement; the tilt comes purely from a heavily
averaged, calibration-corrected accelerometer reading. The microSD CSV remains
the data of record; the dashboard is live telemetry plus remote control.

- **Board:** M5Stack CoreS3 (ESP32-S3, 16 MB flash, 8 MB PSRAM, 2.0" touch IPS, microSD, BM8563 RTC, AXP2101 PMU)
- **IMU:** Bosch **BMI270** via **M5Unified** `M5.Imu` (no raw chip driver, BMM150 unused)
- **Toolchain:** PlatformIO + Arduino, board `m5stack-cores3`

---

## Project layout

| File | Purpose |
|------|---------|
| `platformio.ini` | Build configuration (M5Unified is the only library dependency; the web stack is all ESP32-core built-ins) |
| `src/config.h`   | **All tunable parameters** (WiFi, timezone, thresholds, axes, bubble, cadences) |
| `src/main.cpp`   | Firmware: IMU, stationary detection, calibration, bubble display, SD logging, WiFi + JSON API |
| `src/web_page.h` | The embedded phone dashboard (single self-contained HTML page, NOVA design tokens) |
| `tools/evaluate_level_log.py` | Offline evaluation of the CSV logs → self-contained HTML report |

---

## 1. Configure (optional)

Everything tunable is at the top of **`src/config.h`**. Out of the box the
device hosts its own WiFi (`LevelLogger` / `level1234`) — change those before
deploying:

```c
#define AP_SSID  "LevelLogger"
#define AP_PASS  "level1234"        // WPA2: must be >= 8 characters
```

Optionally switch `WIFI_MODE_SELECT` to `LOGGER_WIFI_STA` / `LOGGER_WIFI_AUTO`
to join a **phone hotspot** instead (set `WIFI_SSID`/`WIFI_PASS`; the ESP32 is
2.4 GHz-only — enable *Maximize Compatibility* on iPhone hotspots). In station
mode the device also gets **NTP** time automatically and stores it to the RTC.

Timezone for timestamps:

```c
#define TZ_INFO            "<+08>-8"     // Asia/Singapore, UTC+8 (POSIX TZ string)
#define TZ_OFFSET_SECONDS  (8 * 3600)    // keep in sync with TZ_INFO
```

The measurement parameters (stationary thresholds, averaging window, dwell, log
cadence, axis signs, level tolerance) in `config.h` are **factory defaults**:
they can be changed at runtime from the dashboard's **Settings panel** and are
persisted in NVS — after first boot, the NVS copy wins.

---

## 2. Build & flash with PlatformIO

```bash
# from the project root
pio run                 # build
pio run -t upload       # build + flash over USB-C
pio device monitor      # serial console @ 115200
```

(or use the PlatformIO VS Code extension: **Build** / **Upload** / **Monitor**.)

The serial console prints WiFi/SD status, calibration load/save, and the
gyro-bias capture on boot.

### If the board won't resolve

If `pio` cannot find the `m5stack-cores3` board, switch to the community
**pioarduino** platform — replace the `platform = espressif32` line in
`platformio.ini` with:

```ini
platform = https://github.com/pioarduino/platform-espressif32/releases/download/53.03.13/platform-espressif32.zip
```

---

## 3. Connect your phone (QR → dashboard)

1. **Tap the top strip** of the device screen (where the IP is shown) — a
   **QR code** appears.
2. **Scan it with the phone camera.** It is a standard `WIFI:` join code: the
   phone asks "Join LevelLogger?" — one tap, no password typing.
3. The network has no internet, so the phone runs its captive-portal check and
   pops up the **"sign in to network" sheet — which IS the dashboard**. If your
   phone doesn't show it (vendor-dependent), open **`http://192.168.4.1/`** in
   the browser (the address is printed on the device screen and below the QR).
   Tell the phone to **stay connected** despite "no internet" — it keeps using
   mobile data for everything else.

Several phones can watch simultaneously. In station mode the QR encodes the
dashboard URL instead (the phone is already on the same hotspot), and
`http://level.local/` works too.

### What the dashboard shows

- **Bullseye level** with degree-graduated rings, tolerance circle and
  auto-ranging full scale (lockable to 0.5°/2°/10°/45°) — green bubble when
  level within tolerance.
- **Pitch & roll readouts** (3 decimals, tabular numerals) with
  SETTLED/MOVING state, **averaging progress** (n/500) and live **measurement
  quality**: std(|a|) in mg and de-biased gyro magnitude, each shown against
  its threshold (the tick at the bar's midpoint is the threshold).
- **Trend chart** (last 5/15 min, backfilled from the device's history ring on
  connect): pitch, roll, optional IMU-temperature trace, settled periods
  shaded, **event markers** flagged.
- **Statistics** over the visible window (settled samples only): mean, σ,
  peak-to-peak, min/max per axis, settled %, ΔT.
- **Controls:** MARK with free-text label + preset chips, START/STOP logging,
  **"Sync clock from phone"** (one tap writes the phone's time to the BM8563
  RTC — fixes `NO_TIME` permanently, no NTP needed), and the **calibration
  wizard** (below) — so you never have to touch the instrument and disturb it.
- A loud **CONNECTION LOST** banner when polling stalls — frozen numbers are
  never silently mistaken for live ones.

The page is a single embedded HTML file (no CDNs — the AP has no internet) and
follows the **NOVA design system** colour tokens (light + dark mode follow the
phone). Nova's corporate typeface (Hexagon Akkurat) is licensed and therefore
not embedded; a close system-font stack with tabular numerals is used.

### JSON API (for your own tooling)

| Endpoint | Method | Purpose |
|----------|--------|---------|
| `/api/status`   | GET  | full live state (~5 Hz poll-safe) |
| `/api/history`  | GET  | trend backfill: `[ms, pitch, roll, temp, flags]` (~15 min @ 1 Hz) |
| `/api/settings` | GET / POST | read / change runtime settings (form-encoded; `reset=1` = factory defaults) |
| `/api/mark`     | POST | write an event row (`label=...`) |
| `/api/log`      | POST | `action=start|stop` |
| `/api/cal`      | POST | `action=start|next|cancel` — drives the wizard |
| `/api/time`     | POST | `epoch=<unix-seconds>` → sets system time + RTC |

---

## 4. On-device controls (touchscreen)

The main screen shows the live bubble level plus three touch buttons along the
bottom:

| Touch | Action |
|-------|--------|
| **Top strip** | Show the **connect-your-phone QR** (tap anywhere to close). |
| **CAL**  | **Tap** = start the calibration wizard. **Hold ~1.5 s** = clear stored calibration (back to UNCALIBRATED). |
| **MARK** | Write an event row to the CSV immediately (even while moving). |
| **LOG**  | Start / stop logging (shows **STOP** while logging). |

During the calibration wizard the bottom buttons become **NEXT** (advance /
finish) and **CANCEL**. The wizard state is shared with the dashboard — you can
drive it from either side.

---

## 5. Granite-plate flip/reversal calibration (step by step)

You need a surface plate that is flat/level to better than the 0.1° target
(e.g. a granite plate good to ~0.011°). Calibration cancels both the BMI270's
zero-g offset **and** any residual plate tilt.

> Start the wizard by tapping **CAL** on the device — or **Start flip
> calibration** on the dashboard, which is the better way: pressing NEXT on
> the phone never bumps the device. The screen and the dashboard show the same
> prompt and progress.

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
  saved and a red `FAILED: …` is shown so you can retry. Tolerances are
  `CAL_FLAT_TOL_G` / `CAL_VERT_TOL_G`.
- **Capture quality gates:** the per-axis std-dev over the *whole* captured
  window must stay below `CAL_MAX_STD_G` — slow creep or vibration that sneaks
  past the instantaneous stationary gate **restarts the capture automatically**
  (shown on screen and dashboard), and hard-fails after `CAL_MAX_RECAPTURES`
  rather than silently saving a poor offset. The IMU temperature is recorded at
  both captures; if it moved more than `CAL_TEMP_WARN_C` between A and B, the
  result carries a **thermal-drift warning**.
- **Known worth:** the success message reports the offsets *and their estimated
  1σ uncertainty* from the capture noise, e.g.
  `Saved. offX −1.23 / offY +0.45 mg, est. uncertainty ±0.05 mg (±0.003 deg).`
- Offsets are **persisted in NVS** and reloaded on boot. With no stored
  calibration the device runs at zero offset and clearly shows **UNCALIBRATED**.
- **Re-run** any time. **Clear** stored calibration by **holding** CAL ~1.5 s on
  the device.
- The BMI270's zero-g offset **drifts with temperature**, so recalibrate if the
  operating temperature changes a lot. The IMU temperature at calibration time is
  stored, and the dashboard shows **ΔT since calibration** with a "consider
  recalibrating" hint past 5 °C. Every CSV row also logs `imu_temp_c`.
  Practical tip: the WiFi radio warms the die — let the device reach its normal
  operating temperature (a few minutes, radio on) *before* calibrating.

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

## 6. Where the CSV lands & how to read it

- A new file is created on the microSD at boot:
  - With a valid clock (RTC set / synced from phone / NTP): **`/level_log_YYYYMMDD_HHMMSS.csv`**
  - Without a clock: an incrementing index, **`/level_log_0001.csv`**, etc.
- The file is **flushed after every write** (`LOG_FLUSH_EVERY`), so data survives
  a power loss on the robot. A missing/failed/removed card is handled gracefully —
  the device keeps running, shows the SD state on screen and dashboard, re-probes
  every few seconds, and recovers automatically when a card is (re)inserted.

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
| `event_flag` | `1` if this row was produced by a **MARK** (device button or dashboard), else `0` |
| `event_label` | Event label (`device-button`, `web-mark`, or your custom dashboard label), CSV-escaped |

Row cadence:
- One **settled** row every `SETTLED_LOG_INTERVAL_MS` (default 1 s) while at rest.
- **Event** rows are written immediately when **MARK** is pressed (device or
  phone), even while moving, so the moment is captured.
- Moving rows are *off* by default; enable from the dashboard Settings panel
  (or `LOG_WHILE_MOVING` as the factory default).

---

## 7. Evaluate the data → HTML report (Python)

`tools/evaluate_level_log.py` turns the CSV logs into a self-contained HTML
report (NOVA-styled, embedded plots — print to PDF from the browser).

```bash
pip install numpy pandas matplotlib
python tools/evaluate_level_log.py level_log_20260611_103000.csv
```

A time-series window opens: **drag** horizontally (on the pitch or roll plot)
to select one or more ranges of measurement points — `u` undo, `r` reset,
`a` whole log, `ENTER` done. Selections are saved next to the log as
`<log>.selections.json`, so the identical report can be regenerated headlessly
with `--reuse`. `--all` skips the GUI and evaluates the whole log; multiple
CSVs can be given and are concatenated.

What the report contains:

- **Session metadata** — files, time span, dwell count, IMU temperature range,
  and the **calibration offsets recovered from the data itself**
  (`raw − corrected` per axis), flagging if the calibration state changed
  mid-log.
- **Time series** (pitch, roll, temperature) with settled periods shaded,
  selections highlighted, events flagged.
- **2-D tilt plot** — every settled sample in the selection, dwell means,
  selection means with 1σ/2σ confidence ellipses, and the ±0.1° target circle.
  Primary axes in degrees; secondary axes in **µm of displacement across the
  plate** (145 mm along pitch, 100 mm along roll — set at the top of the
  script).
- **Selection statistics** — mean, σ, peak-to-peak, min/max, drift (m°/min),
  each also converted to µm on the plate; between-dwell **repeatability** when
  a selection contains several dwells (re-place the plate several times to
  measure mounting repeatability).
- **Allan deviation** from the longest contiguous settled run — stability vs
  averaging time, with the noise floor annotated. Note the logged rows are
  already overlapping window averages (≈ 5 s default), so read the curve for
  τ above the firmware averaging window.

Scope note: the report quantifies **precision** (noise, drift, repeatability).
Judging the **absolute zero** requires a flip measurement (0°/180° pair)
against the granite plate — the same principle as the on-device calibration.

---

## 8. How it works (brief)

- **Sampling:** the accelerometer + gyro are read at `IMU_SAMPLE_RATE_HZ`
  (100 Hz) on a non-blocking `millis()` schedule.
- **Stationary detection:** requires a quiet gyro (de-biased magnitude below
  the gyro threshold) **and** a quiet accelerometer (low `std(|a|)` and `|a|`
  near 1 g), sustained for the dwell time. All thresholds are live-tunable from
  the dashboard and persisted in NVS.
- **Tilt:** while stationary, a moving average of up to `avg samples`
  *consecutive rest* samples is taken (the window resets the instant motion is
  seen, so it never mixes in moving data); the row is flagged `settled` once the
  device has been still for the dwell time, and the average keeps sharpening
  toward the full window as it stays still. The calibration offsets are
  subtracted and pitch/roll are computed with `atan2`. Axis swap is applied
  first, then the signs flip the final pitch/roll (Settings panel).
- **Gyro bias:** measured at boot with a **validated** capture (retried if the
  device was moving) and slowly re-tracked while confidently at rest to follow
  temperature drift.
- **Web stack:** the device answers all DNS queries on its soft-AP (captive
  portal) and serves the dashboard + JSON API with the ESP32 core's
  **synchronous** WebServer. Every HTTP handler runs inside `loop()` — there is
  deliberately no async server task, so firmware state needs no locking. The
  dashboard polls `/api/status` at ~5 Hz; a 1 Hz on-device history ring
  (~15 min) backfills the trend chart on connect.

### On-device bubble level

The main screen is a live **bullseye spirit level**: a bubble drifts toward the
raised side as you tilt the device, and the numeric **pitch and roll** for both
axes are shown beside it (green when settled/high-confidence, amber while moving).
The top strip shows the WiFi address and opens the connect-QR when tapped.

- **Auto-ranging scale** — the vial's full-scale (degrees at the outer ring)
  adjusts automatically with hysteresis: it zooms *in* for sub-degree work (max
  sensitivity) and zooms *out* so the bubble never leaves the vial. The current
  full-scale (e.g. `+/-2.0 deg`) is printed under the bubble. Steps and behaviour
  are set by `BUBBLE_SCALE_STEPS` / `BUBBLE_FILL_FRACTION` in `config.h`.
- The bubble turns **green** when both axes are within the level tolerance.
  Its smoothing is **adaptive** (display only — the measurement/CSV path is
  untouched): at rest it tracks the sharpening N-sample average with a slow EMA
  (`BUBBLE_ALPHA_REST`, rock steady), and the moment motion is detected it
  follows the live tilt with a fast EMA (`BUBBLE_ALPHA_MOVING`, no lag).
- **Axis orientation:** the CoreS3's BMI270 is mounted with its X axis along
  the screen's long edge, so the pitch/roll **swap is on by default** — "pitch"
  is raising the top/bottom edge (bubble moves vertically), "roll" the
  left/right edge. Bench check: the bubble must drift **toward the raised
  edge**; if it mirrors on either axis, flip `PITCH_SIGN`/`ROLL_SIGN` live in
  the dashboard Settings panel.

---

## Notes

- Uses M5Unified `M5.Imu` (BMI270) only — no raw chip driver, no BMM150. See
  **§5 "Accelerometer range"** for the ±2 g note.
- Flip/reversal calibration with validation (drivable from the phone), offsets
  + calibration temperature persisted in NVS, survive reboot, **UNCALIBRATED**
  clearly indicated when absent.
- Static tilt output to 2–3 decimals; only settled readings flagged
  high-confidence.
- CSV with raw + corrected data, flushed per write; tolerates a missing or
  removed card without crashing and remounts automatically.
- **Networking is point-to-point only:** the device's soft-AP serves the
  dashboard to phones nearby; nothing touches a company network. Station mode
  (phone hotspot) is an opt-in compile-time choice. There is no cloud, no
  external service, plain HTTP on a private link.
- The WiFi radio adds a little heat and the BMI270 offset drifts with
  temperature — calibrate at operating temperature; the dashboard's
  **ΔT since calibration** hint tracks this.
