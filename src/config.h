// =============================================================================
//  config.h  —  ALL tunable parameters for the M5Stack CoreS3 Static Level Logger
// -----------------------------------------------------------------------------
//  Everything you are likely to want to change lives in this one file:
//    * Time zone / RTC (the device is offline — no WiFi/NTP)
//    * IMU sampling, averaging and stationary-detection thresholds
//    * Axis / sign conventions (physical mounting is unknown -> easy to flip)
//    * Bubble-level display behaviour, display rate, logging cadence
//
//  Target board : M5Stack CoreS3  (ESP32-S3, BMI270 IMU, BM8563 RTC, AXP2101 PMU)
//  IMU          : Bosch BMI270 via M5Unified (M5.Imu). NOT MPU6886. BMM150 unused.
// =============================================================================
#pragma once

// -----------------------------------------------------------------------------
// 1. Time zone / RTC  (the device is OFFLINE — there is no WiFi/NTP)
// -----------------------------------------------------------------------------
//  Timestamps come from the battery-backed BM8563 RTC. They are real wall-clock
//  only if the RTC was set beforehand (e.g. M5Burner's "Set time", or a build
//  that once had NTP). If the RTC is unset, CSV rows log NO_TIME and files use an
//  incrementing index; the millis column is always valid either way.
//
// POSIX TZ string. Default: Asia/Singapore, UTC+8, no daylight saving.
//   Examples:  Singapore "<+08>-8"   UTC "UTC0"   US-Eastern "EST5EDT,M3.2.0,M11.1.0"
#define TZ_INFO                "<+08>-8"
// Plain offset (seconds) shown in the ISO timestamp (+08:00). Keep in sync with TZ_INFO.
#define TZ_OFFSET_SECONDS      (8 * 3600)
// Epoch past which the system clock is considered "really set" (2023-11-14).
#define TIME_VALID_EPOCH       1700000000

// -----------------------------------------------------------------------------
// 5. IMU sampling & averaging  (accuracy-critical)
// -----------------------------------------------------------------------------
//  ~0.1 deg at rest corresponds to ~1.7 milli-g of accel change near level, so
//  we sample fast and average heavily. The BMI270 is read through M5.Imu only.
#define IMU_SAMPLE_RATE_HZ        100
#define SAMPLE_INTERVAL_MS        (1000 / IMU_SAMPLE_RATE_HZ)   // 10 ms

//  Number of accelerometer samples averaged for one high-confidence reading.
//  500 samples @ 100 Hz = ~5 s of integration once the device is fully still.
#define AVG_SAMPLES               500

// -----------------------------------------------------------------------------
// 6. Accelerometer range note  (read me!)
// -----------------------------------------------------------------------------
//  The spec calls for the +/-2 g range (finest LSB / best tilt resolution near
//  level). M5Unified's high-level M5.Imu API (the abstraction we are required to
//  use) does NOT expose a runtime accel full-scale setter, and getAccel() always
//  returns values already scaled to g for whatever range the driver configured.
//
//  We deliberately do NOT poke the BMI270 ACC_RANGE register (0x41) directly,
//  because that would be a raw chip-driver write AND it would desync M5Unified's
//  internal LSB->g scale factor (giving 4x-wrong readings). See README.md
//  ("Accelerometer range") for how to force +/-2 g inside the M5Unified BMI270
//  driver if your build's default differs. The 0.1 deg target is reached through
//  heavy averaging (AVG_SAMPLES) + flip/reversal offset calibration regardless.

// -----------------------------------------------------------------------------
// 7. Stationary detection (gates which readings count as "settled")
// -----------------------------------------------------------------------------
//  The device only measures at rest. We require BOTH a quiet gyro AND a quiet
//  accelerometer, sustained for a dwell time, before a reading is "settled".
//  The gyro is used ONLY here for rest-detection — never for tilt.

// Gyro magnitude threshold (deg/s). Evaluated AFTER subtracting the measured
// gyro bias (see GYRO_BIAS_* below), so it is immune to the BMI270's zero-rate
// offset. Below this -> "gyro is quiet".
#define STATIONARY_GYRO_THRESH_DPS    0.5f

// Accelerometer noise gate: standard deviation of |a| over a short window.
// Below this -> "accel is quiet" (rejects vibration / handling).
#define STATIONARY_ACCEL_STD_G        0.004f
#define STATIONARY_WINDOW_SAMPLES     25       // 0.25 s window for the std above

// Sanity gate: how far the windowed |a| may stray from 1 g and still be "at
// rest" (rejects sustained linear acceleration). In g.
#define STATIONARY_ACCEL_MAG_TOL_G    0.08f

// How long both conditions must hold continuously before we declare "settled".
#define STATIONARY_DWELL_MS           1000UL
#define DWELL_SAMPLES   ((STATIONARY_DWELL_MS) / (SAMPLE_INTERVAL_MS))   // 100

// -----------------------------------------------------------------------------
// 8. Gyro bias (zero-rate offset) handling
// -----------------------------------------------------------------------------
//  We disable M5Unified's auto-calibration and remove the gyro bias ourselves so
//  behaviour is fully deterministic. Bias is measured at boot (keep still!) and
//  may be slowly re-tracked while the device is confidently at rest, to follow
//  temperature drift. Set GYRO_BIAS_ADAPT_ALPHA to 0 to disable adaptive tracking.
#define GYRO_BIAS_SAMPLES         200      // ~2 s of samples measured at startup
#define GYRO_BIAS_ADAPT_ALPHA     0.001f   // 0 = off; small = slow drift tracking
#define GYRO_BIAS_ADAPT_GATE_DPS  2.0f     // only adapt when raw |gyro-bias| < this

// -----------------------------------------------------------------------------
// 9. Tilt axis / sign convention  (physical mounting on the robot is UNKNOWN)
// -----------------------------------------------------------------------------
//  Convention (device flat, screen up, az ~ +1 g):
//      pitch = atan2( ax, sqrt(ay^2 + az^2) ) * 180/PI    (nose up/down about Y)
//      roll  = atan2( ay, sqrt(ax^2 + az^2) ) * 180/PI    (left/right about X)
//  Flip a sign here if the robot reports the opposite polarity; swap if the two
//  axes are exchanged for your mounting.
#define PITCH_SIGN     (+1.0f)
#define ROLL_SIGN      (+1.0f)
#define SWAP_PITCH_ROLL   false

// -----------------------------------------------------------------------------
// 10. Output / cadence
// -----------------------------------------------------------------------------
#define DISPLAY_INTERVAL_MS       66UL     // ~15 Hz screen refresh (smooth bubble)
#define ANGLE_DECIMALS            3        // decimal places for pitch/roll output

//  The IMU temperature and the battery gauge are read over I2C far less often
//  than the 100 Hz sample loop — cache them to keep the hot path light.
#define TEMP_READ_INTERVAL_MS     500UL    // BMI270 die-temp poll
#define BATTERY_READ_INTERVAL_MS  2000UL   // AXP2101 battery-gauge poll

// -----------------------------------------------------------------------------
// 10b. Bubble (bullseye) spirit-level display
// -----------------------------------------------------------------------------
//  The screen shows a live bubble that drifts toward the raised side, like a
//  real bullseye level. Its full-scale (degrees at the outer ring) AUTO-RANGES:
//  it zooms in for tiny tilts (max sensitivity) and zooms out so the bubble
//  never leaves the vial. The chosen full-scale is printed next to it.
#define BUBBLE_SMOOTH_ALPHA       0.18f    // EMA on the live tilt feeding the bubble
#define LEVEL_TOLERANCE_DEG       0.10f    // within this of level -> bubble turns green
//  Discrete auto-range steps (degrees at the rim). The smallest step that keeps
//  the bubble inside BUBBLE_FILL_FRACTION of the vial is chosen, with hysteresis.
#define BUBBLE_SCALE_STEPS        { 0.5f, 1.0f, 2.0f, 5.0f, 10.0f, 20.0f, 45.0f, 90.0f }
#define BUBBLE_FILL_FRACTION      0.90f    // zoom out when tilt exceeds this * scale
#define BUBBLE_SHRINK_FRACTION    0.55f    // zoom in  when tilt drops below this * lower step

// -----------------------------------------------------------------------------
// 11. CSV logging cadence
// -----------------------------------------------------------------------------
//  Default: log one row per second while settled, plus every event immediately.
#define SETTLED_LOG_INTERVAL_MS   1000UL
//  Optionally also log low-rate rows while MOVING (settled=0). Off by default.
#define LOG_WHILE_MOVING          false
#define MOVING_LOG_INTERVAL_MS    1000UL
//  Flush/sync the file every N writes (1 = after every row -> safest vs power loss).
#define LOG_FLUSH_EVERY           1

// -----------------------------------------------------------------------------
// 11b. Calibration capture validation (sanity-checks the operator's flip)
// -----------------------------------------------------------------------------
//  Before accepting a flip calibration we verify the operator actually did it:
//    * the device was roughly flat (|az| within CAL_FLAT_TOL_G of 1 g) in BOTH
//      captures, i.e. the Z axis was vertical, and
//    * az barely changed between A and B (the 180 deg turn was about the
//      VERTICAL axis, not a horizontal flip) — within CAL_VERT_TOL_G.
//  These are loose sanity gates (not the precision path); a failure is reported
//  and the offsets are NOT saved.
#define CAL_FLAT_TOL_G            0.20f    // |az|-1g allowed at capture (~11 deg)
#define CAL_VERT_TOL_G            0.05f    // |az_A - az_B| allowed across the flip
//  Touch-and-HOLD the on-screen CAL button this long to clear stored calibration.
#define CAL_CLEAR_HOLD_MS         1500UL

// -----------------------------------------------------------------------------
// 12. microSD (CoreS3 SPI bus — verified pins)
// -----------------------------------------------------------------------------
#define SD_SPI_SCK_PIN     36
#define SD_SPI_MISO_PIN    35
#define SD_SPI_MOSI_PIN    37
#define SD_SPI_CS_PIN      4
#define SD_SPI_FREQ_HZ     20000000        // 20 MHz; lower (e.g. 10 MHz) if flaky
#define LOG_FILE_PREFIX    "/level_log_"   // final name: /level_log_YYYYMMDD_HHMMSS.csv
#define SD_RETRY_INTERVAL_MS  5000UL       // re-probe for a card if absent/failed

// -----------------------------------------------------------------------------
// 13. NVS (Preferences) — where calibration offsets are persisted
// -----------------------------------------------------------------------------
#define NVS_NAMESPACE      "levelcal"

// -----------------------------------------------------------------------------
// 14. Serial console
// -----------------------------------------------------------------------------
#define SERIAL_BAUD        115200
