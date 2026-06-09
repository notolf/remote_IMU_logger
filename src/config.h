// =============================================================================
//  config.h  —  ALL tunable parameters for the M5Stack CoreS3 Static Level Logger
// -----------------------------------------------------------------------------
//  Everything you are likely to want to change lives in this one file:
//    * WiFi / soft-AP credentials and mode
//    * NTP / timezone
//    * IMU sampling, averaging and stationary-detection thresholds
//    * Axis / sign conventions (physical mounting is unknown -> easy to flip)
//    * Web push rate, display rate, logging cadence
//
//  Target board : M5Stack CoreS3  (ESP32-S3, BMI270 IMU, BM8563 RTC, AXP2101 PMU)
//  IMU          : Bosch BMI270 via M5Unified (M5.Imu). NOT MPU6886. BMM150 unused.
// =============================================================================
#pragma once

// -----------------------------------------------------------------------------
// 1. WiFi — STATION credentials (the AP you normally join)
// -----------------------------------------------------------------------------
//  Edit these two lines to match your network.
#define WIFI_SSID              "YOUR_WIFI_SSID"
#define WIFI_PASS              "YOUR_WIFI_PASSWORD"

// How long to wait for a station connection before giving up (and, in AUTO
// mode, falling back to the soft-AP).
#define WIFI_CONNECT_TIMEOUT_MS   15000UL

// -----------------------------------------------------------------------------
// 2. WiFi — SOFT-AP fallback (used when the robot roams out of router range)
// -----------------------------------------------------------------------------
//  Connect your PC to this network, then browse to http://192.168.4.1/
#define AP_SSID                "LevelLogger"
#define AP_PASS                "level1234"      // must be >= 8 chars (WPA2)

// -----------------------------------------------------------------------------
// 3. WiFi mode selection
// -----------------------------------------------------------------------------
//  WIFI_MODE_AUTO    : try STATION first, fall back to SOFT-AP on failure  (default)
//  WIFI_MODE_STA     : station only (no AP fallback)
//  WIFI_MODE_AP      : soft-AP only (never try to join a router)
#define WIFI_MODE_AUTO  0
#define WIFI_MODE_STA   1
#define WIFI_MODE_AP    2
#define WIFI_MODE_SELECT   WIFI_MODE_AUTO

// If a station link is up but later drops, how often to attempt a reconnect.
#define WIFI_RECONNECT_INTERVAL_MS   10000UL

// -----------------------------------------------------------------------------
// 4. NTP / timezone  (real wall-clock timestamps; falls back to millis if no NTP)
// -----------------------------------------------------------------------------
#define NTP_SERVER_1           "pool.ntp.org"
#define NTP_SERVER_2           "time.google.com"
// POSIX TZ string. Default: Asia/Singapore, UTC+8, no daylight saving.
//   Examples:  Singapore "<+08>-8"   UTC "UTC0"   US-Eastern "EST5EDT,M3.2.0,M11.1.0"
#define TZ_INFO                "<+08>-8"
// Plain offset (seconds) shown in the ISO timestamp (+08:00). Keep in sync with TZ_INFO.
#define TZ_OFFSET_SECONDS      (8 * 3600)

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
#define WEB_PUSH_INTERVAL_MS      150UL    // ~6.7 Hz live WebSocket push
#define DISPLAY_INTERVAL_MS       200UL    // 5 Hz screen refresh
#define ANGLE_DECIMALS            3        // decimal places for pitch/roll output

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
