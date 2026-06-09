// =============================================================================
//  M5Stack CoreS3 — Static Level Logger
// -----------------------------------------------------------------------------
//  Measures static tilt (pitch & roll) to ~0.1 deg while the robot is at REST,
//  shows it on-screen, mirrors it live to a PC browser over WiFi (WebSocket),
//  lets the operator mark events remotely, and logs every reading to a CSV file
//  on the microSD card.
//
//  Hardware  : M5Stack CoreS3 (ESP32-S3 / BMI270 IMU / BM8563 RTC / AXP2101 PMU)
//  IMU       : Bosch BMI270 via M5Unified's M5.Imu abstraction (NO raw driver,
//              NO BMM150 magnetometer). See config.h §6 for the accel-range note.
//  Framework : Arduino + PlatformIO (board m5stack-cores3)
//
//  All tunable parameters live in config.h. The web page lives in web_page.h.
//
//  KEY IDEAS
//   * The device only ever measures at rest, so there is NO sensor fusion /
//     complementary filter. The gyro is used ONLY to detect "is it moving".
//   * 0.1 deg at level ~= 1.7 milli-g, so a single raw sample cannot get there.
//     We therefore average AVG_SAMPLES accel samples and apply flip-calibrated
//     offsets before computing the angle.
// =============================================================================

#include <M5Unified.h>
#include <WiFi.h>
#include <SPI.h>
#include <FS.h>
#include <SD.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>   // ESP32Async fork (pulls in AsyncTCP)
#include <time.h>
#include <math.h>

#include "config.h"
#include "web_page.h"

// -----------------------------------------------------------------------------
// RGB565 colour fallbacks — M5GFX defines these TFT_* names, but guard anyway so
// the build never breaks on a palette-name change between library versions.
// -----------------------------------------------------------------------------
#ifndef TFT_BLACK
#define TFT_BLACK 0x0000
#endif
#ifndef TFT_WHITE
#define TFT_WHITE 0xFFFF
#endif
#ifndef TFT_RED
#define TFT_RED 0xF800
#endif
#ifndef TFT_GREEN
#define TFT_GREEN 0x07E0
#endif
#ifndef TFT_CYAN
#define TFT_CYAN 0x07FF
#endif
#ifndef TFT_YELLOW
#define TFT_YELLOW 0xFFE0
#endif
#ifndef TFT_ORANGE
#define TFT_ORANGE 0xFDA0
#endif
#ifndef TFT_NAVY
#define TFT_NAVY 0x000F
#endif
#ifndef TFT_DARKGREEN
#define TFT_DARKGREEN 0x03E0
#endif
#ifndef TFT_DARKGREY
#define TFT_DARKGREY 0x7BEF
#endif
#ifndef TFT_MAROON
#define TFT_MAROON 0x7800
#endif

// -----------------------------------------------------------------------------
// Small types
// -----------------------------------------------------------------------------
struct Rect { int x, y, w, h; };

enum CalStep {
  CAL_IDLE, CAL_WAIT_A, CAL_CAPTURE_A, CAL_WAIT_B, CAL_CAPTURE_B, CAL_DONE
};

// Commands handed from the async web task -> main loop (see handleWsText()).
struct PendingCmd {
  bool markEvent  = false;
  bool toggleLog  = false;
  bool calibrate  = false;
  bool calibNext  = false;
  bool calibCancel= false;
  bool clearCalib = false;
  char eventLabel[64] = {0};
};

// =============================================================================
// Globals
// =============================================================================
static bool  g_imuOk = false;

// --- N-sample averaging ring buffer (only ever holds *consecutive* rest samples)
static float  g_bufAx[AVG_SAMPLES], g_bufAy[AVG_SAMPLES], g_bufAz[AVG_SAMPLES];
static int    g_bufHead = 0, g_bufCount = 0;
static double g_sumAx = 0, g_sumAy = 0, g_sumAz = 0;

// --- short window of |a| used for the accel "quiet" test
static float  g_magWin[STATIONARY_WINDOW_SAMPLES];
static int    g_magHead = 0, g_magCount = 0;
static double g_magSum = 0, g_magSumSq = 0;

// --- gyro zero-rate offset we remove ourselves (BMI270 bias drifts w/ temp)
static float  g_gyroBias[3] = {0, 0, 0};

// --- live outputs (averaged = high-confidence; inst = latest single sample) ---
static float  g_pitch = 0, g_roll = 0;             // averaged tilt (settled)
static float  g_avgAx = 0, g_avgAy = 0, g_avgAz = 1; // offset-corrected averaged accel
static float  g_avgRawAx = 0, g_avgRawAy = 0, g_avgRawAz = 1; // averaged RAW accel
static float  g_instPitch = 0, g_instRoll = 0;     // instantaneous tilt
static float  g_instAx = 0, g_instAy = 0, g_instAz = 1;       // offset-corrected
static float  g_instRawAx = 0, g_instRawAy = 0, g_instRawAz = 1; // latest RAW accel
static float  g_gx = 0, g_gy = 0, g_gz = 0;        // latest RAW gyro (deg/s)
static float  g_dispPitch = 0, g_dispRoll = 0;     // smoothed live tilt for the bubble
static bool   g_settled = false;                   // at rest >= dwell time
static int    g_stationaryStreak = 0;              // consecutive rest samples
static float  g_imuTemp = NAN;

// --- cached slow-changing reads (kept off the 100 Hz hot path) ----------------
static int    g_battery = -1;
static bool   g_charging = false;

// --- calibration
static bool    g_calibrated = false;
static float   g_offX = 0, g_offY = 0, g_offZ = 0;
static CalStep g_calStep = CAL_IDLE;
static float   g_calA[3] = {0,0,0}, g_calB[3] = {0,0,0};
static bool    g_calError = false;                 // last calibration validation failed
static String  g_calResultMsg = "";               // shown on the CAL_DONE screen
static Preferences g_prefs;

// --- WiFi / time
static String g_wifiStatus = "init";
static String g_ip = "";
static bool   g_isAP = false;
static bool   g_ntpOk = false;

// --- SD / logging
static bool     g_sdOk = false;
static bool     g_logging = false;
static File     g_logFile;
static String   g_logName = "";
static String   g_sdStatus = "no card";
static uint32_t g_eventCount = 0;
static int      g_flushCounter = 0;

// --- web
static AsyncWebServer g_server(80);
static AsyncWebSocket g_ws("/ws");
static PendingCmd     g_cmd;
static portMUX_TYPE   g_cmdMux = portMUX_INITIALIZER_UNLOCKED;

// --- display
static M5Canvas g_canvas(&M5.Display);
static bool     g_useSprite = false;
static const int BTN_Y = 204, BTN_H = 34, BTN_GAP = 6;
static Rect g_btn[3];                 // CAL / MARK / LOG  (normal mode)
static Rect g_btnCalNext, g_btnCalCancel; // NEXT / CANCEL (calibration mode)

// =============================================================================
// Forward declarations
// =============================================================================
static void measureGyroBias();
static void loadCalibration();
static void saveCalibration();
static void clearCalibration();
static void imuSampleTick();
static void calibrationTick();
static void calibStart();
static void calibAdvance();
static void calibCancel();
static bool sdMount();
static String makeLogName();
static bool openLog();
static void writeRow(bool settled, int eventFlag, const String& label);
static void markEvent(const String& label);
static void toggleLogging();
static void loggingTick();
static void setupWiFi();
static bool startSTA();
static void startAP();
static void syncNtp();
static void wifiTick();
static void setupWeb();
static void onWsEvent(AsyncWebSocket*, AsyncWebSocketClient*, AwsEventType, void*, uint8_t*, size_t);
static void handleWsText(uint8_t* data, size_t len);
static size_t buildStatus(char* buf, size_t cap);
static void processCommands();
static void handleTouch();
static void updateDisplay();
static void computeButtonRects();
static bool timeIsValid();
static void isoTimestamp(char* buf, size_t n);
static String clockString();
static const char* calStepName();
static int calProgress();
static String calPrompt();
static String csvField(const String& s);

// small helpers
static inline bool  isNan(float f) { return f != f; }
static inline bool  hit(int x, int y, const Rect& r) {
  return x >= r.x && x < r.x + r.w && y >= r.y && y < r.y + r.h;
}

// =============================================================================
// Axis / sign convention  (device flat, screen up  ->  pitch = roll = 0)
//   pitch = atan2( ax, sqrt(ay^2 + az^2) )     roll = atan2( ay, sqrt(ax^2+az^2) )
//   The mounting orientation on the robot is unknown, so the polarity is easy to
//   flip with PITCH_SIGN / ROLL_SIGN, and the two axes can be exchanged with
//   SWAP_PITCH_ROLL (all in config.h). Note: because az enters only as az^2,
//   the formula already gives correct small-tilt angles whether the CoreS3 reads
//   az ~ +1 g or ~ -1 g when flat.
// =============================================================================
static void anglesFromCorrected(float ax, float ay, float az, float* p, float* r) {
  float pitch = atan2f(ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG * PITCH_SIGN;
  float roll  = atan2f(ay, sqrtf(ax * ax + az * az)) * RAD_TO_DEG * ROLL_SIGN;
  if (SWAP_PITCH_ROLL) { float t = pitch; pitch = roll; roll = t; }
  *p = pitch; *r = roll;
}

// =============================================================================
// Ring-buffer helpers (N-sample average) and |a| window (stationary test)
// =============================================================================
static void bufReset() { g_bufHead = 0; g_bufCount = 0; g_sumAx = g_sumAy = g_sumAz = 0; }
static void bufAdd(float ax, float ay, float az) {
  if (g_bufCount == AVG_SAMPLES) {                 // full -> subtract oldest
    g_sumAx -= g_bufAx[g_bufHead];
    g_sumAy -= g_bufAy[g_bufHead];
    g_sumAz -= g_bufAz[g_bufHead];
  } else {
    g_bufCount++;
  }
  g_bufAx[g_bufHead] = ax; g_bufAy[g_bufHead] = ay; g_bufAz[g_bufHead] = az;
  g_sumAx += ax; g_sumAy += ay; g_sumAz += az;
  g_bufHead = (g_bufHead + 1) % AVG_SAMPLES;
}

static void magReset() { g_magHead = 0; g_magCount = 0; g_magSum = 0; g_magSumSq = 0; }
static void magAdd(float m) {
  if (g_magCount == STATIONARY_WINDOW_SAMPLES) {
    float old = g_magWin[g_magHead];
    g_magSum   -= old;
    g_magSumSq -= (double)old * old;
  } else {
    g_magCount++;
  }
  g_magWin[g_magHead] = m;
  g_magSum   += m;
  g_magSumSq += (double)m * m;
  g_magHead = (g_magHead + 1) % STATIONARY_WINDOW_SAMPLES;
}

// =============================================================================
// IMU sampling (called at IMU_SAMPLE_RATE_HZ) + stationary detection + tilt
// =============================================================================
static void imuSampleTick() {
  // update() refreshes the BMI270 cache; getImuData() returns it (accel in g,
  // gyro in deg/s) — the pattern from the M5Unified Imu example.
  M5.Imu.update();
  auto d = M5.Imu.getImuData();
  float ax = d.accel.x, ay = d.accel.y, az = d.accel.z;
  float gx = d.gyro.x,  gy = d.gyro.y,  gz = d.gyro.z;
  g_gx = gx; g_gy = gy; g_gz = gz;                 // latest RAW gyro (for the CSV)

  // The BMI270 die temperature changes slowly: poll it at TEMP_READ_INTERVAL_MS
  // instead of every 100 Hz tick to keep the hot path light.
  static uint32_t lastTemp = 0; uint32_t nowMs = millis();
  if (nowMs - lastTemp >= TEMP_READ_INTERVAL_MS) {
    lastTemp = nowMs; float t; if (M5.Imu.getTemp(&t)) g_imuTemp = t;
  }

  // ---- de-biased gyro magnitude (gyro is used ONLY for rest detection) -------
  float dgx = gx - g_gyroBias[0], dgy = gy - g_gyroBias[1], dgz = gz - g_gyroBias[2];
  float gyroMag = sqrtf(dgx * dgx + dgy * dgy + dgz * dgz);
  bool gyroQuiet = (gyroMag < STATIONARY_GYRO_THRESH_DPS);

  // ---- accel "quiet" test: std(|a|) low AND |a| near 1 g ---------------------
  float amag = sqrtf(ax * ax + ay * ay + az * az);
  magAdd(amag);
  bool accelQuiet = false;
  if (g_magCount >= STATIONARY_WINDOW_SAMPLES) {
    float mean = (float)(g_magSum / g_magCount);
    float var  = (float)(g_magSumSq / g_magCount - (double)mean * mean);
    float sd   = var > 0 ? sqrtf(var) : 0;
    accelQuiet = (sd < STATIONARY_ACCEL_STD_G) &&
                 (fabsf(mean - 1.0f) < STATIONARY_ACCEL_MAG_TOL_G);
  }

  // ---- slow adaptive gyro-bias tracking while confidently at rest ------------
  // Follows BMI270 temperature drift. Gated so a real (fast) rotation is never
  // absorbed. Set GYRO_BIAS_ADAPT_ALPHA = 0 in config.h to disable.
  if (GYRO_BIAS_ADAPT_ALPHA > 0.0f && accelQuiet && gyroMag < GYRO_BIAS_ADAPT_GATE_DPS) {
    g_gyroBias[0] += GYRO_BIAS_ADAPT_ALPHA * (gx - g_gyroBias[0]);
    g_gyroBias[1] += GYRO_BIAS_ADAPT_ALPHA * (gy - g_gyroBias[1]);
    g_gyroBias[2] += GYRO_BIAS_ADAPT_ALPHA * (gz - g_gyroBias[2]);
  }

  // ---- instantaneous reading: RAW + offset-corrected (live view, moving rows) -
  g_instRawAx = ax; g_instRawAy = ay; g_instRawAz = az;
  g_instAx = ax - g_offX; g_instAy = ay - g_offY; g_instAz = az - g_offZ;
  anglesFromCorrected(g_instAx, g_instAy, g_instAz, &g_instPitch, &g_instRoll);

  // Smoothed live tilt that drives the on-screen bubble: responsive yet steady.
  g_dispPitch += BUBBLE_SMOOTH_ALPHA * (g_instPitch - g_dispPitch);
  g_dispRoll  += BUBBLE_SMOOTH_ALPHA * (g_instRoll  - g_dispRoll);

  // ---- rest gating + N-sample average ----------------------------------------
  bool instStationary = gyroQuiet && accelQuiet;
  if (instStationary) {
    if (g_stationaryStreak < 1000000) g_stationaryStreak++;
    bufAdd(ax, ay, az);                            // accumulate raw, offset later
    g_avgRawAx = (float)(g_sumAx / g_bufCount);    // averaged RAW accel (for CSV)
    g_avgRawAy = (float)(g_sumAy / g_bufCount);
    g_avgRawAz = (float)(g_sumAz / g_bufCount);
    g_avgAx = g_avgRawAx - g_offX;                 // offset-corrected -> tilt
    g_avgAy = g_avgRawAy - g_offY;
    g_avgAz = g_avgRawAz - g_offZ;
    anglesFromCorrected(g_avgAx, g_avgAy, g_avgAz, &g_pitch, &g_roll);
    g_settled = (g_stationaryStreak >= (int)DWELL_SAMPLES);
  } else {
    // motion: reset accumulators; hold last settled angle for the display
    g_stationaryStreak = 0;
    bufReset();
    g_settled = false;
  }
}

// =============================================================================
// Calibration — granite-plate flip/reversal method
// -----------------------------------------------------------------------------
//  Place flat on the plate (orientation A), capture averaged accel.
//  Rotate 180 deg about the VERTICAL axis (orientation B), capture again.
//      offset = (A + B) / 2     per horizontal axis.
//  Why this works: rotating 180 deg about vertical flips the SIGN of the real
//  horizontal gravity component (any residual plate tilt) while the sensor's
//  own zero-g offset stays put. Averaging A and B therefore cancels the plate
//  tilt and isolates the pure sensor offset, which we then subtract.
//      reading_A = +g*sin(tilt) + offset
//      reading_B = -g*sin(tilt) + offset   ->   (A+B)/2 = offset
//  The Z axis is NOT corrected by the flip (vertical rotation doesn't invert
//  it); a Z-up/Z-down scale calibration is left as an optional extension and
//  g_offZ stays 0 by default.
// =============================================================================
static void calibStart()   { g_calStep = CAL_WAIT_A; Serial.println("[CAL] started"); }
static void calibCancel()  { g_calStep = CAL_IDLE; bufReset(); magReset();
                             g_stationaryStreak = 0; Serial.println("[CAL] cancelled"); }

static void calibAdvance() {                          // "Next" / "Finish" button
  switch (g_calStep) {
    case CAL_WAIT_A:
      bufReset(); magReset(); g_stationaryStreak = 0; g_calStep = CAL_CAPTURE_A; break;
    case CAL_WAIT_B:
      bufReset(); magReset(); g_stationaryStreak = 0; g_calStep = CAL_CAPTURE_B; break;
    case CAL_DONE:
      g_calStep = CAL_IDLE; break;
    default: break;                                   // ignored while capturing
  }
}

static void calibrationTick() {
  // A capture/B capture complete once a full clean N-sample window is collected.
  if (g_calStep == CAL_CAPTURE_A && g_bufCount >= AVG_SAMPLES) {
    g_calA[0] = (float)(g_sumAx / g_bufCount);
    g_calA[1] = (float)(g_sumAy / g_bufCount);
    g_calA[2] = (float)(g_sumAz / g_bufCount);
    Serial.printf("[CAL] A = %.5f %.5f %.5f\n", g_calA[0], g_calA[1], g_calA[2]);
    g_calStep = CAL_WAIT_B;
  } else if (g_calStep == CAL_CAPTURE_B && g_bufCount >= AVG_SAMPLES) {
    g_calB[0] = (float)(g_sumAx / g_bufCount);
    g_calB[1] = (float)(g_sumAy / g_bufCount);
    g_calB[2] = (float)(g_sumAz / g_bufCount);

    // Validate the flip before trusting it: the device must have been roughly
    // flat in both captures (Z vertical) and the 180 deg turn must have been
    // about that vertical axis (az essentially unchanged). Otherwise the plate
    // tilt would not cancel and the offsets would be wrong.
    bool flatA = fabsf(fabsf(g_calA[2]) - 1.0f) < CAL_FLAT_TOL_G;
    bool flatB = fabsf(fabsf(g_calB[2]) - 1.0f) < CAL_FLAT_TOL_G;
    bool vert  = fabsf(g_calA[2] - g_calB[2])    < CAL_VERT_TOL_G;
    if (!flatA || !flatB) {
      g_calError = true;
      g_calResultMsg = "FAILED: device was not flat (Z must be vertical, az ~ 1 g).";
    } else if (!vert) {
      g_calError = true;
      g_calResultMsg = "FAILED: not a 180 deg turn about the VERTICAL axis.";
    } else {
      // flip/reversal: (A+B)/2 cancels residual plate tilt and isolates the
      // sensor offset. Z is left at 0 — a uniform scale cancels in the atan2.
      g_offX = (g_calA[0] + g_calB[0]) * 0.5f;
      g_offY = (g_calA[1] + g_calB[1]) * 0.5f;
      g_offZ = 0.0f;
      g_calibrated = true;
      g_calError = false;
      g_calResultMsg = "Calibration complete and saved to NVS.";
      saveCalibration();
    }
    Serial.printf("[CAL] B = %.5f %.5f %.5f -> %s (offX=%.5f offY=%.5f)\n",
                  g_calB[0], g_calB[1], g_calB[2],
                  g_calError ? "REJECTED" : "OK", g_offX, g_offY);
    g_calStep = CAL_DONE;
  }
}

static const char* calStepName() {
  switch (g_calStep) {
    case CAL_WAIT_A:   return "wait_a";
    case CAL_CAPTURE_A:return "capture_a";
    case CAL_WAIT_B:   return "wait_b";
    case CAL_CAPTURE_B:return "capture_b";
    case CAL_DONE:     return "done";
    default:           return "idle";
  }
}
static int calProgress() {
  if (g_calStep == CAL_CAPTURE_A || g_calStep == CAL_CAPTURE_B)
    return (int)(100L * g_bufCount / AVG_SAMPLES);
  if (g_calStep == CAL_DONE) return 100;
  return 0;
}
static String calPrompt() {
  switch (g_calStep) {
    case CAL_WAIT_A:    return "Step 1/2: Place the device FLAT on the granite plate "
                               "(orientation A). Hold still, then press Next.";
    case CAL_CAPTURE_A: return "Capturing orientation A - keep absolutely still...";
    case CAL_WAIT_B:    return "Step 2/2: Rotate 180 deg about the VERTICAL axis, same "
                               "spot. Hold still, then press Next.";
    case CAL_CAPTURE_B: return "Capturing orientation B - keep absolutely still...";
    case CAL_DONE:      return g_calResultMsg + " Press Finish to exit.";
    default:            return "";
  }
}

// =============================================================================
// Calibration persistence (NVS / Preferences)
// =============================================================================
static void loadCalibration() {
  g_prefs.begin(NVS_NAMESPACE, true);               // read-only
  g_calibrated = g_prefs.getBool("cal", false);
  g_offX = g_prefs.getFloat("offX", 0);
  g_offY = g_prefs.getFloat("offY", 0);
  g_offZ = g_prefs.getFloat("offZ", 0);
  g_prefs.end();
  Serial.printf("[NVS] calibration %s  offsets=(%.5f, %.5f, %.5f)\n",
                g_calibrated ? "LOADED" : "ABSENT -> UNCALIBRATED",
                g_offX, g_offY, g_offZ);
}
static void saveCalibration() {
  g_prefs.begin(NVS_NAMESPACE, false);
  g_prefs.putBool("cal", true);
  g_prefs.putFloat("offX", g_offX);
  g_prefs.putFloat("offY", g_offY);
  g_prefs.putFloat("offZ", g_offZ);
  g_prefs.end();
  Serial.println("[NVS] calibration saved");
}
static void clearCalibration() {
  g_prefs.begin(NVS_NAMESPACE, false);
  g_prefs.clear();
  g_prefs.end();
  g_calibrated = false; g_offX = g_offY = g_offZ = 0;
  Serial.println("[NVS] calibration cleared -> UNCALIBRATED");
}

// Measured once at boot (device must be still). Removes the BMI270 gyro bias so
// the absolute gyro-magnitude rest threshold is meaningful.
static void measureGyroBias() {
  if (!g_imuOk) return;
  if (g_useSprite) {
    g_canvas.fillSprite(TFT_BLACK);
    g_canvas.setTextColor(TFT_YELLOW); g_canvas.setTextSize(2);
    const char* a = "Measuring gyro bias"; const char* b = "KEEP STILL...";
    g_canvas.drawString(a, (g_canvas.width() - g_canvas.textWidth(a)) / 2, 95);
    g_canvas.drawString(b, (g_canvas.width() - g_canvas.textWidth(b)) / 2, 125);
    g_canvas.pushSprite(0, 0);
  }
  double s[3] = {0, 0, 0}; int n = 0;
  for (int i = 0; i < GYRO_BIAS_SAMPLES; i++) {
    M5.Imu.update();
    auto d = M5.Imu.getImuData();
    s[0] += d.gyro.x; s[1] += d.gyro.y; s[2] += d.gyro.z; n++;
    delay(SAMPLE_INTERVAL_MS);
  }
  if (n) { g_gyroBias[0] = s[0] / n; g_gyroBias[1] = s[1] / n; g_gyroBias[2] = s[2] / n; }
  Serial.printf("[IMU] gyro bias = %.4f %.4f %.4f dps (n=%d)\n",
                g_gyroBias[0], g_gyroBias[1], g_gyroBias[2], n);
}

// =============================================================================
// Time helpers
// =============================================================================
static bool timeIsValid() { return time(nullptr) > TIME_VALID_EPOCH; }

static void isoTimestamp(char* buf, size_t n) {
  if (timeIsValid()) {
    time_t now = time(nullptr);
    struct tm lt; localtime_r(&now, &lt);
    int off = TZ_OFFSET_SECONDS; char sign = off >= 0 ? '+' : '-'; off = abs(off);
    snprintf(buf, n, "%04d-%02d-%02dT%02d:%02d:%02d%c%02d:%02d",
             lt.tm_year + 1900, lt.tm_mon + 1, lt.tm_mday,
             lt.tm_hour, lt.tm_min, lt.tm_sec, sign, off / 3600, (off % 3600) / 60);
  } else {
    snprintf(buf, n, "NO_NTP");                     // millis column is still valid
  }
}
static String clockString() {
  if (timeIsValid()) {
    time_t now = time(nullptr); struct tm lt; localtime_r(&now, &lt);
    char b[24];
    snprintf(b, sizeof(b), "%04d-%02d-%02d %02d:%02d:%02d",
             lt.tm_year + 1900, lt.tm_mon + 1, lt.tm_mday, lt.tm_hour, lt.tm_min, lt.tm_sec);
    return String(b);
  }
  return String("no NTP (") + String(millis() / 1000) + "s up)";
}

// =============================================================================
// microSD + CSV logging
// =============================================================================
static bool sdMount() {
  SPI.begin(SD_SPI_SCK_PIN, SD_SPI_MISO_PIN, SD_SPI_MOSI_PIN, SD_SPI_CS_PIN);
  if (!SD.begin(SD_SPI_CS_PIN, SPI, SD_SPI_FREQ_HZ)) {
    Serial.println("[SD] no card / mount failed");
    return false;
  }
  return true;
}
static String makeLogName() {
  char name[48];
  if (timeIsValid()) {                              // wall-clock filename
    time_t now = time(nullptr); struct tm lt; localtime_r(&now, &lt);
    snprintf(name, sizeof(name), "%s%04d%02d%02d_%02d%02d%02d.csv", LOG_FILE_PREFIX,
             lt.tm_year + 1900, lt.tm_mon + 1, lt.tm_mday, lt.tm_hour, lt.tm_min, lt.tm_sec);
  } else {                                          // no clock -> incrementing index
    int idx = 1;
    do { snprintf(name, sizeof(name), "%s%04d.csv", LOG_FILE_PREFIX, idx++); }
    while (SD.exists(name) && idx < 10000);
  }
  return String(name);
}
static bool openLog() {
  if (!sdMount()) { g_sdOk = false; g_sdStatus = "no card"; return false; }
  g_sdOk = true;
  g_logName = makeLogName();
  g_logFile = SD.open(g_logName.c_str(), FILE_WRITE);
  if (!g_logFile) { g_sdStatus = "open error";
                    Serial.printf("[SD] cannot open %s\n", g_logName.c_str()); return false; }
  g_logFile.println("timestamp_iso,millis,pitch_deg,roll_deg,"
                    "ax_g,ay_g,az_g,ax_raw_g,ay_raw_g,az_raw_g,gx_dps,gy_dps,gz_dps,"
                    "settled,calibrated,imu_temp_c,battery_pct,event_flag,event_label");
  g_logFile.flush();
  g_sdStatus = "ready";
  Serial.printf("[SD] logging to %s\n", g_logName.c_str());
  return true;
}
static String csvField(const String& s) {           // quote+escape only if needed
  if (s.indexOf(',') < 0 && s.indexOf('"') < 0 && s.indexOf('\n') < 0) return s;
  String o = "\"";
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '"') o += "\"\"";                        // RFC-4180: double the quote
    else          o += c;
  }
  o += "\""; return o;
}
static void writeRow(bool settled, int eventFlag, const String& label) {
  if (!g_sdOk || !g_logFile) return;                // tolerate missing card
  char ts[40]; isoTimestamp(ts, sizeof(ts));
  // settled rows use the N-sample average; moving/event rows the latest sample.
  float pitch = settled ? g_pitch    : g_instPitch;
  float roll  = settled ? g_roll     : g_instRoll;
  float ax    = settled ? g_avgAx    : g_instAx;      // offset-corrected (-> angle)
  float ay    = settled ? g_avgAy    : g_instAy;
  float az    = settled ? g_avgAz    : g_instAz;
  float rax   = settled ? g_avgRawAx : g_instRawAx;   // raw accelerometer
  float ray   = settled ? g_avgRawAy : g_instRawAy;
  float raz   = settled ? g_avgRawAz : g_instRawAz;

  String line; line.reserve(256);
  line += ts;                       line += ',';
  line += String(millis());         line += ',';
  line += String(pitch, ANGLE_DECIMALS); line += ',';
  line += String(roll,  ANGLE_DECIMALS); line += ',';
  line += String(ax, 5);            line += ',';
  line += String(ay, 5);            line += ',';
  line += String(az, 5);            line += ',';
  line += String(rax, 5);           line += ',';     // raw accel x/y/z (no offset)
  line += String(ray, 5);           line += ',';
  line += String(raz, 5);           line += ',';
  line += String(g_gx, 4);          line += ',';     // raw gyro x/y/z (deg/s)
  line += String(g_gy, 4);          line += ',';
  line += String(g_gz, 4);          line += ',';
  line += (settled ? '1' : '0');    line += ',';
  line += (g_calibrated ? '1' : '0'); line += ',';
  line += (isNan(g_imuTemp) ? String("nan") : String(g_imuTemp, 2)); line += ',';
  line += String(g_battery);        line += ',';
  line += String(eventFlag);        line += ',';
  line += csvField(label);

  if (g_logFile.println(line) == 0) {               // write failed
    g_sdStatus = "write error"; g_sdOk = false;
    g_logFile.close();
    Serial.println("[SD] write failed -> will re-probe card");
    return;
  }
  if (++g_flushCounter >= LOG_FLUSH_EVERY) { g_logFile.flush(); g_flushCounter = 0; }
}

static void markEvent(const String& label) {
  g_eventCount++;
  // Capture the event at the moment it is triggered, even if not settled.
  writeRow(g_settled, 1, label);
  Serial.printf("[EVENT] #%u  '%s'  (settled=%d)\n", g_eventCount, label.c_str(), g_settled);
}
static void toggleLogging() {
  if (!g_sdOk) { Serial.println("[LOG] no SD card -> cannot toggle"); return; }
  g_logging = !g_logging;
  if (g_logFile) g_logFile.flush();
  Serial.printf("[LOG] %s\n", g_logging ? "STARTED" : "STOPPED");
}
static void loggingTick() {
  uint32_t now = millis();
  if (!g_sdOk) {                                     // periodically retry a card
    static uint32_t lastTry = 0;
    if (now - lastTry >= SD_RETRY_INTERVAL_MS) {     // auto-start logging on (re)mount
      lastTry = now;
      if (openLog()) g_logging = true;
    }
    return;
  }
  if (!g_logging) return;
  static uint32_t lastSettled = 0, lastMoving = 0;
  if (g_settled) {
    if (now - lastSettled >= SETTLED_LOG_INTERVAL_MS) { lastSettled = now; writeRow(true, 0, ""); }
  } else if (LOG_WHILE_MOVING) {
    if (now - lastMoving >= MOVING_LOG_INTERVAL_MS)   { lastMoving = now; writeRow(false, 0, ""); }
  }
}

// =============================================================================
// WiFi + NTP
// =============================================================================
static void startAP() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  g_isAP = true; g_ip = WiFi.softAPIP().toString();
  g_wifiStatus = "AP " + g_ip;
  Serial.printf("[WiFi] Soft-AP '%s' (pw '%s') up at %s\n", AP_SSID, AP_PASS, g_ip.c_str());
}
static bool startSTA() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("[WiFi] joining '%s' ...", WIFI_SSID);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < WIFI_CONNECT_TIMEOUT_MS) {
    delay(200); Serial.print('.');
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    g_isAP = false; g_ip = WiFi.localIP().toString();
    g_wifiStatus = "STA " + g_ip;
    Serial.printf("[WiFi] connected, IP %s\n", g_ip.c_str());
    return true;
  }
  Serial.println("[WiFi] station connect FAILED");
  return false;
}
static void syncNtp() {
  configTzTime(TZ_INFO, NTP_SERVER_1, NTP_SERVER_2);
  Serial.print("[NTP] syncing");
  struct tm tm0; uint32_t t0 = millis();
  while (!getLocalTime(&tm0, 500) && millis() - t0 < 8000) Serial.print('.');
  Serial.println();
  if (getLocalTime(&tm0, 100)) {
    g_ntpOk = true;
    Serial.printf("[NTP] OK: %04d-%02d-%02d %02d:%02d:%02d\n",
                  tm0.tm_year + 1900, tm0.tm_mon + 1, tm0.tm_mday,
                  tm0.tm_hour, tm0.tm_min, tm0.tm_sec);
    // Persist UTC to the BM8563 RTC so timestamps survive an AP-only reboot.
    time_t now = time(nullptr); struct tm utc; gmtime_r(&now, &utc);
    m5::rtc_datetime_t dt;
    dt.date.year = utc.tm_year + 1900; dt.date.month = utc.tm_mon + 1;
    dt.date.date = utc.tm_mday;        dt.date.weekDay = utc.tm_wday;
    dt.time.hours = utc.tm_hour; dt.time.minutes = utc.tm_min; dt.time.seconds = utc.tm_sec;
    M5.Rtc.setDateTime(dt);
  } else {
    Serial.println("[NTP] FAILED -> using RTC/millis timestamps");
  }
}
static void setupWiFi() {
#if WIFI_MODE_SELECT == WIFI_MODE_AP
  startAP();
#elif WIFI_MODE_SELECT == WIFI_MODE_STA
  if (startSTA()) syncNtp(); else g_wifiStatus = "STA failed";
#else // WIFI_MODE_AUTO: station first, soft-AP fallback
  if (startSTA()) syncNtp(); else startAP();
#endif
}
static void wifiTick() {
#if WIFI_MODE_SELECT != WIFI_MODE_AP
  if (g_isAP) return;
  static uint32_t lastChk = 0; uint32_t now = millis();
  if (now - lastChk < WIFI_RECONNECT_INTERVAL_MS) return;
  lastChk = now;
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi] link down -> reconnect");
    g_wifiStatus = "STA reconnecting";
    WiFi.reconnect();
  } else {
    g_ip = WiFi.localIP().toString(); g_wifiStatus = "STA " + g_ip;
    if (!g_ntpOk) syncNtp();                         // late NTP if it failed before
  }
#endif
}

// =============================================================================
// Web server + WebSocket
// =============================================================================
static void handleWsText(uint8_t* data, size_t len) {
  // Runs in the AsyncTCP task. Parse here, but only ever hand small flags to the
  // main loop under a short critical section — no SD / IMU access in this context.
  JsonDocument doc;
  if (deserializeJson(doc, (const char*)data, len)) return;
  const char* cmd   = doc["cmd"]   | "";
  const char* label = doc["label"] | "";
  portENTER_CRITICAL(&g_cmdMux);
  if      (!strcmp(cmd, "mark_event"))  { g_cmd.markEvent = true;
            strncpy(g_cmd.eventLabel, label, sizeof(g_cmd.eventLabel) - 1);
            g_cmd.eventLabel[sizeof(g_cmd.eventLabel) - 1] = 0; }
  else if (!strcmp(cmd, "toggle_log"))   g_cmd.toggleLog   = true;
  else if (!strcmp(cmd, "calibrate"))    g_cmd.calibrate   = true;
  else if (!strcmp(cmd, "calib_next"))   g_cmd.calibNext   = true;
  else if (!strcmp(cmd, "calib_cancel")) g_cmd.calibCancel = true;
  else if (!strcmp(cmd, "clear_calib"))  g_cmd.clearCalib  = true;
  portEXIT_CRITICAL(&g_cmdMux);
}
static void onWsEvent(AsyncWebSocket*, AsyncWebSocketClient* c, AwsEventType type,
                      void* arg, uint8_t* data, size_t len) {
  if (type == WS_EVT_DATA) {
    AwsFrameInfo* info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
      handleWsText(data, len);
  } else if (type == WS_EVT_CONNECT) {
    Serial.printf("[WS] client #%u connected\n", c->id());
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("[WS] client #%u disconnected\n", c->id());
  }
}
static void setupWeb() {
  g_ws.onEvent(onWsEvent);
  g_server.addHandler(&g_ws);
  g_server.on("/", HTTP_GET, [](AsyncWebServerRequest* r) {
    r->send_P(200, "text/html", INDEX_HTML);        // served from PROGMEM, no CDN
  });
  g_server.onNotFound([](AsyncWebServerRequest* r) { r->send(404, "text/plain", "not found"); });
  g_server.begin();
  Serial.println("[WEB] HTTP+WS server started on port 80");
}
static size_t buildStatus(char* buf, size_t cap) {
  JsonDocument doc;
  doc["type"]       = "status";
  doc["pitch"]      = g_pitch;
  doc["roll"]       = g_roll;
  doc["settled"]    = g_settled;
  doc["calibrated"] = g_calibrated;
  doc["ax"]         = g_settled ? g_avgAx : g_instAx;
  doc["ay"]         = g_settled ? g_avgAy : g_instAy;
  doc["az"]         = g_settled ? g_avgAz : g_instAz;
  if (!isNan(g_imuTemp)) doc["imu_temp"] = g_imuTemp; else doc["imu_temp"] = nullptr;
  doc["battery"]    = g_battery;
  doc["charging"]   = g_charging;
  doc["events"]     = g_eventCount;
  doc["wifi"]       = g_wifiStatus;
  doc["logging"]    = g_logging;
  doc["sd"]         = g_sdStatus;
  doc["file"]       = g_logName;
  doc["clock"]      = clockString();
  doc["calStep"]    = calStepName();
  doc["calPrompt"]  = calPrompt();
  doc["calProgress"]= calProgress();
  doc["calError"]   = g_calError;
  return serializeJson(doc, buf, cap);          // into a fixed buffer (no heap churn)
}

// =============================================================================
// Command dispatch (main-loop context) + touch buttons
// =============================================================================
static void processCommands() {
  bool mEvent, mToggle, mCal, mNext, mCancel, mClear;
  char label[64];
  portENTER_CRITICAL(&g_cmdMux);
  mEvent  = g_cmd.markEvent;  mToggle = g_cmd.toggleLog; mCal    = g_cmd.calibrate;
  mNext   = g_cmd.calibNext;  mCancel = g_cmd.calibCancel; mClear = g_cmd.clearCalib;
  memcpy(label, g_cmd.eventLabel, sizeof(label));
  g_cmd.markEvent = g_cmd.toggleLog = g_cmd.calibrate = false;
  g_cmd.calibNext = g_cmd.calibCancel = g_cmd.clearCalib = false;
  portEXIT_CRITICAL(&g_cmdMux);

  if (mEvent)  markEvent(String(label));
  if (mToggle) toggleLogging();
  if (mCal)    calibStart();
  if (mNext)   calibAdvance();
  if (mCancel) calibCancel();
  if (mClear)  clearCalibration();
}
static void handleTouch() {
  if (M5.Touch.getCount() == 0) return;
  auto t = M5.Touch.getDetail();
  if (!t.wasPressed()) return;                       // act on the touch-down edge
  int x = t.x, y = t.y;
  if (g_calStep != CAL_IDLE) {                       // calibration mode buttons
    if      (hit(x, y, g_btnCalNext))   calibAdvance();
    else if (hit(x, y, g_btnCalCancel)) calibCancel();
    return;
  }
  if      (hit(x, y, g_btn[0])) calibStart();        // CAL
  else if (hit(x, y, g_btn[1])) markEvent("device-button"); // MARK
  else if (hit(x, y, g_btn[2])) toggleLogging();     // LOG
}

// =============================================================================
// Display (M5GFX via a full-screen sprite to avoid flicker)
// =============================================================================
static void computeButtonRects() {
  int W = M5.Display.width();
  int bw = (W - 4 * BTN_GAP) / 3;
  g_btn[0] = { BTN_GAP,                 BTN_Y, bw, BTN_H };
  g_btn[1] = { BTN_GAP * 2 + bw,        BTN_Y, bw, BTN_H };
  g_btn[2] = { BTN_GAP * 3 + bw * 2,    BTN_Y, bw, BTN_H };
  int half = (W - 3 * BTN_GAP) / 2;
  g_btnCalNext   = { BTN_GAP,            BTN_Y, half, BTN_H };
  g_btnCalCancel = { BTN_GAP * 2 + half, BTN_Y, half, BTN_H };
}
static void drawBtn(const Rect& r, const char* label, uint16_t col) {
  g_canvas.fillRoundRect(r.x, r.y, r.w, r.h, 6, col);
  g_canvas.drawRoundRect(r.x, r.y, r.w, r.h, 6, TFT_WHITE);
  g_canvas.setTextColor(TFT_WHITE); g_canvas.setTextSize(2);
  int tw = g_canvas.textWidth(label), th = g_canvas.fontHeight();
  g_canvas.drawString(label, r.x + (r.w - tw) / 2, r.y + (r.h - th) / 2);
}
static void drawWrapped(const String& txt, int x, int y, int maxw, int lh) {
  int cy = y, start = 0; String line = "";
  while (start < (int)txt.length()) {
    int sp = txt.indexOf(' ', start);
    String word = (sp < 0) ? txt.substring(start) : txt.substring(start, sp);
    String test = line.length() ? line + " " + word : word;
    if (g_canvas.textWidth(test) > maxw && line.length()) {
      g_canvas.drawString(line, x, cy); cy += lh; line = word;
    } else line = test;
    if (sp < 0) break; start = sp + 1;
  }
  if (line.length()) g_canvas.drawString(line, x, cy);
}
static void drawCalScreen() {
  int W = g_canvas.width();
  g_canvas.setTextColor(TFT_CYAN); g_canvas.setTextSize(2);
  g_canvas.drawString("CALIBRATION", 8, 8);
  uint16_t promptCol = (g_calStep == CAL_DONE) ? (g_calError ? TFT_RED : TFT_GREEN)
                                               : TFT_WHITE;
  g_canvas.setTextColor(promptCol); g_canvas.setTextSize(1);
  drawWrapped(calPrompt(), 8, 40, W - 16, 12);
  int pct = calProgress(), by = 150, bw = W - 16, bh = 16;
  g_canvas.drawRect(8, by, bw, bh, TFT_WHITE);
  g_canvas.fillRect(10, by + 2, (bw - 4) * pct / 100, bh - 4, TFT_CYAN);
  char p[8]; snprintf(p, sizeof(p), "%d%%", pct);
  g_canvas.setTextColor(TFT_WHITE);
  g_canvas.drawString(p, (W - g_canvas.textWidth(p)) / 2, by + bh + 6);
  bool capturing = (g_calStep == CAL_CAPTURE_A || g_calStep == CAL_CAPTURE_B);
  drawBtn(g_btnCalNext, g_calStep == CAL_DONE ? "FINISH" : "NEXT",
          capturing ? TFT_DARKGREY : TFT_NAVY);
  drawBtn(g_btnCalCancel, "CANCEL", TFT_MAROON);
}

// Auto-ranging full-scale (degrees at the rim) for the bubble, with hysteresis
// so the chosen step is stable. Mutates a static index -> call once per frame.
static float bubbleFullScaleDeg() {
  static const float steps[] = BUBBLE_SCALE_STEPS;
  static const int   N = (int)(sizeof(steps) / sizeof(steps[0]));
  static int idx = 0;
  float mag = fmaxf(fabsf(g_dispPitch), fabsf(g_dispRoll));
  while (idx < N - 1 && mag > steps[idx] * BUBBLE_FILL_FRACTION) idx++;        // zoom out
  while (idx > 0      && mag < steps[idx - 1] * BUBBLE_SHRINK_FRACTION) idx--; // zoom in
  return steps[idx];
}

// Draw a bullseye spirit level: rings + crosshair + a "level" target ring, with
// the bubble drifting toward the raised side (smoothed live tilt). The vial
// full-scale auto-ranges, so tiny tilts are magnified and big tilts zoom out.
static void drawBubbleLevel(int cx, int cy, int R, float scaleDeg) {
  auto& cv = g_canvas;
  const uint16_t faint = 0x39C7;                     // dim grey gridlines
  cv.drawCircle(cx, cy, R,     TFT_DARKGREY);
  cv.drawCircle(cx, cy, R - 1, TFT_DARKGREY);
  cv.drawCircle(cx, cy, (R * 2) / 3, faint);
  cv.drawCircle(cx, cy, R / 3,       faint);
  cv.drawFastHLine(cx - R, cy, 2 * R, faint);
  cv.drawFastVLine(cx, cy - R, 2 * R, faint);

  // centre "level" tolerance ring (kept visible even at the finest scale)
  int tolR = (int)((LEVEL_TOLERANCE_DEG / scaleDeg) * R);
  if (tolR < 7) tolR = 7;
  cv.drawCircle(cx, cy, tolR, TFT_DARKGREEN);

  // bubble position from the smoothed live tilt, clamped inside the vial
  float nx = g_dispRoll  / scaleDeg;                 // roll  -> horizontal
  float ny = g_dispPitch / scaleDeg;                 // pitch -> vertical
  float rr = sqrtf(nx * nx + ny * ny);
  if (rr > 1.0f) { nx /= rr; ny /= rr; }
  const int br = 13;                                 // bubble radius
  int bx = cx + (int)(nx * (R - br));
  int by = cy - (int)(ny * (R - br));                // screen Y is down -> negate

  bool level = (fabsf(g_dispPitch) <= LEVEL_TOLERANCE_DEG &&
                fabsf(g_dispRoll)  <= LEVEL_TOLERANCE_DEG);
  cv.fillCircle(bx, by, br, level ? TFT_GREEN : TFT_CYAN);
  cv.drawCircle(bx, by, br, TFT_WHITE);
  cv.fillCircle(bx - 4, by - 4, 3, TFT_WHITE);       // glossy highlight

  // full-scale label just under the vial
  char sl[16]; snprintf(sl, sizeof(sl), "+/-%.1f deg", scaleDeg);
  cv.setTextSize(1); cv.setTextColor(TFT_WHITE);
  cv.drawString(sl, cx - cv.textWidth(sl) / 2, cy + R + 2);
}

static void updateDisplay() {
  if (!g_useSprite) {                                // minimal fallback (no PSRAM)
    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setTextColor(g_settled ? TFT_GREEN : TFT_ORANGE);
    M5.Display.setTextSize(3); M5.Display.setCursor(6, 20);
    M5.Display.printf("P %+.*f\n", ANGLE_DECIMALS, g_pitch);
    M5.Display.setCursor(6, 70);
    M5.Display.printf("R %+.*f\n", ANGLE_DECIMALS, g_roll);
    return;
  }
  auto& cv = g_canvas;
  cv.fillSprite(TFT_BLACK);
  int W = cv.width();

  if (g_calStep != CAL_IDLE) { drawCalScreen(); cv.pushSprite(0, 0); return; }

  // ---- top strip: title + battery -------------------------------------------
  cv.setTextSize(1); cv.setTextColor(TFT_CYAN);
  cv.drawString("BUBBLE LEVEL", 6, 4);
  char bs[20]; snprintf(bs, sizeof(bs), "BAT %d%%%s", g_battery, g_charging ? "+" : "");
  cv.setTextColor(g_battery >= 0 && g_battery < 20 ? TFT_RED : TFT_WHITE);
  cv.drawString(bs, W - 6 - cv.textWidth(bs), 4);

  // ---- left: the live auto-ranging bubble (bullseye) level ------------------
  float scaleDeg = bubbleFullScaleDeg();
  drawBubbleLevel(86, 106, 84, scaleDeg);

  // ---- right column: numeric pitch & roll for BOTH axes + status -----------
  const int RX = 178;                                // settled -> precise average,
  float showPitch = g_settled ? g_pitch : g_dispPitch; // else the live smoothed value
  float showRoll  = g_settled ? g_roll  : g_dispRoll;
  uint16_t valCol = g_settled ? TFT_GREEN : TFT_ORANGE;
  char v[16];
  cv.setTextSize(1); cv.setTextColor(TFT_DARKGREY); cv.drawString("PITCH", RX, 24);
  cv.setTextSize(2); cv.setTextColor(valCol);
  snprintf(v, sizeof(v), "%+.*f", ANGLE_DECIMALS, showPitch); cv.drawString(v, RX, 34);
  cv.setTextSize(1); cv.setTextColor(TFT_DARKGREY); cv.drawString("ROLL", RX, 58);
  cv.setTextSize(2); cv.setTextColor(valCol);
  snprintf(v, sizeof(v), "%+.*f", ANGLE_DECIMALS, showRoll); cv.drawString(v, RX, 68);

  cv.setTextSize(1);
  cv.setTextColor(valCol);
  cv.drawString(g_settled ? "SETTLED" : "MOVING", RX, 96);
  cv.setTextColor(g_calibrated ? TFT_GREEN : TFT_RED);
  cv.drawString(g_calibrated ? "CALIBRATED" : "UNCALIBRATED", RX, 110);
  cv.setTextColor(TFT_WHITE);
  cv.drawString(g_wifiStatus, RX, 128);
  char ln[40];
  snprintf(ln, sizeof(ln), "SD:%s%s", g_sdStatus.c_str(), g_logging ? " LOG" : "");
  cv.drawString(ln, RX, 140);
  if (!isNan(g_imuTemp)) snprintf(ln, sizeof(ln), "EV:%lu   %.1fC",
                                  (unsigned long)g_eventCount, g_imuTemp);
  else                   snprintf(ln, sizeof(ln), "EV:%lu", (unsigned long)g_eventCount);
  cv.drawString(ln, RX, 152);
  cv.drawString(clockString(), RX, 164);

  // ---- buttons --------------------------------------------------------------
  drawBtn(g_btn[0], "CAL",  TFT_NAVY);
  drawBtn(g_btn[1], "MARK", TFT_DARKGREEN);
  drawBtn(g_btn[2], g_logging ? "STOP" : "LOG", g_logging ? TFT_MAROON : TFT_DARKGREEN);

  cv.pushSprite(0, 0);
}

// =============================================================================
// setup() / loop()
// =============================================================================
void setup() {
  auto cfg = M5.config();
  cfg.internal_imu = true;          // BMI270 on the CoreS3
  cfg.clear_display = true;
  M5.begin(cfg);

  Serial.begin(SERIAL_BAUD);
  delay(200);
  Serial.println("\n=== M5Stack CoreS3 Static Level Logger ===");

  g_battery  = M5.Power.getBatteryLevel();           // seed the cached gauge
  g_charging = ((int)M5.Power.isCharging() == 1);

  M5.Display.setRotation(1);        // 320x240 landscape
  M5.Display.setBrightness(180);
  M5.Display.fillScreen(TFT_BLACK);
  g_canvas.setColorDepth(16);
  g_useSprite = (g_canvas.createSprite(M5.Display.width(), M5.Display.height()) != nullptr);
  if (!g_useSprite) Serial.println("[GFX] sprite alloc failed -> direct-draw fallback");
  computeButtonRects();

  // --- IMU ------------------------------------------------------------------
  M5.Imu.update();
  float tax, tay, taz;
  g_imuOk = M5.Imu.getAccel(&tax, &tay, &taz);       // BMI270 present?
  if (!g_imuOk) {
    Serial.println("[IMU] BMI270 NOT DETECTED!");
  } else {
    Serial.println("[IMU] BMI270 ready");
    // We manage offsets ourselves: clear M5Unified's internal offset and disable
    // its auto-calibration so accel/gyro behaviour is fully deterministic.
    M5.Imu.clearOffsetData();
    M5.Imu.setCalibration(0, 0, 0);
  }

  measureGyroBias();                // ~2 s, device must be still
  loadCalibration();                // NVS offsets (or UNCALIBRATED)

  // --- clock (seed from RTC before WiFi so filenames can be timestamped) ----
  setenv("TZ", TZ_INFO, 1); tzset();
  M5.Rtc.setSystemTimeFromRtc();    // no-op if the RTC was never set

  // --- WiFi (station or soft-AP) + NTP --------------------------------------
  setupWiFi();
  setupWeb();

  // --- microSD: open the log file; start logging if a card is present -------
  if (openLog()) g_logging = true;

  Serial.printf("[BOOT] done. Open  http://%s/  in a browser.\n", g_ip.c_str());
}

void loop() {
  M5.update();                                       // refresh touch / power
  uint32_t now = millis();

  // 1) IMU sampling at a fixed cadence (non-blocking schedule)
  static uint32_t lastSample = 0;
  if (now - lastSample >= SAMPLE_INTERVAL_MS) {
    lastSample = now;
    if (g_imuOk) imuSampleTick();
  }

  // 2) calibration capture completion
  calibrationTick();

  // 3) commands from web + on-screen touch buttons
  processCommands();
  handleTouch();

  // 4) CSV logging cadence (settled rows; events are immediate)
  loggingTick();

  // 5) live WebSocket push
  static uint32_t lastPush = 0;
  if (now - lastPush >= WEB_PUSH_INTERVAL_MS) {
    lastPush = now;
    if (g_ws.count() > 0) {
      static char buf[768];
      size_t n = buildStatus(buf, sizeof(buf));
      g_ws.textAll(buf, n);
    }
  }

  // 6) screen refresh
  static uint32_t lastDisp = 0;
  if (now - lastDisp >= DISPLAY_INTERVAL_MS) { lastDisp = now; updateDisplay(); }

  // 7) housekeeping: cached battery read, dead-client cleanup, WiFi upkeep
  static uint32_t lastBat = 0;
  if (now - lastBat >= BATTERY_READ_INTERVAL_MS) {
    lastBat = now;
    g_battery  = M5.Power.getBatteryLevel();
    g_charging = ((int)M5.Power.isCharging() == 1);
  }
  static uint32_t lastClean = 0;
  if (now - lastClean >= 1000) { lastClean = now; g_ws.cleanupClients(); }
  wifiTick();

  delay(1);                                          // yield to WiFi/idle tasks
}
