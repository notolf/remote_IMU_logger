// =============================================================================
//  M5Stack CoreS3 — Static Level Logger (touchscreen + phone web dashboard)
// -----------------------------------------------------------------------------
//  Measures static tilt (pitch & roll) to ~0.1 deg while the robot is at REST,
//  shows it live on-screen as a bubble level, lets the operator mark events with
//  the touch buttons, and logs every reading to a CSV file on the microSD card.
//
//  Remote monitoring: the device hosts its own WiFi soft-AP (or joins a phone
//  hotspot in station mode) and serves the embedded dashboard from web_page.h
//  over plain HTTP, with a captive portal so a phone that scans the on-screen
//  QR code lands straight on the live view. The dashboard polls /api/status;
//  commands (mark / log / calibration / clock / settings) are handled by the
//  SYNCHRONOUS WebServer, i.e. inside loop() context — deliberately no async
//  server and no cross-task handoff. The microSD CSV stays the data of record;
//  the dashboard is telemetry plus remote control.
//
//  Hardware  : M5Stack CoreS3 (ESP32-S3 / BMI270 IMU / BM8563 RTC / AXP2101 PMU)
//  IMU       : Bosch BMI270 via M5Unified's M5.Imu abstraction (NO raw driver,
//              NO BMM150 magnetometer). See config.h §6 for the accel-range note.
//  Framework : Arduino + PlatformIO (board m5stack-cores3)
//
//  All tunable parameters live in config.h.
//
//  KEY IDEAS
//   * The device only ever measures at rest, so there is NO sensor fusion /
//     complementary filter. The gyro is used ONLY to detect "is it moving".
//   * 0.1 deg at level ~= 1.7 milli-g, so a single raw sample cannot get there.
//     We therefore average AVG_SAMPLES accel samples and apply flip-calibrated
//     offsets before computing the angle.
// =============================================================================

#include <M5Unified.h>
#include <SPI.h>
#include <FS.h>
#include <SD.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <time.h>
#include <sys/time.h>
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

// =============================================================================
// Globals
// =============================================================================
static bool  g_imuOk = false;

// --- runtime settings: factory defaults come from config.h, the live values
//     are persisted in NVS and editable from the dashboard's Settings panel.
struct Settings {
  float    gyroThreshDps;   // stationary: gyro quiet threshold (deg/s)
  float    accelStdG;       // stationary: std(|a|) threshold (g)
  float    accelMagTolG;    // stationary: ||a| - 1 g| tolerance (g)
  uint32_t dwellMs;         // stationary: dwell before "settled" (ms)
  uint16_t avgSamples;      // averaging window (samples, <= MAX_AVG_SAMPLES)
  uint32_t settledLogMs;    // CSV cadence while settled (ms)
  bool     logWhileMoving;  // also log low-rate rows while moving
  uint32_t movingLogMs;     // CSV cadence while moving (ms)
  float    levelTolDeg;     // "level" tolerance for the bubble / dashboard (deg)
  float    pitchSign;       // +1 / -1
  float    rollSign;        // +1 / -1
  bool     swapPitchRoll;   // exchange the axes (applied BEFORE the signs)
};
static Settings g_set;

// --- N-sample averaging ring buffer (only ever holds *consecutive* rest samples)
//     Sized for the compile-time maximum; the live window is g_set.avgSamples.
//     Sum-of-squares is kept alongside the sums so the per-axis std-dev over
//     the window is O(1) — used to validate calibration capture quality.
static float  g_bufAx[MAX_AVG_SAMPLES], g_bufAy[MAX_AVG_SAMPLES], g_bufAz[MAX_AVG_SAMPLES];
static int    g_bufHead = 0, g_bufCount = 0;
static double g_sumAx = 0, g_sumAy = 0, g_sumAz = 0;
static double g_sumSqAx = 0, g_sumSqAy = 0, g_sumSqAz = 0;

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
static float  g_accelSd = -1;                      // live std(|a|) in g (-1 = warming up)
static float  g_gyroMag = 0;                       // live de-biased |gyro| (deg/s)

// --- cached slow-changing reads (kept off the 100 Hz hot path) ----------------
static int    g_battery = -1;
static bool   g_charging = false;

// --- calibration
static bool    g_calibrated = false;
static float   g_offX = 0, g_offY = 0, g_offZ = 0;
static float   g_calTempC = NAN;                   // IMU temp when cal was captured
static CalStep g_calStep = CAL_IDLE;
static float   g_calA[3] = {0,0,0}, g_calB[3] = {0,0,0};
static float   g_calStdA = 0, g_calStdB = 0;       // capture noise (worst X/Y axis std, g)
static float   g_calTempA = NAN, g_calTempB = NAN; // IMU temp at each capture
static int     g_calRetries = 0;                   // auto-recaptures of the current step
static bool    g_calError = false;                 // last calibration validation failed
static String  g_calResultMsg = "";               // shown on the CAL_DONE screen
static Preferences g_prefs;

// --- SD / logging
static bool     g_sdOk = false;
static bool     g_logging = false;
static File     g_logFile;
static String   g_logName = "";
static String   g_sdStatus = "no card";
static uint32_t g_eventCount = 0;
static uint32_t g_rowsWritten = 0;
static int      g_flushCounter = 0;

// --- trend history ring: 1 Hz pitch/roll/temp, backfills the dashboard chart
struct HistEntry { uint32_t ms; float pitch, roll, temp; uint8_t flags; }; // b0 settled, b1 event
static HistEntry g_hist[HISTORY_LENGTH];
static int  g_histHead = 0, g_histCount = 0;
static bool g_histEventLatch = false;              // event since the last entry

// --- recent event labels (for the dashboard's event list / chart markers)
struct EventRec { uint32_t ms; char label[32]; };
static EventRec g_events[EVENT_RING_LENGTH];
static int g_evHead = 0, g_evCount = 0;

// --- WiFi / web (sync server: handlers run in loop() context)
static WebServer g_http(WEB_SERVER_PORT);
static DNSServer g_dns;
static bool   g_apMode = false;                    // soft-AP (true) vs station
static String g_ip = "";
static bool   g_rtcSyncedFromNtp = false;          // STA mode: NTP -> RTC once
static bool   g_showQr = false;                    // QR overlay on the device LCD
static char   g_qrText[80] = "";

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
static void handleTouch();
static void updateDisplay();
static void computeButtonRects();
static bool timeIsValid();
static void isoTimestamp(char* buf, size_t n);
static String clockString();
static int calProgress();
static String calPrompt();
static String csvField(const String& s);
static void settingsDefaults();
static void settingsLoad();
static void settingsSave();
static void settingsSanitize();
static void historyTick();
static void setupWiFi();
static void setupWeb();
static void wifiTick();
static void drawQrScreen();
static const char* calStepName();

// small helpers
static inline bool  isNan(float f) { return f != f; }
static inline bool  hit(int x, int y, const Rect& r) {
  return x >= r.x && x < r.x + r.w && y >= r.y && y < r.y + r.h;
}
static inline int   dwellSamples() { return (int)(g_set.dwellMs / SAMPLE_INTERVAL_MS); }
static inline float clampF(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// =============================================================================
// Axis / sign convention  (device flat, screen up  ->  pitch = roll = 0)
//   pitch = atan2( ax, sqrt(ay^2 + az^2) )     roll = atan2( ay, sqrt(ax^2+az^2) )
//   The mounting orientation on the robot is unknown, so the axes can be
//   exchanged (swap, applied FIRST) and each final output's polarity flipped
//   (signs) — runtime-tunable from the dashboard, defaults in config.h. Note:
//   because az enters only as az^2, the formula already gives correct
//   small-tilt angles whether the CoreS3 reads az ~ +1 g or ~ -1 g when flat.
// =============================================================================
static void anglesFromCorrected(float ax, float ay, float az, float* p, float* r) {
  float pitch = atan2f(ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
  float roll  = atan2f(ay, sqrtf(ax * ax + az * az)) * RAD_TO_DEG;
  if (g_set.swapPitchRoll) { float t = pitch; pitch = roll; roll = t; }
  *p = pitch * g_set.pitchSign;
  *r = roll  * g_set.rollSign;
}

// =============================================================================
// Ring-buffer helpers (N-sample average) and |a| window (stationary test)
// =============================================================================
static void bufReset() {
  g_bufHead = 0; g_bufCount = 0;
  g_sumAx = g_sumAy = g_sumAz = 0;
  g_sumSqAx = g_sumSqAy = g_sumSqAz = 0;
}
static void bufAdd(float ax, float ay, float az) {
  // The live window is g_set.avgSamples; any runtime change to it goes through
  // bufReset() (enforced in the settings handler), so head/count stay coherent.
  if (g_bufCount >= g_set.avgSamples) {            // full -> subtract oldest
    float ox = g_bufAx[g_bufHead], oy = g_bufAy[g_bufHead], oz = g_bufAz[g_bufHead];
    g_sumAx -= ox;                g_sumAy -= oy;                g_sumAz -= oz;
    g_sumSqAx -= (double)ox * ox; g_sumSqAy -= (double)oy * oy; g_sumSqAz -= (double)oz * oz;
  } else {
    g_bufCount++;
  }
  g_bufAx[g_bufHead] = ax; g_bufAy[g_bufHead] = ay; g_bufAz[g_bufHead] = az;
  g_sumAx += ax;                g_sumAy += ay;                g_sumAz += az;
  g_sumSqAx += (double)ax * ax; g_sumSqAy += (double)ay * ay; g_sumSqAz += (double)az * az;
  g_bufHead = (g_bufHead + 1) % g_set.avgSamples;
}
// Per-axis standard deviation over the current buffer window, from the running
// sums. Doubles keep the catastrophic cancellation harmless at |a| ~ 1 g.
static float bufAxisStd(double sum, double sumSq) {
  if (g_bufCount < 2) return 0;
  double mean = sum / g_bufCount;
  double var  = sumSq / g_bufCount - mean * mean;
  return var > 0 ? sqrtf((float)var) : 0;
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
  g_gyroMag = gyroMag;                               // exposed on the dashboard
  bool gyroQuiet = (gyroMag < g_set.gyroThreshDps);

  // ---- accel "quiet" test: std(|a|) low AND |a| near 1 g ---------------------
  float amag = sqrtf(ax * ax + ay * ay + az * az);
  magAdd(amag);
  bool accelQuiet = false;
  if (g_magCount >= STATIONARY_WINDOW_SAMPLES) {
    float mean = (float)(g_magSum / g_magCount);
    float var  = (float)(g_magSumSq / g_magCount - (double)mean * mean);
    float sd   = var > 0 ? sqrtf(var) : 0;
    g_accelSd  = sd;                                 // exposed on the dashboard
    accelQuiet = (sd < g_set.accelStdG) &&
                 (fabsf(mean - 1.0f) < g_set.accelMagTolG);
  } else {
    g_accelSd = -1;                                  // window still warming up
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
    g_settled = (g_stationaryStreak >= dwellSamples());
  } else {
    // motion: reset accumulators; hold last settled angle for the display
    g_stationaryStreak = 0;
    bufReset();
    g_settled = false;
  }

  // ---- adaptive display smoothing (DISPLAY ONLY — never touches the CSV) -----
  // At rest the bubble tracks the sharpening buffer average with a slow EMA, so
  // it sits rock-steady instead of twitching with single-sample noise; the
  // moment motion is detected it follows the instantaneous tilt with a fast
  // EMA so it never feels laggy.
  float tgtP, tgtR, alpha;
  if (instStationary) { tgtP = g_pitch;     tgtR = g_roll;     alpha = BUBBLE_ALPHA_REST; }
  else                { tgtP = g_instPitch; tgtR = g_instRoll; alpha = BUBBLE_ALPHA_MOVING; }
  g_dispPitch += alpha * (tgtP - g_dispPitch);
  g_dispRoll  += alpha * (tgtR - g_dispRoll);
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
static void calibStart()   { g_calStep = CAL_WAIT_A; g_calRetries = 0;
                             Serial.println("[CAL] started"); }
static void calibCancel()  { g_calStep = CAL_IDLE; bufReset(); magReset();
                             g_stationaryStreak = 0; Serial.println("[CAL] cancelled"); }

static void calibAdvance() {                          // "Next" / "Finish" button
  switch (g_calStep) {
    case CAL_WAIT_A:
      bufReset(); magReset(); g_stationaryStreak = 0; g_calRetries = 0;
      g_calStep = CAL_CAPTURE_A; break;
    case CAL_WAIT_B:
      bufReset(); magReset(); g_stationaryStreak = 0; g_calRetries = 0;
      g_calStep = CAL_CAPTURE_B; break;
    case CAL_DONE:
      g_calStep = CAL_IDLE; break;
    default: break;                                   // ignored while capturing
  }
}

// Capture-quality gate: the per-axis std over the FULL window must be quiet.
// The instantaneous stationary gate only sees a 0.25 s magnitude window, so
// slow creep (cooling plate, settling foam feet) can sneak past it — this
// catches that, restarts the capture automatically, and hard-fails after the
// retry cap instead of silently saving a poor offset.
// Returns true when the capture was accepted.
static bool calCaptureAccept(const char* which, float* stdOut, float* tempOut) {
  float sx = bufAxisStd(g_sumAx, g_sumSqAx);
  float sy = bufAxisStd(g_sumAy, g_sumSqAy);
  float stdMax = fmaxf(sx, sy);                     // offsets are X/Y — gate those axes
  if (stdMax > CAL_MAX_STD_G) {
    if (++g_calRetries <= CAL_MAX_RECAPTURES) {
      Serial.printf("[CAL] capture %s noisy (std %.4f g) -> recapture %d/%d\n",
                    which, stdMax, g_calRetries, CAL_MAX_RECAPTURES);
      bufReset(); g_stationaryStreak = 0;           // stay in the same capture step
      return false;
    }
    g_calError = true;
    g_calResultMsg = "FAILED: too much vibration/drift during capture. "
                     "Check the surface and retry.";
    g_calStep = CAL_DONE;
    return false;
  }
  *stdOut = stdMax;
  *tempOut = g_imuTemp;
  g_calRetries = 0;
  return true;
}

static void calibrationTick() {
  // A capture/B capture complete once a full clean N-sample window is collected.
  if (g_calStep == CAL_CAPTURE_A && g_bufCount >= g_set.avgSamples) {
    if (!calCaptureAccept("A", &g_calStdA, &g_calTempA)) return;
    g_calA[0] = (float)(g_sumAx / g_bufCount);
    g_calA[1] = (float)(g_sumAy / g_bufCount);
    g_calA[2] = (float)(g_sumAz / g_bufCount);
    Serial.printf("[CAL] A = %.5f %.5f %.5f (std %.4f g, %.1f C)\n",
                  g_calA[0], g_calA[1], g_calA[2], g_calStdA, g_calTempA);
    g_calStep = CAL_WAIT_B;
  } else if (g_calStep == CAL_CAPTURE_B && g_bufCount >= g_set.avgSamples) {
    if (!calCaptureAccept("B", &g_calStdB, &g_calTempB)) return;
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
      // sensor offset. Z is left at 0 (it cannot be separated by a rotation
      // about the vertical axis; near level it only contributes a tiny scale
      // error, negligible at the 0.1 deg target).
      g_offX = (g_calA[0] + g_calB[0]) * 0.5f;
      g_offY = (g_calA[1] + g_calB[1]) * 0.5f;
      g_offZ = 0.0f;
      g_calibrated = true;
      g_calError = false;
      saveCalibration();

      // Report what the calibration is worth: offset = (meanA+meanB)/2, so its
      // 1-sigma uncertainty from capture noise is sqrt(stdA^2+stdB^2)/(2*sqrtN).
      // Near level, 1 mg of offset is ~0.0573 deg of angle.
      float sigMg  = 1000.0f * sqrtf(g_calStdA * g_calStdA + g_calStdB * g_calStdB)
                     / (2.0f * sqrtf((float)g_set.avgSamples));
      float sigDeg = sigMg * 0.0573f;
      char m[180];
      snprintf(m, sizeof(m),
               "Saved. offX %+0.2f / offY %+0.2f mg, est. uncertainty "
               "+/-%.2f mg (+/-%.3f deg).",
               g_offX * 1000.0f, g_offY * 1000.0f, sigMg, sigDeg);
      g_calResultMsg = m;
      if (!isNan(g_calTempA) && !isNan(g_calTempB) &&
          fabsf(g_calTempB - g_calTempA) > CAL_TEMP_WARN_C) {
        snprintf(m, sizeof(m),
                 " NOTE: IMU temp moved %.1f C between captures - thermal "
                 "drift may bias the offsets; redo after warm-up if in doubt.",
                 g_calTempB - g_calTempA);
        g_calResultMsg += m;
      }
    }
    Serial.printf("[CAL] B = %.5f %.5f %.5f (std %.4f g, %.1f C) -> %s "
                  "(offX=%.5f offY=%.5f)\n",
                  g_calB[0], g_calB[1], g_calB[2], g_calStdB, g_calTempB,
                  g_calError ? "REJECTED" : "OK", g_offX, g_offY);
    g_calStep = CAL_DONE;
  }
}

static int calProgress() {
  if (g_calStep == CAL_CAPTURE_A || g_calStep == CAL_CAPTURE_B)
    return (int)(100L * g_bufCount / g_set.avgSamples);
  if (g_calStep == CAL_DONE) return 100;
  return 0;
}
static const char* calStepName() {                   // for the JSON API
  switch (g_calStep) {
    case CAL_WAIT_A:    return "wait_a";
    case CAL_CAPTURE_A: return "cap_a";
    case CAL_WAIT_B:    return "wait_b";
    case CAL_CAPTURE_B: return "cap_b";
    case CAL_DONE:      return "done";
    default:            return "idle";
  }
}
static String calPrompt() {
  String retry = g_calRetries
    ? String(" (restarted ") + g_calRetries + "x: vibration/drift)" : String("");
  switch (g_calStep) {
    case CAL_WAIT_A:    return "Step 1/2: Place the device FLAT on the granite plate "
                               "(orientation A). Hold still, then press Next.";
    case CAL_CAPTURE_A: return "Capturing orientation A - keep absolutely still..." + retry;
    case CAL_WAIT_B:    return "Step 2/2: Rotate 180 deg about the VERTICAL axis, same "
                               "spot. Hold still, then press Next.";
    case CAL_CAPTURE_B: return "Capturing orientation B - keep absolutely still..." + retry;
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
  g_calTempC = g_prefs.getFloat("calT", NAN);       // for the drift hint
  g_prefs.end();
  Serial.printf("[NVS] calibration %s  offsets=(%.5f, %.5f, %.5f)\n",
                g_calibrated ? "LOADED" : "ABSENT -> UNCALIBRATED",
                g_offX, g_offY, g_offZ);
}
static void saveCalibration() {
  g_calTempC = g_imuTemp;                           // BMI270 offset drifts with temp:
  g_prefs.begin(NVS_NAMESPACE, false);              // remember when/where we calibrated
  g_prefs.putBool("cal", true);
  g_prefs.putFloat("offX", g_offX);
  g_prefs.putFloat("offY", g_offY);
  g_prefs.putFloat("offZ", g_offZ);
  g_prefs.putFloat("calT", g_calTempC);
  g_prefs.end();
  Serial.println("[NVS] calibration saved");
}
static void clearCalibration() {
  g_prefs.begin(NVS_NAMESPACE, false);
  g_prefs.clear();
  g_prefs.end();
  g_calibrated = false; g_offX = g_offY = g_offZ = 0; g_calTempC = NAN;
  Serial.println("[NVS] calibration cleared -> UNCALIBRATED");
}

// =============================================================================
// Runtime settings persistence (NVS) — defaults in config.h, edited from the
// dashboard's Settings panel. Sanitized on every load/apply so a bad value can
// never wedge the stationary detector.
// =============================================================================
static void settingsDefaults() {
  g_set.gyroThreshDps  = STATIONARY_GYRO_THRESH_DPS;
  g_set.accelStdG      = STATIONARY_ACCEL_STD_G;
  g_set.accelMagTolG   = STATIONARY_ACCEL_MAG_TOL_G;
  g_set.dwellMs        = STATIONARY_DWELL_MS;
  g_set.avgSamples     = AVG_SAMPLES;
  g_set.settledLogMs   = SETTLED_LOG_INTERVAL_MS;
  g_set.logWhileMoving = LOG_WHILE_MOVING;
  g_set.movingLogMs    = MOVING_LOG_INTERVAL_MS;
  g_set.levelTolDeg    = LEVEL_TOLERANCE_DEG;
  g_set.pitchSign      = PITCH_SIGN;
  g_set.rollSign       = ROLL_SIGN;
  g_set.swapPitchRoll  = SWAP_PITCH_ROLL;
}
static void settingsSanitize() {
  g_set.gyroThreshDps = clampF(g_set.gyroThreshDps, 0.05f, 10.0f);
  g_set.accelStdG     = clampF(g_set.accelStdG,   0.0005f, 0.05f);
  g_set.accelMagTolG  = clampF(g_set.accelMagTolG,  0.01f, 0.5f);
  if (g_set.dwellMs < 200)            g_set.dwellMs = 200;
  if (g_set.dwellMs > 10000)          g_set.dwellMs = 10000;
  if (g_set.avgSamples < 50)          g_set.avgSamples = 50;
  if (g_set.avgSamples > MAX_AVG_SAMPLES) g_set.avgSamples = MAX_AVG_SAMPLES;
  if (g_set.settledLogMs < 100)       g_set.settledLogMs = 100;
  if (g_set.settledLogMs > 60000)     g_set.settledLogMs = 60000;
  if (g_set.movingLogMs < 100)        g_set.movingLogMs = 100;
  if (g_set.movingLogMs > 60000)      g_set.movingLogMs = 60000;
  g_set.levelTolDeg = clampF(g_set.levelTolDeg, 0.02f, 5.0f);
  g_set.pitchSign = g_set.pitchSign < 0 ? -1.0f : +1.0f;
  g_set.rollSign  = g_set.rollSign  < 0 ? -1.0f : +1.0f;
}
static void settingsLoad() {
  settingsDefaults();
  g_prefs.begin(NVS_CFG_NAMESPACE, true);
  if (g_prefs.getUChar("v", 0) == 1) {              // versioned for future migration
    g_set.gyroThreshDps  = g_prefs.getFloat("gyTh",  g_set.gyroThreshDps);
    g_set.accelStdG      = g_prefs.getFloat("sgTh",  g_set.accelStdG);
    g_set.accelMagTolG   = g_prefs.getFloat("mgTl",  g_set.accelMagTolG);
    g_set.dwellMs        = g_prefs.getULong("dwMs",  g_set.dwellMs);
    g_set.avgSamples     = g_prefs.getUShort("avgN", g_set.avgSamples);
    g_set.settledLogMs   = g_prefs.getULong("lgMs",  g_set.settledLogMs);
    g_set.logWhileMoving = g_prefs.getBool("lgMv",   g_set.logWhileMoving);
    g_set.movingLogMs    = g_prefs.getULong("mvMs",  g_set.movingLogMs);
    g_set.levelTolDeg    = g_prefs.getFloat("tol",   g_set.levelTolDeg);
    g_set.pitchSign      = g_prefs.getFloat("pSgn",  g_set.pitchSign);
    g_set.rollSign       = g_prefs.getFloat("rSgn",  g_set.rollSign);
    g_set.swapPitchRoll  = g_prefs.getBool("swap",   g_set.swapPitchRoll);
    Serial.println("[CFG] runtime settings loaded from NVS");
  } else {
    Serial.println("[CFG] no stored settings -> factory defaults");
  }
  g_prefs.end();
  settingsSanitize();
  Serial.printf("[CFG] gyro<%.2f dps, sd<%.4f g, dwell=%lu ms, avg=%u, tol=%.2f deg\n",
                g_set.gyroThreshDps, g_set.accelStdG,
                (unsigned long)g_set.dwellMs, g_set.avgSamples, g_set.levelTolDeg);
}
static void settingsSave() {
  g_prefs.begin(NVS_CFG_NAMESPACE, false);
  g_prefs.putUChar("v", 1);
  g_prefs.putFloat("gyTh",  g_set.gyroThreshDps);
  g_prefs.putFloat("sgTh",  g_set.accelStdG);
  g_prefs.putFloat("mgTl",  g_set.accelMagTolG);
  g_prefs.putULong("dwMs",  g_set.dwellMs);
  g_prefs.putUShort("avgN", g_set.avgSamples);
  g_prefs.putULong("lgMs",  g_set.settledLogMs);
  g_prefs.putBool("lgMv",   g_set.logWhileMoving);
  g_prefs.putULong("mvMs",  g_set.movingLogMs);
  g_prefs.putFloat("tol",   g_set.levelTolDeg);
  g_prefs.putFloat("pSgn",  g_set.pitchSign);
  g_prefs.putFloat("rSgn",  g_set.rollSign);
  g_prefs.putBool("swap",   g_set.swapPitchRoll);
  g_prefs.end();
  Serial.println("[CFG] runtime settings saved");
}

// Centered two-line boot/status banner (used during the blocking startup steps
// so the screen never looks frozen before the live bubble view takes over).
static void bootBanner(const char* l1, const char* l2, uint16_t color) {
  if (g_useSprite) {
    g_canvas.fillSprite(TFT_BLACK);
    g_canvas.setTextColor(color); g_canvas.setTextSize(2);
    g_canvas.drawString(l1, (g_canvas.width() - g_canvas.textWidth(l1)) / 2, 95);
    if (l2 && *l2)
      g_canvas.drawString(l2, (g_canvas.width() - g_canvas.textWidth(l2)) / 2, 125);
    g_canvas.pushSprite(0, 0);
  } else {
    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setTextColor(color); M5.Display.setTextSize(2);
    M5.Display.setCursor(16, 95);  M5.Display.print(l1);
    if (l2 && *l2) { M5.Display.setCursor(16, 125); M5.Display.print(l2); }
  }
}

// Measured at boot (device must be still). Removes the BMI270 gyro bias so the
// absolute gyro-magnitude rest threshold is meaningful. The capture is
// VALIDATED: if the per-axis std-dev says the device was moving (boot in hand,
// robot vibrating), it retries instead of poisoning rest detection with a bad
// bias that the slow adaptive tracker might never recover from.
static void measureGyroBias() {
  if (!g_imuOk) return;
  for (int attempt = 1; attempt <= GYRO_BIAS_MAX_ATTEMPTS; attempt++) {
    bootBanner("Measuring gyro",
               attempt == 1 ? "KEEP STILL..." : "MOVED! retrying - KEEP STILL",
               TFT_YELLOW);
    double s[3] = {0, 0, 0}, sq[3] = {0, 0, 0}; int n = 0;
    for (int i = 0; i < GYRO_BIAS_SAMPLES; i++) {
      M5.Imu.update();
      auto d = M5.Imu.getImuData();
      float g3[3] = { d.gyro.x, d.gyro.y, d.gyro.z };
      for (int a = 0; a < 3; a++) { s[a] += g3[a]; sq[a] += (double)g3[a] * g3[a]; }
      n++;
      delay(SAMPLE_INTERVAL_MS);
    }
    float maxSd = 0;
    for (int a = 0; a < 3; a++) {
      double mean = s[a] / n, var = sq[a] / n - mean * mean;
      float sd = var > 0 ? sqrtf((float)var) : 0;
      if (sd > maxSd) maxSd = sd;
    }
    if (maxSd <= GYRO_BIAS_MAX_STD_DPS || attempt == GYRO_BIAS_MAX_ATTEMPTS) {
      g_gyroBias[0] = s[0] / n; g_gyroBias[1] = s[1] / n; g_gyroBias[2] = s[2] / n;
      Serial.printf("[IMU] gyro bias = %.4f %.4f %.4f dps (n=%d, sd=%.3f%s)\n",
                    g_gyroBias[0], g_gyroBias[1], g_gyroBias[2], n, maxSd,
                    maxSd > GYRO_BIAS_MAX_STD_DPS ? " NOISY - accepted anyway" : "");
      return;
    }
    Serial.printf("[IMU] gyro bias capture noisy (sd=%.3f dps) -> retry %d/%d\n",
                  maxSd, attempt + 1, GYRO_BIAS_MAX_ATTEMPTS);
  }
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
    snprintf(buf, n, "NO_TIME");                    // RTC unset; millis is still valid
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
  return String("no clock (") + String(millis() / 1000) + "s up)";
}

// =============================================================================
// microSD + CSV logging
// =============================================================================
static bool sdMount() {
  // Drop any stale mount first: after a card removal / write error, SD.begin()
  // alone would return true on the dead driver state and never re-probe the
  // card, leaving the logger stuck at "open error" until reboot.
  SD.end();
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
  if (!g_logFile) { g_sdStatus = "open error"; g_sdOk = false; // keep the retry loop alive
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
  g_rowsWritten++;
  if (++g_flushCounter >= LOG_FLUSH_EVERY) { g_logFile.flush(); g_flushCounter = 0; }
}

static void markEvent(const String& label) {
  g_eventCount++;
  EventRec& e = g_events[g_evHead];                 // remember for the dashboard
  e.ms = millis();
  strncpy(e.label, label.c_str(), sizeof(e.label) - 1);
  e.label[sizeof(e.label) - 1] = '\0';
  g_evHead = (g_evHead + 1) % EVENT_RING_LENGTH;
  if (g_evCount < EVENT_RING_LENGTH) g_evCount++;
  g_histEventLatch = true;                          // flag it in the trend history
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
    if (now - lastSettled >= g_set.settledLogMs) { lastSettled = now; writeRow(true, 0, ""); }
  } else if (g_set.logWhileMoving) {
    if (now - lastMoving >= g_set.movingLogMs)   { lastMoving = now; writeRow(false, 0, ""); }
  }
}

// One trend-history entry per second: the same value the CSV would log (the
// sharpened average once settled, the instantaneous reading otherwise), so the
// dashboard chart can be backfilled on connect.
static void historyTick() {
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last < HISTORY_INTERVAL_MS) return;
  last = now;
  if (!g_imuOk) return;
  HistEntry& h = g_hist[g_histHead];
  h.ms    = now;
  h.pitch = g_settled ? g_pitch : g_instPitch;
  h.roll  = g_settled ? g_roll  : g_instRoll;
  h.temp  = g_imuTemp;
  h.flags = (uint8_t)((g_settled ? 1 : 0) | (g_histEventLatch ? 2 : 0));
  g_histEventLatch = false;
  g_histHead = (g_histHead + 1) % HISTORY_LENGTH;
  if (g_histCount < HISTORY_LENGTH) g_histCount++;
}

// =============================================================================
// WiFi — soft-AP by default (zero infrastructure), station/hotspot optional
// =============================================================================
static void startAP() {
  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL, 0, AP_MAX_CLIENTS);
  g_apMode = true;
  g_ip = WiFi.softAPIP().toString();
  g_dns.start(53, "*", WiFi.softAPIP());          // captive portal: all DNS -> us
  Serial.printf("[WiFi] soft-AP '%s' %s at %s\n",
                AP_SSID, ok ? "up" : "FAILED", g_ip.c_str());
}

static bool startSTA() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(MDNS_HOSTNAME);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("[WiFi] joining '%s' ...\n", WIFI_SSID);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < WIFI_CONNECT_TIMEOUT_MS)
    delay(100);
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi] station connect FAILED");
    return false;
  }
  g_apMode = false;
  g_ip = WiFi.localIP().toString();
  // The hotspot normally has internet -> NTP; persisted to the RTC in wifiTick.
  configTzTime(TZ_INFO, NTP_SERVER_1, NTP_SERVER_2);
  if (MDNS.begin(MDNS_HOSTNAME)) MDNS.addService("http", "tcp", WEB_SERVER_PORT);
  Serial.printf("[WiFi] connected, IP %s (http://%s.local/)\n",
                g_ip.c_str(), MDNS_HOSTNAME);
  return true;
}

static void setupWiFi() {
  WiFi.persistent(false);
  if (strlen(AP_PASS) < 8)
    Serial.println("[WiFi] WARNING: AP_PASS shorter than 8 chars -> WPA2 soft-AP will fail");
#if WIFI_MODE_SELECT == LOGGER_WIFI_STA
  if (!startSTA()) g_ip = "0.0.0.0";
#elif WIFI_MODE_SELECT == LOGGER_WIFI_AUTO
  if (!startSTA()) startAP();
#else
  startAP();
#endif
  // QR content: WiFi-join string in AP mode (captive portal does the rest),
  // dashboard URL in station mode (the phone is already on that network).
  if (g_apMode) snprintf(g_qrText, sizeof(g_qrText), "WIFI:T:WPA;S:%s;P:%s;;", AP_SSID, AP_PASS);
  else          snprintf(g_qrText, sizeof(g_qrText), "http://%s/", g_ip.c_str());
}

// Station upkeep (reconnect; one-time NTP -> RTC persist). Cheap, 1 Hz.
static void wifiTick() {
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last < 1000) return;
  last = now;
  if (g_apMode) return;
  static uint32_t lastRetry = 0;
  if (WiFi.status() != WL_CONNECTED) {
    if (now - lastRetry >= WIFI_RECONNECT_INTERVAL_MS) { lastRetry = now; WiFi.reconnect(); }
    return;
  }
  String ip = WiFi.localIP().toString();            // refresh after a reconnect:
  if (ip != g_ip) {                                 // the DHCP lease may have changed
    g_ip = ip;
    snprintf(g_qrText, sizeof(g_qrText), "http://%s/", g_ip.c_str());
    Serial.printf("[WiFi] IP now %s\n", g_ip.c_str());
  }
  if (!g_rtcSyncedFromNtp && timeIsValid()) {
    time_t t = time(nullptr);
    M5.Rtc.setDateTime(gmtime(&t));                 // BM8563 stores UTC
    g_rtcSyncedFromNtp = true;
    Serial.println("[RTC] set from NTP");
  }
}

// =============================================================================
// Web API + dashboard. SYNCHRONOUS WebServer: every handler below runs inside
// loop() (g_http.handleClient()), so they may touch any global directly — the
// async-task/spinlock plumbing of the old web build is deliberately gone.
// =============================================================================
static void jsonEscapeInto(String& out, const char* s) {
  for (; *s; s++) {
    char c = *s;
    if (c == '"' || c == '\\')   { out += '\\'; out += c; }
    else if ((uint8_t)c < 0x20)  out += ' ';
    else                         out += c;
  }
}

static String buildStatusJson() {
  String j; j.reserve(1100);
  char t[192];
  time_t ep = time(nullptr);
  snprintf(t, sizeof(t), "{\"up\":%lu,\"epoch\":%lld,\"clock\":%d,",
           (unsigned long)millis(), (long long)(timeIsValid() ? ep : 0),
           timeIsValid() ? 1 : 0);
  j += t;
  snprintf(t, sizeof(t),
           "\"pitch\":%.4f,\"roll\":%.4f,\"ip\":%.4f,\"ir\":%.4f,\"dp\":%.4f,\"dr\":%.4f,",
           g_pitch, g_roll, g_instPitch, g_instRoll, g_dispPitch, g_dispRoll);
  j += t;
  snprintf(t, sizeof(t), "\"settled\":%d,\"streak\":%d,\"n\":%d,\"nT\":%u,\"dwellMs\":%lu,",
           g_settled ? 1 : 0, g_stationaryStreak, g_bufCount,
           (unsigned)g_set.avgSamples, (unsigned long)g_set.dwellMs);
  j += t;
  snprintf(t, sizeof(t), "\"sig\":%.5f,\"gyro\":%.3f,\"sigTh\":%.5f,\"gyroTh\":%.3f,\"tol\":%.3f,",
           g_accelSd, g_gyroMag, g_set.accelStdG, g_set.gyroThreshDps, g_set.levelTolDeg);
  j += t;
  snprintf(t, sizeof(t), "\"cal\":%d,\"ox\":%.5f,\"oy\":%.5f,\"calT\":",
           g_calibrated ? 1 : 0, g_offX, g_offY);
  j += t;
  j += isNan(g_calTempC) ? String("null") : String(g_calTempC, 1);
  snprintf(t, sizeof(t), ",\"cs\":\"%s\",\"cp\":%d,\"ce\":%d,\"cr\":%d,\"cm\":\"",
           calStepName(), calProgress(), g_calError ? 1 : 0, g_calRetries);
  j += t;
  jsonEscapeInto(j, g_calResultMsg.c_str());
  j += "\",\"temp\":";
  j += isNan(g_imuTemp) ? String("null") : String(g_imuTemp, 2);
  snprintf(t, sizeof(t), ",\"bat\":%d,\"chg\":%d,\"sd\":%d,\"sds\":\"",
           g_battery, g_charging ? 1 : 0, g_sdOk ? 1 : 0);
  j += t;
  jsonEscapeInto(j, g_sdStatus.c_str());
  j += "\",\"log\":"; j += (g_logging ? '1' : '0');
  j += ",\"file\":\""; jsonEscapeInto(j, g_logName.c_str());
  snprintf(t, sizeof(t), "\",\"rows\":%lu,\"ev\":%lu,\"evs\":[",
           (unsigned long)g_rowsWritten, (unsigned long)g_eventCount);
  j += t;
  for (int i = 0; i < g_evCount; i++) {             // newest first
    int idx = (g_evHead - 1 - i + 2 * EVENT_RING_LENGTH) % EVENT_RING_LENGTH;
    snprintf(t, sizeof(t), "%s[%lu,\"", i ? "," : "", (unsigned long)g_events[idx].ms);
    j += t;
    jsonEscapeInto(j, g_events[idx].label);
    j += "\"]";
  }
  snprintf(t, sizeof(t), "],\"wifi\":\"%s\",\"clients\":%d,\"rssi\":%d}",
           g_apMode ? "AP" : "STA",
           g_apMode ? (int)WiFi.softAPgetStationNum() : -1,
           g_apMode ? 0 : (int)WiFi.RSSI());
  j += t;
  return j;
}

static String settingsJson() {
  char t[300];
  snprintf(t, sizeof(t),
           "{\"gyroTh\":%.3f,\"sigTh\":%.5f,\"magTol\":%.3f,\"dwellMs\":%lu,\"avgN\":%u,"
           "\"logMs\":%lu,\"logMove\":%d,\"moveMs\":%lu,\"tol\":%.3f,"
           "\"psign\":%d,\"rsign\":%d,\"swap\":%d}",
           g_set.gyroThreshDps, g_set.accelStdG, g_set.accelMagTolG,
           (unsigned long)g_set.dwellMs, (unsigned)g_set.avgSamples,
           (unsigned long)g_set.settledLogMs, g_set.logWhileMoving ? 1 : 0,
           (unsigned long)g_set.movingLogMs, g_set.levelTolDeg,
           g_set.pitchSign > 0 ? 1 : -1, g_set.rollSign > 0 ? 1 : -1,
           g_set.swapPitchRoll ? 1 : 0);
  return String(t);
}

// Trend backfill: entries as [ms,pitch,roll,temp|null,flags], oldest first.
// Chunked so the ~900-entry worst case never needs one big RAM buffer.
static void sendHistory() {
  g_http.setContentLength(CONTENT_LENGTH_UNKNOWN);
  g_http.send(200, "application/json", "");
  char t[80];
  snprintf(t, sizeof(t), "{\"now\":%lu,\"e\":[", (unsigned long)millis());
  String chunk = t; chunk.reserve(2300);
  for (int i = 0; i < g_histCount; i++) {
    int idx = (g_histHead - g_histCount + i + HISTORY_LENGTH) % HISTORY_LENGTH;
    const HistEntry& h = g_hist[idx];
    if (isNan(h.temp))
      snprintf(t, sizeof(t), "%s[%lu,%.4f,%.4f,null,%u]",
               i ? "," : "", (unsigned long)h.ms, h.pitch, h.roll, h.flags);
    else
      snprintf(t, sizeof(t), "%s[%lu,%.4f,%.4f,%.2f,%u]",
               i ? "," : "", (unsigned long)h.ms, h.pitch, h.roll, h.temp, h.flags);
    chunk += t;
    if (chunk.length() > 2048) { g_http.sendContent(chunk); chunk = ""; }
  }
  chunk += "]}";
  g_http.sendContent(chunk);
  g_http.sendContent("");                           // terminate the chunked stream
}

// Phones probe these URLs to detect captive portals; redirecting them to the
// dashboard makes the "sign in to network" sheet open it automatically.
static void captiveRedirect() {
  g_http.sendHeader("Location", String("http://") + g_ip + "/", true);
  g_http.send(302, "text/plain", "");
}

static void setupWeb() {
  g_http.on("/", HTTP_GET, []() { g_http.send_P(200, "text/html", WEB_PAGE_HTML); });

  g_http.on("/api/status",  HTTP_GET, []() { g_http.send(200, "application/json", buildStatusJson()); });
  g_http.on("/api/history", HTTP_GET, sendHistory);
  g_http.on("/api/settings", HTTP_GET, []() { g_http.send(200, "application/json", settingsJson()); });

  g_http.on("/api/settings", HTTP_POST, []() {
    if (g_http.hasArg("reset")) {
      settingsDefaults();
      bufReset(); g_stationaryStreak = 0; g_settled = false;
    } else {
      uint16_t oldAvg = g_set.avgSamples;
      if (g_http.hasArg("gyroTh"))  g_set.gyroThreshDps  = g_http.arg("gyroTh").toFloat();
      if (g_http.hasArg("sigTh"))   g_set.accelStdG      = g_http.arg("sigTh").toFloat();
      if (g_http.hasArg("magTol"))  g_set.accelMagTolG   = g_http.arg("magTol").toFloat();
      if (g_http.hasArg("dwellMs")) g_set.dwellMs        = (uint32_t)g_http.arg("dwellMs").toInt();
      if (g_http.hasArg("avgN"))    g_set.avgSamples     = (uint16_t)g_http.arg("avgN").toInt();
      if (g_http.hasArg("logMs"))   g_set.settledLogMs   = (uint32_t)g_http.arg("logMs").toInt();
      if (g_http.hasArg("logMove")) g_set.logWhileMoving = g_http.arg("logMove").toInt() != 0;
      if (g_http.hasArg("moveMs"))  g_set.movingLogMs    = (uint32_t)g_http.arg("moveMs").toInt();
      if (g_http.hasArg("tol"))     g_set.levelTolDeg    = g_http.arg("tol").toFloat();
      if (g_http.hasArg("psign"))   g_set.pitchSign      = g_http.arg("psign").toFloat();
      if (g_http.hasArg("rsign"))   g_set.rollSign       = g_http.arg("rsign").toFloat();
      if (g_http.hasArg("swap"))    g_set.swapPitchRoll  = g_http.arg("swap").toInt() != 0;
      settingsSanitize();
      if (g_set.avgSamples != oldAvg) {             // window changed -> restart the average
        bufReset(); g_stationaryStreak = 0; g_settled = false;
      }
    }
    settingsSanitize();
    settingsSave();
    g_http.send(200, "application/json", settingsJson());
  });

  g_http.on("/api/mark", HTTP_POST, []() {
    String label = g_http.arg("label");
    label.trim();
    if (label.length() > 31) label = label.substring(0, 31);
    for (size_t i = 0; i < label.length(); i++) {   // keep CSV/JSON trivially safe
      char c = label[i];
      if (c == '"' || c == '\\' || (uint8_t)c < 0x20) label.setCharAt(i, '\'');
    }
    if (!label.length()) label = "web-mark";
    markEvent(label);
    g_http.send(200, "application/json",
                String("{\"ok\":1,\"ev\":") + String(g_eventCount) + "}");
  });

  g_http.on("/api/log", HTTP_POST, []() {
    String a = g_http.arg("action");
    if      (a == "start" && !g_logging) toggleLogging();
    else if (a == "stop"  &&  g_logging) toggleLogging();
    g_http.send(200, "application/json",
                String("{\"ok\":1,\"log\":") + (g_logging ? "1" : "0") + "}");
  });

  g_http.on("/api/cal", HTTP_POST, []() {
    String a = g_http.arg("action");
    if      (a == "start" && g_calStep == CAL_IDLE) { g_showQr = false; calibStart(); }
    else if (a == "next")                           calibAdvance();
    else if (a == "cancel")                         calibCancel();
    g_http.send(200, "application/json",
                String("{\"ok\":1,\"cs\":\"") + calStepName() + "\"}");
  });

  g_http.on("/api/time", HTTP_POST, []() {
    long long ep = atoll(g_http.arg("epoch").c_str());
    if (ep > (long long)TIME_VALID_EPOCH && ep < 4102444800LL) {  // ..year 2100
      struct timeval tv = { (time_t)ep, 0 };
      settimeofday(&tv, nullptr);
      time_t tt = (time_t)ep;
      M5.Rtc.setDateTime(gmtime(&tt));              // BM8563 stores UTC
      Serial.printf("[RTC] set from phone (epoch %lld)\n", ep);
      g_http.send(200, "application/json", "{\"ok\":1}");
    } else {
      g_http.send(400, "application/json", "{\"ok\":0,\"err\":\"bad epoch\"}");
    }
  });

  static const char* probes[] = {
    "/generate_204", "/gen_204",                    // Android
    "/hotspot-detect.html", "/library/test/success.html", // iOS / macOS
    "/connecttest.txt", "/ncsi.txt",                // Windows
    "/success.txt", "/canonical.html"
  };
  for (auto p : probes) g_http.on(p, captiveRedirect);
  g_http.onNotFound(captiveRedirect);

  g_http.begin();
  Serial.printf("[WEB] dashboard at http://%s/\n", g_ip.c_str());
}

// =============================================================================
// Touch buttons (operator input on the device; the phone dashboard mirrors them)
// -----------------------------------------------------------------------------
//  Normal mode buttons:  CAL (tap = start calibration, HOLD = clear calibration)
//                        MARK (log an event row), LOG (start/stop logging).
//  Top strip:            tap = "connect your phone" QR overlay.
//  Calibration wizard:   NEXT (advance / finish), CANCEL.
// =============================================================================
static void handleTouch() {
  // Note: do NOT early-return on getCount()==0 — the release frame often reports
  // zero active touches, and we need wasReleased() there. getDetail(0) is valid.
  auto t = M5.Touch.getDetail();
  int x = t.x, y = t.y;

  // ---- QR overlay: any tap closes it ------------------------------------------
  if (g_showQr) {
    if (t.wasReleased()) g_showQr = false;
    return;
  }

  // ---- calibration wizard: act on release over a button ----------------------
  if (g_calStep != CAL_IDLE) {
    if (t.wasReleased()) {
      if      (hit(x, y, g_btnCalNext))   calibAdvance();
      else if (hit(x, y, g_btnCalCancel)) calibCancel();
    }
    return;
  }

  // ---- normal mode: HOLD on CAL clears stored calibration --------------------
  static uint32_t calHoldStart = 0;
  static bool     calCleared   = false;
  if (t.isPressed() && hit(x, y, g_btn[0])) {
    if (calHoldStart == 0) { calHoldStart = millis(); calCleared = false; }
    else if (!calCleared && millis() - calHoldStart >= CAL_CLEAR_HOLD_MS) {
      clearCalibration();                            // revert to UNCALIBRATED
      calCleared = true;
    }
  } else {
    calHoldStart = 0;                                // released, or moved off CAL
  }

  // ---- normal mode: taps act on release --------------------------------------
  if (t.wasReleased()) {
    if      (y < 24)              g_showQr = true;                   // top strip -> QR
    else if (hit(x, y, g_btn[0])) { if (!calCleared) calibStart(); } // CAL (tap)
    else if (hit(x, y, g_btn[1])) markEvent("device-button");        // MARK
    else if (hit(x, y, g_btn[2])) toggleLogging();                   // LOG
  }
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

// "Connect your phone" overlay (tap the top strip to open, tap anywhere to
// close). In AP mode the QR is a standard WIFI: join string — scanning it joins
// the soft-AP and the captive portal then opens the dashboard by itself. In
// station mode the phone is already on the same network, so the QR is the URL.
static void drawQrScreen() {
  auto& cv = g_canvas;
  cv.fillSprite(TFT_BLACK);
  cv.setTextSize(1); cv.setTextColor(TFT_CYAN);
  cv.drawString("CONNECT YOUR PHONE", 8, 6);

  const int qs = 150, qx = 12, qy = 28;
  cv.fillRect(qx - 5, qy - 5, qs + 10, qs + 10, TFT_WHITE);  // quiet zone
  cv.qrcode(g_qrText, qx, qy, qs);

  int tx = qx + qs + 16, ty = 32;
  cv.setTextColor(TFT_WHITE);
  if (g_apMode) {
    cv.drawString("1. Scan to join", tx, ty);              ty += 14;
    cv.setTextColor(TFT_DARKGREY);
    cv.drawString(AP_SSID, tx + 8, ty);                    ty += 12;
    cv.drawString((String("pw ") + AP_PASS).c_str(), tx + 8, ty); ty += 18;
    cv.setTextColor(TFT_WHITE);
    cv.drawString("2. Dashboard pops", tx, ty);            ty += 12;
    cv.drawString("   up - or open:", tx, ty);             ty += 16;
  } else {
    cv.drawString("Scan to open the", tx, ty);             ty += 12;
    cv.drawString("dashboard:", tx, ty);                   ty += 16;
  }
  cv.setTextColor(TFT_GREEN);
  cv.drawString((String("http://") + g_ip + "/").c_str(), tx, ty);
  cv.setTextColor(TFT_DARKGREY);
  cv.drawString("tap anywhere to close", 12, 224);
  cv.pushSprite(0, 0);
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
  int tolR = (int)((g_set.levelTolDeg / scaleDeg) * R);
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

  bool level = (fabsf(g_dispPitch) <= g_set.levelTolDeg &&
                fabsf(g_dispRoll)  <= g_set.levelTolDeg);
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
  if (g_showQr)              { drawQrScreen(); return; }   // pushes its own sprite

  // ---- top strip: title + WiFi info (tap this strip for the QR) + battery ----
  cv.setTextSize(1); cv.setTextColor(TFT_CYAN);
  cv.drawString("BUBBLE LEVEL", 6, 4);
  char ws[40];
  if (g_apMode)                                snprintf(ws, sizeof(ws), "[QR] AP %s (%d)", g_ip.c_str(), WiFi.softAPgetStationNum());
  else if (WiFi.status() == WL_CONNECTED)      snprintf(ws, sizeof(ws), "[QR] %s", g_ip.c_str());
  else                                         snprintf(ws, sizeof(ws), "[QR] no wifi");
  cv.setTextColor(TFT_DARKGREY);
  cv.drawString(ws, 92, 4);
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
  cv.setTextSize(1); cv.setTextColor(TFT_DARKGREY); cv.drawString("PITCH  deg", RX, 24);
  cv.setTextSize(2); cv.setTextColor(valCol);
  snprintf(v, sizeof(v), "%+.*f", ANGLE_DECIMALS, showPitch); cv.drawString(v, RX, 34);
  cv.setTextSize(1); cv.setTextColor(TFT_DARKGREY); cv.drawString("ROLL   deg", RX, 58);
  cv.setTextSize(2); cv.setTextColor(valCol);
  snprintf(v, sizeof(v), "%+.*f", ANGLE_DECIMALS, showRoll); cv.drawString(v, RX, 68);

  cv.setTextSize(1);
  cv.setTextColor(valCol);
  cv.drawString(g_settled ? "SETTLED" : "MOVING", RX, 98);
  cv.setTextColor(g_calibrated ? TFT_GREEN : TFT_RED);
  cv.drawString(g_calibrated ? "CALIBRATED" : "UNCALIBRATED", RX, 114);
  cv.setTextColor(TFT_WHITE);
  char ln[40];
  snprintf(ln, sizeof(ln), "SD:%s%s", g_sdStatus.c_str(), g_logging ? " LOG" : "");
  cv.drawString(ln, RX, 134);
  if (!isNan(g_imuTemp)) snprintf(ln, sizeof(ln), "EV:%lu   %.1fC",
                                  (unsigned long)g_eventCount, g_imuTemp);
  else                   snprintf(ln, sizeof(ln), "EV:%lu", (unsigned long)g_eventCount);
  cv.drawString(ln, RX, 150);
  cv.drawString(clockString(), RX, 166);

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
  settingsLoad();                   // NVS runtime settings (or factory defaults)

  // --- clock: use the battery-backed BM8563 RTC if it holds a valid time -----
  //  Timestamps are real if the RTC was ever set — by the dashboard's "Sync
  //  clock from phone" button, by NTP (station mode), or externally (M5Burner).
  //  Otherwise filenames fall back to an index and timestamp_iso is logged as
  //  NO_TIME — the millis column is always valid.
  setenv("TZ", TZ_INFO, 1); tzset();
  M5.Rtc.setSystemTimeFromRtc();    // no-op if the RTC was never set

  // --- microSD: open the log file; start logging if a card is present -------
  if (openLog()) g_logging = true;

  // --- WiFi + phone dashboard (soft-AP by default; QR overlay via top strip) -
  bootBanner("Starting WiFi...", "", TFT_CYAN);
  setupWiFi();
  setupWeb();

  Serial.println("[BOOT] done. Tap the top strip for the connect-QR.");
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

  // 2) phone dashboard: captive-portal DNS + HTTP. The handlers run HERE, in
  //    loop() context (synchronous WebServer) — no cross-task state sharing.
  if (g_apMode) g_dns.processNextRequest();
  g_http.handleClient();
  wifiTick();

  // 3) calibration capture completion
  calibrationTick();

  // 4) on-screen touch buttons
  handleTouch();

  // 5) CSV logging cadence (settled rows; events are immediate) + trend history
  loggingTick();
  historyTick();

  // 6) screen refresh (live bubble level)
  static uint32_t lastDisp = 0;
  if (now - lastDisp >= DISPLAY_INTERVAL_MS) { lastDisp = now; updateDisplay(); }

  // 7) cached battery/charging read (kept off the 100 Hz hot path)
  static uint32_t lastBat = 0;
  if (now - lastBat >= BATTERY_READ_INTERVAL_MS) {
    lastBat = now;
    g_battery  = M5.Power.getBatteryLevel();
    g_charging = ((int)M5.Power.isCharging() == 1);
  }

  delay(1);                                          // yield to idle task
}
