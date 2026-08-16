#include "app.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "attitude.h"
#include "imu.h"
#include "baro.h"
#include "gps.h"
#include "mag.h"
#include "nav.h"
#include "motors.h"
#include "communications.h"
#include "receiver.h"
#include "telemetry.h"
#include "sdcard.h"
#include "fault_record.h"
#include "main.h"

#define APP_MOTOR_TEST_MODE 0U
#define APP_CONTROL_LOOP_MS 2U
#define RAD_PER_DEG       0.0174532925f
#define APP_ENABLE_IMU_RUNTIME_TELEMETRY 0U
#define APP_ENABLE_ARM_RUNTIME_TELEMETRY 1U
#define APP_ENABLE_RECEIVER_RUNTIME_TELEMETRY 0U
#define APP_ENABLE_ANGLE_TELEMETRY 0U
/* USB motor-test (bench, ground-only) mode always streams IMU state for motor_test_gui.py,
 * independent of APP_ENABLE_IMU_RUNTIME_TELEMETRY which stays off for real armed flight. */
#define APP_ENABLE_USB_TEST_IMU_TELEMETRY 1U
#define APP_CH_ROLL_INDEX      0U
#define APP_CH_PITCH_INDEX     1U
#define APP_CH_THROTTLE_INDEX  2U
#define APP_CH_YAW_INDEX       3U
#define APP_CH_ARM_INDEX       4U
#define APP_CH_MODE_INDEX      5U
#define APP_CH_TELARM_INDEX    6U
#define APP_ARM_THRESHOLD_US   1500U
/* A single corrupted/noisy CRSF frame can momentarily report the arm channel
 * or link as bad for just one control-loop iteration - require the disarm
 * condition to persist this long before actually disarming, so a real
 * intentional disarm (held stick/switch, real link loss) is unaffected while
 * a one-frame glitch is filtered out. */
#define APP_DISARM_DEBOUNCE_MS 60U
#define APP_THROTTLE_LOW_US    1100U
#define APP_ARM_HOLD_MS        300U
#define APP_BEEPER_TOGGLE_MS   150U
#define APP_STARTUP_BEEP_MS    120U
#define APP_ATTITUDE_ZERO_SETTLE_MS 2000U
#define APP_ATTITUDE_ZERO_AVG_MS 300U
#define APP_USB_TEST_ARM_DELAY_MS 2000U
#define APP_IMU_TELEMETRY_MS    120U
#define APP_ARM_TELEMETRY_MS    150U
#define APP_RX16_TELEMETRY_MS   120U
#define APP_LOW_THROTTLE_MIX_DISABLE_US 40U
/* Ease attitude/yaw correction in over this many ms after crossing the low-throttle
 * gate, instead of releasing full authority in one step - smooths a liftoff bounce
 * when the airframe was resting tilted (e.g. uneven ground) right before takeoff. */
#define APP_LIFTOFF_RAMP_MS     250U
#define APP_CONTROL_DEADBAND_US 20U
#define APP_MOTOR_IDLE_US      1080U
#define APP_BARO_UPDATE_MS     20U
#define APP_BARO_RETRY_MS      1000U
#define APP_GPS_RETRY_MS       3000U
#define APP_MAG_UPDATE_MS      50U
#define APP_MAG_RETRY_MS       1000U
/* Subtracts a term proportional to baro-derived climb rate from throttle to damp
 * Z-axis bounce/sensitivity. Conservative starting gain now that the SPA06 (SPL07-003)
 * baro is confirmed detected/healthy on hardware; hard-clamped to +-APP_BARO_VZ_DAMP_LIMIT_US
 * and only applied while Baro_IsHealthy() - re-tune upward only after a hover confirms the
 * signal is low-noise/low-latency enough to not fight stick throttle inputs. */
#define APP_BARO_VZ_DAMP_GAIN_US_PER_MPS 15.0f
#define APP_BARO_VZ_DAMP_LIMIT_US        60U
/* Ground effect/propwash makes baro pressure very noisy within about a meter of the
 * ground, which otherwise makes the Vz damping term above rapidly flip sign/saturate
 * right at liftoff (audible motor thrust jitter) - suppress it until climbed clear. */
#define APP_BARO_VZ_DAMP_MIN_ALT_M       1.0f
/* ALTHOLD reuses ATTITUDE-mode roll/pitch/yaw angle stabilization. The pilot's raw
 * stick throttle is ALWAYS the primary/dominant throttle signal (exactly like
 * ATTITUDE mode) - ALTHOLD only ever ADDS a small bounded trim on top of it while
 * the stick sits near center, to hold the altitude captured the instant it
 * re-centers. Away from center the trim is zeroed and the pilot has full, direct
 * manual throttle authority. This bounded-trim design means a bad baro reading or a
 * wound-up integral can only ever nudge the output by APP_ALTHOLD_TRIM_LIMIT_US - it
 * can never freeze the motors at idle or run away independent of the stick. Falls
 * back to plain manual throttle (like ATTITUDE mode) if the baro is unhealthy. */
#define APP_ALTHOLD_THROTTLE_DEADBAND_US   40U
/* The hold-throttle reference re-latches to wherever the stick has genuinely
 * settled (within this small window) for this long - NOT just once at mode
 * entry (which can freeze on an unrepresentative value, e.g. idle throttle
 * before liftoff) and NOT every single non-holding iteration (which would
 * re-latch mid-movement, since consecutive samples of a smoothly-moving stick
 * are often within a small window of each other). */
#define APP_ALTHOLD_STICK_STABLE_WINDOW_US 15U
#define APP_ALTHOLD_STICK_SETTLE_MS        300U
#define APP_ALTHOLD_MAX_CLIMB_MPS           2.0f
#define APP_ALTHOLD_ALT_HOLD_KP_MPS_PER_M   0.8f
#define APP_ALTHOLD_VZ_KP_US_PER_MPS        25.0f
#define APP_ALTHOLD_VZ_KI_US_PER_MPS_S       8.0f
#define APP_ALTHOLD_INTEGRAL_LIMIT_US      200U
/* Smooths the final trim output itself (not the P/I gains) - climb-rate error is
 * fed straight from the baro, and even with the baro's own onboard LPF, real
 * sensor/vibration noise translated directly into trim was making the motors
 * respond jerkily on the Z axis. A slow-ish cutoff is fine since altitude
 * corrections are inherently a slow process. */
#define APP_ALTHOLD_TRIM_LPF_HZ             1.5f
#define APP_ALTHOLD_TRIM_LIMIT_US          250U
/* --- NAV_VELOCITY_BRAKE (experimental GPS horizontal velocity brake, phase 1) ---
 * Reuses the ATTITUDE-mode angle controller (roll/pitch) and the ALTHOLD throttle
 * controller (vertical) - this section only adds the outer velocity->angle loop.
 * Engaged when ch6 (the existing mode switch) exceeds this threshold - a 4th band
 * above ALTHOLD's >=1800us, so it needs a transmitter mix that can drive ch6 above
 * 2000us (the existing 3-way RATE/ATTITUDE/ALTHOLD switch positions are unaffected).
 *
 * FLIGHT-TESTED FINDING (2026-08-15): an earlier pitch-sign bug (APP_PITCH_SIGN was
 * applied twice - once converting stick to fwd_cmd_mps, again converting the
 * resulting accel to target_pitch_deg - so the two flips canceled and inverted the
 * pilot's fwd/aft command) was found and fixed. Bench-verified correct while
 * disarmed (no motor current). But a real flight afterwards - fully engaged the
 * whole time (NAVBRAKE requested=176/176 active=176/176) - still would not track
 * the stick and drifted. SD log analysis showed why: compass-vs-yaw tracking error
 * peaked at 25.3deg during that flight (rms 9.3deg) vs. peak 5.2deg on the same
 * airframe disarmed/no-motor-current. Both Nav_RotateBodyToNed() (stick command)
 * and Nav_RotateNedToBody() (velocity-error accel) key off yaw_deg, so a yaw error
 * that size rotates "forward" into materially the wrong geographic direction -
 * independent of, and not fixed by, the pitch-sign fix. This is the same
 * motor-current magnetic interference already called out near
 * APP_MAG_YAW_NUDGE_KP_DPS_PER_DEG below - do not trust NAVBRAKE until that's
 * actually resolved, not just mitigated.
 *
 * FOLLOW-UP (2026-08-16, real flight, 190 samples/6.24s): tracking error is not
 * just ambient motor-current interference - it's substantially worse specifically
 * while the pilot is actively commanding yaw. While holding heading (stick
 * centered, 164 samples): rms 6.4deg peak 22.3deg. While commanding yaw (stick
 * active, 26 samples): rms 21.9deg peak 32.4deg. So active rotation itself
 * degrades compass/gyro agreement well beyond the hover-only baseline - likely
 * magnetometer response lag relative to the gyro-integrated estimate, or the
 * yaw-nudge correction coping worse with a moving target than a steady one.
 * Sharpens, doesn't change, the conclusion above. */
#define APP_NAV_BRAKE_SWITCH_THRESHOLD_US   2000U
/* Conservative first-pass limits (see user-selected 1.5 m/s max velocity):
 * kept at/near the low end of the suggested ranges since this mode is unflown. */
#define APP_NAV_BRAKE_MAX_TILT_DEG          8.0f
#define APP_NAV_BRAKE_MAX_ACCEL_MPS2        1.0f
#define APP_NAV_BRAKE_MAX_VEL_MPS           1.5f
#define APP_NAV_BRAKE_STICK_DEADBAND_US      20U
/* accel_cmd (m/s^2) = Kp * vel_error (m/s). At the max 1.5 m/s error this gives
 * 0.9 m/s^2, leaving headroom below the 1.0 m/s^2 clamp instead of saturating
 * immediately at every large error. */
#define APP_NAV_BRAKE_VELOCITY_KP            0.6f
#define APP_ROLL_SIGN          (1)
#define APP_PITCH_SIGN         (-1)
#define APP_YAW_SIGN           (1)
#define APP_GYRO_YAW_SIGN      (-1)
/* Slow, bounded drift correction only - nudges the gyro-integrated yaw toward the
 * compass's own "rotation since boot" delta (NOT toward absolute magnetic north,
 * so boot heading stays 0 like today). Deliberately NOT a full 9-DOF Mahony fusion
 * (higher risk given this codebase's AHRS bug history) - just corrects long-term
 * free-drift, same role as the existing gyro bias learning above.
 * RE-ENABLED (2026-08-09) after the GPS/compass module was physically relocated
 * further from motors/ESCs/power wiring to address the motor-current interference
 * that caused this to be disabled earlier the same day (100+ deg apparent
 * rotation while holding heading). Re-check compass-vs-yaw tracking error on the
 * next flight before trusting this - if interference is still significant,
 * disable again (set kp back to 0.0f) rather than re-tune the field-strength gate
 * further blind. */
#define APP_MAG_YAW_NUDGE_KP_DPS_PER_DEG 0.05f
#define APP_MAG_YAW_NUDGE_MAX_DPS        2.0f
/* Motor/ESC current is a well-known source of magnetic interference that can
 * swing the compass heading by 100+ degrees at flight throttle even though
 * nothing physically rotated - trust the heading only while the total field
 * STRENGTH still looks like the undisturbed reference sample; a real magnetic
 * disturbance changes magnitude as well as direction, unlike a real yaw turn. */
#define APP_MAG_TRUST_MAX_MAG_DEVIATION_FRAC 0.35f
/* These express each filter's intended cutoff directly; the alpha applied each iteration is
 * derived from the actual measured dt_s (see App_LpfAlpha()) instead of assuming a fixed
 * APP_CONTROL_LOOP_MS sample time, since the real loop period varies well beyond 2ms under load. */
#define APP_GYRO_RATE_LPF_HZ   70.0f  /* gyro feedback filter (motor/prop vibration). Feeds the rate
                                        * loop's P-error/D-source only - AHRS and GLOG capture still
                                        * see the raw, unfiltered gyro. */
#define APP_DTERM_LPF_HZ       20.0f  /* D-term-only noise filter. */
#define APP_FF_LPF_HZ          13.0f  /* feedforward-only smoothing filter.
                                        * Lowered from ~29Hz: higher Kff was overshooting on fast
                                        * stick edges before this got smoothed out. */
#define APP_MODE_SWITCH_THRESHOLD_US 1500U
/* Ch6 3-position switch: <1500=RATE, 1500-1799=ATTITUDE, >=1800=ALTHOLD. */
#define APP_ALTHOLD_SWITCH_THRESHOLD_US 1800U
#define APP_ATTITUDE_MAX_ANGLE_DEG   35.0f
#define APP_ATTITUDE_ANGLE_KP_DPS_PER_DEG 5.0f
#define APP_ATTITUDE_KP_MIN_DPS_PER_DEG 0.2f
/* Was capped at 12; that ceiling saturated the 300 dps rate-cmd clamp at only
 * ~25 deg of error, leaving no headroom to tighten small-error tracking bandwidth. */
#define APP_ATTITUDE_KP_MAX_DPS_PER_DEG 25.0f
#define APP_ATTITUDE_MAX_ANGLE_MIN_DEG 5.0f
#define APP_ATTITUDE_MAX_ANGLE_MAX_DEG 70.0f
#define APP_GYRO_STILL_DPS     0.5f
#define APP_ACCEL_STILL_TOL_G  0.08f
#define APP_YAW_BIAS_ALPHA     0.001f
#define APP_YAW_BIAS_SETTLE_SAMPLES 1000U
#define APP_ADC_MAX_COUNT      65535.0f
#define APP_ADC_REF_V          3.3f
#define APP_BATTERY_DIVIDER_RATIO 11.13f
#define APP_BATTERY_FILTER_ALPHA 0.60f
#define APP_BATTERY_ADC_SAMPLES 2U
#define APP_BATTERY_SAMPLE_MS   120U
#define APP_BATTERY_CHANNEL      ADC_CHANNEL_10
/* Rate-PID authority is normalized against this fixed pack voltage (the 3S
 * baseline the gains were tuned on), not a per-cell value - otherwise a
 * straight cell-count swap re-references itself and cancels out to ~1.0. */
#define APP_VOLTAGE_COMP_REFERENCE_V       11.1f
#define APP_VOLTAGE_COMP_FACTOR_MIN        0.7f
#define APP_VOLTAGE_COMP_FACTOR_MAX        1.3f
#define APP_RATE_CMD_MAX_ROLL_DPS   300.0f
#define APP_RATE_CMD_MAX_PITCH_DPS  300.0f
#define APP_RATE_CMD_MAX_YAW_DPS    220.0f
#define APP_RATE_KP_ROLL_DEFAULT_US_PER_DPS 0.90f
#define APP_RATE_KP_PITCH_DEFAULT_US_PER_DPS 0.90f
#define APP_RATE_KP_YAW_DEFAULT_US_PER_DPS  0.80f
#define APP_RATE_KI_ROLL_DEFAULT_US_PER_DPS_S 0.00f
#define APP_RATE_KI_PITCH_DEFAULT_US_PER_DPS_S 0.00f
#define APP_RATE_KI_YAW_DEFAULT_US_PER_DPS_S  0.00f
#define APP_RATE_KD_ROLL_DEFAULT_US_PER_DPS_PER_S 0.00f
#define APP_RATE_KD_PITCH_DEFAULT_US_PER_DPS_PER_S 0.00f
#define APP_RATE_KD_YAW_DEFAULT_US_PER_DPS_PER_S  0.00f
#define APP_RATE_KFF_ROLL_DEFAULT_US_PER_DPS_PER_S 0.00f
#define APP_RATE_KFF_PITCH_DEFAULT_US_PER_DPS_PER_S 0.00f
#define APP_RATE_KFF_YAW_DEFAULT_US_PER_DPS_PER_S  0.00f
#define APP_RATE_TERM_LIMIT_US      320
#define APP_RATE_KP_MIN_US_PER_DPS  0.0f
#define APP_RATE_KP_MAX_US_PER_DPS  4.0f
#define APP_RATE_KI_MIN_US_PER_DPS_S 0.0f
#define APP_RATE_KI_MAX_US_PER_DPS_S 2.0f
#define APP_RATE_KD_MIN_US_PER_DPS_PER_S 0.0f
#define APP_RATE_KD_MAX_US_PER_DPS_PER_S 0.2f
#define APP_RATE_KFF_MIN_US_PER_DPS_PER_S 0.0f
#define APP_RATE_KFF_MAX_US_PER_DPS_PER_S 0.5f
#define APP_PWM_MIN_US         988U
#define APP_PWM_MID_US         1500U
#define APP_PWM_MAX_US         2012U
#define APP_THROTTLE_MAX_US    1880U
#define APP_CRSF_MIN_RAW       172U
#define APP_CRSF_MAX_RAW       1811U

#define APP_PID_FLASH_MAGIC    0x50494447UL
#define APP_PID_FLASH_VERSION  3UL
#define APP_PID_FLASH_ADDRESS  0x081E0000UL

/* Motor position mapping used by the mixer:
 * S1 = Front-Left, S2 = Front-Right, S3 = Rear-Right, S4 = Rear-Left
 */

static volatile uint8_t g_usb_motor_test_enabled = 0U;
static volatile uint8_t g_usb_motor_test_motor_index = 1U;
static volatile uint16_t g_usb_motor_test_pulse_us = 1100U;
static volatile uint8_t g_attitude_zero_request = 0U;

typedef enum
{
  APP_FLIGHT_MODE_RATE = 0U,
  APP_FLIGHT_MODE_ATTITUDE = 1U,
  APP_FLIGHT_MODE_ALTHOLD = 2U,
  APP_FLIGHT_MODE_NAV_VELOCITY_BRAKE = 3U,
} App_FlightMode_t;

typedef enum
{
  APP_PID_CMD_NONE = 0,
  APP_PID_CMD_SET_AND_SAVE = 1,
  APP_PID_CMD_SAVE = 2,
  APP_PID_CMD_LOAD = 3,
  APP_PID_CMD_DEFAULT = 4,
  APP_PID_CMD_ATT_SET_AND_SAVE = 5,
  APP_PID_CMD_ATT_DEFAULT = 6,
} App_PidCommand_t;

typedef struct
{
  uint32_t magic;
  uint32_t version;
  App_RatePidGains_t gains;
  App_AttitudeGains_t att_gains;
  uint32_t crc32;
  uint32_t reserved[6];
} App_PidFlashBlob_t;

/* Pre-kff on-flash axis/gains layout, kept only to migrate older saved blobs. */
typedef struct
{
  float kp;
  float ki;
  float kd;
} App_RatePidAxisGainsLegacy_t;

typedef struct
{
  App_RatePidAxisGainsLegacy_t roll;
  App_RatePidAxisGainsLegacy_t pitch;
  App_RatePidAxisGainsLegacy_t yaw;
} App_RatePidGainsLegacy_t;

typedef struct
{
  uint32_t magic;
  uint32_t version;
  App_RatePidGainsLegacy_t gains;
  uint32_t crc32;
  uint32_t reserved[2];
} App_PidFlashBlobV1_t;

typedef struct
{
  uint32_t magic;
  uint32_t version;
  App_RatePidGainsLegacy_t gains;
  App_AttitudeGains_t att_gains;
  uint32_t crc32;
  uint32_t reserved[1];
} App_PidFlashBlobV2_t;

typedef union
{
  App_PidFlashBlob_t blob;
  uint32_t words[24];
} App_PidFlashPage_t;

#if defined(__GNUC__)
#define APP_FLASHWORD_ALIGN __attribute__((aligned(32)))
#else
#define APP_FLASHWORD_ALIGN
#endif

_Static_assert(sizeof(App_PidFlashBlob_t) <= sizeof(App_PidFlashPage_t), "PID flash blob too large");

static App_RatePidGains_t g_rate_pid_gains = {
  {APP_RATE_KP_ROLL_DEFAULT_US_PER_DPS, APP_RATE_KI_ROLL_DEFAULT_US_PER_DPS_S, APP_RATE_KD_ROLL_DEFAULT_US_PER_DPS_PER_S, APP_RATE_KFF_ROLL_DEFAULT_US_PER_DPS_PER_S},
  {APP_RATE_KP_PITCH_DEFAULT_US_PER_DPS, APP_RATE_KI_PITCH_DEFAULT_US_PER_DPS_S, APP_RATE_KD_PITCH_DEFAULT_US_PER_DPS_PER_S, APP_RATE_KFF_PITCH_DEFAULT_US_PER_DPS_PER_S},
  {APP_RATE_KP_YAW_DEFAULT_US_PER_DPS, APP_RATE_KI_YAW_DEFAULT_US_PER_DPS_S, APP_RATE_KD_YAW_DEFAULT_US_PER_DPS_PER_S, APP_RATE_KFF_YAW_DEFAULT_US_PER_DPS_PER_S},
};
static App_AttitudeGains_t g_attitude_gains = {
  APP_ATTITUDE_ANGLE_KP_DPS_PER_DEG,
  APP_ATTITUDE_ANGLE_KP_DPS_PER_DEG,
  APP_ATTITUDE_MAX_ANGLE_DEG,
};
static volatile App_PidCommand_t g_pid_command = APP_PID_CMD_NONE;
static volatile App_RatePidGains_t g_pid_command_gains;
static volatile App_AttitudeGains_t g_attitude_command_gains;
static volatile uint32_t g_pid_command_queued_count = 0U;
static volatile uint32_t g_pid_command_handled_count = 0U;

typedef enum
{
  APP_MAG_CAL_CMD_NONE = 0,
  APP_MAG_CAL_CMD_START = 1,
  APP_MAG_CAL_CMD_STOP = 2,
} App_MagCalCommand_t;

static volatile App_MagCalCommand_t g_mag_cal_command = APP_MAG_CAL_CMD_NONE;

static uint8_t g_boot_log_pending = 1U;
static uint8_t g_boot_pid_loaded = 0U;
static float g_roll_gyro_bias_dps = 0.0f;
static float g_pitch_gyro_bias_dps = 0.0f;
static float g_yaw_gyro_bias_dps = 0.0f;
static float g_roll_gyro_bias_sum_dps = 0.0f;
static float g_pitch_gyro_bias_sum_dps = 0.0f;
static float g_yaw_gyro_bias_sum_dps = 0.0f;
static uint32_t g_gyro_bias_stationary_sample_count = 0U;
static uint8_t g_gyro_bias_ready = 0U;

extern ADC_HandleTypeDef hadc1;

#define APP_BOOT_LOG_SIZE 2048U
static char g_boot_log_buffer[APP_BOOT_LOG_SIZE];
static uint16_t g_boot_log_pos = 0U;

/* Temporary high-rate (500Hz) raw rate-gyro capture for noise/vibration diagnosis.
 * Fills once per arm cycle, dumped over UART/USB only while disarmed via "GLOG DUMP". */
#define APP_GLOG_MAX_SAMPLES 2000U
static int16_t g_glog_gx_x10[APP_GLOG_MAX_SAMPLES];
static int16_t g_glog_gy_x10[APP_GLOG_MAX_SAMPLES];
static int16_t g_glog_gz_x10[APP_GLOG_MAX_SAMPLES];
static volatile uint16_t g_glog_count = 0U;
static volatile uint8_t g_glog_capturing = 0U;
static volatile uint8_t g_glog_dump_pending = 0U;
static volatile uint8_t g_glog_armed_state = 0U;
/* Runtime toggle for the high-volume bench IMU/NAV telemetry stream while armed (blocking
 * UART6 writes add control-loop latency, so this defaults off/safe for real flight and is
 * only turned on for prop-off bench diagnostics via App_RequestArmedTelemetryEnabled()). */
static volatile uint8_t g_armed_test_telemetry_enabled = 0U;

void App_RequestGyroLogDump(void)
{
  g_glog_dump_pending = 1U;
}

/* SD commands block on HAL_Delay()/HAL_GetTick() timeouts internally. The USB
 * OTG_FS interrupt runs at NVIC priority 0 (highest) while SysTick runs at
 * priority 15 (lowest, see TICK_INT_PRIORITY), so calling them directly from
 * the USB RX callback (interrupt context) freezes HAL_GetTick() forever and
 * deadlocks the whole MCU. Defer them to this main-loop service instead. */
typedef enum
{
  APP_SD_CMD_NONE = 0,
  APP_SD_CMD_INIT,
  APP_SD_CMD_STATUS,
  APP_SD_CMD_RBLOCK,
  APP_SD_CMD_WBLOCK
} App_SdCmd_t;

static volatile App_SdCmd_t g_sd_cmd_pending = APP_SD_CMD_NONE;
static volatile uint32_t g_sd_cmd_block = 0U;

void App_RequestSdInit(void)
{
  g_sd_cmd_pending = APP_SD_CMD_INIT;
}

void App_RequestSdStatus(void)
{
  g_sd_cmd_pending = APP_SD_CMD_STATUS;
}

void App_RequestSdReadBlock(uint32_t block)
{
  g_sd_cmd_block = block;
  g_sd_cmd_pending = APP_SD_CMD_RBLOCK;
}

void App_RequestSdWriteBlock(uint32_t block)
{
  g_sd_cmd_block = block;
  g_sd_cmd_pending = APP_SD_CMD_WBLOCK;
}

static void App_SdLogLoadSuperblock(void);

static void App_ServiceSdCommands(void)
{
  App_SdCmd_t cmd = g_sd_cmd_pending;
  uint32_t block;
  SD_Status st;
  uint8_t buf[SD_BLOCK_SIZE];
  uint16_t i;

  if (cmd == APP_SD_CMD_NONE)
  {
    return;
  }
  g_sd_cmd_pending = APP_SD_CMD_NONE;

  switch (cmd)
  {
  case APP_SD_CMD_INIT:
    st = SD_Init();
    if (st == SD_OK)
    {
      App_SdLogLoadSuperblock();
    }
    printf("SD_INIT[status=%d hc=%u]\r\n", (int)st, (unsigned int)SD_IsHighCapacity());
    break;

  case APP_SD_CMD_STATUS:
    printf("SD_STATUS[init=%u hc=%u]\r\n", (unsigned int)SD_IsInitialized(), (unsigned int)SD_IsHighCapacity());
    break;

  case APP_SD_CMD_RBLOCK:
    block = g_sd_cmd_block;
    st = SD_ReadBlock(block, buf);
    if (st != SD_OK)
    {
      printf("SD_RBLOCK[status=%d]\r\n", (int)st);
      break;
    }
    printf("SD_RBLOCK[block=%lu]=[", (unsigned long)block);
    for (i = 0U; i < SD_BLOCK_SIZE; i++)
    {
      printf("%02X", buf[i]);
    }
    printf("]\r\n");
    break;

  case APP_SD_CMD_WBLOCK:
    block = g_sd_cmd_block;
    memset(buf, 0xA5, sizeof(buf));
    st = SD_WriteBlock(block, buf);
    printf("SD_WBLOCK[block=%lu status=%d]\r\n", (unsigned long)block, (int)st);
    break;

  default:
    break;
  }
}

/* Continuous flight data logger, written directly to the SD card so a whole
 * flight (not just the last few seconds of RAM) can be pulled and analyzed
 * later with tools/sdlog_analyze.py. Block 0 is a small superblock holding
 * the next free data block, so each flight (and each reboot) appends after
 * the last one instead of overwriting it. Records are buffered one SD block
 * (16 records) at a time; a full block is handed off to SD_WriteBlockBegin()
 * and polled asynchronously via App_SdLogServiceWrite() (see below) so the
 * card's internal flash-program time - measured up to ~65ms on this card,
 * previously a real control-loop stall that showed up as uncommanded-looking
 * attitude transients in flight logs - no longer blocks App_Update(). */
#define APP_SDLOG_MAGIC 0x4B484C47UL /* "KHLG" */
#define APP_SDLOG_DECIMATION 10U /* every 10th 2ms control tick -> 50Hz log rate */
#define APP_SDLOG_FLAG_ARMED 0x01U
#define APP_SDLOG_FLAG_MODE_SHIFT 1U
#define APP_SDLOG_FLAG_MODE_MASK 0x06U
#define APP_SDLOG_FLAG_LINK_ACTIVE 0x08U /* receiver_state.link_active, to tell a switch glitch apart from an RF link dropout */
#define APP_SDLOG_FLAG_MAG_HEALTHY 0x10U

#define APP_SDLOG_NAV_FLAG_REQUESTED    0x01U
#define APP_SDLOG_NAV_FLAG_ACTIVE       0x02U
#define APP_SDLOG_NAV_FLAG_TILT_LIMITED 0x04U
#define APP_SDLOG_NAV_FLAG_ACCEL_LIMITED 0x08U
#define APP_SDLOG_NAV_FLAG_NEW_SAMPLE   0x10U
#define APP_SDLOG_NAV_FLAG_REJECTED     0x20U
#define APP_SDLOG_NAV_FLAG_REF_VALID    0x40U

/* Record grew from the original 32 bytes as fields were added over time (see repo
 * history) - matches tools/sdlog_analyze.py's struct format, which must be updated
 * in lockstep with any change here. */
typedef struct __attribute__((packed))
{
  uint32_t time_ms;
  int16_t setpoint_roll_dps;
  int16_t setpoint_pitch_dps;
  int16_t setpoint_yaw_dps;
  int16_t gyro_roll_dps_x10;
  int16_t gyro_pitch_dps_x10;
  int16_t gyro_yaw_dps_x10;
  int16_t pid_roll_us;
  int16_t pid_pitch_us;
  int16_t pid_yaw_us;
  uint16_t motor_fl_us;
  uint16_t motor_fr_us;
  uint16_t motor_rr_us;
  uint16_t motor_rl_us;
  uint8_t battery_decivolts;
  uint8_t flags;
  int16_t pitch_deg_x10;
  int16_t roll_deg_x10;
  int16_t target_pitch_deg_x10;
  int16_t target_roll_deg_x10;
  int16_t baro_alt_cm;
  int16_t baro_vz_cms;
  int16_t throttle_cmd_us;
  int16_t throttle_actual_us;
  uint16_t arm_us;
  int16_t yaw_deg_x10;
  uint16_t mag_heading_x10;
  /* --- GPS navigation foundation / NAV_VELOCITY_BRAKE fields (added with the
   * GPS nav phase-1 feature) --- */
  uint8_t nav_flags;             /* APP_SDLOG_NAV_FLAG_* bits */
  uint8_t nav_invalid_reason;    /* Nav_InvalidReason_t */
  uint8_t nav_fix_type;
  uint8_t nav_num_sv;
  uint16_t nav_h_acc_cm;
  uint16_t nav_age_ms;
  uint16_t nav_update_period_ms;
  uint16_t nav_consecutive_valid;
  uint16_t nav_dropout_count;
  int16_t nav_north_m_x10;
  int16_t nav_east_m_x10;
  int16_t nav_raw_vel_n_x100;
  int16_t nav_raw_vel_e_x100;
  int16_t nav_filt_vel_n_x100;
  int16_t nav_filt_vel_e_x100;
  int16_t nav_desired_vel_n_x100;
  int16_t nav_desired_vel_e_x100;
  int16_t nav_vel_error_n_x100;
  int16_t nav_vel_error_e_x100;
  int16_t nav_accel_cmd_n_x1000;
  int16_t nav_accel_cmd_e_x1000;
  int16_t nav_accel_cmd_fwd_x1000;
  int16_t nav_accel_cmd_right_x1000;
  int16_t pilot_roll_stick_us;
  int16_t pilot_pitch_stick_us;
} App_SdLogRecord_t;

#define APP_SDLOG_RECORDS_PER_BLOCK (SD_BLOCK_SIZE / sizeof(App_SdLogRecord_t))

static uint8_t g_sdlog_ready = 0U;      /* superblock loaded, card usable for logging */
static uint8_t g_sdlog_active = 0U;     /* currently capturing a flight */
static uint32_t g_sdlog_next_free_block = 1U;
static uint32_t g_sdlog_last_flight_start_block = 1U; /* first block of the most recent flight, for SDLOG DUMP LAST */
static uint32_t g_sdlog_flight_next_block = 0U;
static uint8_t g_sdlog_block_buf[SD_BLOCK_SIZE];
static uint16_t g_sdlog_buf_count = 0U;

/* Second buffer + pending flag let a full block be handed off to the card
 * asynchronously (see App_SdLogServiceWrite()) while new records keep
 * accumulating in g_sdlog_block_buf, so the card's internal flash-program
 * time (the slow/variable part, previously a multi-tens-of-ms control-loop
 * stall) is polled a byte at a time from the main loop instead of blocking it. */
static uint8_t g_sdlog_write_buf[SD_BLOCK_SIZE];
static uint8_t g_sdlog_write_pending = 0U;

static void App_SdLogServiceWrite(void)
{
  if ((g_sdlog_write_pending != 0U) && (SD_WriteBlockPoll() >= 0))
  {
    g_sdlog_write_pending = 0U;
  }
}

static void App_SdLogAwaitWrite(void)
{
  while (g_sdlog_write_pending != 0U)
  {
    App_SdLogServiceWrite();
  }
}

static void App_SdLogFlushBufferAsync(void)
{
  /* Never wait on storage from the armed control path. If the card has not
   * completed the previous block, drop this block instead of delaying motor
   * control or preventing an arm-switch disarm from being processed. */
  if (g_sdlog_write_pending != 0U)
  {
    return;
  }

  memcpy(g_sdlog_write_buf, g_sdlog_block_buf, sizeof(g_sdlog_write_buf));
  if (SD_WriteBlockBegin(g_sdlog_flight_next_block, g_sdlog_write_buf) == SD_OK)
  {
    g_sdlog_write_pending = 1U;
    g_sdlog_flight_next_block++;
  }
}

static void App_SdLogSaveSuperblock(void)
{
  uint8_t buf[SD_BLOCK_SIZE] = {0};
  uint32_t magic = APP_SDLOG_MAGIC;

  memcpy(&buf[0], &magic, sizeof(magic));
  memcpy(&buf[4], &g_sdlog_next_free_block, sizeof(g_sdlog_next_free_block));
  memcpy(&buf[8], &g_sdlog_last_flight_start_block, sizeof(g_sdlog_last_flight_start_block));
  (void)SD_WriteBlock(0U, buf);
}

static void App_SdLogLoadSuperblock(void)
{
  uint8_t buf[SD_BLOCK_SIZE];
  uint32_t magic;

  g_sdlog_ready = 0U;
  g_sdlog_next_free_block = 1U;
  g_sdlog_last_flight_start_block = 1U;

  if (SD_ReadBlock(0U, buf) != SD_OK)
  {
    return;
  }

  memcpy(&magic, &buf[0], sizeof(magic));
  if (magic == APP_SDLOG_MAGIC)
  {
    memcpy(&g_sdlog_next_free_block, &buf[4], sizeof(g_sdlog_next_free_block));
    if (g_sdlog_next_free_block < 1U)
    {
      g_sdlog_next_free_block = 1U;
    }
    /* Older superblocks (written before SDLOG DUMP LAST existed) don't have this
     * field - it reads back as 0 from the zero-filled buffer, handled below. */
    memcpy(&g_sdlog_last_flight_start_block, &buf[8], sizeof(g_sdlog_last_flight_start_block));
    if ((g_sdlog_last_flight_start_block < 1U) || (g_sdlog_last_flight_start_block >= g_sdlog_next_free_block))
    {
      g_sdlog_last_flight_start_block = 1U;
    }
  }
  else
  {
    /* Blank/foreign card - claim it with a fresh superblock. */
    g_sdlog_next_free_block = 1U;
    App_SdLogSaveSuperblock();
  }
  g_sdlog_ready = 1U;
}

static void App_SdLogArmStart(void)
{
  if (g_sdlog_ready != 0U)
  {
    g_sdlog_flight_next_block = g_sdlog_next_free_block;
    g_sdlog_last_flight_start_block = g_sdlog_next_free_block;
    g_sdlog_buf_count = 0U;
    g_sdlog_active = 1U;
  }
}

static void App_SdLogFlushFlight(void)
{
  /* Disarm already stops motor output, so blocking here doesn't cost control timing. */
  App_SdLogAwaitWrite();

  if (g_sdlog_buf_count > 0U)
  {
    memset(&g_sdlog_block_buf[g_sdlog_buf_count * sizeof(App_SdLogRecord_t)],
           0,
           sizeof(g_sdlog_block_buf) - (g_sdlog_buf_count * sizeof(App_SdLogRecord_t)));
    (void)SD_WriteBlock(g_sdlog_flight_next_block, g_sdlog_block_buf);
    g_sdlog_flight_next_block++;
    g_sdlog_buf_count = 0U;
  }
  g_sdlog_active = 0U;
  g_sdlog_next_free_block = g_sdlog_flight_next_block;
  App_SdLogSaveSuperblock();
}

static void App_SdLogAppendRecord(const App_SdLogRecord_t *rec)
{
  if ((g_sdlog_active == 0U) || (g_sdlog_ready == 0U))
  {
    return;
  }

  memcpy(&g_sdlog_block_buf[g_sdlog_buf_count * sizeof(App_SdLogRecord_t)], rec, sizeof(*rec));
  g_sdlog_buf_count++;

  if (g_sdlog_buf_count >= APP_SDLOG_RECORDS_PER_BLOCK)
  {
    App_SdLogFlushBufferAsync();
    g_sdlog_buf_count = 0U;
  }
}

/* Full-rate (undecimated) rolling black-box buffer of the most recent stretch of
 * flight, kept in RAM_D3 (0x38000000, 64KB - unused by anything else in this
 * project; contents survive an IWDG/software reset, only lost on a true power
 * cycle). Captured every armed control-loop iteration alongside (not instead of)
 * the normal decimated persisted SD log, so a mid-flight hang/crash that the
 * ~50Hz log would only catch coarsely can still be inspected at full loop rate
 * on the next boot. Recovered records are written out as their own "flight"
 * segment in the persisted SD log (via the same arm/append/flush path used for a
 * real flight), so existing tooling (SDLOG DUMP LAST, sdlog_analyze.py) needs no
 * changes to see them. */
#define APP_BLACKBOX_RING_ADDR     0x38000200UL
#define APP_BLACKBOX_RING_MAGIC    0x4B42524Cu /* 'KBRL' */
#define APP_BLACKBOX_RING_CAPACITY 400U

typedef struct __attribute__((packed))
{
  uint32_t magic;
  uint32_t write_index;
  uint32_t count;
  App_SdLogRecord_t ring[APP_BLACKBOX_RING_CAPACITY];
} App_BlackboxRing_t;

#define APP_BLACKBOX_RING ((volatile App_BlackboxRing_t *)APP_BLACKBOX_RING_ADDR)

static void App_BlackboxCapture(const App_SdLogRecord_t *rec)
{
  volatile App_BlackboxRing_t *bb = APP_BLACKBOX_RING;

  if (bb->magic != APP_BLACKBOX_RING_MAGIC)
  {
    bb->magic = APP_BLACKBOX_RING_MAGIC;
    bb->write_index = 0U;
    bb->count = 0U;
  }

  memcpy((void *)&bb->ring[bb->write_index], rec, sizeof(*rec));
  bb->write_index = (bb->write_index + 1U) % APP_BLACKBOX_RING_CAPACITY;
  if (bb->count < APP_BLACKBOX_RING_CAPACITY)
  {
    bb->count++;
  }
}

/* Called once at boot (after SD_Init()/App_SdLogLoadSuperblock()) - if the previous
 * session left a valid ring behind, writes it out as a new flight segment. */
static void App_BlackboxDumpIfPresent(void)
{
  volatile App_BlackboxRing_t *bb = APP_BLACKBOX_RING;
  uint32_t n;
  uint32_t start_idx;
  uint32_t i;

  if ((bb->magic != APP_BLACKBOX_RING_MAGIC) || (bb->count == 0U))
  {
    return;
  }

  n = bb->count;
  start_idx = (n >= APP_BLACKBOX_RING_CAPACITY) ? bb->write_index : 0U;

  printf("BLACKBOX[records=%lu]\r\n", (unsigned long)n);

  App_SdLogArmStart();
  for (i = 0U; i < n; i++)
  {
    App_SdLogRecord_t rec;
    uint32_t idx = (start_idx + i) % APP_BLACKBOX_RING_CAPACITY;

    memcpy(&rec, (const void *)&bb->ring[idx], sizeof(rec));
    App_SdLogAppendRecord(&rec);
  }
  App_SdLogFlushFlight();

  bb->magic = 0U;
}

/* Reports (over UART6) and clears the persistent fault record left by stm32h7xx_it.c's
 * fault handlers, if any - a reprint at boot in case nothing was listening live. */
static void App_ReportAndClearFaultRecord(void)
{
  volatile FaultRecord_t *rec = FAULT_RECORD;
  char name[sizeof(rec->name)];

  if (rec->magic != FAULT_RECORD_MAGIC)
  {
    return;
  }

  memcpy(name, (const void *)rec->name, sizeof(name));
  printf("FAULT_PERSISTED[%s] pc=0x%08lX lr=0x%08lX cfsr=0x%08lX hfsr=0x%08lX mmfar=0x%08lX bfar=0x%08lX\r\n",
         name, (unsigned long)rec->pc, (unsigned long)rec->lr, (unsigned long)rec->cfsr,
         (unsigned long)rec->hfsr, (unsigned long)rec->mmfar, (unsigned long)rec->bfar);

  rec->magic = 0U;
}

typedef enum
{
  APP_SDLOGCMD_NONE = 0,
  APP_SDLOGCMD_STATUS,
  APP_SDLOGCMD_DUMP,
  APP_SDLOGCMD_DUMP_LAST,
  APP_SDLOGCMD_ERASE
} App_SdLogCmd_t;

#define APP_SDLOG_DUMP_BLOCKS_PER_CALL 4U
/* Used to gate how many blocks get pushed into the UART6 TX queue per
 * App_Update() call against how much room is actually free (see
 * APP_SDLOG_DUMP_BLOCKS_PER_CALL's cap above) - at 115200 baud the 2048-byte
 * queue drains far slower than this loop can fill it, so pushing blindly up
 * to the fixed per-call cap regardless of backlog (found 2026-08-15) silently
 * dropped most of every block after the first one or two via the queue's own
 * "drop when full" overflow handling.
 *
 * One block's printed line is "SDLOG[" + up to 5 digits + "]=" + 1024 hex
 * chars + "\r\n", up to ~1039 bytes - but requiring only that much headroom
 * (tried 2026-08-15) still let occasional blocks come back partially
 * truncated: nothing else can push into this queue while one block is being
 * printed (this loop is a single uninterrupted main-loop call), so the
 * margin should have held, but it was too close to the worst case to be
 * confident it always does. Requiring the queue to be essentially fully
 * drained first removes that doubt entirely, at the cost of a small amount of
 * pipelining - acceptable since this dump is already deliberately spread
 * across many App_Update() calls rather than optimized for raw throughput. */
#define APP_SDLOG_DUMP_QUEUE_FREE_THRESHOLD_BYTES 2000U

static volatile App_SdLogCmd_t g_sdlog_cmd_pending = APP_SDLOGCMD_NONE;
static uint8_t g_sdlog_dump_active = 0U;
static uint32_t g_sdlog_dump_block = 0U;
static uint32_t g_sdlog_dump_end_block = 0U;

void App_RequestSdLogStatus(void)
{
  g_sdlog_cmd_pending = APP_SDLOGCMD_STATUS;
}

void App_RequestSdLogDump(void)
{
  g_sdlog_cmd_pending = APP_SDLOGCMD_DUMP;
}

void App_RequestSdLogDumpLast(void)
{
  g_sdlog_cmd_pending = APP_SDLOGCMD_DUMP_LAST;
}

void App_RequestSdLogErase(void)
{
  g_sdlog_cmd_pending = APP_SDLOGCMD_ERASE;
}

void App_RequestArmedTelemetryEnabled(uint8_t enabled)
{
  g_armed_test_telemetry_enabled = (enabled != 0U) ? 1U : 0U;
  printf("TELARM[%s]\r\n", (g_armed_test_telemetry_enabled != 0U) ? "ON" : "OFF");
}

void App_PrintArmedTelemetryStatus(void)
{
  /* Read-only echo of the current flag - lets any client (field tester, GUI) resync its
   * displayed/base toggle state without side effects, in case another client changed it. */
  printf("TELARM[%s]\r\n", (g_armed_test_telemetry_enabled != 0U) ? "ON" : "OFF");
}

static void App_ServiceSdLog(void)
{
  App_SdLogCmd_t cmd = g_sdlog_cmd_pending;
  uint8_t buf[SD_BLOCK_SIZE];
  uint32_t i;

  App_SdLogServiceWrite();

  if (cmd != APP_SDLOGCMD_NONE)
  {
    g_sdlog_cmd_pending = APP_SDLOGCMD_NONE;

    if (cmd == APP_SDLOGCMD_STATUS)
    {
      printf("SDLOG_STATUS[ready=%u next_free_block=%lu record_bytes=%u]\r\n",
             (unsigned int)g_sdlog_ready,
             (unsigned long)g_sdlog_next_free_block,
             (unsigned int)sizeof(App_SdLogRecord_t));
    }
    else if (cmd == APP_SDLOGCMD_ERASE)
    {
      /* Not a real physical erase (12000+ blocks would take forever) - just rewinds
       * the superblock's write pointer to block 1 so the next flight starts
       * overwriting from the beginning, same effect for SDLOG DUMP's purposes. */
      if (g_glog_armed_state != 0U)
      {
        printf("SDLOG_ERASE[FAIL armed]\r\n");
      }
      else
      {
        g_sdlog_next_free_block = 1U;
        g_sdlog_last_flight_start_block = 1U;
        g_sdlog_active = 0U;
        g_sdlog_buf_count = 0U;
        g_sdlog_write_pending = 0U;
        App_SdLogSaveSuperblock();
        printf("SDLOG_ERASE[OK]\r\n");
      }
    }
    else if ((cmd == APP_SDLOGCMD_DUMP) || (cmd == APP_SDLOGCMD_DUMP_LAST))
    {
      if (g_glog_armed_state != 0U)
      {
        printf("SDLOG_DUMP[BUSY armed]\r\n");
      }
      else if ((g_sdlog_ready == 0U) || (g_sdlog_next_free_block <= 1U))
      {
        printf("SDLOG_DUMP[EMPTY]\r\n");
      }
      else
      {
        /* DUMP LAST starts at the most recent flight's first block instead of
         * block 1, so pulling recent data doesn't re-stream the entire card's
         * history every time (that grows every arm and never gets shorter). */
        g_sdlog_dump_block = (cmd == APP_SDLOGCMD_DUMP_LAST) ? g_sdlog_last_flight_start_block : 1U;
        g_sdlog_dump_end_block = g_sdlog_next_free_block - 1U;
        g_sdlog_dump_active = 1U;
        printf("SDLOG_DUMP[START first=%lu last=%lu record_bytes=%u]\r\n",
               (unsigned long)g_sdlog_dump_block,
               (unsigned long)g_sdlog_dump_end_block,
               (unsigned int)sizeof(App_SdLogRecord_t));
      }
    }
  }

  /* Dumping is chunked across many App_Update() calls (a few blocks at a
   * time, further throttled below against actual UART6 TX queue headroom)
   * instead of one long synchronous loop, so a large dump never stalls
   * telemetry/receiver servicing for more than a few SD block reads. */
  for (i = 0U; (i < APP_SDLOG_DUMP_BLOCKS_PER_CALL) && (g_sdlog_dump_active != 0U); i++)
  {
    uint16_t b;

    /* Historical note (2026-08-15): a SDLOG DUMP block's hex output used to come
     * back over the WiFi bridge with other telemetry lines spliced into the
     * MIDDLE of a block's 1024 hex characters. Root-caused and fixed in
     * Communications_QueuePush()/Pop() (communications.c) - the UART6 TX ring
     * buffer's head/tail read-modify-write wasn't atomic against
     * HAL_UART_TxCpltCallback re-kicking the same queue from ISR context, so
     * under this loop's sustained back-to-back byte pushes the drain side could
     * desync and start reading stale bytes from an earlier message still sitting
     * in the reused buffer array. Never reproduced over USB because USB CDC
     * bypasses this queue entirely (see Communications_FlushUsbTxBuffer()).
     *
     * That fix stopped the splicing, but exposed a second, separate problem:
     * with the queue now correctly ordered, most of every block past the
     * first one or two came back cleanly TRUNCATED instead - the queue is
     * simply too small (2048 bytes) and too slow to drain (115200 baud) to
     * absorb APP_SDLOG_DUMP_BLOCKS_PER_CALL blocks (~4KB) every 2ms, so it
     * fills and starts dropping almost immediately regardless of ordering.
     * Stop pushing more blocks this call once there isn't comfortably enough
     * room left for one more - the remaining blocks are simply picked up on a
     * later call once the ISR has had real time to drain what's queued. */
    if (Communications_Uart6TxQueueFreeBytes() < APP_SDLOG_DUMP_QUEUE_FREE_THRESHOLD_BYTES)
    {
      break;
    }

    if (SD_ReadBlock(g_sdlog_dump_block, buf) == SD_OK)
    {
      printf("SDLOG[%lu]=", (unsigned long)g_sdlog_dump_block);
      for (b = 0U; b < SD_BLOCK_SIZE; b++)
      {
        printf("%02X", buf[b]);
      }
      printf("\r\n");
    }
    else
    {
      printf("SDLOG_DUMP[READ_ERR block=%lu]\r\n", (unsigned long)g_sdlog_dump_block);
    }

    if (g_sdlog_dump_block >= g_sdlog_dump_end_block)
    {
      g_sdlog_dump_active = 0U;
      printf("SDLOG_DUMP[END]\r\n");
    }
    else
    {
      g_sdlog_dump_block++;
    }
  }
}

static void App_ServiceGyroLogDump(void)
{
  uint16_t i;

  if (g_glog_dump_pending == 0U)
  {
    return;
  }
  g_glog_dump_pending = 0U;

  if (g_glog_armed_state != 0U)
  {
    printf("GLOG[BUSY armed]\r\n");
    return;
  }

  printf("GLOG[START count=%u rate_hz=%lu]\r\n",
         (unsigned int)g_glog_count,
         (unsigned long)(1000U / APP_CONTROL_LOOP_MS));
  for (i = 0U; i < g_glog_count; i++)
  {
    printf("GLOG[%u]=[%d %d %d]\r\n",
           (unsigned int)i,
           (int)g_glog_gx_x10[i],
           (int)g_glog_gy_x10[i],
           (int)g_glog_gz_x10[i]);
  }
  printf("GLOG[END]\r\n");
}

static uint16_t App_CrsfRawToUs(uint16_t raw)
{
  uint32_t scaled;

  if (raw <= APP_CRSF_MIN_RAW)
  {
    return APP_PWM_MIN_US;
  }

  if (raw >= APP_CRSF_MAX_RAW)
  {
    return APP_PWM_MAX_US;
  }

  scaled = (uint32_t)(raw - APP_CRSF_MIN_RAW) * (APP_PWM_MAX_US - APP_PWM_MIN_US);
  scaled = (scaled / (APP_CRSF_MAX_RAW - APP_CRSF_MIN_RAW)) + APP_PWM_MIN_US;
  return (uint16_t)scaled;
}

static uint8_t App_UpdateGyroBias(float ax_g,
                                  float ay_g,
                                  float az_g,
                                  float gx_dps,
                                  float gy_dps,
                                  float gz_dps,
                                  uint8_t motors_armed,
                                  uint16_t throttle_us)
{
  float accel_mag_sq;
  float accel_min_sq;
  float accel_max_sq;
  uint8_t stationary;

  if ((motors_armed != 0U) && (throttle_us > APP_THROTTLE_LOW_US))
  {
    return 0U;
  }

  accel_mag_sq = (ax_g * ax_g) + (ay_g * ay_g) + (az_g * az_g);
  accel_min_sq = (1.0f - APP_ACCEL_STILL_TOL_G) * (1.0f - APP_ACCEL_STILL_TOL_G);
  accel_max_sq = (1.0f + APP_ACCEL_STILL_TOL_G) * (1.0f + APP_ACCEL_STILL_TOL_G);
  stationary = ((accel_mag_sq >= accel_min_sq) &&
                (accel_mag_sq <= accel_max_sq) &&
                (gx_dps > -APP_GYRO_STILL_DPS) && (gx_dps < APP_GYRO_STILL_DPS) &&
                (gy_dps > -APP_GYRO_STILL_DPS) && (gy_dps < APP_GYRO_STILL_DPS) &&
                (gz_dps > -APP_GYRO_STILL_DPS) && (gz_dps < APP_GYRO_STILL_DPS)) ? 1U : 0U;

  if (stationary == 0U)
  {
    return 0U;
  }

  if (g_gyro_bias_ready == 0U)
  {
    g_roll_gyro_bias_sum_dps += gx_dps;
    g_pitch_gyro_bias_sum_dps += gy_dps;
    g_yaw_gyro_bias_sum_dps += gz_dps;
    g_gyro_bias_stationary_sample_count++;

    if (g_gyro_bias_stationary_sample_count >= APP_YAW_BIAS_SETTLE_SAMPLES)
    {
      g_roll_gyro_bias_dps = g_roll_gyro_bias_sum_dps / ((float)g_gyro_bias_stationary_sample_count);
      g_pitch_gyro_bias_dps = g_pitch_gyro_bias_sum_dps / ((float)g_gyro_bias_stationary_sample_count);
      g_yaw_gyro_bias_dps = g_yaw_gyro_bias_sum_dps / ((float)g_gyro_bias_stationary_sample_count);
      g_gyro_bias_ready = 1U;
    }
  }
  else
  {
    g_roll_gyro_bias_dps = ((1.0f - APP_YAW_BIAS_ALPHA) * g_roll_gyro_bias_dps) + (APP_YAW_BIAS_ALPHA * gx_dps);
    g_pitch_gyro_bias_dps = ((1.0f - APP_YAW_BIAS_ALPHA) * g_pitch_gyro_bias_dps) + (APP_YAW_BIAS_ALPHA * gy_dps);
    g_yaw_gyro_bias_dps = ((1.0f - APP_YAW_BIAS_ALPHA) * g_yaw_gyro_bias_dps) + (APP_YAW_BIAS_ALPHA * gz_dps);
  }

  return g_gyro_bias_ready;
}

static App_FlightMode_t App_SelectFlightMode(uint16_t mode_us)
{
  if (mode_us > APP_NAV_BRAKE_SWITCH_THRESHOLD_US)
  {
    return APP_FLIGHT_MODE_NAV_VELOCITY_BRAKE;
  }

  if (mode_us >= APP_ALTHOLD_SWITCH_THRESHOLD_US)
  {
    return APP_FLIGHT_MODE_ALTHOLD;
  }

  if (mode_us < APP_MODE_SWITCH_THRESHOLD_US)
  {
    return APP_FLIGHT_MODE_RATE;
  }

  return APP_FLIGHT_MODE_ATTITUDE;
}

static const char *App_FlightModeName(App_FlightMode_t mode)
{
  switch (mode)
  {
    case APP_FLIGHT_MODE_NAV_VELOCITY_BRAKE:
      return "NAVBRAKE";
    case APP_FLIGHT_MODE_ALTHOLD:
      return "ALTHOLD";
    case APP_FLIGHT_MODE_ATTITUDE:
      return "ATTITUDE";
    case APP_FLIGHT_MODE_RATE:
    default:
      return "RATE";
  }
}

static float App_StickOffsetUsToAngleDeg(int32_t stick_offset_us, float max_angle_deg)
{
  float normalized;

  if (stick_offset_us > 0)
  {
    if (stick_offset_us <= (int32_t)APP_CONTROL_DEADBAND_US)
    {
      stick_offset_us = 0;
    }
    else
    {
      stick_offset_us -= (int32_t)APP_CONTROL_DEADBAND_US;
    }
  }
  else if (stick_offset_us < 0)
  {
    if (stick_offset_us >= -(int32_t)APP_CONTROL_DEADBAND_US)
    {
      stick_offset_us = 0;
    }
    else
    {
      stick_offset_us += (int32_t)APP_CONTROL_DEADBAND_US;
    }
  }

  normalized = ((float)stick_offset_us) / ((float)((int32_t)APP_PWM_MAX_US - (int32_t)APP_PWM_MID_US));

  if (normalized > 1.0f)
  {
    normalized = 1.0f;
  }
  else if (normalized < -1.0f)
  {
    normalized = -1.0f;
  }

  return normalized * max_angle_deg;
}

static uint8_t App_ReadBatteryVoltage(float *battery_voltage_v, uint32_t *adc_raw)
{
  ADC_ChannelConfTypeDef sConfig;
  uint32_t sum_samples;
  uint32_t sample;
  uint32_t throwaway_sample;
  uint32_t i;
  float adc_pin_voltage_v;

  if ((battery_voltage_v == NULL) || (adc_raw == NULL))
  {
    return 0U;
  }

  sConfig.Channel = APP_BATTERY_CHANNEL;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_387CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    return 0U;
  }

  if (HAL_ADC_Start(&hadc1) != HAL_OK)
  {
    return 0U;
  }

  if (HAL_ADC_PollForConversion(&hadc1, 3U) != HAL_OK)
  {
    (void)HAL_ADC_Stop(&hadc1);
    return 0U;
  }

  throwaway_sample = HAL_ADC_GetValue(&hadc1);
  (void)throwaway_sample;
  (void)HAL_ADC_Stop(&hadc1);

  sum_samples = 0U;
  for (i = 0U; i < APP_BATTERY_ADC_SAMPLES; i++)
  {
    if (HAL_ADC_Start(&hadc1) != HAL_OK)
    {
      return 0U;
    }

    if (HAL_ADC_PollForConversion(&hadc1, 3U) != HAL_OK)
    {
      (void)HAL_ADC_Stop(&hadc1);
      return 0U;
    }

    sum_samples += HAL_ADC_GetValue(&hadc1);
    (void)HAL_ADC_Stop(&hadc1);
  }

  sample = sum_samples / APP_BATTERY_ADC_SAMPLES;

  adc_pin_voltage_v = (((float)sample) / APP_ADC_MAX_COUNT) * APP_ADC_REF_V;
  *battery_voltage_v = adc_pin_voltage_v * APP_BATTERY_DIVIDER_RATIO;
  *adc_raw = sample;
  return 1U;
}

static uint16_t App_ClampPulseUs(int32_t pulse_us)
{
  if (pulse_us < (int32_t)APP_PWM_MIN_US)
  {
    return APP_PWM_MIN_US;
  }

  if (pulse_us > (int32_t)APP_PWM_MAX_US)
  {
    return APP_PWM_MAX_US;
  }

  return (uint16_t)pulse_us;
}

static int32_t App_ApplyDeadbandUs(int32_t value, int32_t deadband_us)
{
  if (value > 0)
  {
    if (value <= deadband_us)
    {
      return 0;
    }

    value -= deadband_us;
  }
  else if (value < 0)
  {
    if (value >= -deadband_us)
    {
      return 0;
    }

    value += deadband_us;
  }

  return value;
}

static float App_ClampFloat(float value, float min_value, float max_value)
{
  if (value < min_value)
  {
    return min_value;
  }

  if (value > max_value)
  {
    return max_value;
  }

  return value;
}

/* Single-pole EMA alpha derived from the actual sample time so the filter holds its
 * intended cutoff even when the loop period drifts (SD writes, telemetry, jitter). */
static float App_LpfAlpha(float dt_s, float cutoff_hz)
{
  float tau_s = 1.0f / (2.0f * 3.14159265f * cutoff_hz);
  return 1.0f - expf(-dt_s / tau_s);
}

static float App_ComputeVoltageCompFactor(float battery_voltage_v)
{
  if (battery_voltage_v < 1.0f)
  {
    return 1.0f;
  }

  return App_ClampFloat(APP_VOLTAGE_COMP_REFERENCE_V / battery_voltage_v,
                        APP_VOLTAGE_COMP_FACTOR_MIN,
                        APP_VOLTAGE_COMP_FACTOR_MAX);
}

static float App_StickOffsetUsToRateDps(int32_t stick_offset_us, float max_rate_dps)
{
  float normalized;

  stick_offset_us = App_ApplyDeadbandUs(stick_offset_us, (int32_t)APP_CONTROL_DEADBAND_US);
  normalized = ((float)stick_offset_us) / ((float)((int32_t)APP_PWM_MAX_US - (int32_t)APP_PWM_MID_US));

  if (normalized > 1.0f)
  {
    normalized = 1.0f;
  }
  else if (normalized < -1.0f)
  {
    normalized = -1.0f;
  }

  return normalized * max_rate_dps;
}

/* NAV_VELOCITY_BRAKE pilot stick mapping: same shape as App_StickOffsetUsToRateDps()
 * (deadband + linear normalize to +-1) but scaled to a velocity instead of a rate. */
static float App_NavStickOffsetToVelocityMps(int32_t stick_offset_us, float max_vel_mps)
{
  float normalized;

  stick_offset_us = App_ApplyDeadbandUs(stick_offset_us, (int32_t)APP_NAV_BRAKE_STICK_DEADBAND_US);
  normalized = ((float)stick_offset_us) / ((float)((int32_t)APP_PWM_MAX_US - (int32_t)APP_PWM_MID_US));
  normalized = App_ClampFloat(normalized, -1.0f, 1.0f);

  return normalized * max_vel_mps;
}

static int32_t App_ClampControlTerm(int32_t value, int32_t limit)
{
  if (value > limit)
  {
    return limit;
  }

  if (value < -limit)
  {
    return -limit;
  }

  return value;
}

static int32_t App_ClampInt32(int32_t value, int32_t min_value, int32_t max_value)
{
  if (value < min_value)
  {
    return min_value;
  }

  if (value > max_value)
  {
    return max_value;
  }

  return value;
}

static uint32_t App_Crc32(const uint8_t *data, size_t len)
{
  uint32_t crc = 0xFFFFFFFFUL;
  size_t i;
  uint8_t bit;

  if (data == NULL)
  {
    return 0U;
  }

  for (i = 0U; i < len; i++)
  {
    crc ^= (uint32_t)data[i];
    for (bit = 0U; bit < 8U; bit++)
    {
      if ((crc & 1UL) != 0U)
      {
        crc = (crc >> 1U) ^ 0xEDB88320UL;
      }
      else
      {
        crc >>= 1U;
      }
    }
  }

  return ~crc;
}

static uint8_t App_IsFiniteInRange(float value, float min_value, float max_value)
{
  if (value != value)
  {
    return 0U;
  }

  if ((value < min_value) || (value > max_value))
  {
    return 0U;
  }

  return 1U;
}

static uint8_t App_AreRatePidGainsValid(const App_RatePidGains_t *gains)
{
  if (gains == NULL)
  {
    return 0U;
  }

  if ((App_IsFiniteInRange(gains->roll.kp, APP_RATE_KP_MIN_US_PER_DPS, APP_RATE_KP_MAX_US_PER_DPS) == 0U) ||
      (App_IsFiniteInRange(gains->pitch.kp, APP_RATE_KP_MIN_US_PER_DPS, APP_RATE_KP_MAX_US_PER_DPS) == 0U) ||
      (App_IsFiniteInRange(gains->yaw.kp, APP_RATE_KP_MIN_US_PER_DPS, APP_RATE_KP_MAX_US_PER_DPS) == 0U) ||
      (App_IsFiniteInRange(gains->roll.ki, APP_RATE_KI_MIN_US_PER_DPS_S, APP_RATE_KI_MAX_US_PER_DPS_S) == 0U) ||
      (App_IsFiniteInRange(gains->pitch.ki, APP_RATE_KI_MIN_US_PER_DPS_S, APP_RATE_KI_MAX_US_PER_DPS_S) == 0U) ||
      (App_IsFiniteInRange(gains->yaw.ki, APP_RATE_KI_MIN_US_PER_DPS_S, APP_RATE_KI_MAX_US_PER_DPS_S) == 0U) ||
      (App_IsFiniteInRange(gains->roll.kd, APP_RATE_KD_MIN_US_PER_DPS_PER_S, APP_RATE_KD_MAX_US_PER_DPS_PER_S) == 0U) ||
      (App_IsFiniteInRange(gains->pitch.kd, APP_RATE_KD_MIN_US_PER_DPS_PER_S, APP_RATE_KD_MAX_US_PER_DPS_PER_S) == 0U) ||
      (App_IsFiniteInRange(gains->yaw.kd, APP_RATE_KD_MIN_US_PER_DPS_PER_S, APP_RATE_KD_MAX_US_PER_DPS_PER_S) == 0U) ||
      (App_IsFiniteInRange(gains->roll.kff, APP_RATE_KFF_MIN_US_PER_DPS_PER_S, APP_RATE_KFF_MAX_US_PER_DPS_PER_S) == 0U) ||
      (App_IsFiniteInRange(gains->pitch.kff, APP_RATE_KFF_MIN_US_PER_DPS_PER_S, APP_RATE_KFF_MAX_US_PER_DPS_PER_S) == 0U) ||
      (App_IsFiniteInRange(gains->yaw.kff, APP_RATE_KFF_MIN_US_PER_DPS_PER_S, APP_RATE_KFF_MAX_US_PER_DPS_PER_S) == 0U))
  {
    return 0U;
  }

  return 1U;
}

static uint8_t App_AreAttitudeGainsValid(const App_AttitudeGains_t *gains)
{
  if (gains == NULL)
  {
    return 0U;
  }

  if ((App_IsFiniteInRange(gains->roll_kp, APP_ATTITUDE_KP_MIN_DPS_PER_DEG, APP_ATTITUDE_KP_MAX_DPS_PER_DEG) == 0U) ||
      (App_IsFiniteInRange(gains->pitch_kp, APP_ATTITUDE_KP_MIN_DPS_PER_DEG, APP_ATTITUDE_KP_MAX_DPS_PER_DEG) == 0U) ||
      (App_IsFiniteInRange(gains->max_angle_deg, APP_ATTITUDE_MAX_ANGLE_MIN_DEG, APP_ATTITUDE_MAX_ANGLE_MAX_DEG) == 0U))
  {
    return 0U;
  }

  return 1U;
}

void App_AppendBootLog(const char *str)
{
  size_t len;
  
  if ((str == NULL) || (g_boot_log_pos >= APP_BOOT_LOG_SIZE))
  {
    return;
  }

  len = strlen(str);
  if (len == 0U)
  {
    return;
  }

  if ((g_boot_log_pos + len) >= APP_BOOT_LOG_SIZE)
  {
    len = APP_BOOT_LOG_SIZE - g_boot_log_pos - 1U;
  }

  memcpy(&g_boot_log_buffer[g_boot_log_pos], str, len);
  g_boot_log_pos += len;
  g_boot_log_buffer[g_boot_log_pos] = '\0';
}

const char *App_GetBootLog(void)
{
  return g_boot_log_buffer;
}

void App_GetRatePidGains(App_RatePidGains_t *gains)
{
  if (gains == NULL)
  {
    return;
  }

  __disable_irq();
  *gains = g_rate_pid_gains;
  __enable_irq();
}

uint8_t App_SetRatePidGains(const App_RatePidGains_t *gains)
{
  if (App_AreRatePidGainsValid(gains) == 0U)
  {
    return 0U;
  }

  __disable_irq();
  g_rate_pid_gains = *gains;
  __enable_irq();

  return 1U;
}

void App_ResetRatePidDefaults(void)
{
  App_RatePidGains_t defaults = {
    {APP_RATE_KP_ROLL_DEFAULT_US_PER_DPS, APP_RATE_KI_ROLL_DEFAULT_US_PER_DPS_S, APP_RATE_KD_ROLL_DEFAULT_US_PER_DPS_PER_S, APP_RATE_KFF_ROLL_DEFAULT_US_PER_DPS_PER_S},
    {APP_RATE_KP_PITCH_DEFAULT_US_PER_DPS, APP_RATE_KI_PITCH_DEFAULT_US_PER_DPS_S, APP_RATE_KD_PITCH_DEFAULT_US_PER_DPS_PER_S, APP_RATE_KFF_PITCH_DEFAULT_US_PER_DPS_PER_S},
    {APP_RATE_KP_YAW_DEFAULT_US_PER_DPS, APP_RATE_KI_YAW_DEFAULT_US_PER_DPS_S, APP_RATE_KD_YAW_DEFAULT_US_PER_DPS_PER_S, APP_RATE_KFF_YAW_DEFAULT_US_PER_DPS_PER_S},
  };

  (void)App_SetRatePidGains(&defaults);
}

void App_GetAttitudeGains(App_AttitudeGains_t *gains)
{
  if (gains == NULL)
  {
    return;
  }

  __disable_irq();
  *gains = g_attitude_gains;
  __enable_irq();
}

uint8_t App_SetAttitudeGains(const App_AttitudeGains_t *gains)
{
  if (App_AreAttitudeGainsValid(gains) == 0U)
  {
    return 0U;
  }

  __disable_irq();
  g_attitude_gains = *gains;
  __enable_irq();
  return 1U;
}

void App_ResetAttitudeDefaults(void)
{
  App_AttitudeGains_t defaults = {
    APP_ATTITUDE_ANGLE_KP_DPS_PER_DEG,
    APP_ATTITUDE_ANGLE_KP_DPS_PER_DEG,
    APP_ATTITUDE_MAX_ANGLE_DEG,
  };

  (void)App_SetAttitudeGains(&defaults);
}

static void App_ConvertLegacyRatePidGains(const App_RatePidGainsLegacy_t *legacy, App_RatePidGains_t *out)
{
  out->roll.kp = legacy->roll.kp;
  out->roll.ki = legacy->roll.ki;
  out->roll.kd = legacy->roll.kd;
  out->roll.kff = 0.0f;
  out->pitch.kp = legacy->pitch.kp;
  out->pitch.ki = legacy->pitch.ki;
  out->pitch.kd = legacy->pitch.kd;
  out->pitch.kff = 0.0f;
  out->yaw.kp = legacy->yaw.kp;
  out->yaw.ki = legacy->yaw.ki;
  out->yaw.kd = legacy->yaw.kd;
  out->yaw.kff = 0.0f;
}

uint8_t App_LoadRatePidGains(void)
{
  const App_PidFlashBlob_t *stored;
  const App_PidFlashBlobV1_t *stored_v1;
  const App_PidFlashBlobV2_t *stored_v2;
  App_RatePidGains_t converted;
  uint32_t expected_crc;

  stored = (const App_PidFlashBlob_t *)APP_PID_FLASH_ADDRESS;

  printf("PID_LOAD_DBG: magic=0x%08lX ver=%lu\r\n",
         (unsigned long)stored->magic, (unsigned long)stored->version);

  if (stored->magic != APP_PID_FLASH_MAGIC)
  {
    printf("PID_LOAD_DBG: bad magic (expected=0x%08lX)\r\n",
           (unsigned long)APP_PID_FLASH_MAGIC);
    return 0U;
  }

  if (stored->version == 1UL)
  {
    stored_v1 = (const App_PidFlashBlobV1_t *)APP_PID_FLASH_ADDRESS;
    expected_crc = App_Crc32((const uint8_t *)stored_v1, offsetof(App_PidFlashBlobV1_t, crc32));
    if (expected_crc != stored_v1->crc32)
    {
      printf("PID_LOAD_DBG: v1 crc mismatch stored=0x%08lX computed=0x%08lX\r\n",
             (unsigned long)stored_v1->crc32, (unsigned long)expected_crc);
      return 0U;
    }

    App_ConvertLegacyRatePidGains(&stored_v1->gains, &converted);
    if (App_AreRatePidGainsValid(&converted) == 0U)
    {
      printf("PID_LOAD_DBG: v1 gains out of range\r\n");
      return 0U;
    }

    (void)App_SetRatePidGains(&converted);
    printf("PID_LOAD_DBG: loaded legacy v1 blob (attitude defaults kept, kff=0)\r\n");
    return 1U;
  }

  if (stored->version == 2UL)
  {
    stored_v2 = (const App_PidFlashBlobV2_t *)APP_PID_FLASH_ADDRESS;
    expected_crc = App_Crc32((const uint8_t *)stored_v2, offsetof(App_PidFlashBlobV2_t, crc32));
    if (expected_crc != stored_v2->crc32)
    {
      printf("PID_LOAD_DBG: v2 crc mismatch stored=0x%08lX computed=0x%08lX\r\n",
             (unsigned long)stored_v2->crc32, (unsigned long)expected_crc);
      return 0U;
    }

    App_ConvertLegacyRatePidGains(&stored_v2->gains, &converted);
    if (App_AreRatePidGainsValid(&converted) == 0U)
    {
      printf("PID_LOAD_DBG: v2 gains out of range\r\n");
      return 0U;
    }

    (void)App_SetRatePidGains(&converted);

    if (App_AreAttitudeGainsValid(&stored_v2->att_gains) != 0U)
    {
      (void)App_SetAttitudeGains(&stored_v2->att_gains);
    }
    else
    {
      App_ResetAttitudeDefaults();
    }
    printf("PID_LOAD_DBG: loaded legacy v2 blob (kff=0)\r\n");
    return 1U;
  }

  if (stored->version != APP_PID_FLASH_VERSION)
  {
    printf("PID_LOAD_DBG: bad version=%lu (expected %lu)\r\n",
           (unsigned long)stored->version, (unsigned long)APP_PID_FLASH_VERSION);
    return 0U;
  }

  expected_crc = App_Crc32((const uint8_t *)stored, offsetof(App_PidFlashBlob_t, crc32));
  if (expected_crc != stored->crc32)
  {
    printf("PID_LOAD_DBG: crc mismatch stored=0x%08lX computed=0x%08lX\r\n",
           (unsigned long)stored->crc32, (unsigned long)expected_crc);
    return 0U;
  }

  if (App_AreRatePidGainsValid(&stored->gains) == 0U)
  {
    printf("PID_LOAD_DBG: gains out of range\r\n");
    return 0U;
  }

  (void)App_SetRatePidGains(&stored->gains);

  if (App_AreAttitudeGainsValid(&stored->att_gains) != 0U)
  {
    (void)App_SetAttitudeGains(&stored->att_gains);
  }
  else
  {
    App_ResetAttitudeDefaults();
    printf("PID_LOAD_DBG: attitude gains invalid, reset to defaults\r\n");
  }

  return 1U;
}

uint8_t App_SaveRatePidGains(void)
{
  FLASH_EraseInitTypeDef erase;
  uint32_t sector_error = 0U;
  uint32_t address;
  App_PidFlashPage_t APP_FLASHWORD_ALIGN page;
  const App_PidFlashBlob_t *written;
  uint8_t write_index;

  memset(&page, 0xFF, sizeof(page));
  page.blob.magic = APP_PID_FLASH_MAGIC;
  page.blob.version = APP_PID_FLASH_VERSION;
  App_GetRatePidGains(&page.blob.gains);
  App_GetAttitudeGains(&page.blob.att_gains);
  page.blob.crc32 = App_Crc32((const uint8_t *)&page.blob, offsetof(App_PidFlashBlob_t, crc32));

  if (HAL_FLASH_Unlock() != HAL_OK)
  {
    printf("PID_SAVE_DBG: unlock fail err=0x%08lX\r\n", (unsigned long)HAL_FLASH_GetError());
    return 0U;
  }

  memset(&erase, 0, sizeof(erase));
  erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase.Banks = FLASH_BANK_2;
  erase.Sector = FLASH_SECTOR_7;
  erase.NbSectors = 1U;
  erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

  if (HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK)
  {
    printf("PID_SAVE_DBG: erase fail sector_err=%lu flash_err=0x%08lX\r\n",
           (unsigned long)sector_error, (unsigned long)HAL_FLASH_GetError());
    (void)HAL_FLASH_Lock();
    return 0U;
  }

  address = APP_PID_FLASH_ADDRESS;
  for (write_index = 0U; write_index < 3U; write_index++)
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                          address,
                          (uint32_t)&page.words[write_index * 8U]) != HAL_OK)
    {
      printf("PID_SAVE_DBG: write fail word=%u addr=0x%08lX err=0x%08lX\r\n",
             (unsigned)write_index, (unsigned long)address, (unsigned long)HAL_FLASH_GetError());
      (void)HAL_FLASH_Lock();
      return 0U;
    }
    address += 32U;
  }

  (void)HAL_FLASH_Lock();

  written = (const App_PidFlashBlob_t *)APP_PID_FLASH_ADDRESS;
  if ((written->magic != APP_PID_FLASH_MAGIC) || (written->version != APP_PID_FLASH_VERSION))
  {
    printf("PID_SAVE_DBG: verify header fail magic=0x%08lX ver=%lu\r\n",
           (unsigned long)written->magic, (unsigned long)written->version);
    return 0U;
  }

  if (App_Crc32((const uint8_t *)written, offsetof(App_PidFlashBlob_t, crc32)) != written->crc32)
  {
    printf("PID_SAVE_DBG: verify crc fail stored=0x%08lX computed=0x%08lX\r\n",
           (unsigned long)written->crc32,
           (unsigned long)App_Crc32((const uint8_t *)written, offsetof(App_PidFlashBlob_t, crc32)));
    return 0U;
  }

  return 1U;
}

uint8_t App_RequestRatePidSetAndSave(const App_RatePidGains_t *gains)
{
  if (App_AreRatePidGainsValid(gains) == 0U)
  {
    return 0U;
  }

  __disable_irq();
  g_pid_command_gains = *gains;
  g_pid_command = APP_PID_CMD_SET_AND_SAVE;
  g_pid_command_queued_count++;
  __enable_irq();

  return 1U;
}

uint8_t App_RequestAttitudeSetAndSave(const App_AttitudeGains_t *gains)
{
  if (App_AreAttitudeGainsValid(gains) == 0U)
  {
    return 0U;
  }

  __disable_irq();
  g_attitude_command_gains = *gains;
  g_pid_command = APP_PID_CMD_ATT_SET_AND_SAVE;
  g_pid_command_queued_count++;
  __enable_irq();

  return 1U;
}

void App_RequestRatePidSave(void)
{
  __disable_irq();
  g_pid_command = APP_PID_CMD_SAVE;
  g_pid_command_queued_count++;
  __enable_irq();
}

void App_RequestRatePidLoad(void)
{
  __disable_irq();
  g_pid_command = APP_PID_CMD_LOAD;
  g_pid_command_queued_count++;
  __enable_irq();
}

void App_RequestRatePidDefaults(void)
{
  __disable_irq();
  g_pid_command = APP_PID_CMD_DEFAULT;
  g_pid_command_queued_count++;
  __enable_irq();
}

void App_RequestAttitudeDefaults(void)
{
  __disable_irq();
  g_pid_command = APP_PID_CMD_ATT_DEFAULT;
  g_pid_command_queued_count++;
  __enable_irq();
}

void App_RequestAttitudeZero(void)
{
  __disable_irq();
  g_attitude_zero_request = 1U;
  __enable_irq();
}

void App_RequestMagCalStart(void)
{
  __disable_irq();
  g_mag_cal_command = APP_MAG_CAL_CMD_START;
  __enable_irq();
}

void App_RequestMagCalStop(void)
{
  __disable_irq();
  g_mag_cal_command = APP_MAG_CAL_CMD_STOP;
  __enable_irq();
}

void App_GetPidCommandDebug(uint32_t *queued_count,
							uint32_t *handled_count,
							uint32_t *pending_cmd)
{
  __disable_irq();
  if (queued_count != NULL)
  {
    *queued_count = g_pid_command_queued_count;
  }

  if (handled_count != NULL)
  {
    *handled_count = g_pid_command_handled_count;
  }

  if (pending_cmd != NULL)
  {
    *pending_cmd = (uint32_t)g_pid_command;
  }
  __enable_irq();
}

void App_PrintPidDebug(void)
{
  const App_PidFlashBlob_t *stored;
  uint32_t queued_count;
  uint32_t handled_count;
  uint32_t pending_cmd;
  uint32_t computed_crc;
  uint8_t header_ok;
  uint8_t crc_ok;
  uint8_t gains_ok;
  App_RatePidGains_t active;

  stored = (const App_PidFlashBlob_t *)APP_PID_FLASH_ADDRESS;
  computed_crc = App_Crc32((const uint8_t *)stored, offsetof(App_PidFlashBlob_t, crc32));
  header_ok = (uint8_t)((stored->magic == APP_PID_FLASH_MAGIC) &&
                        (stored->version == APP_PID_FLASH_VERSION));
  crc_ok = (uint8_t)(computed_crc == stored->crc32);
  gains_ok = App_AreRatePidGainsValid(&stored->gains);

  App_GetPidCommandDebug(&queued_count, &handled_count, &pending_cmd);
  App_GetRatePidGains(&active);

  printf("PID_DEBUG[q=%lu h=%lu p=%lu]\r\n",
         (unsigned long)queued_count,
         (unsigned long)handled_count,
         (unsigned long)pending_cmd);
  printf("PID_FLASH[addr=0x%08lX magic=0x%08lX ver=%lu crc=0x%08lX calc=0x%08lX header=%u crc_ok=%u gains_ok=%u]\r\n",
         (unsigned long)APP_PID_FLASH_ADDRESS,
         (unsigned long)stored->magic,
         (unsigned long)stored->version,
         (unsigned long)stored->crc32,
         (unsigned long)computed_crc,
         (unsigned)header_ok,
         (unsigned)crc_ok,
         (unsigned)gains_ok);
  Telemetry_PrintRatePid(&active, "debug_active");

  if ((header_ok != 0U) && (crc_ok != 0U) && (gains_ok != 0U))
  {
    Telemetry_PrintRatePid(&stored->gains, "debug_flash");
  }
}

static void App_ProcessPendingPidCommand(void)
{
  App_PidCommand_t cmd;
  App_RatePidGains_t cmd_gains;
  App_AttitudeGains_t cmd_att_gains;
  App_RatePidGains_t active;
  App_AttitudeGains_t active_att;
  uint8_t op_ok;

  __disable_irq();
  cmd = g_pid_command;
  cmd_gains = g_pid_command_gains;
  cmd_att_gains = g_attitude_command_gains;
  g_pid_command = APP_PID_CMD_NONE;
  __enable_irq();

  if (cmd == APP_PID_CMD_NONE)
  {
    return;
  }

  __disable_irq();
  g_pid_command_handled_count++;
  __enable_irq();

  /* HAL_FLASHEx_Erase() blocks the whole control loop for ~1-2s (STM32H7 sector erase) -
   * refuse any command that reaches App_SaveRatePidGains() while armed, mirroring the
   * existing SDLOG DUMP "BUSY armed" guard, so tuning saves never freeze the aircraft mid-flight. */
  if ((g_glog_armed_state != 0U) &&
      ((cmd == APP_PID_CMD_SET_AND_SAVE) || (cmd == APP_PID_CMD_SAVE) ||
       (cmd == APP_PID_CMD_ATT_SET_AND_SAVE) || (cmd == APP_PID_CMD_ATT_DEFAULT)))
  {
    printf("PID_SAVE[FAIL armed]\r\n");
    return;
  }

  switch (cmd)
  {
    case APP_PID_CMD_SET_AND_SAVE:
      if (App_SetRatePidGains(&cmd_gains) == 0U)
      {
        printf("PID_SET[FAIL]\r\n");
        break;
      }

      op_ok = App_SaveRatePidGains();
      App_GetRatePidGains(&active);
      Telemetry_PrintRatePid(&active, op_ok != 0U ? "set_saved" : "set_unsaved");
      printf("PID_SET[%s]\r\n", (op_ok != 0U) ? "OK" : "OK_NO_SAVE");
      printf("PID_SAVE[%s]\r\n", (op_ok != 0U) ? "OK" : "FAIL");
      break;

    case APP_PID_CMD_SAVE:
      op_ok = App_SaveRatePidGains();
      App_GetRatePidGains(&active);
      Telemetry_PrintRatePid(&active, op_ok != 0U ? "save_ok" : "save_fail");
      printf("PID_SAVE[%s]\r\n", (op_ok != 0U) ? "OK" : "FAIL");
      break;

    case APP_PID_CMD_LOAD:
      op_ok = App_LoadRatePidGains();
      if (op_ok == 0U)
      {
        App_ResetRatePidDefaults();
      }
      App_GetRatePidGains(&active);
      Telemetry_PrintRatePid(&active, op_ok != 0U ? "load_ok" : "load_default");
      printf("PID_LOAD[%s]\r\n", (op_ok != 0U) ? "OK" : "DEFAULT");
      break;

    case APP_PID_CMD_DEFAULT:
      App_ResetRatePidDefaults();
      App_GetRatePidGains(&active);
      Telemetry_PrintRatePid(&active, "default");
      break;

    case APP_PID_CMD_ATT_SET_AND_SAVE:
      if (App_SetAttitudeGains(&cmd_att_gains) == 0U)
      {
        printf("ATT_SET[FAIL]\r\n");
        break;
      }

      op_ok = App_SaveRatePidGains();
      App_GetAttitudeGains(&active_att);
      printf("ATT_SET[%s]\r\n", (op_ok != 0U) ? "OK" : "OK_NO_SAVE");
      printf("ATT_SAVE[%s]\r\n", (op_ok != 0U) ? "OK" : "FAIL");
      printf("ATT[src=%s]=[ROLL_KP %.4f PITCH_KP %.4f MAX_ANG %.2f]\r\n",
             (op_ok != 0U) ? "set_saved" : "set_unsaved",
             (double)active_att.roll_kp,
             (double)active_att.pitch_kp,
             (double)active_att.max_angle_deg);
      break;

    case APP_PID_CMD_ATT_DEFAULT:
      App_ResetAttitudeDefaults();
      op_ok = App_SaveRatePidGains();
      App_GetAttitudeGains(&active_att);
      printf("ATT_DEFAULT[%s]\r\n", (op_ok != 0U) ? "OK" : "OK_NO_SAVE");
      printf("ATT_SAVE[%s]\r\n", (op_ok != 0U) ? "OK" : "FAIL");
      printf("ATT[src=%s]=[ROLL_KP %.4f PITCH_KP %.4f MAX_ANG %.2f]\r\n",
             (op_ok != 0U) ? "default_saved" : "default_unsaved",
             (double)active_att.roll_kp,
             (double)active_att.pitch_kp,
             (double)active_att.max_angle_deg);
      break;

    default:
      break;
  }
}

static void App_ProcessPendingMagCalCommand(void)
{
  App_MagCalCommand_t cmd;

  __disable_irq();
  cmd = g_mag_cal_command;
  g_mag_cal_command = APP_MAG_CAL_CMD_NONE;
  __enable_irq();

  if (cmd == APP_MAG_CAL_CMD_NONE)
  {
    return;
  }

  if (cmd == APP_MAG_CAL_CMD_START)
  {
    Mag_CalStart();
    printf("MAG_CAL[STARTED]\r\n");
    return;
  }

  /* Mag_CalStop() calls HAL_FLASHEx_Erase() (~1-2s block) on success - refuse
   * while armed, mirroring the PID-save armed guard above. */
  if (g_glog_armed_state != 0U)
  {
    printf("MAG_CAL[FAIL armed]\r\n");
    return;
  }

  if (Mag_CalStop() != 0U)
  {
    printf("MAG_CAL[OK ox=%.4f oy=%.4f sx=%.4f sy=%.4f]\r\n",
           (double)Mag_GetCalOffsetX(), (double)Mag_GetCalOffsetY(),
           (double)Mag_GetCalScaleX(), (double)Mag_GetCalScaleY());
  }
  else
  {
    printf("MAG_CAL[FAIL not_started_or_range_too_small]\r\n");
  }
}

void App_SetUsbMotorTest(uint8_t enabled, uint8_t motor_index, uint16_t pulse_us)
{
  if (motor_index < 1U)
  {
    motor_index = 1U;
  }
  else if (motor_index > 4U)
  {
    motor_index = 4U;
  }

  if (pulse_us < APP_PWM_MIN_US)
  {
    pulse_us = APP_PWM_MIN_US;
  }
  else if (pulse_us > APP_PWM_MAX_US)
  {
    pulse_us = APP_PWM_MAX_US;
  }

  __disable_irq();
  g_usb_motor_test_enabled = (enabled != 0U) ? 1U : 0U;
  g_usb_motor_test_motor_index = motor_index;
  g_usb_motor_test_pulse_us = pulse_us;
  __enable_irq();
}

static void App_EmitBootLog(void)
{
  Telemetry_PrintImuLoggerStart();
  Telemetry_PrintRatePid(&g_rate_pid_gains, g_boot_pid_loaded ? "boot" : "boot_default");
  if (IMU_GetType() != IMU_TYPE_UNKNOWN)
  {
    Telemetry_PrintImuDetected(IMU_GetType(), IMU_GetWhoAmI());
  }
  else
  {
    Telemetry_PrintImuDetectionFailed();
  }
}

void App_Init(void)
{
  Motors_Init();
  Motors_SetOutputEnabled(APP_MOTOR_TEST_MODE);
  Motors_StopAll();

  (void)HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

  Attitude_Init();
  Receiver_Init();

  g_boot_pid_loaded = App_LoadRatePidGains();
  if (g_boot_pid_loaded == 0U)
  {
    App_ResetRatePidDefaults();
  }

  (void)IMU_DetectAndInit();
  if (Baro_Init() == HAL_OK)
  {
    printf("BARO_INIT[OK addr=0x%02X]\r\n", (unsigned int)Baro_GetDetectedAddr7());
  }
  else
  {
    printf("BARO_INIT[FAIL chip_id=0x%02X]\r\n", (unsigned int)Baro_GetLastChipId());
  }

  /* GPS is only used by NAV_VELOCITY_BRAKE - not initialized at boot; App_Update()'s
   * retry only attempts GPS_Init() once that mode is actually selected. */

  if (Mag_Init() == HAL_OK)
  {
    printf("MAG_INIT[OK chip_id=0x%02X]\r\n", (unsigned int)Mag_GetLastChipId());
  }
  else
  {
    printf("MAG_INIT[FAIL chip_id=0x%02X]\r\n", (unsigned int)Mag_GetLastChipId());
  }

  if (SD_Init() == SD_OK)
  {
    App_SdLogLoadSuperblock();
  }

  App_ReportAndClearFaultRecord();
  App_BlackboxDumpIfPresent();

  Nav_Init();
}

void App_Update(void)
{
  static uint32_t detect_retry_counter = 0U;
  static uint32_t last_tick_ms = 0U;
  static uint8_t last_motor_test_step = 0xFFU;
  static uint8_t last_telarm_switch_high = 0xFFU; /* sentinel forces an initial sync on first read */

  if (g_boot_log_pending != 0U)
  {
    if (HAL_GetTick() >= 2000U)
    {
      g_boot_log_pending = 0U;
      App_EmitBootLog();
    }
  }
  static uint32_t last_receiver_telemetry_ms = 0U;
  static uint32_t last_rx16_telemetry_ms = 0U;
  static uint32_t last_mode_telemetry_ms = 0U;
  static uint32_t last_imu_telemetry_ms = 0U;
#if APP_ENABLE_ARM_RUNTIME_TELEMETRY
  static uint32_t last_arm_telemetry_ms = 0U;
#endif
  static uint32_t last_imu_error_ms = 0U;
  static uint8_t usb_test_was_enabled = 0U;
  static uint32_t usb_test_arm_start_ms = 0U;
  static uint8_t motors_armed = 0U;
  static uint8_t trim_captured = 0U;
  static uint32_t arm_hold_start_ms = 0U;
  static uint8_t disarm_condition_active = 0U;
  static uint32_t disarm_condition_start_ms = 0U;
  static uint8_t startup_safety_checked = 0U;
  static uint8_t startup_arm_blocked = 0U;
  static uint8_t beeper_on = 0U;
  static uint32_t last_beeper_toggle_ms = 0U;
  static uint32_t sdlog_decim_counter = 0U;
  static uint16_t roll_center_us = APP_PWM_MID_US;
  static uint16_t pitch_center_us = APP_PWM_MID_US;
  static uint16_t yaw_center_us = APP_PWM_MID_US;
  static float measured_roll_rate_dps = 0.0f;
  static float measured_pitch_rate_dps = 0.0f;
  static float measured_yaw_rate_dps = 0.0f;
  static float filtered_gyro_roll_rate_dps = 0.0f;
  static float filtered_gyro_pitch_rate_dps = 0.0f;
  static float filtered_gyro_yaw_rate_dps = 0.0f;
  static float roll_integral_dps_s = 0.0f;
  static float pitch_integral_dps_s = 0.0f;
  static float yaw_integral_dps_s = 0.0f;
  static float dterm_filt_roll_rate_dps = 0.0f;
  static float dterm_filt_pitch_rate_dps = 0.0f;
  static float dterm_filt_yaw_rate_dps = 0.0f;
  static float prev_dterm_filt_roll_rate_dps = 0.0f;
  static float prev_dterm_filt_pitch_rate_dps = 0.0f;
  static float prev_dterm_filt_yaw_rate_dps = 0.0f;
  static float ff_filt_cmd_roll_rate_dps = 0.0f;
  static float ff_filt_cmd_pitch_rate_dps = 0.0f;
  static float ff_filt_cmd_yaw_rate_dps = 0.0f;
  static float prev_ff_filt_cmd_roll_rate_dps = 0.0f;
  static float prev_ff_filt_cmd_pitch_rate_dps = 0.0f;
  static float prev_ff_filt_cmd_yaw_rate_dps = 0.0f;
  static uint8_t pid_state_initialized = 0U;
  static uint8_t attitude_zero_captured = 0U;
  static float startup_roll_offset_deg = 0.0f;
  static float startup_pitch_offset_deg = 0.0f;
  static float startup_yaw_offset_deg = 0.0f;
  static float startup_roll_offset_sum_deg = 0.0f;
  static float startup_pitch_offset_sum_deg = 0.0f;
  static float startup_yaw_offset_sum_deg = 0.0f;
  static uint32_t startup_zero_avg_sample_count = 0U;
  static uint8_t startup_beep_active = 0U;
  static uint32_t startup_beep_start_ms = 0U;
  static uint8_t mag_yaw_ref_captured = 0U;
  static float mag_heading_at_ref_deg = 0.0f;
  static float mag_ref_field_g = 0.0f;
  static float mag_yaw_nudge_dps = 0.0f;
  static float battery_voltage_filtered_v = 0.0f;
  static uint8_t battery_voltage_valid = 0U;
  static uint32_t battery_adc_raw = 0U;
  static uint32_t last_battery_sample_ms = 0U;
  static uint32_t last_baro_sample_ms = 0U;
  static uint32_t last_baro_retry_ms = 0U;
  static uint32_t last_gps_retry_ms = 0U;
  static uint32_t last_mag_sample_ms = 0U;
  static uint32_t last_mag_retry_ms = 0U;
  static uint8_t liftoff_ramp_active = 0U;
  static uint32_t liftoff_ramp_start_ms = 0U;
  static uint8_t althold_holding = 0U;
  static float althold_target_alt_m = 0.0f;
  static float althold_integral_us = 0.0f;
  static uint16_t althold_center_throttle_us = APP_PWM_MID_US;
  static uint16_t althold_settle_ref_us = APP_PWM_MID_US;
  static uint32_t althold_settle_start_ms = 0U;
  static float althold_trim_filtered_us = 0.0f;
  static uint8_t nav_brake_active = 0U;
  static uint8_t nav_brake_disqualified_latch = 0U;
  /* Gates ALL GPS init/retry/parsing and Nav_Update() to only when NAV_VELOCITY_BRAKE
   * is the currently selected mode - updated wherever flight_mode is (re)computed
   * below, so it always reflects the mode switch with at most one iteration of lag. */
  static App_FlightMode_t last_known_flight_mode = APP_FLIGHT_MODE_RATE;
  int32_t althold_trim_us;
  uint32_t liftoff_ramp_elapsed_ms;
  float liftoff_ramp_factor;
  IMU_RawData_t imu_raw;
  uint8_t motor_test_step;
  uint32_t now_ms;
  receiver_state_t receiver_state;
  float dt_s;
  float gyro_lpf_alpha;
  float dterm_lpf_alpha;
  float ff_lpf_alpha;
  float ax_g;
  float ay_g;
  float az_g;
  float gx_dps;
  float gy_dps;
  float gz_dps;
  float pitch_deg;
  float roll_deg;
  float yaw_deg;
  /* Raw (pre attitude-zero-offset) tilt for Mag_GetHeadingDeg() - captured right
   * after Attitude_GetBoardAnglesDeg() below, before pitch_deg/roll_deg get the
   * startup-offset subtracted out for control-loop use. Tilt compensation needs
   * the board's true physical tilt relative to gravity, not a value zeroed to
   * whatever attitude it happened to be at when the pilot last zeroed it. */
  float mag_tilt_roll_deg = 0.0f;
  float mag_tilt_pitch_deg = 0.0f;
  float battery_voltage_v;
  App_FlightMode_t flight_mode;
  uint16_t mode_us;
  float target_roll_deg;
  float target_pitch_deg;
  float roll_angle_error_deg;
  float pitch_angle_error_deg;
  Nav_State_t nav_state;
  uint8_t nav_brake_requested;
  uint8_t nav_brake_tilt_limited;
  uint8_t nav_brake_accel_limited;
  float nav_brake_desired_north_vel_mps;
  float nav_brake_desired_east_vel_mps;
  float nav_brake_north_vel_error_mps;
  float nav_brake_east_vel_error_mps;
  float nav_brake_north_accel_cmd_mps2;
  float nav_brake_east_accel_cmd_mps2;
  float nav_brake_fwd_accel_cmd_mps2;
  float nav_brake_right_accel_cmd_mps2;
  int16_t pilot_roll_stick_us;
  int16_t pilot_pitch_stick_us;
  uint16_t roll_us;
  uint16_t pitch_us;
  uint16_t throttle_us;
  uint16_t yaw_us;
  uint16_t arm_us;
  float cmd_roll_rate_dps;
  float cmd_pitch_rate_dps;
  float cmd_yaw_rate_dps;
  float roll_rate_error_dps;
  float pitch_rate_error_dps;
  float yaw_rate_error_dps;
  float roll_rate_derivative_dps_per_s;
  float pitch_rate_derivative_dps_per_s;
  float yaw_rate_derivative_dps_per_s;
  float roll_rate_ff_dps_per_s;
  float pitch_rate_ff_dps_per_s;
  float yaw_rate_ff_dps_per_s;
  float roll_term_f;
  float pitch_term_f;
  float yaw_term_f;
  float integral_limit_roll;
  float integral_limit_pitch;
  float integral_limit_yaw;
  float voltage_comp_factor;
  App_RatePidGains_t active_pid_gains;
  App_AttitudeGains_t active_attitude_gains;
  int32_t roll_term;
  int32_t pitch_term;
  int32_t yaw_term;
  int32_t throttle_term;
  int32_t baro_damp_term_us;
  int32_t throttle_offset_us;
  int32_t althold_throttle_us;
  float climb_rate_setpoint_mps;
  float climb_rate_error_mps;
  uint8_t baro_healthy_now;
  int32_t m_front_left;
  int32_t m_front_right;
  int32_t m_rear_right;
  int32_t m_rear_left;
  int32_t mix_max;
  int32_t mix_min;
  int32_t mix_offset;
  uint16_t s1_us;
  uint16_t s2_us;
  uint16_t s3_us;
  uint16_t s4_us;
  uint8_t arm_switch_high;
  uint8_t throttle_low;
  uint8_t usb_motor_test_enabled;
  uint8_t usb_motor_test_motor_index;
  uint16_t usb_motor_test_pulse_us;

  s1_us = APP_PWM_MIN_US;
  s2_us = APP_PWM_MIN_US;
  s3_us = APP_PWM_MIN_US;
  s4_us = APP_PWM_MIN_US;
  arm_switch_high = 0U;
  throttle_low = 0U;
  throttle_us = APP_PWM_MIN_US;
  nav_brake_requested = 0U;
  nav_brake_tilt_limited = 0U;
  nav_brake_accel_limited = 0U;
  nav_brake_desired_north_vel_mps = 0.0f;
  nav_brake_desired_east_vel_mps = 0.0f;
  nav_brake_north_vel_error_mps = 0.0f;
  nav_brake_east_vel_error_mps = 0.0f;
  nav_brake_north_accel_cmd_mps2 = 0.0f;
  nav_brake_east_accel_cmd_mps2 = 0.0f;
  nav_brake_fwd_accel_cmd_mps2 = 0.0f;
  nav_brake_right_accel_cmd_mps2 = 0.0f;
  pilot_roll_stick_us = 0;
  pilot_pitch_stick_us = 0;
  memset(&nav_state, 0, sizeof(nav_state));

  HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);

  __disable_irq();
  usb_motor_test_enabled = g_usb_motor_test_enabled;
  usb_motor_test_motor_index = g_usb_motor_test_motor_index;
  usb_motor_test_pulse_us = g_usb_motor_test_pulse_us;
  __enable_irq();

  now_ms = HAL_GetTick();

  if ((startup_beep_active != 0U) && ((now_ms - startup_beep_start_ms) >= APP_STARTUP_BEEP_MS))
  {
    startup_beep_active = 0U;
    HAL_GPIO_WritePin(BEEPER_GPIO_Port, BEEPER_Pin, GPIO_PIN_RESET);
  }

  Communications_ServiceEscPassthrough();
  Communications_ServiceUart6Commands();
  App_ProcessPendingPidCommand();
  App_ProcessPendingMagCalCommand();
  App_ServiceGyroLogDump();
  App_ServiceSdCommands();
  App_ServiceSdLog();

  if ((now_ms - last_baro_sample_ms) >= APP_BARO_UPDATE_MS)
  {
    if ((Baro_IsInitialized() == 0U) && ((now_ms - last_baro_retry_ms) >= APP_BARO_RETRY_MS))
    {
      if (Baro_Init() == HAL_OK)
      {
        printf("BARO_INIT[OK addr=0x%02X]\r\n", (unsigned int)Baro_GetDetectedAddr7());
      }
      else
      {
        printf("BARO_INIT[FAIL chip_id=0x%02X]\r\n", (unsigned int)Baro_GetLastChipId());
      }
      last_baro_retry_ms = now_ms;
    }
    (void)Baro_Update();
    last_baro_sample_ms = now_ms;
  }

  /* GPS_Init() blocks waiting for UBX ACKs (up to ~1s worst case) - only retry
   * while disarmed, matching the SD/PID-save armed-guard pattern above. GPS is
   * only used by NAV_VELOCITY_BRAKE - no GPS init/retry/parsing in any other mode. */
  if ((last_known_flight_mode == APP_FLIGHT_MODE_NAV_VELOCITY_BRAKE) &&
      (GPS_IsConfigured() == 0U) && (g_glog_armed_state == 0U) &&
      ((now_ms - last_gps_retry_ms) >= APP_GPS_RETRY_MS))
  {
    if (GPS_Init() == HAL_OK)
    {
      printf("GPS_INIT[OK]\r\n");
    }
    else
    {
      printf("GPS_INIT[FAIL prt_ack=%u msg_ack=%u]\r\n", (unsigned int)GPS_GetLastPrtAcked(),
             (unsigned int)GPS_GetLastMsgAcked());
    }
    last_gps_retry_ms = now_ms;
  }

  if ((now_ms - last_mag_sample_ms) >= APP_MAG_UPDATE_MS)
  {
    if ((Mag_IsInitialized() == 0U) && ((now_ms - last_mag_retry_ms) >= APP_MAG_RETRY_MS))
    {
      printf("MAG_INIT[%s chip_id=0x%02X]\r\n", (Mag_Init() == HAL_OK) ? "OK" : "FAIL",
             (unsigned int)Mag_GetLastChipId());
      last_mag_retry_ms = now_ms;
    }
    (void)Mag_Update();
    last_mag_sample_ms = now_ms;
  }

  /* Nav_Update() (and the GPS data it consumes) is only relevant to NAV_VELOCITY_BRAKE -
   * skipped entirely in every other mode, leaving nav_state at its last value. */
  if (last_known_flight_mode == APP_FLIGHT_MODE_NAV_VELOCITY_BRAKE)
  {
    Nav_Update(now_ms, motors_armed);
    Nav_GetState(&nav_state);
  }

  if (usb_motor_test_enabled != 0U)
  {
    Motors_SetOutputEnabled(1U);

    if (usb_test_was_enabled == 0U)
    {
      usb_test_was_enabled = 1U;
      usb_test_arm_start_ms = now_ms;
    }

    if ((now_ms - usb_test_arm_start_ms) >= APP_USB_TEST_ARM_DELAY_MS)
    {
      switch (usb_motor_test_motor_index)
      {
        case 1U:
          s1_us = usb_motor_test_pulse_us;
          break;
        case 2U:
          s2_us = usb_motor_test_pulse_us;
          break;
        case 3U:
          s3_us = usb_motor_test_pulse_us;
          break;
        case 4U:
          s4_us = usb_motor_test_pulse_us;
          break;
        default:
          break;
      }
    }

    /* Physical channels map as: CH1=LA, CH2=LF, CH3=RA, CH4=RF. */
    Motors_WriteUs(s4_us, s1_us, s3_us, s2_us);

    if ((now_ms - last_receiver_telemetry_ms) >= 1000U)
    {
      last_receiver_telemetry_ms = now_ms;
    }

    if ((now_ms - last_rx16_telemetry_ms) >= APP_RX16_TELEMETRY_MS)
    {
      Receiver_GetState(&receiver_state);
      Telemetry_PrintReceiverState16(&receiver_state);
      last_rx16_telemetry_ms = now_ms;
    }

    if ((now_ms - last_mode_telemetry_ms) >= 250U)
    {
      mode_us = App_CrsfRawToUs(receiver_state.channels[APP_CH_MODE_INDEX]);
      flight_mode = App_SelectFlightMode(mode_us);
      last_known_flight_mode = flight_mode;
      Telemetry_PrintFlightMode(App_FlightModeName(flight_mode), mode_us);
      last_mode_telemetry_ms = now_ms;
    }

#if APP_ENABLE_ARM_RUNTIME_TELEMETRY
    if ((now_ms - last_arm_telemetry_ms) >= APP_ARM_TELEMETRY_MS)
    {
      Telemetry_PrintArmState(1U,
                              0U,
                              0U,
                              usb_motor_test_pulse_us,
                              s1_us,
                              s2_us,
                              s3_us,
                              s4_us);
      last_arm_telemetry_ms = now_ms;
    }
#endif

    if ((now_ms - last_battery_sample_ms) >= APP_BATTERY_SAMPLE_MS)
    {
      if (App_ReadBatteryVoltage(&battery_voltage_v, &battery_adc_raw) != 0U)
      {
        if (battery_voltage_valid == 0U)
        {
          battery_voltage_filtered_v = battery_voltage_v;
          battery_voltage_valid = 1U;
        }
        else
        {
          battery_voltage_filtered_v += APP_BATTERY_FILTER_ALPHA * (battery_voltage_v - battery_voltage_filtered_v);
        }
      }
      last_battery_sample_ms = now_ms;
    }

    if ((IMU_GetType() == IMU_TYPE_UNKNOWN) && ((detect_retry_counter++ % 10U) == 0U))
    {
      if (IMU_DetectAndInit() == HAL_OK)
      {
        Telemetry_PrintImuDetected(IMU_GetType(), IMU_GetWhoAmI());
      }
    }

    if ((IMU_GetType() != IMU_TYPE_UNKNOWN) && (IMU_ReadRawAligned(&imu_raw) == HAL_OK))
    {
      ax_g = ((float)imu_raw.accel_x) / IMU_ACCEL_LSB_PER_G;
      ay_g = ((float)imu_raw.accel_y) / IMU_ACCEL_LSB_PER_G;
      az_g = ((float)imu_raw.accel_z) / IMU_ACCEL_LSB_PER_G;
      gx_dps = ((float)imu_raw.gyro_x) / IMU_GYRO_LSB_PER_DPS;
      gy_dps = ((float)imu_raw.gyro_y) / IMU_GYRO_LSB_PER_DPS;
      gz_dps = ((float)APP_GYRO_YAW_SIGN) * (((float)imu_raw.gyro_z) / IMU_GYRO_LSB_PER_DPS);
      (void)App_UpdateGyroBias(ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, 1U, usb_motor_test_pulse_us);
      gx_dps -= g_roll_gyro_bias_dps;
      gy_dps -= g_pitch_gyro_bias_dps;
      gz_dps -= g_yaw_gyro_bias_dps;
      gz_dps += mag_yaw_nudge_dps;

      gyro_lpf_alpha = App_LpfAlpha(((float)APP_CONTROL_LOOP_MS) * 0.001f, APP_GYRO_RATE_LPF_HZ);
      filtered_gyro_roll_rate_dps += gyro_lpf_alpha * (gx_dps - filtered_gyro_roll_rate_dps);
      filtered_gyro_pitch_rate_dps += gyro_lpf_alpha * (gy_dps - filtered_gyro_pitch_rate_dps);
      filtered_gyro_yaw_rate_dps += gyro_lpf_alpha * (gz_dps - filtered_gyro_yaw_rate_dps);

      measured_roll_rate_dps = filtered_gyro_roll_rate_dps;
      measured_pitch_rate_dps = filtered_gyro_pitch_rate_dps;
      measured_yaw_rate_dps = filtered_gyro_yaw_rate_dps;

      Attitude_UpdateIMU(gx_dps * RAD_PER_DEG,
                         gy_dps * RAD_PER_DEG,
                         gz_dps * RAD_PER_DEG,
                         ax_g,
                         ay_g,
                         az_g,
                         ((float)APP_CONTROL_LOOP_MS) * 0.001f);
      Attitude_GetBoardAnglesDeg(&pitch_deg, &roll_deg, &yaw_deg);
      mag_tilt_roll_deg = roll_deg;
      mag_tilt_pitch_deg = pitch_deg;

      if (g_attitude_zero_request != 0U)
      {
        __disable_irq();
        g_attitude_zero_request = 0U;
        __enable_irq();
        startup_roll_offset_deg = roll_deg;
        startup_pitch_offset_deg = pitch_deg;
        startup_yaw_offset_deg = yaw_deg;
        attitude_zero_captured = 1U;
        startup_beep_active = 1U;
        startup_beep_start_ms = now_ms;
        HAL_GPIO_WritePin(BEEPER_GPIO_Port, BEEPER_Pin, GPIO_PIN_SET);
        printf("ATT_ZERO[OK]\r\n");
      }

      if (attitude_zero_captured == 0U)
      {
        if (now_ms >= (APP_ATTITUDE_ZERO_SETTLE_MS - APP_ATTITUDE_ZERO_AVG_MS))
        {
          startup_roll_offset_sum_deg += roll_deg;
          startup_pitch_offset_sum_deg += pitch_deg;
          startup_yaw_offset_sum_deg += yaw_deg;
          startup_zero_avg_sample_count++;
        }

        if ((now_ms >= APP_ATTITUDE_ZERO_SETTLE_MS) && (startup_zero_avg_sample_count > 0U))
        {
          startup_roll_offset_deg = startup_roll_offset_sum_deg / ((float)startup_zero_avg_sample_count);
          startup_pitch_offset_deg = startup_pitch_offset_sum_deg / ((float)startup_zero_avg_sample_count);
          startup_yaw_offset_deg = startup_yaw_offset_sum_deg / ((float)startup_zero_avg_sample_count);
          attitude_zero_captured = 1U;
          startup_beep_active = 1U;
          startup_beep_start_ms = now_ms;
          HAL_GPIO_WritePin(BEEPER_GPIO_Port, BEEPER_Pin, GPIO_PIN_SET);
        }
      }

      roll_deg = Attitude_WrapAngle180(roll_deg - startup_roll_offset_deg);
      pitch_deg = Attitude_WrapAngle180(pitch_deg - startup_pitch_offset_deg);
      yaw_deg = Attitude_WrapAngle180(yaw_deg - startup_yaw_offset_deg);

      /* Compute the next iteration's slow yaw drift-correction nudge here (see
       * APP_MAG_YAW_NUDGE_* above) - compares the compass's own rotation-since-ref
       * against the AHRS's rotation-since-boot, so boot heading stays 0 as today. */
      if ((attitude_zero_captured != 0U) && (Mag_IsHealthy() != 0U))
      {
        float mag_field_now_g = sqrtf((Mag_GetXGauss() * Mag_GetXGauss()) +
                                       (Mag_GetYGauss() * Mag_GetYGauss()) +
                                       (Mag_GetZGauss() * Mag_GetZGauss()));

        if (mag_yaw_ref_captured == 0U)
        {
          mag_heading_at_ref_deg = Mag_GetHeadingDeg(mag_tilt_roll_deg, mag_tilt_pitch_deg);
          mag_ref_field_g = mag_field_now_g;
          mag_yaw_ref_captured = 1U;
          mag_yaw_nudge_dps = 0.0f;
        }
        else if ((mag_ref_field_g > 0.01f) &&
                 (fabsf(mag_field_now_g - mag_ref_field_g) / mag_ref_field_g > APP_MAG_TRUST_MAX_MAG_DEVIATION_FRAC))
        {
          mag_yaw_nudge_dps = 0.0f; /* field strength shifted too much - likely motor/ESC current interference */
        }
        else
        {
          float mag_delta_deg = Attitude_WrapAngle180(Mag_GetHeadingDeg(mag_tilt_roll_deg, mag_tilt_pitch_deg) - mag_heading_at_ref_deg);
          float yaw_error_deg = Attitude_WrapAngle180(mag_delta_deg - yaw_deg);

          mag_yaw_nudge_dps = App_ClampFloat(yaw_error_deg * APP_MAG_YAW_NUDGE_KP_DPS_PER_DEG,
                                              -APP_MAG_YAW_NUDGE_MAX_DPS, APP_MAG_YAW_NUDGE_MAX_DPS);
        }
      }
      else
      {
        mag_yaw_nudge_dps = 0.0f;
      }

#if APP_ENABLE_USB_TEST_IMU_TELEMETRY
      if ((now_ms - last_imu_telemetry_ms) >= APP_IMU_TELEMETRY_MS)
      {
        Telemetry_PrintImuState(ax_g,
                                ay_g,
                                az_g,
                                gx_dps,
                                gy_dps,
                                gz_dps,
                                pitch_deg,
                                roll_deg,
                                yaw_deg);
        if (battery_voltage_valid != 0U)
        {
          Telemetry_PrintBatteryState(battery_voltage_filtered_v, battery_adc_raw);
        }
        Telemetry_PrintBaroState(Baro_GetAltitudeM(), Baro_GetClimbRateMps(), Baro_IsHealthy());
        Telemetry_PrintGpsState(GPS_IsConfigured(), GPS_IsHealthy(), GPS_GetFixType(), GPS_GetNumSatellites(),
                               GPS_GetLatitudeDeg(), GPS_GetLongitudeDeg(), GPS_GetAltitudeM());
        Telemetry_PrintMagState(Mag_IsHealthy(), Mag_GetXGauss(), Mag_GetYGauss(), Mag_GetZGauss(),
                               Mag_GetHeadingDeg(mag_tilt_roll_deg, mag_tilt_pitch_deg));
        Telemetry_PrintNavState(nav_state.valid, nav_state.reference_valid, (uint8_t)nav_state.invalid_reason,
                               nav_state.fix_type, nav_state.num_sv, nav_state.h_acc_m,
                               nav_state.age_ms, nav_state.update_period_ms,
                               nav_state.consecutive_valid, nav_state.consecutive_invalid,
                               nav_state.duplicate_count, nav_state.rejected_count, nav_state.dropout_count);
        Telemetry_PrintNavPosVel(nav_state.north_m, nav_state.east_m,
                                nav_state.raw_north_vel_mps, nav_state.raw_east_vel_mps,
                                nav_state.filtered_north_vel_mps, nav_state.filtered_east_vel_mps);
        last_imu_telemetry_ms = now_ms;
      }
#endif
    }
    else if ((now_ms - last_imu_error_ms) >= 1000U)
    {
#if APP_ENABLE_USB_TEST_IMU_TELEMETRY
      Telemetry_PrintImuReadFailed(IMU_GetType(), IMU_GetWhoAmI());
#endif
      last_imu_error_ms = now_ms;
    }

    HAL_Delay(APP_CONTROL_LOOP_MS);
    return;
  }

  usb_test_was_enabled = 0U;

  if (APP_MOTOR_TEST_MODE != 0U)
  {
    motor_test_step = Motors_RunTestPattern(now_ms);
    if (motor_test_step != last_motor_test_step)
    {
      Telemetry_PrintMotorTestStep(motor_test_step);
      last_motor_test_step = motor_test_step;
    }
  }

  if (last_tick_ms == 0U)
  {
    dt_s = ((float)APP_CONTROL_LOOP_MS) * 0.001f;
  }
  else
  {
    dt_s = ((float)(now_ms - last_tick_ms)) * 0.001f;
  }
  last_tick_ms = now_ms;

  Receiver_Update(now_ms);
  Receiver_GetState(&receiver_state);

  if (APP_MOTOR_TEST_MODE == 0U)
  {
    arm_us = App_CrsfRawToUs(receiver_state.channels[APP_CH_ARM_INDEX]);
    roll_us = App_CrsfRawToUs(receiver_state.channels[APP_CH_ROLL_INDEX]);
    pitch_us = App_CrsfRawToUs(receiver_state.channels[APP_CH_PITCH_INDEX]);
    throttle_us = App_CrsfRawToUs(receiver_state.channels[APP_CH_THROTTLE_INDEX]);
    yaw_us = App_CrsfRawToUs(receiver_state.channels[APP_CH_YAW_INDEX]);
    mode_us = App_CrsfRawToUs(receiver_state.channels[APP_CH_MODE_INDEX]);
    flight_mode = App_SelectFlightMode(mode_us);
    last_known_flight_mode = flight_mode;
    arm_switch_high = (uint8_t)(arm_us >= APP_ARM_THRESHOLD_US);
    throttle_low = (uint8_t)(throttle_us <= APP_THROTTLE_LOW_US);

    /* RC channel 7 controls the armed bench telemetry stream (hi=on, lo=off) - source of
     * truth is the switch position itself, not a latched command, so it can never drift
     * out of sync with what the pilot is holding. Edge-triggered so this never prints (or
     * touches the flag) more than once per actual transition. */
    {
      uint16_t telarm_us = App_CrsfRawToUs(receiver_state.channels[APP_CH_TELARM_INDEX]);
      uint8_t telarm_switch_high = (uint8_t)(telarm_us >= APP_ARM_THRESHOLD_US);

      if (telarm_switch_high != last_telarm_switch_high)
      {
        last_telarm_switch_high = telarm_switch_high;
        App_RequestArmedTelemetryEnabled(telarm_switch_high);
      }
    }
    /* Reflects the mode-switch position for telemetry/bench visibility even while
     * disarmed - the armed block below recomputes/uses this for actual control. */
    nav_brake_requested = (flight_mode == APP_FLIGHT_MODE_NAV_VELOCITY_BRAKE) ? 1U : 0U;

    if ((startup_safety_checked == 0U) && (receiver_state.link_active != 0U))
    {
      startup_safety_checked = 1U;
      if (arm_switch_high != 0U)
      {
        startup_arm_blocked = 1U;
      }
    }

    if (startup_arm_blocked != 0U)
    {
      motors_armed = 0U;
      trim_captured = 0U;
      arm_hold_start_ms = 0U;

      /* Keep valid minimum PWM while startup arm safety is active so ESCs
       * can still complete their own ready sequence. */
      Motors_SetOutputEnabled(1U);
      Motors_StopAll();

      if ((now_ms - last_beeper_toggle_ms) >= APP_BEEPER_TOGGLE_MS)
      {
        beeper_on ^= 1U;
        HAL_GPIO_WritePin(BEEPER_GPIO_Port,
                          BEEPER_Pin,
                          (beeper_on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        last_beeper_toggle_ms = now_ms;
      }

      if (arm_switch_high == 0U)
      {
        startup_arm_blocked = 0U;
        beeper_on = 0U;
        HAL_GPIO_WritePin(BEEPER_GPIO_Port, BEEPER_Pin, GPIO_PIN_RESET);
      }
    }
    else
    {
      if (beeper_on != 0U)
      {
        beeper_on = 0U;
        HAL_GPIO_WritePin(BEEPER_GPIO_Port, BEEPER_Pin, GPIO_PIN_RESET);
      }

      if ((receiver_state.link_active == 0U) || (arm_switch_high == 0U))
      {
        if (disarm_condition_active == 0U)
        {
          disarm_condition_active = 1U;
          disarm_condition_start_ms = now_ms;
        }
      }
      else
      {
        disarm_condition_active = 0U;
      }

      if ((disarm_condition_active != 0U) &&
          ((now_ms - disarm_condition_start_ms) >= APP_DISARM_DEBOUNCE_MS))
      {
        uint8_t was_armed = motors_armed;
        const char *disarm_reason = (receiver_state.link_active == 0U) ? "RX_LINK_LOST" : "ARM_SWITCH_LOW";

        motors_armed = 0U;
        g_glog_armed_state = 0U;
        Motors_SetOutputEnabled(1U);
        Motors_StopAll();

        if (was_armed != 0U)
        {
          printf("ARM_EVENT[DISARM reason=%s link=%u arm_us=%u age_ms=%lu]\r\n",
                 disarm_reason,
                 (unsigned int)receiver_state.link_active,
                 (unsigned int)arm_us,
                 (unsigned long)(now_ms - receiver_state.last_frame_ms));
        }

        if ((was_armed != 0U) && (g_sdlog_active != 0U))
        {
          App_SdLogFlushFlight();
        }
        trim_captured = 0U;
        arm_hold_start_ms = 0U;
        roll_integral_dps_s = 0.0f;
        pitch_integral_dps_s = 0.0f;
        yaw_integral_dps_s = 0.0f;
        pid_state_initialized = 0U;
        nav_brake_active = 0U;
        nav_brake_disqualified_latch = 0U;
      }

      Motors_SetOutputEnabled(1U);

      if (motors_armed == 0U)
      {
        Motors_StopAll();
        s1_us = APP_PWM_MIN_US;
        s2_us = APP_PWM_MIN_US;
        s3_us = APP_PWM_MIN_US;
        s4_us = APP_PWM_MIN_US;

        if ((receiver_state.link_active != 0U) &&
            (arm_switch_high != 0U) &&
            (throttle_low != 0U))
        {
          if (arm_hold_start_ms == 0U)
          {
            arm_hold_start_ms = now_ms;
          }
          else if ((now_ms - arm_hold_start_ms) >= APP_ARM_HOLD_MS)
          {
            motors_armed = 1U;
            trim_captured = 0U;
            g_glog_count = 0U;
            g_glog_capturing = 1U;
            App_SdLogArmStart();
          }
        }
        else
        {
          arm_hold_start_ms = 0U;
        }
      }

      g_glog_armed_state = motors_armed;

      if (motors_armed != 0U)
      {
        App_GetRatePidGains(&active_pid_gains);
        App_GetAttitudeGains(&active_attitude_gains);

        if (trim_captured == 0U)
        {
          roll_center_us = roll_us;
          pitch_center_us = pitch_us;
          yaw_center_us = yaw_us;
          trim_captured = 1U;
        }

        pilot_roll_stick_us = (int16_t)((int32_t)roll_us - (int32_t)roll_center_us);
        pilot_pitch_stick_us = (int16_t)((int32_t)pitch_us - (int32_t)pitch_center_us);
        nav_brake_requested = (flight_mode == APP_FLIGHT_MODE_NAV_VELOCITY_BRAKE) ? 1U : 0U;

        if (nav_brake_requested != 0U)
        {
          /* Engagement requirements (section 9): valid+referenced GPS nav, a
           * usable attitude/yaw solution, altitude hold available, link up,
           * and not latched out from a previous in-mode GPS loss. */
          uint8_t nav_engage_ok = (uint8_t)((nav_state.valid != 0U) &&
                                            (nav_state.reference_valid != 0U) &&
                                            (attitude_zero_captured != 0U) &&
                                            (Baro_IsHealthy() != 0U) &&
                                            (receiver_state.link_active != 0U) &&
                                            (nav_brake_disqualified_latch == 0U));

          if (nav_engage_ok != 0U)
          {
            float fwd_cmd_mps;
            float right_cmd_mps;
            float desired_north_vel_mps;
            float desired_east_vel_mps;

            if (nav_brake_active == 0U)
            {
              /* Fresh engagement this iteration: explicitly latch the local
               * reference now (sec.4) so this always works even if the
               * disarmed auto-latch never had the chance to run. */
              Nav_LatchReference();
            }

            /* NOTE: unlike target_pitch_deg below, fwd_cmd_mps is NOT sign-flipped by
             * APP_PITCH_SIGN here - Nav_RotateBodyToNed()'s "fwd" is a true physical
             * FRD quantity (see nav.h), and target_pitch_deg already applies the one
             * flip needed to convert an accel command into this codebase's pitch-angle
             * convention. Flipping here too would cancel that flip and invert the
             * pilot's pitch command (this bit the first flight test: full forward
             * stick drifted aft, regardless of heading). */
            fwd_cmd_mps = App_NavStickOffsetToVelocityMps(pilot_pitch_stick_us, APP_NAV_BRAKE_MAX_VEL_MPS);
            right_cmd_mps = ((float)APP_ROLL_SIGN) *
                            App_NavStickOffsetToVelocityMps(pilot_roll_stick_us, APP_NAV_BRAKE_MAX_VEL_MPS);

            /* Pilot command flow (sec.7): body-relative stick -> yaw-rotated
             * desired N/E velocity -> compare to filtered GPS velocity ->
             * earth-frame accel command -> body-frame accel -> angle command. */
            Nav_RotateBodyToNed(fwd_cmd_mps, right_cmd_mps, yaw_deg,
                               &desired_north_vel_mps, &desired_east_vel_mps);

            nav_brake_north_vel_error_mps = desired_north_vel_mps - nav_state.filtered_north_vel_mps;
            nav_brake_east_vel_error_mps = desired_east_vel_mps - nav_state.filtered_east_vel_mps;

            nav_brake_north_accel_cmd_mps2 = App_ClampFloat(APP_NAV_BRAKE_VELOCITY_KP * nav_brake_north_vel_error_mps,
                                                            -APP_NAV_BRAKE_MAX_ACCEL_MPS2, APP_NAV_BRAKE_MAX_ACCEL_MPS2);
            nav_brake_east_accel_cmd_mps2 = App_ClampFloat(APP_NAV_BRAKE_VELOCITY_KP * nav_brake_east_vel_error_mps,
                                                           -APP_NAV_BRAKE_MAX_ACCEL_MPS2, APP_NAV_BRAKE_MAX_ACCEL_MPS2);

            Nav_RotateNedToBody(nav_brake_north_accel_cmd_mps2, nav_brake_east_accel_cmd_mps2, yaw_deg,
                               &nav_brake_fwd_accel_cmd_mps2, &nav_brake_right_accel_cmd_mps2);

            target_pitch_deg = ((float)APP_PITCH_SIGN) *
                               Nav_AccelToAngleDeg(nav_brake_fwd_accel_cmd_mps2, APP_NAV_BRAKE_MAX_TILT_DEG);
            target_roll_deg = ((float)APP_ROLL_SIGN) *
                              Nav_AccelToAngleDeg(nav_brake_right_accel_cmd_mps2, APP_NAV_BRAKE_MAX_TILT_DEG);

            nav_brake_desired_north_vel_mps = desired_north_vel_mps;
            nav_brake_desired_east_vel_mps = desired_east_vel_mps;
            nav_brake_tilt_limited = (uint8_t)((fabsf(target_pitch_deg) >= (APP_NAV_BRAKE_MAX_TILT_DEG - 0.01f)) ||
                                               (fabsf(target_roll_deg) >= (APP_NAV_BRAKE_MAX_TILT_DEG - 0.01f)));
            nav_brake_accel_limited = (uint8_t)((fabsf(nav_brake_north_accel_cmd_mps2) >= (APP_NAV_BRAKE_MAX_ACCEL_MPS2 - 0.001f)) ||
                                                (fabsf(nav_brake_east_accel_cmd_mps2) >= (APP_NAV_BRAKE_MAX_ACCEL_MPS2 - 0.001f)));

            nav_brake_active = 1U;
          }
          else
          {
            if (nav_brake_active != 0U)
            {
              const char *lost_reason;

              /* Was engaged, lost validity this iteration - log the exact
               * reason (sec.9) and latch out until the pilot leaves and
               * reselects this mode. Never retain a stale nav command: fall
               * straight through to the plain stick-angle behavior below. */
              if (nav_state.valid == 0U)
              {
                lost_reason = Nav_InvalidReasonName(nav_state.invalid_reason);
              }
              else if (nav_state.reference_valid == 0U)
              {
                lost_reason = "NO_REFERENCE";
              }
              else if (attitude_zero_captured == 0U)
              {
                lost_reason = "ATTITUDE_NOT_READY";
              }
              else if (Baro_IsHealthy() == 0U)
              {
                lost_reason = "BARO_UNHEALTHY";
              }
              else if (receiver_state.link_active == 0U)
              {
                lost_reason = "RX_LINK_LOST";
              }
              else
              {
                lost_reason = "GATE_UNKNOWN";
              }
              printf("NAV_LOST[reason=%s]\r\n", lost_reason);
              nav_brake_disqualified_latch = 1U;
            }
            nav_brake_active = 0U;

            target_roll_deg = ((float)APP_ROLL_SIGN) * App_StickOffsetUsToAngleDeg((int32_t)roll_us - (int32_t)roll_center_us,
                                                                                    active_attitude_gains.max_angle_deg);
            target_pitch_deg = ((float)APP_PITCH_SIGN) * App_StickOffsetUsToAngleDeg((int32_t)pitch_us - (int32_t)pitch_center_us,
                                                                                      active_attitude_gains.max_angle_deg);
            nav_brake_desired_north_vel_mps = 0.0f;
            nav_brake_desired_east_vel_mps = 0.0f;
            nav_brake_north_vel_error_mps = 0.0f;
            nav_brake_east_vel_error_mps = 0.0f;
            nav_brake_north_accel_cmd_mps2 = 0.0f;
            nav_brake_east_accel_cmd_mps2 = 0.0f;
            nav_brake_fwd_accel_cmd_mps2 = 0.0f;
            nav_brake_right_accel_cmd_mps2 = 0.0f;
            nav_brake_tilt_limited = 0U;
            nav_brake_accel_limited = 0U;
          }

          roll_angle_error_deg = Attitude_WrapAngle180(target_roll_deg - roll_deg);
          pitch_angle_error_deg = Attitude_WrapAngle180(target_pitch_deg - pitch_deg);

          cmd_roll_rate_dps = App_ClampFloat(roll_angle_error_deg * active_attitude_gains.roll_kp,
                                             -APP_RATE_CMD_MAX_ROLL_DPS,
                                             APP_RATE_CMD_MAX_ROLL_DPS);
          cmd_pitch_rate_dps = App_ClampFloat(pitch_angle_error_deg * active_attitude_gains.pitch_kp,
                                              -APP_RATE_CMD_MAX_PITCH_DPS,
                                              APP_RATE_CMD_MAX_PITCH_DPS);
        }
        else if ((flight_mode == APP_FLIGHT_MODE_ATTITUDE) || (flight_mode == APP_FLIGHT_MODE_ALTHOLD))
        {
          /* Leaving NAV_VELOCITY_BRAKE always clears the disqualify latch -
           * re-selecting the mode later always gets a fresh engagement chance. */
          nav_brake_active = 0U;
          nav_brake_disqualified_latch = 0U;

          target_roll_deg = ((float)APP_ROLL_SIGN) * App_StickOffsetUsToAngleDeg((int32_t)roll_us - (int32_t)roll_center_us,
                                                                                  active_attitude_gains.max_angle_deg);
          target_pitch_deg = ((float)APP_PITCH_SIGN) * App_StickOffsetUsToAngleDeg((int32_t)pitch_us - (int32_t)pitch_center_us,
                                                                                    active_attitude_gains.max_angle_deg);
          roll_angle_error_deg = Attitude_WrapAngle180(target_roll_deg - roll_deg);
          pitch_angle_error_deg = Attitude_WrapAngle180(target_pitch_deg - pitch_deg);

          cmd_roll_rate_dps = App_ClampFloat(roll_angle_error_deg * active_attitude_gains.roll_kp,
                                             -APP_RATE_CMD_MAX_ROLL_DPS,
                                             APP_RATE_CMD_MAX_ROLL_DPS);
          cmd_pitch_rate_dps = App_ClampFloat(pitch_angle_error_deg * active_attitude_gains.pitch_kp,
                                              -APP_RATE_CMD_MAX_PITCH_DPS,
                                              APP_RATE_CMD_MAX_PITCH_DPS);
        }
        else
        {
          nav_brake_active = 0U;
          nav_brake_disqualified_latch = 0U;

          cmd_roll_rate_dps = ((float)APP_ROLL_SIGN) * App_StickOffsetUsToRateDps((int32_t)roll_us - (int32_t)roll_center_us,
                             APP_RATE_CMD_MAX_ROLL_DPS);
          cmd_pitch_rate_dps = ((float)APP_PITCH_SIGN) * App_StickOffsetUsToRateDps((int32_t)pitch_us - (int32_t)pitch_center_us,
                               APP_RATE_CMD_MAX_PITCH_DPS);
        }
        cmd_yaw_rate_dps = ((float)APP_YAW_SIGN) * App_StickOffsetUsToRateDps((int32_t)yaw_us - (int32_t)yaw_center_us,
                               APP_RATE_CMD_MAX_YAW_DPS);

        roll_rate_error_dps = cmd_roll_rate_dps - measured_roll_rate_dps;
        pitch_rate_error_dps = cmd_pitch_rate_dps - measured_pitch_rate_dps;
        yaw_rate_error_dps = cmd_yaw_rate_dps - measured_yaw_rate_dps;

        if ((dt_s < 0.0005f) || (dt_s > 0.050f))
        {
          dt_s = ((float)APP_CONTROL_LOOP_MS) * 0.001f;
        }

        if (pid_state_initialized == 0U)
        {
          dterm_filt_roll_rate_dps = measured_roll_rate_dps;
          dterm_filt_pitch_rate_dps = measured_pitch_rate_dps;
          dterm_filt_yaw_rate_dps = measured_yaw_rate_dps;
          prev_dterm_filt_roll_rate_dps = measured_roll_rate_dps;
          prev_dterm_filt_pitch_rate_dps = measured_pitch_rate_dps;
          prev_dterm_filt_yaw_rate_dps = measured_yaw_rate_dps;
          ff_filt_cmd_roll_rate_dps = cmd_roll_rate_dps;
          ff_filt_cmd_pitch_rate_dps = cmd_pitch_rate_dps;
          ff_filt_cmd_yaw_rate_dps = cmd_yaw_rate_dps;
          prev_ff_filt_cmd_roll_rate_dps = cmd_roll_rate_dps;
          prev_ff_filt_cmd_pitch_rate_dps = cmd_pitch_rate_dps;
          prev_ff_filt_cmd_yaw_rate_dps = cmd_yaw_rate_dps;
          pid_state_initialized = 1U;
        }

        roll_integral_dps_s += roll_rate_error_dps * dt_s;
        pitch_integral_dps_s += pitch_rate_error_dps * dt_s;
        yaw_integral_dps_s += yaw_rate_error_dps * dt_s;

        if (active_pid_gains.roll.ki > 0.000001f)
        {
          integral_limit_roll = ((float)APP_RATE_TERM_LIMIT_US) / active_pid_gains.roll.ki;
          roll_integral_dps_s = App_ClampFloat(roll_integral_dps_s, -integral_limit_roll, integral_limit_roll);
        }

        if (active_pid_gains.pitch.ki > 0.000001f)
        {
          integral_limit_pitch = ((float)APP_RATE_TERM_LIMIT_US) / active_pid_gains.pitch.ki;
          pitch_integral_dps_s = App_ClampFloat(pitch_integral_dps_s, -integral_limit_pitch, integral_limit_pitch);
        }

        if (active_pid_gains.yaw.ki > 0.000001f)
        {
          integral_limit_yaw = ((float)APP_RATE_TERM_LIMIT_US) / active_pid_gains.yaw.ki;
          yaw_integral_dps_s = App_ClampFloat(yaw_integral_dps_s, -integral_limit_yaw, integral_limit_yaw);
        }

        /* D term uses its own low-pass-filtered rate signal so raw gyro noise/spikes don't kick it. */
        dterm_lpf_alpha = App_LpfAlpha(dt_s, APP_DTERM_LPF_HZ);
        dterm_filt_roll_rate_dps += dterm_lpf_alpha * (measured_roll_rate_dps - dterm_filt_roll_rate_dps);
        dterm_filt_pitch_rate_dps += dterm_lpf_alpha * (measured_pitch_rate_dps - dterm_filt_pitch_rate_dps);
        dterm_filt_yaw_rate_dps += dterm_lpf_alpha * (measured_yaw_rate_dps - dterm_filt_yaw_rate_dps);

        roll_rate_derivative_dps_per_s = -(dterm_filt_roll_rate_dps - prev_dterm_filt_roll_rate_dps) / dt_s;
        pitch_rate_derivative_dps_per_s = -(dterm_filt_pitch_rate_dps - prev_dterm_filt_pitch_rate_dps) / dt_s;
        yaw_rate_derivative_dps_per_s = -(dterm_filt_yaw_rate_dps - prev_dterm_filt_yaw_rate_dps) / dt_s;

        prev_dterm_filt_roll_rate_dps = dterm_filt_roll_rate_dps;
        prev_dterm_filt_pitch_rate_dps = dterm_filt_pitch_rate_dps;
        prev_dterm_filt_yaw_rate_dps = dterm_filt_yaw_rate_dps;

        /* Feedforward tracks the commanded rate's own derivative, bypassing the error/I/D path entirely. */
        ff_lpf_alpha = App_LpfAlpha(dt_s, APP_FF_LPF_HZ);
        ff_filt_cmd_roll_rate_dps += ff_lpf_alpha * (cmd_roll_rate_dps - ff_filt_cmd_roll_rate_dps);
        ff_filt_cmd_pitch_rate_dps += ff_lpf_alpha * (cmd_pitch_rate_dps - ff_filt_cmd_pitch_rate_dps);
        ff_filt_cmd_yaw_rate_dps += ff_lpf_alpha * (cmd_yaw_rate_dps - ff_filt_cmd_yaw_rate_dps);

        roll_rate_ff_dps_per_s = (ff_filt_cmd_roll_rate_dps - prev_ff_filt_cmd_roll_rate_dps) / dt_s;
        pitch_rate_ff_dps_per_s = (ff_filt_cmd_pitch_rate_dps - prev_ff_filt_cmd_pitch_rate_dps) / dt_s;
        yaw_rate_ff_dps_per_s = (ff_filt_cmd_yaw_rate_dps - prev_ff_filt_cmd_yaw_rate_dps) / dt_s;

        prev_ff_filt_cmd_roll_rate_dps = ff_filt_cmd_roll_rate_dps;
        prev_ff_filt_cmd_pitch_rate_dps = ff_filt_cmd_pitch_rate_dps;
        prev_ff_filt_cmd_yaw_rate_dps = ff_filt_cmd_yaw_rate_dps;

        roll_term_f = (active_pid_gains.roll.kp * roll_rate_error_dps) +
                      (active_pid_gains.roll.ki * roll_integral_dps_s) +
                      (active_pid_gains.roll.kd * roll_rate_derivative_dps_per_s) +
                      (active_pid_gains.roll.kff * roll_rate_ff_dps_per_s);
        pitch_term_f = (active_pid_gains.pitch.kp * pitch_rate_error_dps) +
                       (active_pid_gains.pitch.ki * pitch_integral_dps_s) +
                       (active_pid_gains.pitch.kd * pitch_rate_derivative_dps_per_s) +
                       (active_pid_gains.pitch.kff * pitch_rate_ff_dps_per_s);
        yaw_term_f = (active_pid_gains.yaw.kp * yaw_rate_error_dps) +
                     (active_pid_gains.yaw.ki * yaw_integral_dps_s) +
                     (active_pid_gains.yaw.kd * yaw_rate_derivative_dps_per_s) +
                     (active_pid_gains.yaw.kff * yaw_rate_ff_dps_per_s);

        /* Higher pack voltage yields more real thrust/torque per PID microsecond,
         * so scale authority down (or up on sag) to keep response consistent. */
        voltage_comp_factor = (battery_voltage_valid != 0U) ?
                              App_ComputeVoltageCompFactor(battery_voltage_filtered_v) : 1.0f;
        roll_term_f *= voltage_comp_factor;
        pitch_term_f *= voltage_comp_factor;
        yaw_term_f *= voltage_comp_factor;

        roll_term = App_ClampControlTerm((int32_t)roll_term_f, APP_RATE_TERM_LIMIT_US);
        pitch_term = App_ClampControlTerm((int32_t)pitch_term_f, APP_RATE_TERM_LIMIT_US);
        yaw_term = App_ClampControlTerm((int32_t)yaw_term_f, APP_RATE_TERM_LIMIT_US);
        baro_healthy_now = Baro_IsHealthy();
        althold_trim_us = 0;
        if (((flight_mode == APP_FLIGHT_MODE_ALTHOLD) || (flight_mode == APP_FLIGHT_MODE_NAV_VELOCITY_BRAKE)) &&
            (baro_healthy_now != 0U) &&
            (Baro_GetAltitudeM() >= APP_BARO_VZ_DAMP_MIN_ALT_M))
        {
          /* Also gated on altitude (same APP_BARO_VZ_DAMP_MIN_ALT_M ground-effect
           * threshold already used for baro Vz damping above) - without this, a
           * brief natural pause in the pilot's throttle push during liftoff/climb-
           * out (routine - throttle is rarely pushed in one perfectly smooth
           * motion) could satisfy the settle timer and latch a hold target while
           * altitude/climb-rate readings are still ground-effect-noisy, producing
           * a jumpy unwanted correction right at liftoff instead of a clean climb. */
          /* Re-latch the hover-throttle reference to wherever the stick has
           * genuinely SETTLED (stayed within a small window for a while), not
           * a fixed APP_PWM_MID_US=1500 (real hover throttle is wherever this
           * aircraft's thrust/weight puts it, observed ~1200-1270us here,
           * nowhere near mid-stick) and not just once at mode entry (which can
           * freeze on an unrepresentative value like idle throttle before
           * liftoff, and then never update again once the stick moves outside
           * the deadband of that bad reference). */
          throttle_offset_us = (int32_t)throttle_us - (int32_t)althold_settle_ref_us;
          if ((throttle_offset_us > (int32_t)APP_ALTHOLD_STICK_STABLE_WINDOW_US) ||
              (throttle_offset_us < -(int32_t)APP_ALTHOLD_STICK_STABLE_WINDOW_US))
          {
            althold_settle_ref_us = (uint16_t)throttle_us;
            althold_settle_start_ms = now_ms;
          }
          else if ((now_ms - althold_settle_start_ms) >= APP_ALTHOLD_STICK_SETTLE_MS)
          {
            althold_center_throttle_us = althold_settle_ref_us;
          }

          throttle_offset_us = (int32_t)throttle_us - (int32_t)althold_center_throttle_us;
          if ((throttle_offset_us > -(int32_t)APP_ALTHOLD_THROTTLE_DEADBAND_US) &&
              (throttle_offset_us < (int32_t)APP_ALTHOLD_THROTTLE_DEADBAND_US))
          {
            if (althold_holding == 0U)
            {
              althold_target_alt_m = Baro_GetAltitudeM();
              althold_integral_us = 0.0f;
              althold_holding = 1U;
            }

            climb_rate_setpoint_mps = App_ClampFloat(APP_ALTHOLD_ALT_HOLD_KP_MPS_PER_M *
                                                      (althold_target_alt_m - Baro_GetAltitudeM()),
                                                      -APP_ALTHOLD_MAX_CLIMB_MPS,
                                                      APP_ALTHOLD_MAX_CLIMB_MPS);
            climb_rate_error_mps = climb_rate_setpoint_mps - Baro_GetClimbRateMps();
            althold_integral_us = App_ClampFloat(althold_integral_us +
                                                 (climb_rate_error_mps * APP_ALTHOLD_VZ_KI_US_PER_MPS_S * dt_s),
                                                 -(float)APP_ALTHOLD_INTEGRAL_LIMIT_US,
                                                 (float)APP_ALTHOLD_INTEGRAL_LIMIT_US);

            althold_trim_us = App_ClampInt32((int32_t)((climb_rate_error_mps * APP_ALTHOLD_VZ_KP_US_PER_MPS) +
                                                        althold_integral_us),
                                             -(int32_t)APP_ALTHOLD_TRIM_LIMIT_US,
                                             (int32_t)APP_ALTHOLD_TRIM_LIMIT_US);
          }
          else
          {
            /* Pilot is actively commanding a climb/descend - give full manual
             * authority over raw throttle_us, no hold trim fighting the stick.
             * The settle-tracking above will re-latch althold_center_throttle_us
             * once the stick genuinely stops here, so hold resumes at the new
             * altitude without needing to return to the old reference. */
            althold_holding = 0U;
            althold_integral_us = 0.0f;
          }
        }
        else
        {
          althold_holding = 0U;
          althold_integral_us = 0.0f;
          althold_settle_ref_us = (uint16_t)throttle_us;
          althold_settle_start_ms = now_ms;
        }
        /* Smooth the trim itself (not just the gains feeding it) - noisy baro
         * climb-rate readings were translating directly into jerky per-iteration
         * motor changes on the Z axis. Filters toward 0 the same way when not
         * holding, so re-engaging/disengaging the hold is a smooth ramp too. */
        {
          float althold_trim_lpf_alpha = App_LpfAlpha(dt_s, APP_ALTHOLD_TRIM_LPF_HZ);
          althold_trim_filtered_us += althold_trim_lpf_alpha *
                                      ((float)althold_trim_us - althold_trim_filtered_us);
        }
        althold_trim_us = (int32_t)althold_trim_filtered_us;
        /* ALTHOLD never overrides the pilot's raw throttle - it only ever adds a
         * small bounded trim on top, so a bad baro reading/integral can nudge but
         * never freeze or run away with the actual output. */
        althold_throttle_us = (int32_t)throttle_us + althold_trim_us;

        baro_damp_term_us = 0;
        if ((APP_BARO_VZ_DAMP_GAIN_US_PER_MPS != 0.0f) && (baro_healthy_now != 0U) &&
            (Baro_GetAltitudeM() >= APP_BARO_VZ_DAMP_MIN_ALT_M))
        {
          baro_damp_term_us = (int32_t)(-APP_BARO_VZ_DAMP_GAIN_US_PER_MPS * Baro_GetClimbRateMps());
          baro_damp_term_us = App_ClampInt32(baro_damp_term_us,
                                            -((int32_t)APP_BARO_VZ_DAMP_LIMIT_US),
                                            (int32_t)APP_BARO_VZ_DAMP_LIMIT_US);
        }
        throttle_term = althold_throttle_us + baro_damp_term_us;
        if (throttle_term < (int32_t)APP_MOTOR_IDLE_US)
        {
          throttle_term = APP_MOTOR_IDLE_US;
        }
        throttle_term = App_ClampInt32(throttle_term,
                                       (int32_t)APP_MOTOR_IDLE_US,
                                       (int32_t)APP_THROTTLE_MAX_US);

        /* Keep startup spool-up symmetric: suppress attitude/yaw correction
         * very close to idle so one motor does not start noticeably earlier. */
        if (throttle_term <= ((int32_t)APP_MOTOR_IDLE_US + (int32_t)APP_LOW_THROTTLE_MIX_DISABLE_US))
        {
          roll_term = 0;
          pitch_term = 0;
          yaw_term = 0;
          roll_integral_dps_s = 0.0f;
          pitch_integral_dps_s = 0.0f;
          yaw_integral_dps_s = 0.0f;
          pid_state_initialized = 0U;
          liftoff_ramp_active = 1U;
          liftoff_ramp_start_ms = now_ms;
        }
        else if (liftoff_ramp_active != 0U)
        {
          liftoff_ramp_elapsed_ms = now_ms - liftoff_ramp_start_ms;
          if (liftoff_ramp_elapsed_ms >= APP_LIFTOFF_RAMP_MS)
          {
            liftoff_ramp_active = 0U;
          }
          else
          {
            liftoff_ramp_factor = (float)liftoff_ramp_elapsed_ms / (float)APP_LIFTOFF_RAMP_MS;
            roll_term = (int32_t)((float)roll_term * liftoff_ramp_factor);
            pitch_term = (int32_t)((float)pitch_term * liftoff_ramp_factor);
            yaw_term = (int32_t)((float)yaw_term * liftoff_ramp_factor);
          }
        }

        m_front_left = throttle_term + pitch_term + roll_term - yaw_term;
        m_front_right = throttle_term + pitch_term - roll_term + yaw_term;
        m_rear_right = throttle_term - pitch_term - roll_term - yaw_term;
        m_rear_left = throttle_term - pitch_term + roll_term + yaw_term;

        /* Keep mixer authority at high throttle by shifting all motors together
         * instead of clipping each output independently. */
        mix_max = m_front_left;
        if (m_front_right > mix_max)
        {
          mix_max = m_front_right;
        }
        if (m_rear_right > mix_max)
        {
          mix_max = m_rear_right;
        }
        if (m_rear_left > mix_max)
        {
          mix_max = m_rear_left;
        }

        mix_min = m_front_left;
        if (m_front_right < mix_min)
        {
          mix_min = m_front_right;
        }
        if (m_rear_right < mix_min)
        {
          mix_min = m_rear_right;
        }
        if (m_rear_left < mix_min)
        {
          mix_min = m_rear_left;
        }

        if (mix_max > (int32_t)APP_PWM_MAX_US)
        {
          mix_offset = mix_max - (int32_t)APP_PWM_MAX_US;
          m_front_left -= mix_offset;
          m_front_right -= mix_offset;
          m_rear_right -= mix_offset;
          m_rear_left -= mix_offset;
        }

        mix_min = m_front_left;
        if (m_front_right < mix_min)
        {
          mix_min = m_front_right;
        }
        if (m_rear_right < mix_min)
        {
          mix_min = m_rear_right;
        }
        if (m_rear_left < mix_min)
        {
          mix_min = m_rear_left;
        }

        if (mix_min < (int32_t)APP_PWM_MIN_US)
        {
          mix_offset = (int32_t)APP_PWM_MIN_US - mix_min;
          m_front_left += mix_offset;
          m_front_right += mix_offset;
          m_rear_right += mix_offset;
          m_rear_left += mix_offset;
        }

        s1_us = App_ClampPulseUs(m_front_left);
        s2_us = App_ClampPulseUs(m_front_right);
        s3_us = App_ClampPulseUs(m_rear_right);
        s4_us = App_ClampPulseUs(m_rear_left);

        /* Physical channels map as: CH1=LA, CH2=LF, CH3=RA, CH4=RF. */
        Motors_WriteUs(s4_us, s1_us, s3_us, s2_us);

        if (g_sdlog_active != 0U)
        {
          App_SdLogRecord_t sdlog_rec;

          sdlog_rec.time_ms = now_ms;
          sdlog_rec.setpoint_roll_dps = (int16_t)cmd_roll_rate_dps;
          sdlog_rec.setpoint_pitch_dps = (int16_t)cmd_pitch_rate_dps;
          sdlog_rec.setpoint_yaw_dps = (int16_t)cmd_yaw_rate_dps;
          sdlog_rec.gyro_roll_dps_x10 = (int16_t)(measured_roll_rate_dps * 10.0f);
          sdlog_rec.gyro_pitch_dps_x10 = (int16_t)(measured_pitch_rate_dps * 10.0f);
          sdlog_rec.gyro_yaw_dps_x10 = (int16_t)(measured_yaw_rate_dps * 10.0f);
          sdlog_rec.pid_roll_us = (int16_t)roll_term;
          sdlog_rec.pid_pitch_us = (int16_t)pitch_term;
          sdlog_rec.pid_yaw_us = (int16_t)yaw_term;
          sdlog_rec.motor_fl_us = s1_us;
          sdlog_rec.motor_fr_us = s2_us;
          sdlog_rec.motor_rr_us = s3_us;
          sdlog_rec.motor_rl_us = s4_us;
          sdlog_rec.battery_decivolts = (uint8_t)App_ClampFloat(battery_voltage_filtered_v * 10.0f, 0.0f, 255.0f);
          sdlog_rec.flags = (uint8_t)(APP_SDLOG_FLAG_ARMED |
                                      (((uint8_t)flight_mode << APP_SDLOG_FLAG_MODE_SHIFT) & APP_SDLOG_FLAG_MODE_MASK) |
                                      ((receiver_state.link_active != 0U) ? APP_SDLOG_FLAG_LINK_ACTIVE : 0U) |
                                      ((Mag_IsHealthy() != 0U) ? APP_SDLOG_FLAG_MAG_HEALTHY : 0U));
          sdlog_rec.pitch_deg_x10 = (int16_t)(pitch_deg * 10.0f);
          sdlog_rec.roll_deg_x10 = (int16_t)(roll_deg * 10.0f);
          /* target_roll/pitch_deg are only meaningful (set this iteration) in an angle-mode branch. */
          if ((flight_mode == APP_FLIGHT_MODE_ATTITUDE) || (flight_mode == APP_FLIGHT_MODE_ALTHOLD) ||
              (flight_mode == APP_FLIGHT_MODE_NAV_VELOCITY_BRAKE))
          {
            sdlog_rec.target_pitch_deg_x10 = (int16_t)(target_pitch_deg * 10.0f);
            sdlog_rec.target_roll_deg_x10 = (int16_t)(target_roll_deg * 10.0f);
          }
          else
          {
            sdlog_rec.target_pitch_deg_x10 = 0;
            sdlog_rec.target_roll_deg_x10 = 0;
          }
          sdlog_rec.baro_alt_cm = (int16_t)(Baro_GetAltitudeM() * 100.0f);
          sdlog_rec.baro_vz_cms = (int16_t)(Baro_GetClimbRateMps() * 100.0f);
          sdlog_rec.throttle_cmd_us = (int16_t)throttle_us;
          sdlog_rec.throttle_actual_us = (int16_t)throttle_term;
          sdlog_rec.arm_us = arm_us;
          sdlog_rec.yaw_deg_x10 = (int16_t)(yaw_deg * 10.0f);
          sdlog_rec.mag_heading_x10 = (uint16_t)(Mag_GetHeadingDeg(mag_tilt_roll_deg, mag_tilt_pitch_deg) * 10.0f);

          sdlog_rec.nav_flags = (uint8_t)((nav_brake_requested != 0U ? APP_SDLOG_NAV_FLAG_REQUESTED : 0U) |
                                          (nav_brake_active != 0U ? APP_SDLOG_NAV_FLAG_ACTIVE : 0U) |
                                          (nav_brake_tilt_limited != 0U ? APP_SDLOG_NAV_FLAG_TILT_LIMITED : 0U) |
                                          (nav_brake_accel_limited != 0U ? APP_SDLOG_NAV_FLAG_ACCEL_LIMITED : 0U) |
                                          (nav_state.new_sample != 0U ? APP_SDLOG_NAV_FLAG_NEW_SAMPLE : 0U) |
                                          ((nav_state.rejected_count > 0U) &&
                                           (nav_state.invalid_reason == NAV_INVALID_REASON_POSITION_JUMP ||
                                            nav_state.invalid_reason == NAV_INVALID_REASON_VELOCITY_JUMP ||
                                            nav_state.invalid_reason == NAV_INVALID_REASON_BAD_UPDATE_INTERVAL) ?
                                           APP_SDLOG_NAV_FLAG_REJECTED : 0U) |
                                          (nav_state.reference_valid != 0U ? APP_SDLOG_NAV_FLAG_REF_VALID : 0U));
          sdlog_rec.nav_invalid_reason = (uint8_t)nav_state.invalid_reason;
          sdlog_rec.nav_fix_type = nav_state.fix_type;
          sdlog_rec.nav_num_sv = nav_state.num_sv;
          sdlog_rec.nav_h_acc_cm = (uint16_t)App_ClampFloat(nav_state.h_acc_m * 100.0f, 0.0f, 65535.0f);
          sdlog_rec.nav_age_ms = (uint16_t)((nav_state.age_ms > 65535U) ? 65535U : nav_state.age_ms);
          sdlog_rec.nav_update_period_ms = (uint16_t)((nav_state.update_period_ms > 65535U) ? 65535U : nav_state.update_period_ms);
          sdlog_rec.nav_consecutive_valid = (uint16_t)((nav_state.consecutive_valid > 65535U) ? 65535U : nav_state.consecutive_valid);
          sdlog_rec.nav_dropout_count = (uint16_t)((nav_state.dropout_count > 65535U) ? 65535U : nav_state.dropout_count);
          sdlog_rec.nav_north_m_x10 = (int16_t)(nav_state.north_m * 10.0f);
          sdlog_rec.nav_east_m_x10 = (int16_t)(nav_state.east_m * 10.0f);
          sdlog_rec.nav_raw_vel_n_x100 = (int16_t)(nav_state.raw_north_vel_mps * 100.0f);
          sdlog_rec.nav_raw_vel_e_x100 = (int16_t)(nav_state.raw_east_vel_mps * 100.0f);
          sdlog_rec.nav_filt_vel_n_x100 = (int16_t)(nav_state.filtered_north_vel_mps * 100.0f);
          sdlog_rec.nav_filt_vel_e_x100 = (int16_t)(nav_state.filtered_east_vel_mps * 100.0f);
          sdlog_rec.nav_desired_vel_n_x100 = (int16_t)(nav_brake_desired_north_vel_mps * 100.0f);
          sdlog_rec.nav_desired_vel_e_x100 = (int16_t)(nav_brake_desired_east_vel_mps * 100.0f);
          sdlog_rec.nav_vel_error_n_x100 = (int16_t)(nav_brake_north_vel_error_mps * 100.0f);
          sdlog_rec.nav_vel_error_e_x100 = (int16_t)(nav_brake_east_vel_error_mps * 100.0f);
          sdlog_rec.nav_accel_cmd_n_x1000 = (int16_t)(nav_brake_north_accel_cmd_mps2 * 1000.0f);
          sdlog_rec.nav_accel_cmd_e_x1000 = (int16_t)(nav_brake_east_accel_cmd_mps2 * 1000.0f);
          sdlog_rec.nav_accel_cmd_fwd_x1000 = (int16_t)(nav_brake_fwd_accel_cmd_mps2 * 1000.0f);
          sdlog_rec.nav_accel_cmd_right_x1000 = (int16_t)(nav_brake_right_accel_cmd_mps2 * 1000.0f);
          sdlog_rec.pilot_roll_stick_us = pilot_roll_stick_us;
          sdlog_rec.pilot_pitch_stick_us = pilot_pitch_stick_us;

          App_BlackboxCapture(&sdlog_rec);
          if ((sdlog_decim_counter++ % APP_SDLOG_DECIMATION) == 0U)
          {
            App_SdLogAppendRecord(&sdlog_rec);
          }
        }
      }
    }
  }

  if ((now_ms - last_rx16_telemetry_ms) >= APP_RX16_TELEMETRY_MS)
  {
    Telemetry_PrintReceiverState16(&receiver_state);
    last_rx16_telemetry_ms = now_ms;
  }

  if ((now_ms - last_mode_telemetry_ms) >= 250U)
  {
    Telemetry_PrintFlightMode(App_FlightModeName(flight_mode), mode_us);
    last_mode_telemetry_ms = now_ms;
  }

#if APP_ENABLE_ARM_RUNTIME_TELEMETRY
  if ((now_ms - last_arm_telemetry_ms) >= APP_ARM_TELEMETRY_MS)
  {
    Telemetry_PrintArmState(motors_armed,
                            arm_switch_high,
                            throttle_low,
                            throttle_us,
                            s1_us,
                            s2_us,
                            s3_us,
                            s4_us);
    last_arm_telemetry_ms = now_ms;
  }
#endif

  if ((now_ms - last_battery_sample_ms) >= APP_BATTERY_SAMPLE_MS)
  {
    if (App_ReadBatteryVoltage(&battery_voltage_v, &battery_adc_raw) != 0U)
    {
      if (battery_voltage_valid == 0U)
      {
        battery_voltage_filtered_v = battery_voltage_v;
        battery_voltage_valid = 1U;
      }
      else
      {
        battery_voltage_filtered_v += APP_BATTERY_FILTER_ALPHA * (battery_voltage_v - battery_voltage_filtered_v);
      }
    }
    last_battery_sample_ms = now_ms;
  }

  if ((IMU_GetType() == IMU_TYPE_UNKNOWN) && ((detect_retry_counter++ % 10U) == 0U))
  {
    if (IMU_DetectAndInit() == HAL_OK)
    {
      Telemetry_PrintImuDetected(IMU_GetType(), IMU_GetWhoAmI());
    }
  }

  if ((IMU_GetType() != IMU_TYPE_UNKNOWN) && (IMU_ReadRawAligned(&imu_raw) == HAL_OK))
  {
    ax_g = ((float)imu_raw.accel_x) / IMU_ACCEL_LSB_PER_G;
    ay_g = ((float)imu_raw.accel_y) / IMU_ACCEL_LSB_PER_G;
    az_g = ((float)imu_raw.accel_z) / IMU_ACCEL_LSB_PER_G;
    gx_dps = ((float)imu_raw.gyro_x) / IMU_GYRO_LSB_PER_DPS;
    gy_dps = ((float)imu_raw.gyro_y) / IMU_GYRO_LSB_PER_DPS;
    gz_dps = ((float)APP_GYRO_YAW_SIGN) * (((float)imu_raw.gyro_z) / IMU_GYRO_LSB_PER_DPS);
    (void)App_UpdateGyroBias(ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, motors_armed, throttle_us);
    gx_dps -= g_roll_gyro_bias_dps;
    gy_dps -= g_pitch_gyro_bias_dps;
    gz_dps -= g_yaw_gyro_bias_dps;
    gz_dps += mag_yaw_nudge_dps;

    gyro_lpf_alpha = App_LpfAlpha(dt_s, APP_GYRO_RATE_LPF_HZ);
    filtered_gyro_roll_rate_dps += gyro_lpf_alpha * (gx_dps - filtered_gyro_roll_rate_dps);
    filtered_gyro_pitch_rate_dps += gyro_lpf_alpha * (gy_dps - filtered_gyro_pitch_rate_dps);
    filtered_gyro_yaw_rate_dps += gyro_lpf_alpha * (gz_dps - filtered_gyro_yaw_rate_dps);

    measured_roll_rate_dps = filtered_gyro_roll_rate_dps;
    measured_pitch_rate_dps = filtered_gyro_pitch_rate_dps;
    measured_yaw_rate_dps = filtered_gyro_yaw_rate_dps;

    /* GLOG captures the raw, unfiltered gyro (not measured_*_rate_dps) so vibration
     * frequency/amplitude is still visible for diagnosis even with feedback filtered. */
    if ((g_glog_capturing != 0U) && (g_glog_count < APP_GLOG_MAX_SAMPLES))
    {
      g_glog_gx_x10[g_glog_count] = (int16_t)(gx_dps * 10.0f);
      g_glog_gy_x10[g_glog_count] = (int16_t)(gy_dps * 10.0f);
      g_glog_gz_x10[g_glog_count] = (int16_t)(gz_dps * 10.0f);
      g_glog_count++;
      if (g_glog_count >= APP_GLOG_MAX_SAMPLES)
      {
        g_glog_capturing = 0U;
      }
    }

    Attitude_UpdateIMU(gx_dps * RAD_PER_DEG,
                       gy_dps * RAD_PER_DEG,
                       gz_dps * RAD_PER_DEG,
                       ax_g,
                       ay_g,
                       az_g,
                       dt_s);
    Attitude_GetBoardAnglesDeg(&pitch_deg, &roll_deg, &yaw_deg);
    mag_tilt_roll_deg = roll_deg;
    mag_tilt_pitch_deg = pitch_deg;

    if (g_attitude_zero_request != 0U)
    {
      __disable_irq();
      g_attitude_zero_request = 0U;
      __enable_irq();
      startup_roll_offset_deg = roll_deg;
      startup_pitch_offset_deg = pitch_deg;
      startup_yaw_offset_deg = yaw_deg;
      attitude_zero_captured = 1U;
      startup_beep_active = 1U;
      startup_beep_start_ms = now_ms;
      HAL_GPIO_WritePin(BEEPER_GPIO_Port, BEEPER_Pin, GPIO_PIN_SET);
      printf("ATT_ZERO[OK]\r\n");
    }

    if (attitude_zero_captured == 0U)
    {
      if (now_ms >= (APP_ATTITUDE_ZERO_SETTLE_MS - APP_ATTITUDE_ZERO_AVG_MS))
      {
        startup_roll_offset_sum_deg += roll_deg;
        startup_pitch_offset_sum_deg += pitch_deg;
        startup_yaw_offset_sum_deg += yaw_deg;
        startup_zero_avg_sample_count++;
      }

      if ((now_ms >= APP_ATTITUDE_ZERO_SETTLE_MS) && (startup_zero_avg_sample_count > 0U))
      {
        startup_roll_offset_deg = startup_roll_offset_sum_deg / ((float)startup_zero_avg_sample_count);
        startup_pitch_offset_deg = startup_pitch_offset_sum_deg / ((float)startup_zero_avg_sample_count);
        startup_yaw_offset_deg = startup_yaw_offset_sum_deg / ((float)startup_zero_avg_sample_count);
        attitude_zero_captured = 1U;
        startup_beep_active = 1U;
        startup_beep_start_ms = now_ms;
        HAL_GPIO_WritePin(BEEPER_GPIO_Port, BEEPER_Pin, GPIO_PIN_SET);
      }
    }

    roll_deg = Attitude_WrapAngle180(roll_deg - startup_roll_offset_deg);
    pitch_deg = Attitude_WrapAngle180(pitch_deg - startup_pitch_offset_deg);
    yaw_deg = Attitude_WrapAngle180(yaw_deg - startup_yaw_offset_deg);

    /* Compute the next iteration's slow yaw drift-correction nudge here (see
     * APP_MAG_YAW_NUDGE_* above) - compares the compass's own rotation-since-ref
     * against the AHRS's rotation-since-boot, so boot heading stays 0 as today. */
    if ((attitude_zero_captured != 0U) && (Mag_IsHealthy() != 0U))
    {
      float mag_field_now_g = sqrtf((Mag_GetXGauss() * Mag_GetXGauss()) +
                                     (Mag_GetYGauss() * Mag_GetYGauss()) +
                                     (Mag_GetZGauss() * Mag_GetZGauss()));

      if (mag_yaw_ref_captured == 0U)
      {
        mag_heading_at_ref_deg = Mag_GetHeadingDeg(mag_tilt_roll_deg, mag_tilt_pitch_deg);
        mag_ref_field_g = mag_field_now_g;
        mag_yaw_ref_captured = 1U;
        mag_yaw_nudge_dps = 0.0f;
      }
      else if ((mag_ref_field_g > 0.01f) &&
               (fabsf(mag_field_now_g - mag_ref_field_g) / mag_ref_field_g > APP_MAG_TRUST_MAX_MAG_DEVIATION_FRAC))
      {
        mag_yaw_nudge_dps = 0.0f; /* field strength shifted too much - likely motor/ESC current interference */
      }
      else
      {
        float mag_delta_deg = Attitude_WrapAngle180(Mag_GetHeadingDeg(mag_tilt_roll_deg, mag_tilt_pitch_deg) - mag_heading_at_ref_deg);
        float yaw_error_deg = Attitude_WrapAngle180(mag_delta_deg - yaw_deg);

        mag_yaw_nudge_dps = App_ClampFloat(yaw_error_deg * APP_MAG_YAW_NUDGE_KP_DPS_PER_DEG,
                                            -APP_MAG_YAW_NUDGE_MAX_DPS, APP_MAG_YAW_NUDGE_MAX_DPS);
      }
    }
    else
    {
      mag_yaw_nudge_dps = 0.0f;
    }

    /* Stream while disarmed, or while armed with bench telemetry toggled on. */
    if ((APP_ENABLE_IMU_RUNTIME_TELEMETRY || (g_armed_test_telemetry_enabled != 0U) || (motors_armed == 0U)) &&
        ((now_ms - last_imu_telemetry_ms) >= APP_IMU_TELEMETRY_MS))
    {
      Telemetry_PrintImuState(ax_g,
                              ay_g,
                              az_g,
                              gx_dps,
                              gy_dps,
                              gz_dps,
                              pitch_deg,
                              roll_deg,
                              yaw_deg);
      if (battery_voltage_valid != 0U)
      {
        Telemetry_PrintBatteryState(battery_voltage_filtered_v, battery_adc_raw);
      }
      Telemetry_PrintBaroState(Baro_GetAltitudeM(), Baro_GetClimbRateMps(), Baro_IsHealthy());
      Telemetry_PrintGpsState(GPS_IsConfigured(), GPS_IsHealthy(), GPS_GetFixType(), GPS_GetNumSatellites(),
                             GPS_GetLatitudeDeg(), GPS_GetLongitudeDeg(), GPS_GetAltitudeM());
      Telemetry_PrintMagState(Mag_IsHealthy(), Mag_GetXGauss(), Mag_GetYGauss(), Mag_GetZGauss(),
                             Mag_GetHeadingDeg(mag_tilt_roll_deg, mag_tilt_pitch_deg));
      Telemetry_PrintNavState(nav_state.valid, nav_state.reference_valid, (uint8_t)nav_state.invalid_reason,
                             nav_state.fix_type, nav_state.num_sv, nav_state.h_acc_m,
                             nav_state.age_ms, nav_state.update_period_ms,
                             nav_state.consecutive_valid, nav_state.consecutive_invalid,
                             nav_state.duplicate_count, nav_state.rejected_count, nav_state.dropout_count);
      Telemetry_PrintNavPosVel(nav_state.north_m, nav_state.east_m,
                              nav_state.raw_north_vel_mps, nav_state.raw_east_vel_mps,
                              nav_state.filtered_north_vel_mps, nav_state.filtered_east_vel_mps);
      {
        /* Bench-test diagnostic (section 14): while DISARMED, motors_armed's mixer
         * block above never ran, so target_roll/pitch_deg and the nav_brake_* accel
         * fields are stale/irrelevant - recompute a "would-be" nav-brake command
         * here purely for bench validation (sign conventions, yaw rotation, GPS
         * velocity braking direction) using the SAME named transformation
         * functions as the real controller. This NEVER reaches the motors (motors
         * are already forced to idle/stopped while disarmed) and is clearly a
         * distinct print (bench values only substituted while disarmed below). */
        uint8_t bench_requested = nav_brake_requested;
        uint8_t bench_active = nav_brake_active;
        uint8_t bench_tilt_limited = nav_brake_tilt_limited;
        uint8_t bench_accel_limited = nav_brake_accel_limited;
        float bench_desired_n = nav_brake_desired_north_vel_mps;
        float bench_desired_e = nav_brake_desired_east_vel_mps;
        float bench_err_n = nav_brake_north_vel_error_mps;
        float bench_err_e = nav_brake_east_vel_error_mps;
        float bench_accel_n = nav_brake_north_accel_cmd_mps2;
        float bench_accel_e = nav_brake_east_accel_cmd_mps2;
        float bench_accel_fwd = nav_brake_fwd_accel_cmd_mps2;
        float bench_accel_right = nav_brake_right_accel_cmd_mps2;
        float bench_target_roll_deg = target_roll_deg;
        float bench_target_pitch_deg = target_pitch_deg;

        if (motors_armed == 0U)
        {
          /* No APP_PITCH_SIGN flip here - see the matching comment on the armed
           * path above; this bench block must stay in sync with it. */
          float fwd_cmd_mps = App_NavStickOffsetToVelocityMps((int32_t)pitch_us - (int32_t)pitch_center_us,
                                                              APP_NAV_BRAKE_MAX_VEL_MPS);
          float right_cmd_mps = ((float)APP_ROLL_SIGN) *
                                App_NavStickOffsetToVelocityMps((int32_t)roll_us - (int32_t)roll_center_us,
                                                                APP_NAV_BRAKE_MAX_VEL_MPS);
          float desired_n;
          float desired_e;

          Nav_RotateBodyToNed(fwd_cmd_mps, right_cmd_mps, yaw_deg, &desired_n, &desired_e);

          bench_err_n = desired_n - nav_state.filtered_north_vel_mps;
          bench_err_e = desired_e - nav_state.filtered_east_vel_mps;
          bench_accel_n = App_ClampFloat(APP_NAV_BRAKE_VELOCITY_KP * bench_err_n,
                                         -APP_NAV_BRAKE_MAX_ACCEL_MPS2, APP_NAV_BRAKE_MAX_ACCEL_MPS2);
          bench_accel_e = App_ClampFloat(APP_NAV_BRAKE_VELOCITY_KP * bench_err_e,
                                         -APP_NAV_BRAKE_MAX_ACCEL_MPS2, APP_NAV_BRAKE_MAX_ACCEL_MPS2);
          Nav_RotateNedToBody(bench_accel_n, bench_accel_e, yaw_deg, &bench_accel_fwd, &bench_accel_right);
          bench_target_pitch_deg = ((float)APP_PITCH_SIGN) * Nav_AccelToAngleDeg(bench_accel_fwd, APP_NAV_BRAKE_MAX_TILT_DEG);
          bench_target_roll_deg = ((float)APP_ROLL_SIGN) * Nav_AccelToAngleDeg(bench_accel_right, APP_NAV_BRAKE_MAX_TILT_DEG);
          bench_desired_n = desired_n;
          bench_desired_e = desired_e;
          bench_tilt_limited = (uint8_t)((fabsf(bench_target_pitch_deg) >= (APP_NAV_BRAKE_MAX_TILT_DEG - 0.01f)) ||
                                         (fabsf(bench_target_roll_deg) >= (APP_NAV_BRAKE_MAX_TILT_DEG - 0.01f)));
          bench_accel_limited = (uint8_t)((fabsf(bench_accel_n) >= (APP_NAV_BRAKE_MAX_ACCEL_MPS2 - 0.001f)) ||
                                          (fabsf(bench_accel_e) >= (APP_NAV_BRAKE_MAX_ACCEL_MPS2 - 0.001f)));
        }

        Telemetry_PrintNavBrake(bench_requested, bench_active, bench_tilt_limited, bench_accel_limited,
                               bench_desired_n, bench_desired_e,
                               bench_err_n, bench_err_e,
                               bench_accel_n, bench_accel_e,
                               bench_accel_fwd, bench_accel_right,
                               bench_target_roll_deg, bench_target_pitch_deg);
         printf("NAVGATE[nav ref att baro link latch]=[%u %u %u %u %u %u]\r\n",
           (unsigned int)nav_state.valid,
           (unsigned int)nav_state.reference_valid,
           (unsigned int)attitude_zero_captured,
           (unsigned int)Baro_IsHealthy(),
           (unsigned int)receiver_state.link_active,
           (unsigned int)nav_brake_disqualified_latch);
      }
      last_imu_telemetry_ms = now_ms;
    }
#if APP_ENABLE_ANGLE_TELEMETRY
    Telemetry_PrintAngles(pitch_deg, roll_deg, yaw_deg);
#endif
  }
  else if (APP_ENABLE_IMU_RUNTIME_TELEMETRY || (g_armed_test_telemetry_enabled != 0U) || (motors_armed == 0U))
  {
    Telemetry_PrintImuReadFailed(IMU_GetType(), IMU_GetWhoAmI());
  }

  HAL_Delay(APP_CONTROL_LOOP_MS);
}
