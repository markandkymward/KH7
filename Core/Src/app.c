#include "app.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "attitude.h"
#include "imu.h"
#include "baro.h"
#include "vert_ekf.h"
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
/* Low-battery beeper: fast triple-chirp every 2s, audibly distinct from the
 * continuous arm-blocked toggle and the single startup-ready chirp. Warns from
 * the aircraft's onboard piezo only - this firmware has no OSD and no
 * bidirectional CRSF telemetry back to the transmitter, so there is no
 * in-goggles/on-radio alert path, only what's audible near the aircraft.
 * Threshold is 3.5V/cell: comfortably above both typical ESC low-voltage
 * cutoff (~3.0-3.3V/cell) and the ~2.6V/cell the pack was flown down to on
 * 2026-08-21 (10.3-10.5V on a 4S), so there's real time to land before either
 * an ESC LVC event or genuine cell damage. Cell count is inferred once from
 * the first post-boot reading (smallest N with N*4.25V >= reading), mirroring
 * tools/sdlog_analyze.py's infer_nominal_battery_v(). */
#define APP_LOW_BATTERY_WARN_CELL_V    3.5f
#define APP_LOW_BATTERY_MAX_CHARGE_CELL_V 4.25f
#define APP_LOW_BATTERY_BEEP_CYCLE_MS  2000U
#define APP_LOW_BATTERY_BEEP_ON_MS     80U
#define APP_LOW_BATTERY_BEEP_GAP_MS    160U
/* Tiered low-battery response (2026-08-21, after a deep-discharge test flight
 * bottomed at ~2.6V/cell with no working local warning at the time):
 *   1. APP_LOW_BATTERY_WARN_CELL_V (3.5V/cell)     -> beeper + CRSF telemetry
 *      alarm only, no flight-behavior change.
 *   2. APP_LOW_BATTERY_CRITICAL_CELL_V (3.2V/cell)  -> throttle ceiling clamp
 *      (see APP_LOW_BATTERY_THROTTLE_CEILING_US) - pilot keeps full
 *      roll/pitch/yaw authority, but can no longer sustain hover/climb
 *      indefinitely, forcing a gradual controlled descent instead of an
 *      abrupt cutoff or uncontrolled ESC low-voltage-cutoff event.
 *   3. APP_LOW_BATTERY_DISARM_CELL_V (3.0V/cell)    -> hard auto-disarm via
 *      the existing debounced clean-disarm path (same as RX-loss/arm-switch),
 *      as a last-resort backstop if the pilot hasn't landed by then. Still
 *      above the ~2.6V/cell danger zone that flight reached, and above
 *      typical ESC LVC thresholds so it should trip well before either. */
#define APP_LOW_BATTERY_CRITICAL_CELL_V     3.2f
#define APP_LOW_BATTERY_CRITICAL_HYST_V     0.1f /* release margin, avoids rapid on/off right at the boundary */
#define APP_LOW_BATTERY_DISARM_CELL_V       3.0f
/* Well above this session's observed hover throttle (~1230-1290us) so the
 * pilot keeps meaningful climb/maneuver margin, well below APP_THROTTLE_MAX_US
 * (1880) so aggressive full-power draws are no longer possible. */
#define APP_LOW_BATTERY_THROTTLE_CEILING_US 1500U
#define APP_CRSF_BATTERY_TELEM_MS      1000U /* radio-side alarm only needs a slow update rate */
#define APP_CRSF_ATTITUDE_TELEM_MS     100U  /* ~10Hz, smooth-ish display */
#define APP_CRSF_VARIO_TELEM_MS        200U  /* ~5Hz */
#define APP_CRSF_FLIGHTMODE_TELEM_MS   500U  /* ~2Hz - changes rarely */
#define APP_CRSF_GPS_TELEM_MS          500U  /* ~2Hz - position doesn't need to be fast */
#define APP_ATTITUDE_ZERO_SETTLE_MS 2000U
/* Raised from 300ms on 2026-08-22: reproduced on 3 separate power-ons with the
 * board sitting genuinely still and level the whole time - the DISPLAYED
 * (offset-corrected) pitch read a consistent ~-10.8deg, while a live
 * comparison against MAGTILT (which logs roll/pitch BEFORE this offset is
 * subtracted) showed the RAW AHRS pitch had already settled to ~-0.4deg
 * (correctly near level) by the time of the comparison, meaning the ~10.4deg
 * baked into startup_pitch_offset_deg was captured before the AHRS had
 * actually converged. Attitude_Init() (attitude.c) starts the Mahony filter
 * from a level quaternion every boot regardless of true orientation, and a
 * brief real disturbance during power-on handling/USB connection (or a noisy
 * early accelerometer reading before the sensor electrically settles) can
 * push it away from level for a couple of seconds before the proportional
 * correction (two_kp=3.0) pulls it back - the old 300ms window, firing right
 * at the gyro-bias-ready boundary (~2.3s post-boot per the 2026-08-17 fix
 * above), was long enough to reliably land inside that recovery window every
 * time, not just occasionally. A much longer average dilutes any such
 * transient with many more genuinely-settled samples instead of trusting a
 * brief snapshot. Does not reintroduce the 2026-08-17 bug - that fix was
 * about WHEN the window starts (keyed off gyro-bias-ready, unchanged here),
 * this only changes how long it runs once started. */
#define APP_ATTITUDE_ZERO_AVG_MS 2000U
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
 * signal is low-noise/low-latency enough to not fight stick throttle inputs.
 *
 * RAISED then REVERTED, both 2026-08-29. First raised 15->30 (limit 60->120) while this
 * term still consumed a dedicated filtered BARO climb-rate - real flight data justified
 * it (neither this term nor the trim were ever seen saturating during real ALTHOLD
 * altitude wander, the same "ceiling wasn't the limiter, the gain was" signature as the
 * trim history below), but a same-day flight test showed the doubled gain made NO
 * measurable difference to the actual altitude wander (fused-height std unchanged:
 * 0.256m vs a 0.240m baseline) - ruling out damping/authority as the limiter rather than
 * fixing anything. Reverted back to 15/60 the same day when this term (along with the
 * relatch gate and climb_rate_error) switched from baro's climb-rate to
 * VertEkf_GetClimbRateMps() entirely (see APP_ALTHOLD_VZ_KP_US_PER_MPS's comment above
 * for that switch and why its own gains were also lowered, not raised, for the same
 * less-lagged-signal reasoning) - no evidence supports keeping the raise for a
 * fundamentally different, snappier input signal. Re-validate with the same
 * stick-held-steady bench/hover discipline as every other change to this loop before
 * trusting it for real hands-off flight. */
#define APP_BARO_VZ_DAMP_GAIN_US_PER_MPS 15.0f
#define APP_BARO_VZ_DAMP_LIMIT_US        60U
/* Live-tunable override for the damping gain above (2026-08-30) - a real flight at
 * raised inner-loop VZ Kp/Ki (see that constant's comment) showed raising P/I gain did
 * NOT meaningfully change a real, repeating ~8-10s-period height wander (~0.4-0.5m
 * amplitude) - consistent with a delay-dominated oscillation, which more proportional/
 * integral authority against a lagged signal does not fix and can worsen. Damping
 * (a rate-opposing term) is the textbook lever for exactly this failure mode instead.
 * The 2026-08-29 negative result on doubling this same gain (comment above) predates
 * the switch to VertEkf_GetClimbRateMps() for this whole loop and used a since-removed
 * dedicated filtered baro signal - not evidence against retrying now on the current,
 * different signal path. `damp_us` never exceeded 12 of its 60us limit on the flight
 * that motivated this - real headroom exists. Exposed live-tunable for the same
 * incremental, GET/Read-verified-before-flying discipline as every other gain tonight.
 * Bounds (APP_BARO_VZ_DAMP_GAIN_MIN/MAX, APP_BARO_VZ_DAMP_LIMIT_MIN/MAX) live in app.h,
 * next to the getter/setter declarations - same pattern as every other live-tunable
 * ALTHOLD gain this session. */
static float g_baro_vz_damp_gain_us_per_mps = APP_BARO_VZ_DAMP_GAIN_US_PER_MPS;
static uint32_t g_baro_vz_damp_limit_us = APP_BARO_VZ_DAMP_LIMIT_US;
/* Added 2026-08-22: this term is computed fresh every 2ms control-loop tick straight
 * from the instantaneous climb-rate estimate with no smoothing of its own - unlike the
 * ALTHOLD trim above it, which IS filtered (APP_ALTHOLD_TRIM_LPF_HZ) before reaching the
 * motors. That raw pass-through got noticeably noisier the same night after
 * BARO_VZ_COMPL_FILTER_TAU_S was shortened 2.0s->0.5s (see baro.c) to fix climb-rate sign
 * lag during maneuvers - trading noise-smoothing for faster correction there directly
 * shows up here, since this term applies that faster/noisier signal unfiltered. A light
 * dedicated LPF here cuts the sample-to-sample jitter this term itself contributes to
 * throttle without touching the underlying climb-rate estimate (which other consumers,
 * e.g. ALTHOLD's trim integrator, still see raw/fast). Kept well above the ALTHOLD trim's
 * 1.5Hz since this term's whole purpose is reacting quickly to counter thrust-response
 * lag - only enough smoothing to cut obvious per-sample chatter, not to slow it down. */
#define APP_BARO_VZ_DAMP_LPF_HZ           5.0f
/* Ground effect/propwash makes baro pressure noisy close to the ground, which
 * otherwise makes the Vz damping term above (and the ALTHOLD trim gate that reuses
 * this same threshold) rapidly flip sign/saturate right at liftoff (audible motor
 * thrust jitter) - suppress both until climbed clear. Lowered from 1.0m on
 * 2026-08-22: with the baro propwash altitude compensation added the same night
 * (see baro.c), readings are trustworthy much closer to the ground, and 1.0m was
 * gating ALTHOLD assistance off for 43-54% of two real flights whose median hover
 * altitude was ~1.0m - confirmed via a host-side replay of the ALTHOLD control law
 * against those flights' logged baro data, which showed the trim itself staying
 * small/well-damped (nowhere near its clamp) whenever active, meaning the "cyclical
 * altitude chasing" complaint was this gate flickering ALTHOLD on/off across the
 * pilot's normal hover height, not a control-loop stability bug.
 *
 * LOWERED AGAIN 2026-08-30, this time from 0.3f to vert_ekf.c's own
 * VERT_EKF_GROUND_EFFECT_ZONE_M (2x rotor diameter = 0.254m for this
 * airframe's 5-inch props), via VertEkf_GetGroundEffectZoneM() - real flight
 * data that night (liftoff_capture_20260830_accelclamp.txt) showed `ah_hold`
 * dropping to 0 nine times in one flight with the throttle stick VERIFIED
 * pinned at a single fixed PWM value the entire time (checked the raw RX16
 * telemetry directly - not a perception issue), every single drop coinciding
 * with fused height dipping to 0.13-0.29m: ordinary, already-characterized
 * height noise crossing this gate on flights held at deliberately low
 * (0.31-0.55m) targets, not any real stick input. The prior 0.3f value was
 * chosen ad hoc when this constant was first lowered from 1.0f (see above) -
 * never itself derived from data. 0.254m is not a fresh guess either: it is
 * the EXACT height vert_ekf.c's own baro ground-effect noise model already
 * uses as the point its extra R-inflation has fully tapered to zero (see
 * VERT_EKF_GROUND_EFFECT_ZONE_M's comment) - it was never consistent for this
 * hard on/off gate to demand MORE clearance (0.3m) than the smooth model
 * backing the estimate it gates already considers ground-effect-clear. Now a
 * single shared value (via the getter) instead of two independent numbers
 * that could silently drift apart. */
#define APP_BARO_VZ_DAMP_MIN_ALT_M       VertEkf_GetGroundEffectZoneM()
/* Added 2026-08-23 after tracing a real, felt Z-axis jolt right at liftoff across every
 * flight on the card: reported altitude bounces back and forth across
 * APP_BARO_VZ_DAMP_MIN_ALT_M several times within under a second during the actual
 * ground-to-air transition (both the propwash pressure artifact settling out AND the
 * real climb starting at once), so the plain >= comparison above re-triggers a FRESH
 * proportional (and, in ALTHOLD, integral) correction from whatever momentarily-noisy
 * climb-rate reading exists at each crossing - measured up to +4.5m/s spikes right at
 * this boundary. That sudden throttle correction, combined with the mixer, is tightly
 * time-correlated (same or adjacent sample, every flight examined) with large roll/pitch
 * rate spikes (40-110+ deg/s) and individual motors pinned to their PWM rails - this
 * gate flickering is believed to be the dominant cause of the jolt (an ATTITUDE-only
 * flight with no ALTHOLD trim engaged, so only the smaller-clamped Vz-damping term was
 * subject to the same flicker, showed only a mild version). Fixed with a minimum
 * continuous-dwell debounce (see ground_effect_clear below) instead of widening this
 * threshold - a single noisy dip below now requires the full dwell again before
 * re-engaging, rather than instantly re-arming on the next noisy tick above it. */
#define APP_GROUND_EFFECT_CLEAR_DWELL_MS 300U
/* REVERTED 2026-08-21: a same-day attempt to map the full throttle stick range to
 * a commanded climb rate (fixing "Z axis very sensitive to throttle") caused a
 * real in-flight uncontrolled climb. Root cause: althold_center_throttle_us is a
 * fragile one-shot latch that can lock onto a transient, lower-than-intended
 * value during a dynamic liftoff. In this original design that's benign - being
 * "outside the deadband" just falls back to direct manual throttle. In the
 * full-range redesign it meant a PERSISTENT climb-rate command with no natural
 * way for the pilot to get back into the deadband (their normal hover throttle
 * permanently read as "above center"). Reverted to known-safe raw-passthrough
 * behavior. See memory kh7-althold-throttle-incident for the full writeup.
 *
 * Same day, separately: also widened the deadband (was 40U) and reset the
 * hover-throttle reference fresh on every arm (see the arm-transition block
 * above) and decoupled the reference tracking itself from the ground-effect
 * altitude gate (see where althold_settle_ref_us/althold_center_throttle_us are
 * updated below) - a stale/wrong reference was the most likely explanation for
 * "altitude is very difficult to hold" even in this unmodified raw-passthrough
 * design, independent of the incident above.
 *
 * ALTHOLD reuses ATTITUDE-mode roll/pitch/yaw angle stabilization. Throttle
 * stick is FULL AUTHORITY, climb-rate-command design (2026-08-23, replacing
 * an earlier raw-passthrough-plus-small-trim design): centered (within
 * APP_ALTHOLD_THROTTLE_DEADBAND_US) LOCKS the altitude captured the instant
 * it centers; off-center commands a climb/descend RATE proportional to
 * stick deflection (full stick = APP_ALTHOLD_MAX_CLIMB_MPS), not a raw
 * throttle value - the stick no longer passes through to the motors at all
 * once engaged, not even below the ground-effect/baro-health gate (see
 * APP_ALTHOLD_LIFTOFF_ASSIST_MAX_US's comment).
 *
 * "Center" is NOT a fixed PWM value (APP_PWM_MID_US=1500 was tried first and
 * reverted the same day - see althold_settled_center_us's declaration
 * comment for why a fixed value can't work here). It's wherever the pilot's
 * stick genuinely SETTLES (same settle-latch idiom used elsewhere in this
 * file, e.g. yaw_settle_ref_us, but a much longer duration - see
 * APP_ALTHOLD_THROTTLE_SETTLE_MS's comment for why) - hold the stick still
 * long enough and that position becomes center, exactly matching how a
 * pilot naturally flies: find a comfortable throttle, hold it, expect the
 * aircraft to hold that height from there. The stick's physical position
 * never has to correspond to any particular thrust value -
 * it only ever means "settled here" vs "displaced from there by this much" -
 * so it works regardless of this airframe's actual hover throttle.
 *
 * The altitude fed into the hold/gate logic is ALSO not baro alone - it comes
 * from vert_ekf.c (2026-08-23, replaced by a real Kalman filter 2026-08-29 -
 * see that file's top-of-file design comment), which fuses baro against
 * rangefinder/TF-Luna specifically because baro has a real, characterized
 * bias/transient right at liftoff that the range sensors are immune to.
 *
 * This exact design (full-range stick-to-climb-rate) was attempted once
 * before and reverted after a real in-flight uncontrolled climb (2026-08-21,
 * see kh7-althold-throttle-incident memory) - root cause was NOT the
 * climb-rate mapping itself, it was that the hover-throttle reference lived
 * entirely inside the altitude gate (froze at a stale cross-flight value
 * until the gate opened, then got used for a full-authority correction with
 * no warm-up) and was never reset on arm. Both are fixed here:
 * althold_hover_throttle_us updates every iteration ALTHOLD/NAVBRAKE is
 * selected regardless of the gate (see its declaration and the throttle
 * block below), and is reset to a safe seed on every arm (see the arm-
 * transition block). It's also hard-clamped to
 * [APP_ALTHOLD_HOVER_EST_MIN_US, APP_ALTHOLD_HOVER_EST_MAX_US] and can only
 * drift at APP_ALTHOLD_HOVER_EST_KI_US_PER_MPS_S per second of sustained
 * climb-rate error - it cannot jump. The fast/reactive trim on top of it
 * keeps the exact same bound as the old design
 * (APP_ALTHOLD_TRIM_LIMIT_US) for the same reason: even a wrong hover
 * estimate can only be fought by a bounded amount on any single iteration.
 * Whatever this whole block computes is still clamped to
 * [APP_MOTOR_IDLE_US, APP_THROTTLE_MAX_US] same as manual flight (see
 * throttle_term below) - full authority here is bounded by the same ceiling
 * full-stick manual throttle already has, not something new.
 *
 * Treat the first flight after any change in this block as a deliberate
 * low-altitude hover test, not normal flying - the prior incident triggered
 * exactly at the ground-effect gate crossing. */
#define APP_ALTHOLD_THROTTLE_DEADBAND_US   100U
/* Stillness tolerance shared with yaw-hold's own settle-latch (see
 * yaw_settle_ref_us/yaw_settle_start_ms below) - same idiom, two independent
 * instances (throttle center, yaw heading). The DURATION threshold is
 * deliberately NOT shared - see APP_ALTHOLD_THROTTLE_SETTLE_MS below. */
#define APP_ALTHOLD_STICK_STABLE_WINDOW_US 15U
#define APP_ALTHOLD_STICK_SETTLE_MS        300U
/* Throttle-specific settle duration (2026-08-23) - deliberately much longer
 * than yaw-hold's 300ms above. Confirmed via a host-side replay of a real
 * flight log: a 300ms pause is common and unremarkable during a smooth,
 * continuous stick push (climbing out), but the settle-latch can't tell that
 * apart from a deliberate "I've arrived, hold here." A real incident:
 * stick paused near 1190us for ~270ms while climbing steadily from a
 * 988us-latched center (commanding a strong, correct climb), which relatched
 * center to 1190us - the SAME stick position an instant later now read as a
 * small NEGATIVE offset from the new center instead of a large positive one,
 * and commanded throttle fell off a cliff (1253us actual -> 1140us) with no
 * change in what the pilot was actually doing - the concrete cause of a
 * reported "aircraft only bounces off the ground." A spring-centered yaw
 * stick doesn't have this ambiguity (release = center, unambiguous), which
 * is why 300ms was fine there but not here. 1200ms requires genuinely
 * holding still for over a second - well past a natural mid-push hesitation,
 * still fast enough to feel responsive once actually parked. */
#define APP_ALTHOLD_THROTTLE_SETTLE_MS     1200U
/* Second condition (2026-08-23, alongside the settle-duration fix above) for
 * committing a relatch: the aircraft must also actually BE near-level, not
 * just the stick being still. The settle-duration fix alone still had a gap -
 * a genuine ~0.8s "let's see how this feels" pause DURING an ongoing climb
 * (stick still, but the climb-rate estimate nowhere near zero) could still
 * relatch center right there, silently locking a hold at whatever (possibly
 * low) altitude that pause happened to occur at - reported as the aircraft
 * "just gives up." Requiring near-zero climb rate too means a relatch can
 * only commit once the aircraft has actually stopped moving, not just the
 * stick. */
#define APP_ALTHOLD_RELATCH_MAX_CLIMB_MPS  0.4f
/* Third condition for committing a relatch (2026-08-23): the settle
 * reference must not be within this many us of either physical stick
 * extreme. A real incident: holding full-down stick to land is, by
 * definition, a perfectly still stick once pinned at the mechanical limit -
 * it satisfied both conditions above and relatched center to the stick's own
 * position, instantly turning a commanded max-rate descent into an altitude
 * LOCK (the same now-"centered" stick read as offset=0). The aircraft never
 * landed - see kh7-althold-full-authority-redesign memory. Excluding both
 * extremes means holding full-down (or full-up) always keeps commanding
 * max-rate descent (or climb), never silently converts to a hold. */
#define APP_ALTHOLD_RELATCH_EXCLUDE_MARGIN_US 100U
#define APP_ALTHOLD_MAX_CLIMB_MPS           2.0f
/* REVERTED back to 0.8 (2026-08-25, same day) - raising this to 1.5 caused a real
 * in-flight uncontrolled climb that could not be arrested with the stick and
 * required an emergency in-flight disarm to stop. Root cause NOT yet understood -
 * the SD card ejected during/after the emergency disarm, so the log of this exact
 * flight may be incomplete or unrecoverable. Do not raise this gain again without
 * first understanding why a theoretically-bounded change (climb_rate_setpoint was
 * still clamped to +-APP_ALTHOLD_MAX_CLIMB_MPS regardless of this value, by
 * design) produced an unarrestable real climb - that reasoning was evidently
 * missing something. Treat this exactly like the 2026-08-21/2026-08-22 incidents:
 * re-attempt only in much smaller steps with a dedicated low-altitude bench/hover
 * test, and only after the root cause of THIS incident is understood, not just
 * because the number is reverted. */
#define APP_ALTHOLD_ALT_HOLD_KP_MPS_PER_M   0.8f
/* Live-tunable override for APP_ALTHOLD_ALT_HOLD_KP_MPS_PER_M (2026-08-29) - added
 * so this specific gain can be nudged from the GUI during a cautious low-hover test
 * instead of needing a reflash per attempt, given a real flight confirmed the
 * position-hold loop wanders ~0.5m with the throttle stick held perfectly flat
 * (not sensor noise - the fused height tracked lidar accurately the whole time).
 * RAM-only, resets to the safe compiled-in default on every boot - deliberately NOT
 * persisted to flash, so a value left over from a tuning session can never silently
 * carry into a later one. Bounded well below 1.5 - see APP_ALTHOLD_ALT_HOLD_KP_MPS_PER_M's
 * comment above for the exact real incident (an unarrestable in-flight climb,
 * emergency disarm, root cause never understood) that value caused; this cap keeps
 * any GUI-driven increase from ever reaching it. */
static float g_althold_alt_hold_kp_mps_per_m = APP_ALTHOLD_ALT_HOLD_KP_MPS_PER_M;
/* App_GetAltholdAltHoldKp()/App_SetAltholdAltHoldKp() are defined further below,
 * next to App_IsFiniteInRange() which the setter's validity check needs. */

/* Live-tunable override for APP_ALTHOLD_MAX_CLIMB_MPS (2026-08-30) - the ceiling on
 * both manually-commanded climb/descend rate (off-center stick) AND the position-hold
 * loop's own correction rate. Exposed to the GUI on request, same RAM-only/reset-on-
 * boot/firmware-bounded pattern as the hold-gain live-tunable above. Unlike that gain,
 * this specific constant has no direct incident history of its own (it was
 * deliberately left untouched while OTHER gains were tuned/reverted around it - see
 * kh7-althold-oscillation-yawhold-todo memory), but it still directly caps how fast
 * this aircraft can move vertically, so the bound below (APP_ALTHOLD_MAX_CLIMB_MIN/MAX,
 * app.h) is a cautious ~2x headroom above the long-standing default, not an unbounded
 * slider. */
static float g_althold_max_climb_mps = APP_ALTHOLD_MAX_CLIMB_MPS;

/* Integral on the OUTER position-hold loop (2026-08-30) - the hold P-term
 * (App_GetAltholdAltHoldKp() * position_error) is pure proportional, which real
 * flight data showed settles with a persistent steady-state droop: fused height sat
 * 8.7-15.1cm BELOW the held target across three separate hold episodes in one flight,
 * consistently in the same direction (not symmetric noise). Same class of bug as
 * kh7-pitch-steady-state-droop earlier this session (a P-only chain settling short of
 * its target), fixed the same way there by adding an integral term. Starting value
 * (0.05) chosen by simulating candidate gains against the REAL captured target/fused
 * trajectory from that flight before ever writing this code: 0.03-0.12 all stayed
 * well inside the integral limit with no sign of runaway, 0.05 sits in the
 * conservative middle of that range. Live-tunable (App_GetAltholdPosKi()/
 * App_SetAltholdPosKi()) so it can be dialed to 0 (fully off) or adjusted without a
 * reflash if real flight shows it's too weak or too aggressive - same RAM-only/
 * reset-on-boot pattern as every other live-tunable ALTHOLD gain tonight. Separate
 * accumulator (althold_pos_integral_mps) and separate clamp
 * (APP_ALTHOLD_POS_INTEGRAL_LIMIT_MPS) from the INNER climb-rate loop's own integral
 * (althold_integral_us) - these are two different loops, not the same term reused.
 *
 * 2026-08-30: the open-loop-replay validation above was NOT sufficient - real flight
 * at this exact value (0.05) caused a severe, reproducible closed-loop instability
 * (fused height cycling ~0.1-1.7m even with the target held flat), confirmed via
 * GUI-verified GET/Read on 4 of 5 flights, one ending in a flip; nowhere near
 * trim/damp saturation, so it's not a clamp/authority issue. Default lowered to 0.0f
 * here so a power cycle doesn't silently re-arm the known-bad value (this RAM-only
 * value had already reverted and confused one earlier "confirmation" test before a
 * GET/Read caught it - verify live value before flying, don't assume). A genuinely
 * verified Ki=0 flight is still pending as of this note - do not raise this off 0
 * without that, PLUS closed-loop (not just open-loop trajectory replay) validation
 * against the halved inner climb-rate-loop gains. See
 * kh7_althold_oscillation_yawhold_todo memory for full incident details. */
#define APP_ALTHOLD_POS_KI_PER_S2            0.0f
#define APP_ALTHOLD_POS_INTEGRAL_LIMIT_MPS   0.5f
static float g_althold_pos_ki_per_s2 = APP_ALTHOLD_POS_KI_PER_S2;

/* Gentle fine-adjustment rate for stick offset WITHIN the hold deadband
 * (2026-08-23) - see the "Centered" branch's comment for why this exists.
 * Deliberately much slower than APP_ALTHOLD_MAX_CLIMB_MPS (the full
 * off-center climb/descend rate) so nudging while holding still feels
 * clearly different from actively commanding a climb. */
#define APP_ALTHOLD_HOLD_NUDGE_MAX_MPS      0.3f
/* Deadzone on the nudge above (2026-08-29) - real ALTHOLD-on-EKF flight telemetry
 * (the new Telemetry_PrintAltholdState() target_cm field) showed the held target
 * continuously ramping - never flat - throughout every hold, root-caused to this
 * exact nudge: it had no threshold of its own, so ANY nonzero throttle_offset_us
 * walked the target, without bound, for the whole hold duration. Back-calculated
 * from the observed drift rate (~0.02 m/s over a 10s window): an average stick
 * offset of just 6.7us from the latched center fully explains it - smaller than
 * any human can hold a stick to, meaning this drift was guaranteed on every
 * single hold, for any pilot, regardless of skill. Not related to gains, baro
 * noise, or vert_ekf.c at all - this bug predates all of that (added 2026-08-23).
 * 20us rejects that level of stick noise/tremor while still leaving most of the
 * +-100us deadband available for a genuinely intentional nudge. */
#define APP_ALTHOLD_HOLD_NUDGE_DEADZONE_US  20U
/* Safe starting point for althold_hover_throttle_us on every arm, and what it
 * stays pinned at for the whole below-gate liftoff-assist phase (see
 * APP_ALTHOLD_LIFTOFF_ASSIST_MAX_US below - there's no baro feedback to learn
 * from pre-gate by design, so this never moves until the gate opens and the
 * real closed loop takes over). Not meant to be a good guess by itself - see
 * that constant for how liftoff actually reaches real hover/climb throttle
 * from here - just a known, predictable, comfortably-clear-of-idle value. */
#define APP_ALTHOLD_HOVER_EST_SEED_US    1150.0f
#define APP_ALTHOLD_HOVER_EST_MIN_US     1100U
/* LOWERED from 1550 to 1330 (2026-08-25) after a real incident: the aircraft climbed
 * hard, pinned this estimate at its old 1550 ceiling, then got stuck "arrested
 * against the ceiling" for 7+ seconds with FULL-DOWN STICK HELD THE WHOLE TIME
 * having no effect - required an emergency in-flight disarm. Root cause: this
 * ceiling combined with the fast trim's bounded authority
 * (APP_ALTHOLD_TRIM_LIMIT_US=250, the most it can ever subtract) sets a WORST-CASE
 * FLOOR of ceiling-250, regardless of stick position - at the old 1550 that floor
 * was 1300us, which was apparently enough thrust to sustain this airframe near
 * level flight, so even a maxed-out descend command (correctly computed - the
 * stick-command math itself was working) could never bring real output low enough
 * to actually descend. This wasn't unique to the gain change that triggered it
 * this time (APP_ALTHOLD_ALT_HOLD_KP_MPS_PER_M, since reverted) - it's a latent
 * structural gap that any sufficiently sustained climb could have wound the
 * estimate into. Fixed at the source: 1330 guarantees ceiling-250 <=
 * APP_MOTOR_IDLE_US(1080), so the worst possible case - this estimate pinned at
 * its ceiling AND trim pinned at its floor, simultaneously - still reaches idle,
 * restoring "full-down stick always actually reaches idle" as a real, provable
 * guarantee instead of one that happened to hold only when this estimate stayed
 * away from its ceiling. */
#define APP_ALTHOLD_HOVER_EST_MAX_US     1330U
/* Deliberately much smaller than APP_ALTHOLD_VZ_KI_US_PER_MPS_S below - this
 * drives the persistent base estimate, not the fast reactive trim, so it
 * should only fully correct a sustained error over several seconds, never in
 * one or two iterations. Raised 3.0->9.0 (2026-09-04) after real flight data
 * showed this Ki, combined with the resets-every-arm safety discipline above
 * (see APP_ALTHOLD_HOVER_EST_SEED_US's reset comment - deliberate, from a real
 * 2026-08-21 incident, NOT touched here), meant every single flight needed a
 * genuinely painful ~30-40s of weak, barely-airborne thrust before the
 * estimate reached this airframe's real ~1250-1290us hover point - two
 * consecutive flights that night never got there at all. 3x faster still
 * takes ~10-15s to fully correct a sustained error (nowhere near "one or two
 * iterations"), it's still a bounded, gradual, per-arm-reset estimate, not a
 * carried-over or assumed value - just less punishing to actually fly. */
#define APP_ALTHOLD_HOVER_EST_KI_US_PER_MPS_S 9.0f
/* Below-gate liftoff assist (2026-08-23): full stick deflection from
 * althold_settled_center_us adds/subtracts this many us around
 * althold_hover_throttle_us, OPEN LOOP - no baro feedback at all, since baro
 * is known unreliable during exactly this window (see
 * kh7-baro-liftoff-transient memory). An earlier version of this centered on
 * a fixed APP_PWM_MID_US=1500 instead of the settled center - reverted the
 * same day: raw PWM 1500 is nowhere near this airframe's real ~1200-1270us
 * hover throttle, making it impossible to comfortably approach the post-gate
 * hold center without climbing hard the whole way there (same class of
 * stick-semantics discontinuity at the gate as the 2026-08-21 incident, just
 * manifesting as "can't get there" instead of "runaway"). Centering on the
 * settled stick position instead keeps stick meaning consistent across the
 * gate with NO dependence on this airframe's specific hover throttle at all -
 * see althold_settled_center_us's declaration comment. Sized so idle seed
 * (1150) plus full-stick assist (1150+350=1500) alone would already reach
 * the real observed liftoff throttle (~1210-1215us mean) well within half
 * stick travel - conservative, not twitchy. */
#define APP_ALTHOLD_LIFTOFF_ASSIST_MAX_US   350U
/* ALTHOLD's ABSOLUTE ALTITUDE reference is vert_ekf.c's VertEkf_GetHeightM() as of
 * 2026-08-29 - replaces the hand-rolled baro+rangefinder complementary fusion
 * (App_GetFusedAltitudeM(), removed) that lived here before. See vert_ekf.c's
 * top-of-file design comment for why a real Kalman filter with per-measurement R
 * replaces that fusion's fixed-rate correction/outlier-reject/resync-watchdog logic
 * outright rather than extending it further, and kh7-baro-liftoff-transient /
 * kh7-althold-full-authority-redesign / kh7-althold-hover-est-ceiling-incident
 * memory for exactly how much real-flight iteration went into the version this
 * replaces.
 *
 * CLIMB RATE history, both 2026-08-29: FIRST attempt used VertEkf_GetClimbRateMps()
 * directly as a drop-in swap for Baro_GetClimbRateMps() - reverted the same day after a
 * live ALTHOLD-on-EKF test flight showed a real ~20-50cm altitude oscillation that
 * wasn't there before. Root cause: the fused HEIGHT was accurate throughout (tracked
 * lidar_h_cm closely the whole time), so this wasn't a sensor/estimator error - it was
 * vert_ekf.c's climb-rate being driven by real-time accel integration, snappier/less
 * lagged than baro's own complementary filter, changing the PID loop's effective
 * gain/phase enough to oscillate even though the gains themselves didn't change. Reverted
 * to baro's climb-rate (with a dedicated input-side filter added, then later removed -
 * see APP_ALTHOLD_VZ_KP_US_PER_MPS's comment below), and real indoor+outdoor flight data
 * confirmed baro's own noise floor (not gain, not damping, not ground-effect/
 * recirculation - see kh7-althold-oscillation-yawhold-todo memory) as the dominant
 * remaining driver of ALTHOLD altitude variance. SECOND attempt (same day): switched to
 * VertEkf_GetClimbRateMps() again, this time PAIRED with a real gain reduction
 * (APP_ALTHOLD_VZ_KP_US_PER_MPS/_KI_US_PER_MPS_S roughly halved) instead of a drop-in
 * swap, since a less-lagged signal needs less raw gain for the same effective loop
 * authority - see that constant's comment for the full reasoning. NOT YET
 * FLIGHT-VALIDATED as of this comment - treat exactly like every other change to this
 * loop, incremental stick-held-steady testing required before trusting it. */
/* REVERTED 2026-08-22 (same day as the increase below): raising these caused a
 * real, clean limit-cycle oscillation - confirmed directly in a flight log with
 * the throttle stick held dead flat for 18+ seconds: altitude cycled between
 * 0.35m and 1.42m (~1m/3.5ft swing) with a ~4-6s period, and trim oscillated
 * right along with it (ranging -11 to -77us, never crossing positive) - the
 * classic signature of integral gain too aggressive relative to the aircraft's
 * real thrust-response lag (overcorrect, forced to reverse, overcorrect the
 * other way). The earlier persistent-hover-reference fix and shortened
 * complementary-filter tau (see kh7-althold-oscillation-yawhold-todo memory)
 * were confirmed still working correctly in this same flight (stick-steady
 * behavior, not a reference/sign bug) - this was purely the gain increase
 * overshooting into instability. Back to the original, previously-stable
 * values. If more authority is wanted again, re-attempt in much smaller steps
 * with a dedicated stick-held-steady bench/hover test to check for oscillation
 * BEFORE calling it validated - a "does it saturate" check alone (which is what
 * motivated the increase last time) does not rule out this failure mode. */
/* LOWERED 2026-08-29 (25->12, 8->4) alongside switching this loop's climb-rate SOURCE
 * from Baro_GetClimbRateMps() to VertEkf_GetClimbRateMps() (see the big comment above
 * APP_ALTHOLD_LIFTOFF_ASSIST_MAX_US for the full history of that switch, including the
 * FIRST attempt the same day that caused a real ~20-50cm oscillation and was reverted).
 * This second attempt pairs the source swap with a real gain reduction instead of a
 * drop-in swap: vert_ekf's climb-rate is driven by real-time accel integration and
 * reacts with less lag than baro's own complementary filter, which effectively raises
 * this loop's gain/phase margin impact even at the SAME numeric Kp/Ki - roughly halving
 * both is a conservative first step, not a precisely derived value (no closed-loop
 * system ID was done, just the general principle that a less-lagged input needs less
 * raw gain for the same effective authority). A dedicated input-side filter on baro's
 * climb-rate (APP_ALTHOLD_VZ_FILTER_HZ, added earlier the same day) is REMOVED here,
 * superseded by moving off baro's climb-rate for this loop entirely - see
 * kh7-althold-oscillation-yawhold-todo memory for why baro's own noise floor was
 * identified as the dominant remaining driver of ALTHOLD altitude variance (confirmed
 * environment-independent via matched indoor/outdoor data), and kh7-vert-ekf-design for
 * why increasing damping instead (tried right before this) had no measurable effect -
 * ruling out authority/damping as the limiter and motivating this SOURCE change instead
 * of another gain-only attempt. Treat this exactly like every other change to this
 * exact loop: NOT YET FLIGHT-VALIDATED as of this comment, requires the same
 * incremental, stick-held-steady, hand-ready-to-disarm testing discipline as the
 * 2026-08-22 and 2026-08-29 gain incidents on this file. */
#define APP_ALTHOLD_VZ_KP_US_PER_MPS        12.0f
#define APP_ALTHOLD_VZ_KI_US_PER_MPS_S       4.0f
/* Live-tunable overrides for the two constants above (2026-08-30) - added after a
 * flight-confirmed-clean 70s hold (once the night's other real bugs - Ki windup,
 * momentum-handoff capture, EKF accel/lidar glitches, ground-effect-gate cycling -
 * were fixed separately) showed `trim_us` never exceeding 16 of its 250us ceiling
 * the ENTIRE flight, alongside a real, slower (~8-18s) residual height wave that
 * trim barely correlated with (r=-0.38) - strong evidence this inner loop has
 * substantial unused corrective authority now that the ACTUAL causes of the
 * earlier oscillation are gone, not that it's already working as hard as it safely
 * can. Exposed live-tunable specifically so this can be tested incrementally,
 * in small steps, with a GET/Read verification before each attempt (per the
 * lesson learned earlier this exact session about a stale RAM-only value), rather
 * than picking one new hardcoded number and reflashing blind. Do NOT trust an
 * open-loop replay of old error data to validate a higher value here - that
 * exact method already failed once tonight (see APP_ALTHOLD_POS_KI_PER_S2's
 * comment) precisely because it cannot reveal a closed-loop instability. Bounds
 * deliberately capped at the ORIGINAL pre-2026-08-29 values (25/8) - this is
 * restoring toward a value this loop has run at before (with baro's climb-rate,
 * not vert_ekf's - so even the max of this range is not proven safe with the
 * current source, just not a step into genuinely unprecedented territory).
 * Bounds (APP_ALTHOLD_VZ_KP_MIN/MAX, APP_ALTHOLD_VZ_KI_MIN/MAX) live in app.h,
 * next to the getter/setter declarations - same pattern as every other
 * live-tunable ALTHOLD gain this session. */
static float g_althold_vz_kp_us_per_mps = APP_ALTHOLD_VZ_KP_US_PER_MPS;
static float g_althold_vz_ki_us_per_mps_s = APP_ALTHOLD_VZ_KI_US_PER_MPS_S;
#define APP_ALTHOLD_INTEGRAL_LIMIT_US      200U
/* Smooths the final trim output itself (not the P/I gains) - climb-rate error is
 * fed straight from the baro, and even with the baro's own onboard LPF, real
 * sensor/vibration noise translated directly into trim was making the motors
 * respond jerkily on the Z axis. A slow-ish cutoff is fine since altitude
 * corrections are inherently a slow process. */
/* REVERTED to 250 alongside the VZ_KP/KI/INTEGRAL_LIMIT revert above - see that
 * comment. Raising this to 350 alongside the gain increase was part of the
 * same overshoot; back to the original, previously-stable value. */
#define APP_ALTHOLD_TRIM_LPF_HZ             1.5f
#define APP_ALTHOLD_TRIM_LIMIT_US          250U
/* --- NAV_POSHOLD (2026-09-04 rewrite) ---
 * Replaces the old NAV_VELOCITY_BRAKE mode + separate independent poshold aux-
 * switch overlay (both deleted) with ONE unified GPS mode, per explicit user
 * request after two rounds of real-flight failures in the old split design
 * (a growing velocity-error oscillation, then an actual attitude-tracking
 * divergence - see git history / kh7_gps_data_and_poshold_design memory for the
 * full incident record). Engaged when ch6 exceeds this threshold, same physical
 * slot the old top band used - reuses the ATTITUDE-mode angle controller
 * (roll/pitch) and the ALTHOLD throttle controller (vertical), this section only
 * adds the outer position/velocity->angle loop. Behavior (the whole mode in one
 * paragraph): roll/pitch sticks centered -> command zero drift (hold the last
 * settled position once GPS-measured speed confirms it's actually stopped, or
 * brake to a stop first if it wasn't); sticks off-center -> stick maps directly
 * to a commanded NE velocity, same as manual flying; sticks return to center ->
 * always recapture the CURRENT position as a fresh hold target (per the pilot
 * spec: "when you release the sticks, recapture position" - deliberately no
 * memory of any earlier target, see the removed APP_NAVPOS_TARGET_MEMORY_MS
 * for why that was wrong). Yaw is NOT re-litigated here - consumes the same
 * yaw_deg/mag-nudge pipeline every other mode already shares. If the GPS engage
 * gate ever fails while this mode is selected, falls straight through to plain
 * stick-to-angle control (identical to ATTITUDE mode) rather than doing nothing -
 * the pilot always has manual attitude control as a fallback. */
#define APP_NAV_BRAKE_SWITCH_THRESHOLD_US   2000U
/* Raised again 2026-09-04 (10.0->15.0, 1.6->2.5): confirmed calm-air (no wind)
 * flight still showed correction ramping too slowly to arrest drift within a
 * couple seconds even after the first round of increases - with no external
 * disturbance to explain the shortfall, the ceilings themselves were still
 * the limit, not a real-world force this loop can never fully cancel. */
/* APP_NAVPOS_MAX_TILT_DEG is no longer the hold's own tilt ceiling (see
 * 2026-09-04 below) - kept only for the disarmed bench-diagnostic block's
 * "would-be" print, which never reaches the motors. */
#define APP_NAVPOS_MAX_TILT_DEG              15.0f
/* Raised again 2026-09-04 (2.5->8.0): the hold's angle output is now capped by
 * active_attitude_gains.max_angle_deg (same authority as manual flying, per
 * the pilot's explicit request after the gentler ceilings above still felt
 * "no different" in real flight) rather than a separate, lower tilt ceiling -
 * this accel cap is raised well past what's needed to reach a ~35deg angle
 * (g*tan(35)=6.9 m/s^2) so it no longer silently re-introduces a lower
 * effective ceiling of its own. */
#define APP_NAVPOS_MAX_ACCEL_MPS2            8.0f
/* Raised 2026-09-04 (20->80, matching APP_YAWHOLD_DEADBAND_US's already-proven
 * value): real flight data showed a real pilot's stick never sits inside a
 * +-20us window for more than a fraction of a second - ordinary hand tremor
 * repeatedly popped it outside the deadband, so navpos_active kept dropping
 * and re-latching a BRAND NEW current-position target every time, chasing
 * wherever the aircraft happened to be at each restart instead of ever
 * holding one point - reported as "still drifts" even with the hold engaged
 * most of the time. */
#define APP_NAVPOS_STICK_DEADBAND_US        80U
/* Raised 2026-09-04 (0.6->1.2 Kp, and MAX_ACCEL/MAX_TILT below 1.0->1.6 m/s^2,
 * 8.0->10.0 deg): real flight data showed the hold converging in the right
 * direction but too slowly against real outdoor disturbance - a sustained
 * ~0.8 m/s velocity error only produced ~2-2.5deg of tilt (nowhere near
 * either the old 1.0 m/s^2 accel cap or the 8deg tilt cap), so the low Kp
 * itself was the bottleneck, not either ceiling. Safe to push harder here for
 * the same reason the Kp/Ki raise above was: this chain is now used ONLY for
 * the centered-stick hold, structurally isolated from manual flying (which
 * uses the separate full-authority ATTITUDE-mode mapping), so it can't
 * destabilize active piloting the way the two historical incidents did. */
#define APP_NAVPOS_VELOCITY_KP               2.0f
/* Off-center stick -> commanded velocity ceiling (manual flying under this mode). */
#define APP_NAVPOS_MAX_STICK_VEL_MPS         1.5f
/* Centered-stick position-hold outer loop: position error (Kp+Ki) -> desired NE
 * velocity (clamped to this, separate from the manual stick ceiling above since
 * holding against real disturbance may need more authority than a pilot's own
 * gentle stick input ever asks for) -> feeds the SAME velocity-error->accel->angle
 * chain above - no second, independently-tuned physics path. */
/* Raised 2026-09-04 (0.30->0.6, 0.03->0.08): real flight data showed the hold
 * settling at a persistent 0.2-0.6m standoff from its target rather than
 * converging - reported as "drifts, should hold the GPS position". At the old
 * Kp, a 0.5m error only requested a 0.15 m/s correction, well under this
 * loop's own 1.0 m/s/1.0 m/s^2 ceilings (neither was saturating), so the
 * bottleneck was gain, not authority. Safe to raise now in a way it wasn't
 * before: manual flying (off-center stick) no longer shares this chain at all
 * - it uses the separate, full-authority ATTITUDE-mode stick mapping - so this
 * gain only affects the gentle station-keeping hold, not piloted maneuvering,
 * unlike the two past incidents where raising a SHARED gain destabilized
 * active flying. Still well under APP_NAVPOS_KP_MAX/KI_MAX (0.8/0.15). */
#define APP_NAVPOS_KP_DEFAULT                0.60f  /* m/s commanded per meter of position error */
#define APP_NAVPOS_KI_DEFAULT                0.08f  /* m/s per meter-second of accumulated error */
#define APP_NAVPOS_MAX_INTEGRAL_MPS          0.6f   /* hard clamp, independent of ki (isfinite-guarded below) */
/* Raised 2026-09-04 (1.0->1.5) alongside the accel/tilt ceiling raise above,
 * so a larger position error can request a correspondingly faster correction
 * instead of hitting this velocity ceiling before the stronger accel/tilt
 * authority ever gets used. */
#define APP_NAVPOS_MAX_HOLD_VEL_MPS          1.5f
/* Tighter than NAV_MAX_HORIZONTAL_ACC_M's 5.0m general engage gate - a "hold" at
 * 5m accuracy would visibly wander, not matching pilot expectation. Live-checked
 * hAcc this session read 0.70m on an 18-satellite 3D fix, so ~2-3m is achievable
 * in practice, not just in theory. */
#define APP_NAVPOS_MAX_HORIZONTAL_ACC_M       2.5f
/* Velocity-gated relatch (mirrors APP_ALTHOLD_RELATCH_MAX_CLIMB_MPS's incident-
 * driven lesson): "stick centered" alone is not sufficient evidence the aircraft
 * has actually stopped moving - require GPS-measured horizontal speed to already
 * be near zero before latching a target, or a hold engaged mid-drift would just
 * freeze the drift in place. */
#define APP_NAVPOS_RELATCH_MAX_VEL_MPS        0.4f
static float g_navpos_kp_per_s = APP_NAVPOS_KP_DEFAULT;
static float g_navpos_ki_per_s2 = APP_NAVPOS_KI_DEFAULT;
/* App_GetNavPosKp()/App_SetNavPosKp()/App_GetNavPosKi()/App_SetNavPosKi() are
 * defined further below, next to App_IsFiniteInRange() which the setters need. */

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
 * further blind.
 *
 * GAIN RAISED (2026-08-16) after a disarmed, motionless-on-a-table bench test
 * showed the old 0.05/2.0 pair was nowhere near enough authority: roll/pitch
 * stayed flat to 0.1deg for 90s (strong two_kp=3.0 accel correction holding
 * them), while yaw free-drifted 0.5->40.7deg with an ACCELERATING rate
 * (~0.02deg/s early, ~1.9deg/s by the end) even though the raw tilt-compensated
 * compass heading sat rock-stable at 352.5-354.2deg the whole time - i.e. the
 * reference was trustworthy and available, the correction was just too weak to
 * use it (already saturated at -2.0dps while yaw error was -40deg). No motor
 * current is flowing during this bench test, so the field-deviation
 * interference gate above is the thing standing between this larger gain and a
 * repeat of the pre-relocation runaway once armed - re-validate with the same
 * bench test after any further change, and re-check real-flight tracking error
 * (see the FOLLOW-UP comment near APP_NAV_BRAKE_SWITCH_THRESHOLD_US above)
 * before trusting NAVBRAKE with motors spinning. */
/* REDESIGNED (2026-08-17), vector-based instead of angle-subtraction: every
 * earlier version of this correction computed error as a raw degree
 * difference (mag_delta_deg - yaw_deg), which has a hard discontinuity at
 * +-180deg - right at that boundary "which way is shortest" flips on noise
 * (confirmed live: nudge chattering exactly between +max/-max every sample),
 * and even a slew limiter plus a special-cased forced-escape-direction hack
 * only left yaw parked in a bounded oscillation near the boundary instead of
 * converging. Real Mahony/Madgwick AHRS filters avoid this class of bug
 * entirely by correcting from a CROSS PRODUCT of measured vs. reference
 * vectors, not a subtracted angle - see the sinf() at the injection site
 * below. sin(current - reference) matches a plain angle error for small
 * errors (sin(x) ~= x near 0) but SMOOTHLY tapers to exactly zero at the
 * antipodal point instead of staying pinned at max magnitude right up to a
 * discontinuous jump - there is no wrap boundary left to chatter at, so the
 * escape-zone hack this comment used to describe is gone, not just tuned. */
/* DISABLED AGAIN (2026-08-17): a fast-lock gain (Kp 2.5, cap 90dps) tested
 * clean on the bench (stationary hold, and a ~150deg hand rotation converging
 * in ~1.8-2.2s with no overshoot) but made in-flight yaw slewing MUCH worse
 * once actually flown - real flight vibration regularly pushes accelerometer
 * trust-gating past its limits (see the vibration findings elsewhere this
 * session), and a much stronger correction reacting to that noisier/
 * vibration-corrupted heading amplifies bad signal far more violently than
 * the original conservative gain did. The field-deviation interference gate
 * below only catches gross field-strength shifts, not general vibration
 * noise on the heading itself. No gain has been flight-validated as safe yet
 * - zeroing both constants disables the nudge entirely (vector-based math and
 * sign fixes stay in place) until this gets properly tuned against real
 * flight data, not more bench guesses.
 *
 * RE-ENABLED (2026-08-19), conservative gain: the 2026-08-18 real-flight
 * finding that motivated urgency here (a confirmed +15.3dps sustained
 * rightward yaw-rate bias with genuinely centered stick) did NOT reproduce in
 * a clean 100%-centered-stick 55.7s retest the same week (full-flight mean
 * +1.29dps, wandering -3.35..+4.80dps across 5s windows - ordinary noise, not
 * a fixed torque imbalance) - see kh7-yaw-system-state memory. That removes
 * the "fighting a large, real, sustained disturbance" scenario the 2.5/90
 * gain got amplified by. Picked well below that failed gain rather than
 * re-attempting anything close to it: ~10x the original too-weak 0.05/2.0
 * pair (which under-corrected because it saturated at just 2.0dps against
 * real drift, not because 0.05 itself was necessarily wrong), but only ~1/5
 * the Kp and ~1/6 the cap of the gain that amplified vibration noise. Treat
 * this as a first real-flight data point, not a final value - re-check
 * compass-vs-yaw tracking error (sdlog_analyze.py's mag_delta/tracking-err
 * stats) on the next flight before trusting it further, same as every
 * previous change to this correction. */
#define APP_MAG_YAW_NUDGE_KP_DPS_PER_DEG 0.5f
#define APP_MAG_YAW_NUDGE_MAX_DPS        15.0f
/* Slew-limits the APPLIED nudge (not the target) so it ramps smoothly even
 * across a fast target change, rather than stepping instantly - a general
 * control-loop courtesy, not a workaround (the vector-based error above has
 * no discontinuity left to need rate-limiting away). Sized so a full
 * -max..+max reversal still takes about 1s, matching the fast-lock intent above. */
#define APP_MAG_YAW_NUDGE_SLEW_DPS_PER_S 180.0f
/* BUG FOUND (2026-08-17) during flight-test prep: the reference heading used to
 * anchor the nudge above was a single raw compass sample taken the instant
 * attitude-zero settled - if that one sample was a transient/bad reading (e.g.
 * right after the QMC5883L's own power-on settling), every later heading gets
 * compared against a wrong reference. A ~180deg reference error is the worst
 * case for any angle-based P-controller: "which way is shortest" is undefined
 * right at 180deg and flips on noise, so the nudge chatters at its +/-max cap
 * every iteration instead of converging - confirmed live (gz alternating
 * exactly +8.0/-8.0 dps every sample, yaw hopping across the +-180 wrap
 * boundary, reproducing at every power-on). Fixed by averaging the reference
 * over APP_MAG_REF_AVG_MS instead of trusting one sample - same pattern already
 * used for the attitude-zero capture above (APP_ATTITUDE_ZERO_AVG_MS), and
 * circularly (unit-vector sin/cos averaging) since heading wraps at 360. */
#define APP_MAG_REF_AVG_MS 300U
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
/* Yaw angle-hold (2026-08-22) - a small, bounded assist that pulls back toward
 * a captured target heading whenever the yaw stick is centered, in
 * ATTITUDE/ALTHOLD/NAV_VELOCITY_BRAKE (matching where roll/pitch already get
 * angle-holding); RATE mode is untouched, yaw stays pure rate passthrough
 * there by design (acro-style). This is the first cut of a previously-missing
 * feature (yaw was rate-control-only everywhere - see kh7-althold-oscillation-
 * yawhold-todo memory), kept deliberately conservative given everything
 * learned tuning ALTHOLD's gains this same session: max assist rate is a small
 * fraction (~14%) of full manual yaw authority (APP_RATE_CMD_MAX_YAW_DPS), and
 * the gain saturates at only a 10deg error, so it can't apply a large,
 * sudden correction. Bench-test for smooth, non-oscillatory tracking (yaw
 * setpoint vs gyro correlation, same method used to tune the rate PIDs) before
 * trusting it in flight - do not assume gentle bench behavior generalizes to
 * flight without checking, per kh7-flight-test-workflow memory. */
/* Briefly halved 2026-08-22 (Kp 3.0->1.5, ceiling 30->15) after a re-test flight showed
 * setpoint_yaw_dps pegged at the ceiling for 45% of the flight with what looked like a
 * limit-cycle oscillation in yaw_deg - assumed at the time to be a P-gain-too-aggressive
 * problem (same failure class as the ALTHOLD Ki oscillation earlier this session). A
 * SECOND re-test at the halved gains still showed a monotonic yaw_deg runaway (not an
 * oscillation this time, which is what exposed the real bug - see the sign-flip comment
 * at the cmd_yaw_rate_dps assignment near the capture site below), proving gain magnitude
 * was never the actual problem: the correction term had the wrong sign and was reliably
 * pushing yaw_deg further from target, at whatever rate the gain/ceiling allowed. With
 * the sign now fixed, restored to the original values - re-bench-test for correct-
 * direction, settling (not diverging) tracking before the next real flight; do not
 * reflexively assume oscillation-shaped symptoms mean "lower the gain" without first
 * checking the correction is even pointed the right way. */
#define APP_YAWHOLD_KP_DPS_PER_DEG    3.0f
#define APP_YAWHOLD_MAX_RATE_DPS     30.0f
/* Dedicated engagement deadband, NOT the tiny APP_CONTROL_DEADBAND_US (20us) -
 * that constant exists to reject stick jitter inside the direct stick->rate
 * mapping, where being off by a few dps doesn't matter. Bench-tested
 * 2026-08-22: with APP_CONTROL_DEADBAND_US, a single clean stick-centered
 * twist-and-release left yaw sitting 20+ degrees off target for 7+ seconds
 * before yaw-hold's own engagement condition happened to be satisfied - too
 * tight for normal stick/receiver jitter and human precision holding a stick
 * at center. ALTHOLD's analogous "is the stick centered enough to engage
 * assist" gate uses 100us (APP_ALTHOLD_THROTTLE_DEADBAND_US) - matching that
 * scale here instead. */
#define APP_YAWHOLD_DEADBAND_US      80U
/* Live-tuned 2026-08-18 via SD-log correlation testing (setpoint_*_dps vs gyro_*_dps
 * per axis across multiple real flights) - see the "KH7 yaw system state" memory note
 * for the full before/after numbers. Kp raised from 0.90/0.90/0.80 after yaw tracking
 * correlation was found much weaker than roll/pitch (0.046-0.29 vs 0.8-0.94); yaw Kp was
 * walked up through 0.55/0.6/0.66 before settling at parity with roll/pitch. Kd added
 * from 0 after that - roll/yaw showed a clear, repeatable correlation improvement; pitch's
 * effect was inconclusive (flight-to-flight noise swamped the signal) but kept at 0.03
 * with no evidence of a regression. */
#define APP_RATE_KP_ROLL_DEFAULT_US_PER_DPS 0.70f
#define APP_RATE_KP_PITCH_DEFAULT_US_PER_DPS 0.70f
#define APP_RATE_KP_YAW_DEFAULT_US_PER_DPS  0.70f
#define APP_RATE_KI_ROLL_DEFAULT_US_PER_DPS_S 0.00f
#define APP_RATE_KI_PITCH_DEFAULT_US_PER_DPS_S 0.00f
#define APP_RATE_KI_YAW_DEFAULT_US_PER_DPS_S  0.00f
#define APP_RATE_KD_ROLL_DEFAULT_US_PER_DPS_PER_S 0.025f
#define APP_RATE_KD_PITCH_DEFAULT_US_PER_DPS_PER_S 0.03f
#define APP_RATE_KD_YAW_DEFAULT_US_PER_DPS_PER_S  0.02f
#define APP_RATE_KFF_ROLL_DEFAULT_US_PER_DPS_PER_S 0.00f
#define APP_RATE_KFF_PITCH_DEFAULT_US_PER_DPS_PER_S 0.00f
#define APP_RATE_KFF_YAW_DEFAULT_US_PER_DPS_PER_S  0.00f
#define APP_RATE_TERM_LIMIT_US      320
/* Fallback integral clamp used when ki is too small (including exactly 0, the
 * shipped default on all three axes as of 2026-08-20) for the normal
 * APP_RATE_TERM_LIMIT_US/ki bound to be computed - previously the whole
 * clamp block was skipped in that case, leaving the integral completely
 * unbounded. Purely a numeric-safety ceiling (nowhere close to the
 * dps*s magnitude any legitimate flight condition would reach) - if a single
 * bad (non-finite) value ever entered the integral from an upstream glitch,
 * an unbounded integral let it persist forever (NaN/Inf poison the running
 * sum permanently); this makes sure the integral always has SOME bound to
 * fall back into. Confirmed via SD log forensics 2026-08-20: pitch's rate-PID
 * output went to exactly 0 for the remainder of a flight starting mid-flight,
 * consistent with ki(=0) * Inf/NaN integral = NaN, (int32_t)NaN == 0 on this
 * ARM FPU - root trigger for the initial non-finite value still not
 * identified, see APP_PITCH_NONFINITE_DEBUG below. */
#define APP_RATE_INTEGRAL_SAFETY_LIMIT_DPS_S 10000.0f
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

/* Persistent storage for the ALTHOLD live-tunables (2026-08-30) - see
 * App_SaveAltholdSettings()/App_LoadAltholdSettings() below. Deliberately a SEPARATE
 * flash blob/sector from the rate-PID one above, not an extension of it - keeps this
 * new, less-tested save/load path from having any chance of corrupting or being
 * corrupted by the existing, heavily-relied-on PID gain storage. Bank 2 Sector 6
 * (0x081C0000) - the sector immediately below the PID blob's Sector 7, still well
 * outside the firmware image itself (which lives in Bank 1). */
#define APP_ALTHOLD_FLASH_MAGIC    0x484F4C44UL /* "HOLD" */
#define APP_ALTHOLD_FLASH_VERSION  1UL
#define APP_ALTHOLD_FLASH_ADDRESS  0x081C0000UL

typedef struct
{
  uint32_t magic;
  uint32_t version;
  float alt_hold_kp;
  float max_climb_mps;
  float pos_ki;
  float vz_kp;
  float vz_ki;
  float damp_gain;
  float damp_limit_us;
  uint32_t crc32;
  uint32_t reserved[6];
} App_AltholdFlashBlob_t;

typedef union
{
  App_AltholdFlashBlob_t blob;
  uint32_t words[24];
} App_AltholdFlashPage_t;

_Static_assert(sizeof(App_AltholdFlashBlob_t) <= sizeof(App_AltholdFlashPage_t), "ALTHOLD flash blob too large");

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
  APP_FLIGHT_MODE_NAV_POSHOLD = 3U,
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
extern I2C_HandleTypeDef hi2c1;

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
static volatile uint8_t g_gps_scan_pending = 0U;
static volatile uint8_t g_gps_factory_reset_pending = 0U;
static volatile uint8_t g_i2c1_scan_pending = 0U;
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
/* How often to sync g_sdlog_next_free_block to the superblock DURING an active
 * flight, not just at a clean disarm - see App_SdLogSaveSuperblock() call in
 * App_Update()'s armed logging path. Without this, a crash that never reaches
 * a clean disarm (App_SdLogFlushFlight()) leaves the superblock's next_free_block
 * pointing at wherever the PREVIOUS flight ended - the real data from the
 * crashed flight is still physically written to the card, but SDLOG DUMP/DUMP
 * LAST only ever read up to next_free_block, so it's invisible, AND the next
 * arm event resets the write cursor back to that same stale point and starts
 * overwriting it. This bit three real incidents in one session (confirmed via
 * manual per-block SD RBLOCK reads recovering real flight data - severe
 * battery sag, real motor PWM - well past the stale next_free_block). 1s
 * throttle bounds the worst-case unindexed/overwritable window to ~1s of
 * flight instead of the entire flight; SD_WriteBlock() here is a small
 * (512B) synchronous write, acceptable at this rate even from the armed
 * control-loop path. */
#define APP_SDLOG_SUPERBLOCK_SYNC_MS 1000U
#define APP_SDLOG_FLAG_ARMED 0x01U
#define APP_SDLOG_FLAG_MODE_SHIFT 1U
#define APP_SDLOG_FLAG_MODE_MASK 0x06U
#define APP_SDLOG_FLAG_LINK_ACTIVE 0x08U /* receiver_state.link_active, to tell a switch glitch apart from an RF link dropout */
#define APP_SDLOG_FLAG_MAG_HEALTHY 0x10U
#define APP_SDLOG_FLAG_MAG_NUDGE_GATED 0x20U /* field-deviation interference gate was suppressing the nudge this sample */

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
  uint8_t mag_field_dev_pct; /* |field| deviation from the boot reference, percent (see APP_MAG_TRUST_MAX_MAG_DEVIATION_FRAC) */
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
  int16_t rangefinder_cm_x10; /* ESP32-bridge HC-SR04 ground truth, 0 = no/stale reading -
                                * see App_GetRangefinderCm()'s comment */
  int16_t luna_cm_x10; /* ESP32-bridge TF-Luna LiDAR, 0 = no/stale reading - see
                         * App_GetLunaCm()'s comment */
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
  APP_SDLOGCMD_DUMP_FROM,
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
static uint32_t g_sdlog_dump_from_requested_block = 0U;

void App_RequestGpsScan(void)
{
  g_gps_scan_pending = 1U;
}

void App_RequestGpsFactoryReset(void)
{
  g_gps_factory_reset_pending = 1U;
}

void App_RequestI2c1Scan(void)
{
  g_i2c1_scan_pending = 1U;
}

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

/* Dumps from an arbitrary caller-supplied block through the current write
 * pointer - lets tooling pull "everything since block X" (e.g. the last N
 * flights, using an X recovered from a prior dump's SDLOG_DUMP[START]
 * header) without re-streaming the whole card like plain SDLOG DUMP does. */
void App_RequestSdLogDumpFrom(uint32_t block)
{
  g_sdlog_dump_from_requested_block = block;
  g_sdlog_cmd_pending = APP_SDLOGCMD_DUMP_FROM;
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

/* ESP32-bridge-reported HC-SR04 rangefinder (2026-08-23) - ground-truth height,
 * originally added purely for calibrating the baro's propwash/ground-effect altitude
 * bias (logged alongside baro_alt_cm on the SAME time_ms axis instead of trying to
 * align two independently-clocked capture streams after the fact - see
 * kh7-sdlog-corruption-bug and kh7-rangefinder-setup memory).
 *
 * As of 2026-08-23 also used to help ground_effect_clear detect clearing ground effect
 * faster/more reliably than baro altitude alone can during the liftoff window (see
 * kh7-baro-liftoff-transient memory - baro itself swings wildly, up to ~230cm off,
 * during exactly the window this gate needs to evaluate).
 *
 * IMPORTANT correction (2026-08-23, same day): this reaches the FC over Serial2/UART6 -
 * a direct, wired, onboard link from the ESP32, NOT WiFi. WiFi only carries the
 * SEPARATE ground-station copy of the same reading (bridge_client_printf's "[BRIDGE]
 * RANGE" line, over the TCP clients[]) - an earlier version of this comment wrongly
 * applied that link's WiFi latency/jitter/reconnect-drop risk to this one too, which
 * doesn't share that path at all. The real remaining risks here are narrower: the
 * ESP32 itself resetting/crashing (UART6 traffic just stops, caught by the staleness
 * check below same as any other failure), transient UART6 line corruption (mitigated
 * by the line-boundary-safe queuing on both directions - see
 * esp32_s3_uart6_wifi_bridge.ino), and the sensor's own inherent limits (near-field
 * dead zone during ESP32-side bootstrap lock, occasional multipath/off-axis echo
 * spikes that survive the median/MAD filter - see kh7-rangefinder-setup memory - and a
 * hard range ceiling around 4-6m). App_GetRangefinderCm() returns 0.0f (an otherwise
 * impossible reading - RANGE_MIN_VALID_CM on the ESP32 side floors real readings well
 * above 0) whenever the data is missing or stale, and every caller must treat that as
 * "no reading available right now, fall back to baro-only" rather than "height is
 * zero." */
#define APP_RANGEFINDER_STALE_MS 500U  /* ESP32 reports ~every 100ms; several missed/rejected
                                        * cycles in a row (not just one) before calling it stale */
static float g_rangefinder_cm = 0.0f;
static uint32_t g_rangefinder_last_update_ms = 0U;
/* Confidence (0.0-1.0) and the ESP32's own micros() at measurement time, added
 * 2026-08-25 alongside the "SENSOR" packet redesign (see
 * esp32_s3_uart6_wifi_bridge.ino's range_filter_apply_ex() comment for what confidence
 * means). Stored for future use - NOT yet consumed by App_GetBestHeightCm() or the
 * fusion below, which still behave exactly as before this redesign. Deciding how much
 * to lean on confidence in the actual control path is a separate, deliberate change,
 * not part of this one. */
static float g_rangefinder_confidence = 0.0f;
static uint32_t g_rangefinder_sensor_ts_us = 0U;
/* Fixed mounting descriptor from the ESP32's "SENSOR_CFG" line - see that sketch's
 * RANGE_MOUNT_AXIS comment. Not consumed anywhere yet (no tilt-compensation is
 * performed on the FC side currently); stored so it's available once that's added. */
static char g_rangefinder_mount_axis[8] = "";
static float g_rangefinder_mount_offset_deg = 0.0f;

/* g_rangefinder_cm is forced to 0.0f (the existing "unavailable" convention) whenever
 * the ESP32 reports this reading invalid - see esp32_s3_uart6_wifi_bridge.ino's
 * SENSOR packet comment - so App_GetRangefinderCm()'s existing staleness/zero-means-
 * unavailable contract below is unchanged. confidence/sensor_ts_us are stored
 * unconditionally (even for an invalid reading - a low-confidence rejection is still
 * useful diagnostic information).
 *
 * `valid` (2026-08-29) is passed through to VertEkf_UpdateRange() SEPARATELY from the
 * cm collapsing above - see that function's comment for why: a genuine 0cm reading
 * (this sensor mounted low enough to be sitting right at ground level, confirmed for
 * real via a liftoff capture showing strong signal strength at raw_cm=0) and an
 * actually-invalid reading both used to collapse to the same "cm==0.0f" wire value by
 * the time they reached the EKF, so the EKF silently discarded real ground-level
 * readings as if they were never sent - costing several real seconds of every flight,
 * including all of liftoff, with the very sensor most needed there absent. */
void App_SetRangefinderCm(float cm, float confidence, uint32_t sensor_ts_us, uint8_t valid)
{
  g_rangefinder_cm = (valid != 0U) ? cm : 0.0f;
  g_rangefinder_last_update_ms = HAL_GetTick();
  g_rangefinder_confidence = confidence;
  g_rangefinder_sensor_ts_us = sensor_ts_us;
  /* True event-driven async update (2026-08-29) - fired the instant a fresh
   * reading arrives, not polled, matching vert_ekf.c's stated design requirement. */
  VertEkf_UpdateRange(cm, confidence, 0U, valid);
}

void App_SetRangefinderMountDescriptor(const char *axis, float offset_deg)
{
  (void)strncpy(g_rangefinder_mount_axis, axis, sizeof(g_rangefinder_mount_axis) - 1U);
  g_rangefinder_mount_axis[sizeof(g_rangefinder_mount_axis) - 1U] = '\0';
  g_rangefinder_mount_offset_deg = offset_deg;
}

/* Returns 0.0f if no fresh reading is available - see the big comment above for why
 * every caller must treat that as "unavailable," not "height is zero." */
static float App_GetRangefinderCm(uint32_t now_ms)
{
  if ((g_rangefinder_last_update_ms == 0U) ||
      ((now_ms - g_rangefinder_last_update_ms) > APP_RANGEFINDER_STALE_MS))
  {
    return 0.0f;
  }
  return g_rangefinder_cm;
}

/* TF-Luna LiDAR (2026-08-25) - added ALONGSIDE the HC-SR04 rangefinder above, not
 * replacing it. Reaches the FC the same way (Serial2/UART6, direct wired link, not
 * WiFi - see App_GetRangefinderCm()'s comment above for the same correction applied
 * here) via a "LUNA <cm>" line from the ESP32 bridge - see
 * esp32_s3_uart6_wifi_bridge.ino's LUNA_RX_PIN comment for the sensor/wiring
 * details. Wider usable range (0.2-8m) and not subject to the sonar's multipath/
 * off-axis scatter, so App_GetBestHeightCm() below prefers it whenever it's fresh
 * and falls back to the sonar otherwise - same staleness convention as the sonar
 * (0.0f = unavailable, never "height is zero"). */
#define APP_LUNA_STALE_MS 500U
static float g_luna_cm = 0.0f;
static uint32_t g_luna_last_update_ms = 0U;
/* Same additions as the rangefinder's above, for the same reason - see
 * App_SetRangefinderCm()'s comment. */
static float g_luna_confidence = 0.0f;
static uint32_t g_luna_sensor_ts_us = 0U;
static char g_luna_mount_axis[8] = "";
static float g_luna_mount_offset_deg = 0.0f;

void App_SetLunaCm(float cm, float confidence, uint32_t sensor_ts_us, uint8_t valid)
{
  g_luna_cm = (valid != 0U) ? cm : 0.0f;
  g_luna_last_update_ms = HAL_GetTick();
  g_luna_confidence = confidence;
  g_luna_sensor_ts_us = sensor_ts_us;
  /* See App_SetRangefinderCm()'s comment - same true event-driven async update, and
   * the same reason `valid` is forwarded separately from the cm collapsing above. */
  VertEkf_UpdateRange(cm, confidence, 1U, valid);
}

void App_SetLunaMountDescriptor(const char *axis, float offset_deg)
{
  (void)strncpy(g_luna_mount_axis, axis, sizeof(g_luna_mount_axis) - 1U);
  g_luna_mount_axis[sizeof(g_luna_mount_axis) - 1U] = '\0';
  g_luna_mount_offset_deg = offset_deg;
}

static float App_GetLunaCm(uint32_t now_ms)
{
  if ((g_luna_last_update_ms == 0U) ||
      ((now_ms - g_luna_last_update_ms) > APP_LUNA_STALE_MS))
  {
    return 0.0f;
  }
  return g_luna_cm;
}

/* Centralizes "which external height sensor do we trust right now" for
 * ground_effect_clear's sensor-based supplement below - prefers TF-Luna
 * whenever fresh, falls back to the HC-SR04 rangefinder, and returns 0.0f
 * (unavailable) only if neither has a fresh reading. */
static float App_GetBestHeightCm(uint32_t now_ms)
{
  float luna_cm = App_GetLunaCm(now_ms);

  if (luna_cm > 0.0f)
  {
    return luna_cm;
  }
  return App_GetRangefinderCm(now_ms);
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
    else if ((cmd == APP_SDLOGCMD_DUMP) || (cmd == APP_SDLOGCMD_DUMP_LAST) || (cmd == APP_SDLOGCMD_DUMP_FROM))
    {
      if (g_glog_armed_state != 0U)
      {
        printf("SDLOG_DUMP[BUSY armed]\r\n");
      }
      else if ((g_sdlog_ready == 0U) || (g_sdlog_next_free_block <= 1U))
      {
        printf("SDLOG_DUMP[EMPTY]\r\n");
      }
      else if ((cmd == APP_SDLOGCMD_DUMP_FROM) &&
               ((g_sdlog_dump_from_requested_block < 1U) ||
                (g_sdlog_dump_from_requested_block > (g_sdlog_next_free_block - 1U))))
      {
        printf("SDLOG_DUMP[BAD_BLOCK requested=%lu valid=[1,%lu]]\r\n",
               (unsigned long)g_sdlog_dump_from_requested_block,
               (unsigned long)(g_sdlog_next_free_block - 1U));
      }
      else
      {
        /* DUMP LAST starts at the most recent flight's first block instead of
         * block 1, so pulling recent data doesn't re-stream the entire card's
         * history every time (that grows every arm and never gets shorter).
         * DUMP FROM lets a caller pick any earlier checkpoint block (e.g. the
         * first block of a previously-dumped flight) to pull everything since. */
        if (cmd == APP_SDLOGCMD_DUMP_LAST)
        {
          g_sdlog_dump_block = g_sdlog_last_flight_start_block;
        }
        else if (cmd == APP_SDLOGCMD_DUMP_FROM)
        {
          g_sdlog_dump_block = g_sdlog_dump_from_requested_block;
        }
        else
        {
          g_sdlog_dump_block = 1U;
        }
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
    return APP_FLIGHT_MODE_NAV_POSHOLD;
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
    case APP_FLIGHT_MODE_NAV_POSHOLD:
      return "NAVPOSHOLD";
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

  stick_offset_us = App_ApplyDeadbandUs(stick_offset_us, (int32_t)APP_NAVPOS_STICK_DEADBAND_US);
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

float App_GetAltholdAltHoldKp(void)
{
  return g_althold_alt_hold_kp_mps_per_m;
}

uint8_t App_SetAltholdAltHoldKp(float kp)
{
  if (App_IsFiniteInRange(kp, APP_ALTHOLD_ALT_HOLD_KP_MIN, APP_ALTHOLD_ALT_HOLD_KP_MAX) == 0U)
  {
    return 0U;
  }
  g_althold_alt_hold_kp_mps_per_m = kp;
  return 1U;
}

float App_GetAltholdMaxClimbMps(void)
{
  return g_althold_max_climb_mps;
}

uint8_t App_SetAltholdMaxClimbMps(float max_climb_mps)
{
  if (App_IsFiniteInRange(max_climb_mps, APP_ALTHOLD_MAX_CLIMB_MIN, APP_ALTHOLD_MAX_CLIMB_MAX) == 0U)
  {
    return 0U;
  }
  g_althold_max_climb_mps = max_climb_mps;
  return 1U;
}

float App_GetAltholdPosKi(void)
{
  return g_althold_pos_ki_per_s2;
}

uint8_t App_SetAltholdPosKi(float ki)
{
  if (App_IsFiniteInRange(ki, APP_ALTHOLD_POS_KI_MIN, APP_ALTHOLD_POS_KI_MAX) == 0U)
  {
    return 0U;
  }
  g_althold_pos_ki_per_s2 = ki;
  return 1U;
}

float App_GetNavPosKp(void)
{
  return g_navpos_kp_per_s;
}

uint8_t App_SetNavPosKp(float kp)
{
  if (App_IsFiniteInRange(kp, APP_NAVPOS_KP_MIN, APP_NAVPOS_KP_MAX) == 0U)
  {
    return 0U;
  }
  g_navpos_kp_per_s = kp;
  return 1U;
}

float App_GetNavPosKi(void)
{
  return g_navpos_ki_per_s2;
}

uint8_t App_SetNavPosKi(float ki)
{
  if (App_IsFiniteInRange(ki, APP_NAVPOS_KI_MIN, APP_NAVPOS_KI_MAX) == 0U)
  {
    return 0U;
  }
  g_navpos_ki_per_s2 = ki;
  return 1U;
}

float App_GetAltholdVzKp(void)
{
  return g_althold_vz_kp_us_per_mps;
}

uint8_t App_SetAltholdVzKp(float kp)
{
  if (App_IsFiniteInRange(kp, APP_ALTHOLD_VZ_KP_MIN, APP_ALTHOLD_VZ_KP_MAX) == 0U)
  {
    return 0U;
  }
  g_althold_vz_kp_us_per_mps = kp;
  return 1U;
}

float App_GetAltholdVzKi(void)
{
  return g_althold_vz_ki_us_per_mps_s;
}

uint8_t App_SetAltholdVzKi(float ki)
{
  if (App_IsFiniteInRange(ki, APP_ALTHOLD_VZ_KI_MIN, APP_ALTHOLD_VZ_KI_MAX) == 0U)
  {
    return 0U;
  }
  g_althold_vz_ki_us_per_mps_s = ki;
  return 1U;
}

float App_GetBaroVzDampGain(void)
{
  return g_baro_vz_damp_gain_us_per_mps;
}

uint8_t App_SetBaroVzDampGain(float gain)
{
  if (App_IsFiniteInRange(gain, APP_BARO_VZ_DAMP_GAIN_MIN, APP_BARO_VZ_DAMP_GAIN_MAX) == 0U)
  {
    return 0U;
  }
  g_baro_vz_damp_gain_us_per_mps = gain;
  return 1U;
}

uint32_t App_GetBaroVzDampLimit(void)
{
  return g_baro_vz_damp_limit_us;
}

uint8_t App_SetBaroVzDampLimit(float limit_us)
{
  if (App_IsFiniteInRange(limit_us, (float)APP_BARO_VZ_DAMP_LIMIT_MIN,
                           (float)APP_BARO_VZ_DAMP_LIMIT_MAX) == 0U)
  {
    return 0U;
  }
  g_baro_vz_damp_limit_us = (uint32_t)limit_us;
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

uint8_t App_LoadAltholdSettings(void)
{
  const App_AltholdFlashBlob_t *stored;
  uint32_t expected_crc;

  stored = (const App_AltholdFlashBlob_t *)APP_ALTHOLD_FLASH_ADDRESS;

  if (stored->magic != APP_ALTHOLD_FLASH_MAGIC)
  {
    printf("ALTHOLD_LOAD_DBG: bad magic=0x%08lX\r\n", (unsigned long)stored->magic);
    return 0U;
  }

  if (stored->version != APP_ALTHOLD_FLASH_VERSION)
  {
    printf("ALTHOLD_LOAD_DBG: bad version=%lu (expected %lu)\r\n",
           (unsigned long)stored->version, (unsigned long)APP_ALTHOLD_FLASH_VERSION);
    return 0U;
  }

  expected_crc = App_Crc32((const uint8_t *)stored, offsetof(App_AltholdFlashBlob_t, crc32));
  if (expected_crc != stored->crc32)
  {
    printf("ALTHOLD_LOAD_DBG: crc mismatch stored=0x%08lX computed=0x%08lX\r\n",
           (unsigned long)stored->crc32, (unsigned long)expected_crc);
    return 0U;
  }

  /* Each setter re-validates its own range - a stored value from a build with looser
   * bounds (or plain corruption that happened to pass CRC) still can't silently apply
   * an out-of-range gain. A rejected field just keeps its compiled-in default. */
  (void)App_SetAltholdAltHoldKp(stored->alt_hold_kp);
  (void)App_SetAltholdMaxClimbMps(stored->max_climb_mps);
  (void)App_SetAltholdPosKi(stored->pos_ki);
  (void)App_SetAltholdVzKp(stored->vz_kp);
  (void)App_SetAltholdVzKi(stored->vz_ki);
  (void)App_SetBaroVzDampGain(stored->damp_gain);
  (void)App_SetBaroVzDampLimit(stored->damp_limit_us);

  printf("ALTHOLD_LOAD_DBG: loaded kp=%.4f maxclimb=%.4f poski=%.4f vzkp=%.4f vzki=%.4f dampgain=%.4f damplimit=%.4f\r\n",
         (double)stored->alt_hold_kp, (double)stored->max_climb_mps, (double)stored->pos_ki,
         (double)stored->vz_kp, (double)stored->vz_ki, (double)stored->damp_gain,
         (double)stored->damp_limit_us);
  return 1U;
}

uint8_t App_SaveAltholdSettings(void)
{
  FLASH_EraseInitTypeDef erase;
  uint32_t sector_error = 0U;
  uint32_t address;
  App_AltholdFlashPage_t APP_FLASHWORD_ALIGN page;
  const App_AltholdFlashBlob_t *written;
  uint8_t write_index;

  /* See this function's declaration comment in app.h - self-contained armed check
   * (unlike the rate-PID save path, whose equivalent check lives in the deferred
   * command dispatcher instead) so every caller is protected automatically. */
  if (g_glog_armed_state != 0U)
  {
    printf("ALTHOLD_SAVE_DBG: refused while armed\r\n");
    return 0U;
  }

  memset(&page, 0xFF, sizeof(page));
  page.blob.magic = APP_ALTHOLD_FLASH_MAGIC;
  page.blob.version = APP_ALTHOLD_FLASH_VERSION;
  page.blob.alt_hold_kp = App_GetAltholdAltHoldKp();
  page.blob.max_climb_mps = App_GetAltholdMaxClimbMps();
  page.blob.pos_ki = App_GetAltholdPosKi();
  page.blob.vz_kp = App_GetAltholdVzKp();
  page.blob.vz_ki = App_GetAltholdVzKi();
  page.blob.damp_gain = App_GetBaroVzDampGain();
  page.blob.damp_limit_us = (float)App_GetBaroVzDampLimit();
  page.blob.crc32 = App_Crc32((const uint8_t *)&page.blob, offsetof(App_AltholdFlashBlob_t, crc32));

  if (HAL_FLASH_Unlock() != HAL_OK)
  {
    printf("ALTHOLD_SAVE_DBG: unlock fail err=0x%08lX\r\n", (unsigned long)HAL_FLASH_GetError());
    return 0U;
  }

  memset(&erase, 0, sizeof(erase));
  erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase.Banks = FLASH_BANK_2;
  erase.Sector = FLASH_SECTOR_6;
  erase.NbSectors = 1U;
  erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

  if (HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK)
  {
    printf("ALTHOLD_SAVE_DBG: erase fail sector_err=%lu flash_err=0x%08lX\r\n",
           (unsigned long)sector_error, (unsigned long)HAL_FLASH_GetError());
    (void)HAL_FLASH_Lock();
    return 0U;
  }

  address = APP_ALTHOLD_FLASH_ADDRESS;
  for (write_index = 0U; write_index < 3U; write_index++)
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                          address,
                          (uint32_t)&page.words[write_index * 8U]) != HAL_OK)
    {
      printf("ALTHOLD_SAVE_DBG: write fail word=%u addr=0x%08lX err=0x%08lX\r\n",
             (unsigned)write_index, (unsigned long)address, (unsigned long)HAL_FLASH_GetError());
      (void)HAL_FLASH_Lock();
      return 0U;
    }
    address += 32U;
  }

  (void)HAL_FLASH_Lock();

  written = (const App_AltholdFlashBlob_t *)APP_ALTHOLD_FLASH_ADDRESS;
  if ((written->magic != APP_ALTHOLD_FLASH_MAGIC) || (written->version != APP_ALTHOLD_FLASH_VERSION))
  {
    printf("ALTHOLD_SAVE_DBG: verify header fail magic=0x%08lX ver=%lu\r\n",
           (unsigned long)written->magic, (unsigned long)written->version);
    return 0U;
  }

  if (App_Crc32((const uint8_t *)written, offsetof(App_AltholdFlashBlob_t, crc32)) != written->crc32)
  {
    printf("ALTHOLD_SAVE_DBG: verify crc fail stored=0x%08lX computed=0x%08lX\r\n",
           (unsigned long)written->crc32,
           (unsigned long)App_Crc32((const uint8_t *)written, offsetof(App_AltholdFlashBlob_t, crc32)));
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
    printf("MAG_CAL[OK cx=%.4f cy=%.4f cz=%.4f wxx=%.4f wyy=%.4f wzz=%.4f wxy=%.4f wxz=%.4f wyz=%.4f]\r\n",
           (double)Mag_GetCalCenterX(), (double)Mag_GetCalCenterY(), (double)Mag_GetCalCenterZ(),
           (double)Mag_GetCalMatrixXX(), (double)Mag_GetCalMatrixYY(), (double)Mag_GetCalMatrixZZ(),
           (double)Mag_GetCalMatrixXY(), (double)Mag_GetCalMatrixXZ(), (double)Mag_GetCalMatrixYZ());
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
  VertEkf_Init();
  Receiver_Init();

  g_boot_pid_loaded = App_LoadRatePidGains();
  if (g_boot_pid_loaded == 0U)
  {
    App_ResetRatePidDefaults();
  }

  /* Silently keeps compiled-in defaults (already assigned via the static
   * initializers) if nothing was ever saved - not a failure, just "no saved
   * settings yet" (e.g. this exact board/build has never called ALTHOLD SAVE). */
  (void)App_LoadAltholdSettings();

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
  /* Throttles the ARM_BLOCKED[...] print below to once/sec instead of every 2ms loop
   * while the pilot holds the arm switch against a blocked gate. */
  static uint32_t last_arm_nav_block_print_ms = 0U;
  static uint8_t disarm_condition_active = 0U;
  static uint32_t disarm_condition_start_ms = 0U;
  static uint8_t startup_safety_checked = 0U;
  static uint8_t startup_arm_blocked = 0U;
  static uint8_t beeper_on = 0U;
  static uint32_t last_beeper_toggle_ms = 0U;
  static uint8_t low_battery_cells_known = 0U;
  static uint8_t low_battery_cell_count = 0U;
  static uint8_t low_battery_beeper_on = 0U;
  static uint8_t low_battery_warned_once = 0U;
  static uint8_t low_battery_critical_active = 0U; /* throttle ceiling - can release (hysteresis) if voltage recovers */
  static uint8_t low_battery_disarm_active = 0U;   /* hard floor - one-way latch for the rest of this boot */
  static uint32_t last_crsf_battery_telem_ms = 0U;
  static uint32_t last_crsf_attitude_telem_ms = 0U;
  static uint32_t last_crsf_vario_telem_ms = 0U;
  static uint32_t last_crsf_flightmode_telem_ms = 0U;
  static uint32_t last_crsf_gps_telem_ms = 0U;
  static uint32_t sdlog_decim_counter = 0U;
  static uint16_t roll_center_us = APP_PWM_MID_US;
  static uint16_t pitch_center_us = APP_PWM_MID_US;
  static uint16_t yaw_center_us = APP_PWM_MID_US;
  /* REVISED 2026-08-22 (same day as the original settle-latch fix): a real
   * flight (yaw-hold-vs-no-yaw-hold comparison, ALTHOLD deliberately excluded
   * to isolate the yaw subsystem) came back with yaw "wandering worse" than
   * before yaw-hold existed. Root cause: unlike ALTHOLD's throttle (which has
   * no natural resting position and genuinely needs continuous re-discovery
   * of wherever hover happens to be), the yaw stick is spring-centered with a
   * FIXED true neutral. Copying ALTHOLD's "settle anywhere the stick sits
   * still" technique wholesale was wrong for yaw - it treated ANY steady
   * stick position held for the settle window as the new "center," including
   * a deliberate sustained turn (very normal - holding a steady yaw
   * deflection for a controlled turn easily exceeds 300ms), corrupting
   * yaw_center_us mid-flight and throwing off both the engagement gate and
   * every subsequent yaw-hold target. Fixed by locking the settle-latch after
   * its FIRST successful commit each arm (yaw_center_locked below) - it still
   * gets to briefly refine away from a bad arm-instant snapshot right after
   * arming (fixing the original bench-tested engagement-delay bug), but can
   * never again drift to wherever the stick happens to be during a real
   * maneuver later in the flight. */
  static uint16_t yaw_settle_ref_us = APP_PWM_MID_US;
  static uint32_t yaw_settle_start_ms = 0U;
  static uint8_t yaw_center_locked = 0U;
  static float measured_roll_rate_dps = 0.0f;
  static float measured_pitch_rate_dps = 0.0f;
  static float measured_yaw_rate_dps = 0.0f;
  static float filtered_gyro_roll_rate_dps = 0.0f;
  static float filtered_gyro_pitch_rate_dps = 0.0f;
  static float filtered_gyro_yaw_rate_dps = 0.0f;
  static float roll_integral_dps_s = 0.0f;
  static float pitch_integral_dps_s = 0.0f;
  static float yaw_integral_dps_s = 0.0f;
  /* Diagnostic-only, see the isfinite() checks near pitch_term_f below - not
   * a control-behavior fix, just catches and reports which rate-PID input
   * first goes non-finite so the actual upstream trigger can be identified
   * (root cause not yet found as of 2026-08-20 - see kh7-yaw-system-state-
   * adjacent notes). One report per arm session so a recurring fault doesn't
   * flood telemetry. */
  static uint8_t roll_nonfinite_reported = 0U;
  static uint8_t pitch_nonfinite_reported = 0U;
  static uint8_t yaw_nonfinite_reported = 0U;
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
  /* BUG FOUND (2026-08-17): the fixed APP_ATTITUDE_ZERO_SETTLE_MS timer races
   * gyro-bias readiness (APP_YAW_BIAS_SETTLE_SAMPLES at the 2ms/500Hz loop is
   * ~1000*2ms=2000ms - the SAME as this timer), so attitude-zero could complete
   * before bias correction ever engages, averaging/zeroing yaw while gz still
   * has its full raw, uncalibrated offset. That raw offset integrating
   * unopposed for ~2s was enough to push yaw close to the +-180deg wrap
   * boundary before any correction (bias or mag-nudge) got a chance to act,
   * which is a degenerate point for an angle-based P-controller (see
   * APP_MAG_YAW_NUDGE_MAX_DPS above) - reproduced at every power-on. Fixed by
   * keying the averaging window off gyro-bias-ready instead of the boot clock. */
  static uint8_t bias_ready_seen_for_zero = 0U;
  static uint32_t bias_ready_since_ms = 0U;
  static uint8_t mag_yaw_ref_captured = 0U;
  static float mag_heading_at_ref_deg = 0.0f;
  static float mag_ref_field_g = 0.0f;
  static float mag_yaw_nudge_dps = 0.0f;
  /* Motor current (proxy: avg motor PWM above idle) feeds Mag_GetHeadingDeg()'s
   * power-induced heading compensation - see mag.c. Updated once, right after
   * motor mixing below (the only non-duplicated place all 4 motor outputs are
   * known this iteration); read by every Mag_GetHeadingDeg() call site
   * regardless of which of the two IMU-processing branches is active this
   * pass, so it's always at most one iteration stale - negligible for a
   * signal this slow-varying. Bench-characterized 2026-08-21: real, repeatable,
   * reversible ~0.05deg heading shift per us of motor PWM above idle (props on,
   * secured stationary, ~10deg observed at hover power - see mag.c). */
  static float g_avg_motor_power_delta_us = 0.0f;
  /* Most recent raw accelerometer reading (g's), updated right after each
   * Attitude_UpdateIMU() call in either IMU-processing branch below - used to
   * feed Attitude_GetVerticalAccelMps2() for the baro climb-rate
   * complementary filter (see baro.c) at the Baro_Update() call site, which
   * runs on its own separate schedule (not necessarily every IMU sample), so
   * it needs a persisted "latest" value rather than a purely local one. */
  static float g_last_ax_g = 0.0f;
  static float g_last_ay_g = 0.0f;
  static float g_last_az_g = 1.0f;
  /* Logged to SD (see mag_field_dev_pct/APP_SDLOG_FLAG_MAG_NUDGE_GATED below)
   * so a captured flight shows directly whether/how much the field-strength
   * interference gate is suppressing the nudge, instead of having to infer it
   * indirectly from noisy mag_heading_deg jumps after the fact. */
  static uint8_t mag_field_dev_pct = 0U;
  static uint8_t mag_nudge_gated = 0U;
  /* Reference-capture averaging state (see APP_MAG_REF_AVG_MS below) - heading is
   * circular, so this averages unit vectors (sin/cos sums), not raw degrees, to
   * avoid a wraparound-crossing average being wrong the same way a single bad
   * sample was. */
  static float mag_ref_avg_sin_sum = 0.0f;
  static float mag_ref_avg_cos_sum = 0.0f;
  static float mag_ref_avg_field_sum_g = 0.0f;
  static uint32_t mag_ref_avg_sample_count = 0U;
  static uint32_t mag_ref_avg_start_ms = 0U;
  static float battery_voltage_filtered_v = 0.0f;
  static uint8_t battery_voltage_valid = 0U;
  static uint32_t battery_adc_raw = 0U;
  static uint32_t last_battery_sample_ms = 0U;
  static uint32_t last_baro_sample_ms = 0U;
  static uint32_t last_baro_retry_ms = 0U;
  static uint32_t last_gps_retry_ms = 0U;
  /* Detects a genuinely FRESH GPS fix (vs. re-processing the same stale one every
   * iteration) for VertEkf_UpdateGps() - see that call site below. */
  static uint32_t last_gps_ekf_pvt_host_ms = 0U;
  static uint32_t last_sdlog_superblock_sync_ms = 0U;
  static uint32_t last_mag_sample_ms = 0U;
  static uint32_t last_mag_retry_ms = 0U;
  static uint8_t liftoff_ramp_active = 0U;
  static uint32_t liftoff_ramp_start_ms = 0U;
  /* Yaw angle-hold - see the capture/use site near cmd_yaw_rate_dps below for
   * the full design writeup. Unlike ALTHOLD's persistent-hover-reference
   * (which had to survive brief correction nudges without losing the pilot's
   * original target), a fresh capture on every deadband re-entry is the
   * CORRECT behavior here: a yaw stick touch is essentially always a
   * deliberate "face this new direction" action, not a drift correction, so
   * there is no equivalent "should this recapture or not" ambiguity to get
   * wrong. */
  static uint8_t yawhold_active = 0U;
  static float yawhold_target_deg = 0.0f;
  static uint8_t althold_holding = 0U;
  static float althold_target_alt_m = 0.0f;
  static float althold_integral_us = 0.0f;
  /* See APP_ALTHOLD_POS_KI_PER_S2's comment - integral on the OUTER position-hold
   * loop, separate from althold_integral_us above (which is the INNER climb-rate
   * loop's integral). Same reset discipline as that one: zeroed on every fresh hold,
   * on leaving the centered branch, and on arm. */
  static float althold_pos_integral_mps = 0.0f;
  /* Slow, full-authority base throttle estimate - see the big comment at the
   * ALTHOLD throttle block below and kh7-althold-throttle-incident memory.
   * Updated EVERY iteration ALTHOLD/NAVBRAKE is selected, regardless of the
   * ground-effect/baro-health gate (deliberately decoupled - gating this was
   * the root cause of the 2026-08-21 incident), so it already holds a
   * realistic value by the time the gate opens instead of starting cold. */
  static float althold_hover_throttle_us = APP_ALTHOLD_HOVER_EST_SEED_US;
  /* Where "center" actually is for throttle-stick classification (centered =
   * lock altitude, off-center = climb/descend rate), replacing a fixed
   * APP_PWM_MID_US=1500 tried and reverted the same day (2026-08-23): this
   * airframe's real hover throttle (~1200-1270us observed) is nowhere near
   * 1500, so a fixed center made it impossible to comfortably approach the
   * hold zone without climbing hard the whole way there. Re-latches to
   * wherever the stick has genuinely SETTLED (stayed within
   * APP_ALTHOLD_STICK_STABLE_WINDOW_US for APP_ALTHOLD_THROTTLE_SETTLE_MS -
   * NOT the shorter APP_ALTHOLD_STICK_SETTLE_MS yaw-hold uses, see that
   * constant's comment for why throttle needs its own, much longer one),
   * tracked UNCONDITIONALLY whenever ALTHOLD/NAVBRAKE is selected regardless
   * of the ground-effect gate (same "don't gate reference-tracking behind
   * the gate" lesson as althold_hover_throttle_us above -
   * kh7-althold-throttle-incident memory) so it's already meaningful by the
   * time full authority engages. Deliberately never tied to any particular
   * throttle VALUE - it only ever means "settled here," so it works
   * regardless of what this or any other airframe's real hover throttle is. */
  static uint16_t althold_settle_ref_us = APP_PWM_MID_US;
  static uint32_t althold_settle_start_ms = 0U;
  static uint16_t althold_settled_center_us = APP_PWM_MID_US;
  /* Remembers whether full closed-loop authority was active on the PREVIOUS
   * iteration, purely to detect the open-loop-to-closed-loop transition edge
   * - see where it's checked, in the full-authority branch below. */
  static uint8_t althold_authority_was_active = 0U;
  static float althold_trim_filtered_us = 0.0f;
  static float baro_damp_term_filtered_us = 0.0f;
  /* Debounced, ONE-WAY-LATCHED (per arm cycle) replacement for a plain
   * altitude>=threshold check - see APP_GROUND_EFFECT_CLEAR_DWELL_MS's comment for
   * why the plain check flickers right at liftoff, and the 2026-08-30 incident
   * comment at this flag's use sites for why it was made one-way (a real emergency
   * mid-flight disarm - a two-way flicker right at the liftoff boundary was
   * silently swapping throttle control laws underneath the pilot's stick).
   * ground_effect_below_since_ms tracks the start of the current continuous stretch
   * at/above the threshold (reset to "now" on every dip below it while NOT YET
   * clear, same idiom as the other settle-latches in this file), and
   * ground_effect_clear latches true once that stretch has held for the full dwell
   * time - and then stays true, ignoring further dips, until the next arm-reset. */
  static uint8_t ground_effect_clear = 0U;
  static uint32_t ground_effect_below_since_ms = 0U;
  /* Same idiom, independent dwell clock, for the rangefinder-based supplemental
   * clear path added 2026-08-23 - see where it's evaluated below and
   * App_GetRangefinderCm()'s comment for why this can only ever ADD a way to
   * reach ground_effect_clear sooner, never replace the baro path above. */
  static uint32_t ground_effect_rangefinder_below_since_ms = 0U;
  /* NAV_POSHOLD state (2026-09-04 rewrite - see the APP_NAVPOS_* block above).
   * navpos_active means a target is currently latched and being held.
   * Deliberately no disqualify-latch requiring a mode-reselect to recover: real
   * flight data showed this GPS throwing frequent brief, self-recovering
   * validity blips, and a one-strike lockout meant one blip could disable
   * holding for the whole rest of a flight. Silently dropping out and
   * re-attempting the (still velocity-gated) relatch on the very next good
   * sample is simpler and matches how the off-center-stick case already
   * behaves - no separate latch state to manage. */
  static uint8_t navpos_active = 0U;
  /* Tracks whether the GPS engage-gate was satisfied on the PREVIOUS iteration
   * (independent of navpos_active, which specifically means "actively holding
   * a target" - off-center manual flying under this mode is engaged but not
   * active). Used solely to trigger exactly one Nav_LatchReference() call per
   * fresh engagement, not on every iteration the gate happens to be open. */
  static uint8_t navpos_was_engaged = 0U;
  /* Separate from navpos_was_engaged: only true immediately after a genuinely
   * fresh entry into NAV_POSHOLD (mode just selected, or just armed into it) -
   * NOT after every brief GPS-staleness blip clears. Real flight data
   * (2026-09-04) showed nav_state.valid flickering false for 100ms-3s several
   * times per minute (GPS_STALE/REACQUIRING) while nav_state.reference_valid
   * stayed continuously true the whole time - the local NED reference was
   * never actually lost, so re-latching (which zeroes filtered_north/east_m at
   * the CURRENT position) on every recovery was needless and harmful: it both
   * discarded a still-good position reference and, combined with the fallback
   * below swinging to full ATTITUDE-mode tilt authority, made stick response
   * snap between two very different feels many times per flight - reported as
   * "cannot control lat/long". */
  static uint8_t navpos_needs_latch = 1U;
  static float navpos_target_north_m = 0.0f;
  static float navpos_target_east_m = 0.0f;
  static float navpos_integral_north_mps = 0.0f;
  static float navpos_integral_east_mps = 0.0f;
  /* Gates ALL GPS init/retry/parsing and Nav_Update() to only when NAV_POSHOLD is
   * the currently selected mode - updated wherever flight_mode is (re)computed
   * below, so it always reflects the mode switch with at most one iteration of
   * lag. */
  static App_FlightMode_t last_known_flight_mode = APP_FLIGHT_MODE_RATE;
  /* pitch_deg/roll_deg/yaw_deg are computed fresh inside whichever of the two
   * IMU-read branches below actually runs this iteration, so they aren't
   * safely readable from the single common call site the CRSF attitude
   * telemetry send lives at - mirrors last_known_flight_mode's role above. */
  static float last_known_pitch_deg = 0.0f;
  static float last_known_roll_deg = 0.0f;
  static float last_known_yaw_deg = 0.0f;
  int32_t althold_trim_us = 0;
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
  float yaw_angle_error_deg;
  Nav_State_t nav_state;
  uint8_t navpos_requested;
  uint8_t navpos_tilt_limited = 0U;
  uint8_t navpos_accel_limited = 0U;
  float navpos_desired_north_vel_mps = 0.0f;
  float navpos_desired_east_vel_mps = 0.0f;
  float navpos_north_vel_error_mps = 0.0f;
  float navpos_east_vel_error_mps = 0.0f;
  float navpos_north_accel_cmd_mps2 = 0.0f;
  float navpos_east_accel_cmd_mps2 = 0.0f;
  float navpos_fwd_accel_cmd_mps2 = 0.0f;
  float navpos_right_accel_cmd_mps2 = 0.0f;
  float navpos_err_north_m = 0.0f;
  float navpos_err_east_m = 0.0f;
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
  int32_t baro_damp_term_us = 0;
  int32_t throttle_offset_us;
  int32_t althold_throttle_us;
  uint8_t althold_authority_active = 0U;
  uint8_t althold_liftoff_assist_active;
  float althold_liftoff_base_us;
  float height_cm_now;
  float althold_fused_alt_m_now = 0.0f;
  float climb_rate_setpoint_mps = 0.0f;
  float climb_rate_error_mps = 0.0f;
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
  navpos_requested = 0U;
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

  /* Low-battery beeper - see APP_LOW_BATTERY_WARN_CELL_V above. Runs whether
   * armed or disarmed; yields to the arm-blocked beeper (startup_arm_blocked)
   * rather than fighting it for the same GPIO pin. */
  if ((low_battery_cells_known != 0U) && (startup_arm_blocked == 0U) &&
      (battery_voltage_filtered_v > 1.0f) &&
      (battery_voltage_filtered_v < (((float)low_battery_cell_count) * APP_LOW_BATTERY_WARN_CELL_V)))
  {
    uint32_t phase_ms = now_ms % APP_LOW_BATTERY_BEEP_CYCLE_MS;
    uint32_t slot_ms = phase_ms % (APP_LOW_BATTERY_BEEP_ON_MS + APP_LOW_BATTERY_BEEP_GAP_MS);
    uint8_t want_on = ((phase_ms < (3U * (APP_LOW_BATTERY_BEEP_ON_MS + APP_LOW_BATTERY_BEEP_GAP_MS))) &&
                       (slot_ms < APP_LOW_BATTERY_BEEP_ON_MS)) ? 1U : 0U;

    if (want_on != low_battery_beeper_on)
    {
      low_battery_beeper_on = want_on;
      HAL_GPIO_WritePin(BEEPER_GPIO_Port, BEEPER_Pin, (want_on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
    if (low_battery_warned_once == 0U)
    {
      low_battery_warned_once = 1U;
      printf("LOW_BATTERY[cells=%u v=%.2f threshold=%.2f]\r\n",
             (unsigned int)low_battery_cell_count, (double)battery_voltage_filtered_v,
             (double)(((float)low_battery_cell_count) * APP_LOW_BATTERY_WARN_CELL_V));
    }
  }
  else if (low_battery_beeper_on != 0U)
  {
    low_battery_beeper_on = 0U;
    HAL_GPIO_WritePin(BEEPER_GPIO_Port, BEEPER_Pin, GPIO_PIN_RESET);
  }

  /* Common-location fallback/correction for cell-count inference - found
   * 2026-08-21 that the two duplicated IMU-branch battery-read blocks had
   * silently diverged (a replace_all edit only matched one of them), so
   * whichever branch actually ran determined whether inference happened at
   * all - if the unpatched one ran, low_battery_cells_known never got set
   * and every tier below stayed permanently disabled all boot, with no
   * warning, no clamp, no disarm, despite the pack reaching ~2.98V/cell.
   * This location is NOT duplicated, runs every iteration regardless of
   * which IMU branch executed, and both (a) catches cases where neither
   * per-branch inference ran and (b) re-infers if the current reading is no
   * longer plausible for the previously-inferred cell count (guards against
   * a bad one-off first sample, a separate risk from the duplication bug). */
  if ((low_battery_cells_known != 0U) &&
      (battery_voltage_filtered_v > ((((float)low_battery_cell_count) * APP_LOW_BATTERY_MAX_CHARGE_CELL_V) + 1.0f)))
  {
    printf("LOW_BATTERY_REINFER[was_cells=%u v=%.2f]\r\n",
           (unsigned int)low_battery_cell_count, (double)battery_voltage_filtered_v);
    low_battery_cells_known = 0U;
  }
  if ((low_battery_cells_known == 0U) && (battery_voltage_valid != 0U) && (battery_voltage_filtered_v > 1.0f))
  {
    uint8_t cells;
    for (cells = 1U; cells < 13U; cells++)
    {
      if (battery_voltage_filtered_v <= (((float)cells) * APP_LOW_BATTERY_MAX_CHARGE_CELL_V))
      {
        break;
      }
    }
    low_battery_cell_count = cells;
    low_battery_cells_known = 1U;
    printf("LOW_BATTERY_CELLS[inferred=%u v=%.2f]\r\n",
           (unsigned int)low_battery_cell_count, (double)battery_voltage_filtered_v);
  }

  /* Tiers 2/3 of the low-battery response - see the defines above for the
   * full rationale. Evaluated here (common to every mode) so the flags are
   * already current by the time the throttle clamp and disarm-condition
   * checks below consume them. */
  if ((low_battery_cells_known != 0U) && (battery_voltage_filtered_v > 1.0f))
  {
    float critical_v = ((float)low_battery_cell_count) * APP_LOW_BATTERY_CRITICAL_CELL_V;
    float critical_release_v = critical_v + (((float)low_battery_cell_count) * APP_LOW_BATTERY_CRITICAL_HYST_V);

    if (low_battery_critical_active == 0U)
    {
      if (battery_voltage_filtered_v < critical_v)
      {
        low_battery_critical_active = 1U;
        printf("LOW_BATTERY_CRITICAL[on v=%.2f threshold=%.2f]\r\n",
               (double)battery_voltage_filtered_v, (double)critical_v);
      }
    }
    else if (battery_voltage_filtered_v >= critical_release_v)
    {
      low_battery_critical_active = 0U;
      printf("LOW_BATTERY_CRITICAL[off v=%.2f]\r\n", (double)battery_voltage_filtered_v);
    }

    if ((low_battery_disarm_active == 0U) &&
        (battery_voltage_filtered_v < (((float)low_battery_cell_count) * APP_LOW_BATTERY_DISARM_CELL_V)))
    {
      low_battery_disarm_active = 1U;
      printf("LOW_BATTERY_DISARM[triggered v=%.2f]\r\n", (double)battery_voltage_filtered_v);
    }
  }

  if (battery_voltage_valid != 0U &&
      ((now_ms - last_crsf_battery_telem_ms) >= APP_CRSF_BATTERY_TELEM_MS))
  {
    Receiver_SendBatteryTelemetry(battery_voltage_filtered_v);
    last_crsf_battery_telem_ms = now_ms;
  }

  /* Remaining iNav-compatible CRSF telemetry frames, at roughly iNav's own
   * rates (attitude fast/smooth, GPS/flight-mode slow since they change
   * rarely) - see the frame builder comments in receiver.c for exact wire
   * format sourcing. */
  if ((now_ms - last_crsf_attitude_telem_ms) >= APP_CRSF_ATTITUDE_TELEM_MS)
  {
    /* Roll negated here only - confirmed inverted on the EdgeTX iNav widget's
     * display 2026-08-21 (this FC's own roll_deg sign convention is otherwise
     * unrelated to and independently correct for flight control; this flip is
     * purely to match what the telemetry display expects). Pitch/yaw matched
     * fine, so left as-is (pitch was initially misreported as reversed and
     * flipped, then corrected back once roll was identified as the actual
     * culprit). */
    Receiver_SendAttitudeTelemetry(last_known_pitch_deg, -last_known_roll_deg, last_known_yaw_deg);
    last_crsf_attitude_telem_ms = now_ms;
  }

  if ((now_ms - last_crsf_vario_telem_ms) >= APP_CRSF_VARIO_TELEM_MS)
  {
    Receiver_SendVarioTelemetry(Baro_GetClimbRateMps());
    last_crsf_vario_telem_ms = now_ms;
  }

  if ((now_ms - last_crsf_flightmode_telem_ms) >= APP_CRSF_FLIGHTMODE_TELEM_MS)
  {
    Receiver_SendFlightModeTelemetry(App_FlightModeName(last_known_flight_mode));
    last_crsf_flightmode_telem_ms = now_ms;
  }

  if ((now_ms - last_crsf_gps_telem_ms) >= APP_CRSF_GPS_TELEM_MS)
  {
    Receiver_SendGpsTelemetry(GPS_GetLatitudeDeg(), GPS_GetLongitudeDeg(), GPS_GetAltitudeM(),
                              GPS_GetGroundSpeedMps(), GPS_GetCourseDeg(), GPS_GetNumSatellites());
    last_crsf_gps_telem_ms = now_ms;
  }

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
    (void)Baro_Update(g_avg_motor_power_delta_us,
                      Attitude_GetVerticalAccelMps2(g_last_ax_g, g_last_ay_g, g_last_az_g));
    VertEkf_UpdateBaro(Baro_GetRawAltitudeM(), Baro_IsHealthy(), g_avg_motor_power_delta_us);
    last_baro_sample_ms = now_ms;
  }

  /* GPS bias trim/divergence check - only ever fires while GPS is actually
   * configured/parsing, which per the comment below is NAV_VELOCITY_BRAKE only in
   * this firmware today; a no-op elsewhere via VertEkf_UpdateGps()'s own health
   * gate, not something changed by adding this call. GPS_GetLastPvtHostMs()
   * changing is what "fresh fix" means here - GPS_GetAltitudeM() alone would
   * report the same stale value between fixes, which would wrongly look like new
   * agreement/disagreement information to the slow trim every single iteration. */
  if (GPS_GetLastPvtHostMs() != last_gps_ekf_pvt_host_ms)
  {
    last_gps_ekf_pvt_host_ms = GPS_GetLastPvtHostMs();
    VertEkf_UpdateGps(GPS_GetAltitudeM(), GPS_GetVerticalAccuracyM(), GPS_IsHealthy(), motors_armed);
  }

  /* GPS_Init() blocks waiting for UBX ACKs (up to ~1s worst case) - only retry
   * while disarmed, matching the SD/PID-save armed-guard pattern above. GPS is
   * only used by NAV_POSHOLD - no GPS init/retry/parsing in any other mode. */
  if ((last_known_flight_mode == APP_FLIGHT_MODE_NAV_POSHOLD) &&
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

  if (g_gps_scan_pending != 0U)
  {
    g_gps_scan_pending = 0U;
    if (g_glog_armed_state != 0U)
    {
      printf("GPS_SCAN[FAIL armed]\r\n");
    }
    else
    {
      GPS_ScanBaud();
      /* Let the normal retry logic above re-attempt the real GPS_Init()
       * handshake on the very next iteration, at whatever baud the scan
       * settled huart3 on. */
      last_gps_retry_ms = 0U;
    }
  }

  if (g_i2c1_scan_pending != 0U)
  {
    g_i2c1_scan_pending = 0U;
    if (g_glog_armed_state != 0U)
    {
      printf("I2C1_SCAN[FAIL armed]\r\n");
    }
    else
    {
      /* Raw 7-bit address sweep of I2C1 (shared by mag.c's QMC5883L at 0x0D
       * and baro.c's SPL06-family chip at 0x76/0x77) - answers "is the address
       * wrong" directly instead of inferring it from FAIL/chip_id=0xFF, which
       * is ambiguous (0xFF is also the QMC5883L's genuine chip-ID value, and
       * also g_last_chip_id's untouched default - see mag.c). Skips the
       * reserved 0x00-0x07/0x78-0x7F ranges per the I2C spec. Added 2026-08-21
       * after the mag (and later baro, on the same bus) went unresponsive
       * post-crash. */
      uint8_t addr7;
      uint8_t found_count = 0U;
      printf("I2C1_SCAN[start]\r\n");
      for (addr7 = 0x08U; addr7 <= 0x77U; addr7++)
      {
        if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr7 << 1), 2U, 5U) == HAL_OK)
        {
          printf("I2C1_SCAN[found addr7=0x%02X addr8=0x%02X]\r\n", (unsigned int)addr7,
                 (unsigned int)(addr7 << 1));
          found_count++;
        }
      }
      printf("I2C1_SCAN[done count=%u]\r\n", (unsigned int)found_count);
    }
  }

  if (g_gps_factory_reset_pending != 0U)
  {
    g_gps_factory_reset_pending = 0U;
    if (g_glog_armed_state != 0U)
    {
      printf("GPS_FACTORY_RESET[FAIL armed]\r\n");
    }
    else
    {
      GPS_FactoryReset();
      last_gps_retry_ms = 0U;
    }
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

  /* Nav_Update() (and the GPS data it consumes) is only relevant to NAV_POSHOLD -
   * skipped entirely otherwise, leaving nav_state at its last value. */
  if (last_known_flight_mode == APP_FLIGHT_MODE_NAV_POSHOLD)
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

          if ((low_battery_cells_known == 0U) && (battery_voltage_v > 1.0f))
          {
            uint8_t cells;
            for (cells = 1U; cells < 13U; cells++)
            {
              if (battery_voltage_v <= (((float)cells) * APP_LOW_BATTERY_MAX_CHARGE_CELL_V))
              {
                break;
              }
            }
            low_battery_cell_count = cells;
            low_battery_cells_known = 1U;
          }
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
      /* BUG FOUND (2026-08-17): this was `+=`, i.e. assumed increasing gz_dps
       * increases yaw_deg. Attitude_GetBoardAnglesDeg() computes
       * `yaw = -atan2f(...)` - note the explicit negation - so the true
       * relationship is inverted: increasing gz_dps DECREASES yaw_deg. With
       * `+=`, a nudge computed to correct a positive error (yaw needs to
       * increase) instead drove yaw the wrong way, making the error grow,
       * making the nudge grow, etc. - genuine positive feedback. Confirmed
       * directly via live MAGDBG telemetry: nudge went increasingly negative
       * (-2.94 -> -20.00 dps, correctly trying to pull yaw down) while yaw
       * simultaneously climbed 5.9 -> 139.0deg in the same window - the
       * "correction" and the observed effect moved in the same direction.
       * This is very likely the actual root cause of every yaw-drift/runaway
       * symptom chased this session, not gain magnitude - the gain increases
       * made the (backwards) feedback loop diverge faster, which is why
       * "raising MAX_DPS alone made the chatter worse, not better" (see the
       * slew-limiter comment above) - a real, retrospectively obvious tell. */
      gz_dps -= mag_yaw_nudge_dps;

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
      g_last_ax_g = ax_g;
      g_last_ay_g = ay_g;
      g_last_az_g = az_g;
      VertEkf_Predict(Attitude_GetVerticalAccelMps2(ax_g, ay_g, az_g),
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
        if ((bias_ready_seen_for_zero == 0U) && (g_gyro_bias_ready != 0U))
        {
          bias_ready_seen_for_zero = 1U;
          bias_ready_since_ms = now_ms;
        }

        if ((bias_ready_seen_for_zero != 0U) && (now_ms >= bias_ready_since_ms))
        {
          startup_roll_offset_sum_deg += roll_deg;
          startup_pitch_offset_sum_deg += pitch_deg;
          startup_yaw_offset_sum_deg += yaw_deg;
          startup_zero_avg_sample_count++;
        }

        if ((bias_ready_seen_for_zero != 0U) &&
            ((now_ms - bias_ready_since_ms) >= APP_ATTITUDE_ZERO_AVG_MS) &&
            (startup_zero_avg_sample_count > 0U))
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
      last_known_pitch_deg = pitch_deg;
      last_known_roll_deg = roll_deg;
      last_known_yaw_deg = yaw_deg;

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
          float heading_now_rad = Mag_GetHeadingDeg(mag_tilt_roll_deg, mag_tilt_pitch_deg, g_avg_motor_power_delta_us) * RAD_PER_DEG;

          if (mag_ref_avg_sample_count == 0U)
          {
            mag_ref_avg_start_ms = now_ms;
          }

          mag_ref_avg_sin_sum += sinf(heading_now_rad);
          mag_ref_avg_cos_sum += cosf(heading_now_rad);
          mag_ref_avg_field_sum_g += mag_field_now_g;
          mag_ref_avg_sample_count++;

          if ((now_ms - mag_ref_avg_start_ms) >= APP_MAG_REF_AVG_MS)
          {
            mag_heading_at_ref_deg = atan2f(mag_ref_avg_sin_sum, mag_ref_avg_cos_sum) * (1.0f / RAD_PER_DEG);
            mag_ref_field_g = mag_ref_avg_field_sum_g / ((float)mag_ref_avg_sample_count);
            mag_yaw_ref_captured = 1U;
            mag_yaw_nudge_dps = 0.0f;
          }
        }
        else if ((mag_ref_field_g > 0.01f) &&
                 (fabsf(mag_field_now_g - mag_ref_field_g) / mag_ref_field_g > APP_MAG_TRUST_MAX_MAG_DEVIATION_FRAC))
        {
          mag_yaw_nudge_dps = 0.0f; /* field strength shifted too much - likely motor/ESC current interference */
        }
        else
        {
          /* Negated 2026-08-21: bench-verified with real MAG+IMU telemetry that
           * Mag_GetHeadingDeg()'s delta and yaw_deg's delta moved in OPPOSITE
           * directions for the same physical rotation (magDelta +360deg vs
           * yawDelta -257deg over the same turn - rms tracking error 90deg,
           * peak 179.9deg, essentially maximally divergent, not just noisy).
           * This nudge's sign was very likely tuned against the OLD, since-fixed
           * mounting-rotation bug (see mag.c) that made the compass run
           * backwards - now that the compass correctly increases with CW
           * rotation, the nudge's assumed sign is mismatched again. Flipping
           * mag_delta_deg here (not yaw_deg, which NAVBRAKE/telemetry/SD log all
           * also depend on, and not Mag_GetHeadingDeg(), which is bench-verified
           * correct against true rotation direction and magnetic north) is the
           * minimal, localized fix. Re-verify with a fresh bench MAG+IMU capture
           * before trusting this in flight. */
          float mag_delta_deg = -Attitude_WrapAngle180(Mag_GetHeadingDeg(mag_tilt_roll_deg, mag_tilt_pitch_deg, g_avg_motor_power_delta_us) - mag_heading_at_ref_deg);
          /* Cross-product-style error: sin() of the angle difference, not the
           * difference itself - matches a plain error for small angles (sin(x) ~= x
           * near 0) but tapers smoothly to zero at the antipodal point instead of a
           * discontinuous wrap. RAD_PER_DEG in the denominator keeps
           * APP_MAG_YAW_NUDGE_KP_DPS_PER_DEG's small-angle meaning ("dps per degree
           * of error") even though the input to Kp is now a sine, not a degree. */
          float yaw_error_sin = sinf((mag_delta_deg - yaw_deg) * RAD_PER_DEG);
          float mag_yaw_nudge_target_dps = App_ClampFloat(
              yaw_error_sin * (APP_MAG_YAW_NUDGE_KP_DPS_PER_DEG / RAD_PER_DEG),
              -APP_MAG_YAW_NUDGE_MAX_DPS, APP_MAG_YAW_NUDGE_MAX_DPS);
          float mag_yaw_nudge_max_step_dps;

          mag_yaw_nudge_max_step_dps = APP_MAG_YAW_NUDGE_SLEW_DPS_PER_S *
                                              (((float)APP_CONTROL_LOOP_MS) * 0.001f);

          mag_yaw_nudge_dps += App_ClampFloat(mag_yaw_nudge_target_dps - mag_yaw_nudge_dps,
                                               -mag_yaw_nudge_max_step_dps, mag_yaw_nudge_max_step_dps);
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
        Telemetry_PrintVertEkfState(VertEkf_IsHealthy(), VertEkf_GetHeightM(), VertEkf_GetClimbRateMps(),
                                    VertEkf_GetAccelBiasMps2(), VertEkf_GetLidarImpliedHeightM(),
                                    VertEkf_GetSonarImpliedHeightM());
        Telemetry_PrintAltholdState(althold_holding, althold_authority_active, althold_target_alt_m,
                                    althold_fused_alt_m_now, climb_rate_setpoint_mps, climb_rate_error_mps,
                                    althold_trim_us, baro_damp_term_us, althold_hover_throttle_us);
        Telemetry_PrintGpsState(GPS_IsConfigured(), GPS_IsHealthy(), GPS_GetFixType(), GPS_GetNumSatellites(),
                               GPS_GetLatitudeDeg(), GPS_GetLongitudeDeg(), GPS_GetAltitudeM());
        /* Added 2026-09-04 alongside the CFG-NAV5 retry fix - this ack was
         * previously invisible in telemetry, so a lost round-trip (leaving
         * the receiver stuck in factory static-hold, freezing lat/lon) had no
         * symptom short of forensically comparing GPS output against known
         * real motion after the fact. */
        printf("GPSNAV5[acked=%u]\r\n", (unsigned int)GPS_GetLastNav5Acked());
        Telemetry_PrintMagState(Mag_IsHealthy(), Mag_GetXGauss(), Mag_GetYGauss(), Mag_GetZGauss(),
                               Mag_GetHeadingDeg(mag_tilt_roll_deg, mag_tilt_pitch_deg, g_avg_motor_power_delta_us));
        Telemetry_PrintMagTiltState(mag_tilt_roll_deg, mag_tilt_pitch_deg);
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
    navpos_requested = (flight_mode == APP_FLIGHT_MODE_NAV_POSHOLD) ? 1U : 0U;

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

      if ((receiver_state.link_active == 0U) || (arm_switch_high == 0U) || (low_battery_disarm_active != 0U))
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
        const char *disarm_reason = (receiver_state.link_active == 0U) ? "RX_LINK_LOST" :
                                    (arm_switch_high == 0U) ? "ARM_SWITCH_LOW" : "LOW_BATTERY";

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
        roll_nonfinite_reported = 0U;
        pitch_nonfinite_reported = 0U;
        yaw_nonfinite_reported = 0U;
        navpos_active = 0U;
        navpos_was_engaged = 0U;
        navpos_needs_latch = 1U;
        navpos_target_north_m = 0.0f;
        navpos_target_east_m = 0.0f;
        navpos_integral_north_mps = 0.0f;
        navpos_integral_east_mps = 0.0f;
        /* INCIDENT (2026-08-30): ground_effect_clear used to be able to revert to 0
         * mid-flight on a single noisy below-threshold height sample even after
         * genuinely clearing - see the latch fix at its own declaration/use sites
         * for the full story. Resetting it here (fresh per arm cycle) is what makes
         * that one-way latch semantics actually work. */
        ground_effect_clear = 0U;
        ground_effect_below_since_ms = now_ms;
        ground_effect_rangefinder_below_since_ms = now_ms;
        /* INCIDENT (2026-08-30): althold_settled_center_us never got reset between
         * arm cycles (only a genuinely fresh 1200ms settle could move it, and idle
         * throttle right after arming is deliberately EXCLUDED from ever settling -
         * see APP_ALTHOLD_RELATCH_EXCLUDE_MARGIN_US's comment). A real repeated-
         * re-arm test showed this leaving a stale settled-center from an earlier
         * arm cycle (or mode) in place: on the next arm, idle throttle read as a
         * huge NEGATIVE offset from that stale reference, and the instant full
         * authority engaged it commanded a hard -2.0 m/s descend instead of a
         * climb - the aircraft would not lift at all. Resetting all three settle
         * variables to the current stick position here means every fresh arm
         * cycle starts from a correct, current baseline; the normal settle-latch
         * logic still re-derives a proper center once the pilot actually holds a
         * real hover position, exactly as before. */
        althold_settle_ref_us = (uint16_t)throttle_us;
        althold_settle_start_ms = now_ms;
        althold_settled_center_us = (uint16_t)throttle_us;

        if (was_armed != 0U)
        {
          /* Force a fresh GPS reference latch before the next arm attempt -
           * a reference latched once per boot session never re-latches
           * otherwise (see Nav_ResetReference()), which could leave
           * NAV_VELOCITY_BRAKE's arm gate stuck blocked until power-cycled.
           * Gated on the armed->disarmed edge (not every disarmed-loop
           * iteration) so it doesn't thrash the reference while idle. */
          Nav_ResetReference();
        }
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
          /* NAVBRAKE has no meaning without a valid nav solution - it would just sit
           * disqualified the moment it engaged. Every other mode is pure body-frame
           * control and doesn't touch nav_state at all, so they stay ungated - this
           * must not block arming for RATE/ATTITUDE/ALTHOLD indoors or anywhere else
           * GPS isn't locked. */
          uint8_t nav_arm_ok = (uint8_t)((flight_mode != APP_FLIGHT_MODE_NAV_POSHOLD) ||
                                         (nav_state.valid != 0U));

          if (nav_arm_ok == 0U)
          {
            arm_hold_start_ms = 0U;
            if ((now_ms - last_arm_nav_block_print_ms) >= 1000U)
            {
              printf("ARM_BLOCKED[reason=NAV_INVALID mode=NAVBRAKE]\r\n");
              last_arm_nav_block_print_ms = now_ms;
            }
          }
          else if (arm_hold_start_ms == 0U)
          {
            arm_hold_start_ms = now_ms;
          }
          else if ((now_ms - arm_hold_start_ms) >= APP_ARM_HOLD_MS)
          {
            motors_armed = 1U;
            trim_captured = 0U;
            /* Fresh arm always gets a fresh chance to calibrate yaw_center_us
             * once (see the settle-latch comment above) - never carry a lock
             * or a settled reference across arms. */
            yaw_center_locked = 0U;
            yaw_settle_ref_us = APP_PWM_MID_US;
            yaw_settle_start_ms = now_ms;
            g_glog_count = 0U;
            g_glog_capturing = 1U;
            App_SdLogArmStart();
            /* Reset ALTHOLD's hover-throttle estimate fresh on every arm - it's
             * `static` and previously (in an earlier design) carried over
             * unchanged from whatever the last flight (possibly a different
             * battery/loadout, hours earlier) left it at. A stale reference here
             * was the root cause of a real uncontrolled-climb incident
             * (2026-08-21, see kh7-althold-throttle-incident memory) - the seed
             * value itself doesn't need to be a good guess, since the below-gate
             * open-loop liftoff assist (see the ALTHOLD throttle block and
             * APP_ALTHOLD_LIFTOFF_ASSIST_MAX_US's comment) gives the pilot full
             * manual authority to lift off from it directly, and the real
             * closed loop refines it quickly once the gate opens. */
            althold_hover_throttle_us = APP_ALTHOLD_HOVER_EST_SEED_US;
            /* Same discipline for the settled-center reference (see its
             * declaration comment) and vert_ekf.c's state (ALTHOLD's altitude/
             * climb-rate source as of 2026-08-29) - neither should carry state
             * across arms/flights either. */
            althold_settle_ref_us = (uint16_t)throttle_us;
            althold_settle_start_ms = now_ms;
            althold_settled_center_us = APP_PWM_MID_US;
            althold_authority_was_active = 0U;
            VertEkf_Reset();
            althold_holding = 0U;
            althold_integral_us = 0.0f;
            althold_pos_integral_mps = 0.0f;
            althold_trim_filtered_us = 0.0f;
            baro_damp_term_filtered_us = 0.0f;
            /* Fresh arm always needs the full ground-effect dwell re-proven, never
             * carries a "clear" verdict over from a previous flight/landing. */
            ground_effect_clear = 0U;
            ground_effect_below_since_ms = now_ms;
            ground_effect_rangefinder_below_since_ms = now_ms;
            /* No explicit reset needed for althold_target_alt_m itself - it's
             * only ever read/used inside the `althold_holding == 0U` capture
             * branch below, and althold_holding is reset to 0U just above, so
             * a fresh arm always re-captures before a stale value could ever
             * be used. */
            /* Same discipline for yaw-hold's target - always start a fresh arm
             * with no held heading, re-captured the moment the yaw stick first
             * settles centered. */
            yawhold_active = 0U;
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
          /* Immediate fallback baseline, same as roll/pitch - if the pilot
           * never holds the stick still long enough for the settle-latch
           * below to lock in a (possibly more accurate) value this arm, this
           * is what yaw_center_us keeps for the whole flight, matching the
           * original one-shot behavior exactly as a worst case. */
          yaw_center_us = yaw_us;
          trim_captured = 1U;
        }
        /* Yaw center: settles ONCE per arm, then locks - see the long comment
         * on yaw_center_locked's declaration above for the full history (why
         * this isn't a one-shot snapshot like roll/pitch, and why it must NOT
         * keep re-settling for the whole flight the way ALTHOLD's throttle-
         * center does). Reuses ALTHOLD's exact settle-window/settle-time
         * constants (not ALTHOLD-specific in meaning - both are just "how
         * tightly and how long must a stick sit still to call it settled"). */
        if (yaw_center_locked == 0U)
        {
          int32_t yaw_settle_offset_us = (int32_t)yaw_us - (int32_t)yaw_settle_ref_us;

          if ((yaw_settle_offset_us > (int32_t)APP_ALTHOLD_STICK_STABLE_WINDOW_US) ||
              (yaw_settle_offset_us < -(int32_t)APP_ALTHOLD_STICK_STABLE_WINDOW_US))
          {
            yaw_settle_ref_us = yaw_us;
            yaw_settle_start_ms = now_ms;
          }
          else if ((now_ms - yaw_settle_start_ms) >= APP_ALTHOLD_STICK_SETTLE_MS)
          {
            yaw_center_us = yaw_settle_ref_us;
            yaw_center_locked = 1U;
          }
        }

        pilot_roll_stick_us = (int16_t)((int32_t)roll_us - (int32_t)roll_center_us);
        pilot_pitch_stick_us = (int16_t)((int32_t)pitch_us - (int32_t)pitch_center_us);
        navpos_requested = (flight_mode == APP_FLIGHT_MODE_NAV_POSHOLD) ? 1U : 0U;

        /* NAV_POSHOLD (2026-09-04 rewrite): one unified GPS mode - see the
         * APP_NAVPOS_* block for the full design. No separate "braking" vs
         * "position hold" split, no independent enable switch: sticks centered
         * commands zero drift (holding once settled), sticks off-center maps
         * directly to a commanded velocity, and returning to center re-latches
         * (or resumes) a hold target. */
        if (navpos_requested != 0U)
        {
          /* Engagement requirement: valid+referenced GPS nav accurate enough to
           * trust, a usable attitude solution, link up. Deliberately no latch-
           * out on loss - falls straight through to plain stick-angle control
           * below (identical to ATTITUDE mode) until the gate passes again, so
           * the pilot always retains manual attitude control as a fallback. */
          uint8_t navpos_engage_ok = (uint8_t)((nav_state.valid != 0U) &&
                                               (nav_state.reference_valid != 0U) &&
                                               (nav_state.h_acc_m <= APP_NAVPOS_MAX_HORIZONTAL_ACC_M) &&
                                               (attitude_zero_captured != 0U) &&
                                               (receiver_state.link_active != 0U));
          uint8_t navpos_sticks_centered = (uint8_t)((fabsf((float)pilot_roll_stick_us) <= (float)APP_NAVPOS_STICK_DEADBAND_US) &&
                                                      (fabsf((float)pilot_pitch_stick_us) <= (float)APP_NAVPOS_STICK_DEADBAND_US));

          if (navpos_was_engaged != 0U && navpos_engage_ok == 0U)
          {
            printf("NAV_LOST[reason=%s]\r\n", Nav_InvalidReasonName(nav_state.invalid_reason));
          }

          /* Per the pilot's explicit spec: off-center stick is manual flying,
           * full stop - it gets the SAME direct, full-authority stick-to-angle
           * mapping as ATTITUDE/ALTHOLD (active_attitude_gains.max_angle_deg),
           * not the capped ~5.8deg GPS velocity/accel chain the old design
           * routed manual flying through. Real flight data showed that capped
           * chain made the aircraft feel like it had "no control authority"
           * during ordinary maneuvering - the ~1.0 m/s^2 accel ceiling was
           * appropriate for a gentle station-keeping correction, never for
           * deliberate pilot-commanded flight. GPS-referenced position hold
           * (the capped, gentle chain) is now used ONLY while sticks are
           * centered and the GPS gate is satisfied - exactly the "sticks
           * centered -> capture GPS position and hold it" behavior asked for,
           * nothing more. */
          if ((navpos_engage_ok != 0U) && (navpos_sticks_centered != 0U))
          {
            if (navpos_needs_latch != 0U)
            {
              /* Genuinely fresh entry into this mode (just selected, or just
               * armed into it) - explicitly latch the local reference now so
               * this always works even if the disarmed auto-latch never had
               * the chance to run. Deliberately NOT re-latched on every gate
               * recovery from a brief GPS blip - see navpos_needs_latch's
               * declaration comment. */
              Nav_LatchReference();
              navpos_needs_latch = 0U;
            }

            if (navpos_active == 0U)
            {
              /* Velocity-gated relatch (mirrors APP_ALTHOLD_RELATCH_MAX_CLIMB_MPS's
               * incident-driven lesson): "stick centered" alone does not prove the
               * aircraft has actually stopped moving - require GPS-measured
               * horizontal speed to already be near zero before latching a target,
               * or a hold engaged mid-drift would just freeze the drift in place. */
              float vel_mag_mps = sqrtf((nav_state.filtered_north_vel_mps * nav_state.filtered_north_vel_mps) +
                                        (nav_state.filtered_east_vel_mps * nav_state.filtered_east_vel_mps));

              if (vel_mag_mps <= APP_NAVPOS_RELATCH_MAX_VEL_MPS)
              {
                /* Always capture the CURRENT position as the fresh hold
                 * target - per the pilot's explicit spec, releasing the
                 * sticks recaptures wherever the aircraft is NOW. */
                navpos_target_north_m = nav_state.filtered_north_m;
                navpos_target_east_m = nav_state.filtered_east_m;
                navpos_integral_north_mps = 0.0f;
                navpos_integral_east_mps = 0.0f;
                navpos_active = 1U;
              }
            }

            if (navpos_active != 0U)
            {
              /* Centered and settled: position error (P+I) -> desired NE
               * velocity -> velocity-error->accel->angle, capped at the
               * gentle APP_NAVPOS_MAX_TILT_DEG ceiling - this is a station-
               * keeping correction, not maneuvering flight, so a soft cap is
               * correct here even though manual flying (above) is not
               * capped this way. */
              float err_n = navpos_target_north_m - nav_state.filtered_north_m;
              float err_e = navpos_target_east_m - nav_state.filtered_east_m;
              float navpos_dt_s = dt_s;
              float desired_north_vel_mps;
              float desired_east_vel_mps;

              if ((navpos_dt_s < 0.0005f) || (navpos_dt_s > 0.050f))
              {
                navpos_dt_s = ((float)APP_CONTROL_LOOP_MS) * 0.001f;
              }

              navpos_integral_north_mps = App_ClampFloat(navpos_integral_north_mps + (err_n * g_navpos_ki_per_s2 * navpos_dt_s),
                                                         -APP_NAVPOS_MAX_INTEGRAL_MPS, APP_NAVPOS_MAX_INTEGRAL_MPS);
              navpos_integral_east_mps = App_ClampFloat(navpos_integral_east_mps + (err_e * g_navpos_ki_per_s2 * navpos_dt_s),
                                                        -APP_NAVPOS_MAX_INTEGRAL_MPS, APP_NAVPOS_MAX_INTEGRAL_MPS);
              /* App_ClampFloat() alone cannot recover an already-non-finite value -
               * see the identical isfinite() guard on the rate-PID integrals below. */
              if (isfinite(navpos_integral_north_mps) == 0)
              {
                navpos_integral_north_mps = 0.0f;
              }
              if (isfinite(navpos_integral_east_mps) == 0)
              {
                navpos_integral_east_mps = 0.0f;
              }

              desired_north_vel_mps = App_ClampFloat((g_navpos_kp_per_s * err_n) + navpos_integral_north_mps,
                                                     -APP_NAVPOS_MAX_HOLD_VEL_MPS, APP_NAVPOS_MAX_HOLD_VEL_MPS);
              desired_east_vel_mps = App_ClampFloat((g_navpos_kp_per_s * err_e) + navpos_integral_east_mps,
                                                    -APP_NAVPOS_MAX_HOLD_VEL_MPS, APP_NAVPOS_MAX_HOLD_VEL_MPS);
              navpos_err_north_m = err_n;
              navpos_err_east_m = err_e;

              navpos_north_vel_error_mps = desired_north_vel_mps - nav_state.filtered_north_vel_mps;
              navpos_east_vel_error_mps = desired_east_vel_mps - nav_state.filtered_east_vel_mps;

              navpos_north_accel_cmd_mps2 = App_ClampFloat(APP_NAVPOS_VELOCITY_KP * navpos_north_vel_error_mps,
                                                            -APP_NAVPOS_MAX_ACCEL_MPS2, APP_NAVPOS_MAX_ACCEL_MPS2);
              navpos_east_accel_cmd_mps2 = App_ClampFloat(APP_NAVPOS_VELOCITY_KP * navpos_east_vel_error_mps,
                                                           -APP_NAVPOS_MAX_ACCEL_MPS2, APP_NAVPOS_MAX_ACCEL_MPS2);

              Nav_RotateNedToBody(navpos_north_accel_cmd_mps2, navpos_east_accel_cmd_mps2, yaw_deg,
                                 &navpos_fwd_accel_cmd_mps2, &navpos_right_accel_cmd_mps2);

              target_pitch_deg = ((float)APP_PITCH_SIGN) *
                                 Nav_AccelToAngleDeg(navpos_fwd_accel_cmd_mps2, active_attitude_gains.max_angle_deg);
              target_roll_deg = ((float)APP_ROLL_SIGN) *
                                Nav_AccelToAngleDeg(navpos_right_accel_cmd_mps2, active_attitude_gains.max_angle_deg);

              navpos_desired_north_vel_mps = desired_north_vel_mps;
              navpos_desired_east_vel_mps = desired_east_vel_mps;
              navpos_tilt_limited = (uint8_t)((fabsf(target_pitch_deg) >= (active_attitude_gains.max_angle_deg - 0.01f)) ||
                                              (fabsf(target_roll_deg) >= (active_attitude_gains.max_angle_deg - 0.01f)));
              navpos_accel_limited = (uint8_t)((fabsf(navpos_north_accel_cmd_mps2) >= (APP_NAVPOS_MAX_ACCEL_MPS2 - 0.001f)) ||
                                               (fabsf(navpos_east_accel_cmd_mps2) >= (APP_NAVPOS_MAX_ACCEL_MPS2 - 0.001f)));
            }
            else
            {
              /* Centered but not yet settled enough to latch a hold target -
               * ACTIVELY brake toward zero velocity through the same
               * velocity-error->accel->angle chain, rather than just
               * commanding level and waiting for drag to bleed off the
               * residual speed. Real flight data showed the level-only
               * version leaving the aircraft coasting at 0.4-0.9 m/s with a
               * literal 0deg commanded tilt for 2+ seconds straight - a
               * quadcopter has nowhere near enough drag to ever decay below
               * APP_NAVPOS_RELATCH_MAX_VEL_MPS on its own, so it never
               * reached the settle gate and just drifted indefinitely with
               * the stick sitting dead center the whole time. */
              float desired_north_vel_mps = 0.0f;
              float desired_east_vel_mps = 0.0f;

              navpos_north_vel_error_mps = desired_north_vel_mps - nav_state.filtered_north_vel_mps;
              navpos_east_vel_error_mps = desired_east_vel_mps - nav_state.filtered_east_vel_mps;

              navpos_north_accel_cmd_mps2 = App_ClampFloat(APP_NAVPOS_VELOCITY_KP * navpos_north_vel_error_mps,
                                                            -APP_NAVPOS_MAX_ACCEL_MPS2, APP_NAVPOS_MAX_ACCEL_MPS2);
              navpos_east_accel_cmd_mps2 = App_ClampFloat(APP_NAVPOS_VELOCITY_KP * navpos_east_vel_error_mps,
                                                           -APP_NAVPOS_MAX_ACCEL_MPS2, APP_NAVPOS_MAX_ACCEL_MPS2);

              Nav_RotateNedToBody(navpos_north_accel_cmd_mps2, navpos_east_accel_cmd_mps2, yaw_deg,
                                 &navpos_fwd_accel_cmd_mps2, &navpos_right_accel_cmd_mps2);

              target_pitch_deg = ((float)APP_PITCH_SIGN) *
                                 Nav_AccelToAngleDeg(navpos_fwd_accel_cmd_mps2, active_attitude_gains.max_angle_deg);
              target_roll_deg = ((float)APP_ROLL_SIGN) *
                                Nav_AccelToAngleDeg(navpos_right_accel_cmd_mps2, active_attitude_gains.max_angle_deg);

              navpos_desired_north_vel_mps = desired_north_vel_mps;
              navpos_desired_east_vel_mps = desired_east_vel_mps;
              navpos_tilt_limited = (uint8_t)((fabsf(target_pitch_deg) >= (active_attitude_gains.max_angle_deg - 0.01f)) ||
                                              (fabsf(target_roll_deg) >= (active_attitude_gains.max_angle_deg - 0.01f)));
              navpos_accel_limited = (uint8_t)((fabsf(navpos_north_accel_cmd_mps2) >= (APP_NAVPOS_MAX_ACCEL_MPS2 - 0.001f)) ||
                                               (fabsf(navpos_east_accel_cmd_mps2) >= (APP_NAVPOS_MAX_ACCEL_MPS2 - 0.001f)));
              navpos_err_north_m = 0.0f;
              navpos_err_east_m = 0.0f;
            }
          }
          else
          {
            /* Manual flying (sticks off-center) or GPS gate not satisfied -
             * plain, full-authority stick-to-angle, identical to ATTITUDE/
             * ALTHOLD. No position-hold state applies here. */
            navpos_active = 0U;
            navpos_integral_north_mps = 0.0f;
            navpos_integral_east_mps = 0.0f;
            navpos_desired_north_vel_mps = 0.0f;
            navpos_desired_east_vel_mps = 0.0f;
            navpos_north_vel_error_mps = 0.0f;
            navpos_east_vel_error_mps = 0.0f;
            navpos_north_accel_cmd_mps2 = 0.0f;
            navpos_east_accel_cmd_mps2 = 0.0f;
            navpos_fwd_accel_cmd_mps2 = 0.0f;
            navpos_right_accel_cmd_mps2 = 0.0f;
            navpos_tilt_limited = 0U;
            navpos_accel_limited = 0U;
            navpos_err_north_m = 0.0f;
            navpos_err_east_m = 0.0f;

            target_roll_deg = ((float)APP_ROLL_SIGN) * App_StickOffsetUsToAngleDeg((int32_t)roll_us - (int32_t)roll_center_us,
                                                                                    active_attitude_gains.max_angle_deg);
            target_pitch_deg = ((float)APP_PITCH_SIGN) * App_StickOffsetUsToAngleDeg((int32_t)pitch_us - (int32_t)pitch_center_us,
                                                                                      active_attitude_gains.max_angle_deg);
          }

          navpos_was_engaged = navpos_engage_ok;

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
          /* Leaving NAV_POSHOLD always clears its engaged/active state -
           * re-selecting the mode later always gets a fresh engagement chance. */
          navpos_was_engaged = 0U;
          navpos_active = 0U;
          navpos_needs_latch = 1U;

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
          navpos_was_engaged = 0U;
          navpos_active = 0U;
          navpos_needs_latch = 1U;

          cmd_roll_rate_dps = ((float)APP_ROLL_SIGN) * App_StickOffsetUsToRateDps((int32_t)roll_us - (int32_t)roll_center_us,
                             APP_RATE_CMD_MAX_ROLL_DPS);
          cmd_pitch_rate_dps = ((float)APP_PITCH_SIGN) * App_StickOffsetUsToRateDps((int32_t)pitch_us - (int32_t)pitch_center_us,
                               APP_RATE_CMD_MAX_PITCH_DPS);
        }
        {
          int32_t yaw_offset_us = (int32_t)yaw_us - (int32_t)yaw_center_us;
          uint8_t yawhold_eligible_mode = (uint8_t)((flight_mode == APP_FLIGHT_MODE_ATTITUDE) ||
                                                     (flight_mode == APP_FLIGHT_MODE_ALTHOLD) ||
                                                     (flight_mode == APP_FLIGHT_MODE_NAV_POSHOLD));

          if ((yawhold_eligible_mode != 0U) &&
              (yaw_offset_us > -(int32_t)APP_YAWHOLD_DEADBAND_US) &&
              (yaw_offset_us < (int32_t)APP_YAWHOLD_DEADBAND_US))
          {
            if (yawhold_active == 0U)
            {
              yawhold_target_deg = yaw_deg;
              yawhold_active = 1U;
            }
            yaw_angle_error_deg = Attitude_WrapAngle180(yawhold_target_deg - yaw_deg);
            /* SIGN BUG FOUND (2026-08-22, after the gain-lowering re-test still showed a
             * monotonic runaway): Attitude_GetBoardAnglesDeg() computes `yaw = -atan2f(...)`
             * (see the gz_dps integration comment above, ~line 3264) - increasing gz_dps
             * DECREASES yaw_deg, and measured_yaw_rate_dps/gyro_yaw_dps (what the rate PID
             * error and this cmd_yaw_rate_dps are both expressed in terms of) follow that
             * same inverted gz_dps convention. A positive yaw_angle_error_deg (target above
             * current, yaw_deg needs to INCREASE) therefore needs a NEGATIVE gz-convention
             * command, not a positive one - the un-negated version above always pushed the
             * opposite way. Confirmed directly in a flight log: gyro_yaw_dps pinned negative
             * (-8 to -15dps, tracking its own commanded ceiling correctly) while yaw_deg
             * climbed +0deg->+31deg over 3.5s in lockstep - the "correction" and the observed
             * drift moved the same direction, growing without bound, every time yaw-hold
             * engaged. This (not gain magnitude) is the root cause of the flip incident, the
             * "worse than before" wandering report, and this runaway - a lower gain only slows
             * the same guaranteed-wrong-direction push. Fixed by negating the error term;
             * gains restored to their original (pre-panic-lowering) values now that the actual
             * bug is fixed - re-bench-test for correct-direction, settling (not diverging)
             * tracking before the next real flight. */
            cmd_yaw_rate_dps = App_ClampFloat(-yaw_angle_error_deg * APP_YAWHOLD_KP_DPS_PER_DEG,
                                               -APP_YAWHOLD_MAX_RATE_DPS,
                                               APP_YAWHOLD_MAX_RATE_DPS);
          }
          else
          {
            yawhold_active = 0U;
            cmd_yaw_rate_dps = ((float)APP_YAW_SIGN) * App_StickOffsetUsToRateDps(yaw_offset_us,
                                   APP_RATE_CMD_MAX_YAW_DPS);
          }
        }

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

        /* App_ClampFloat() alone cannot recover an already-non-finite value -
         * NaN fails both its comparisons and passes straight through unchanged
         * - so a NaN/Inf that reaches the integral needs an explicit isfinite()
         * reset here, not just a magnitude bound, or it stays poisoned forever
         * regardless of the clamp below. */
        if (isfinite(roll_integral_dps_s) == 0)
        {
          roll_integral_dps_s = 0.0f;
        }
        integral_limit_roll = (active_pid_gains.roll.ki > 0.000001f) ?
                              (((float)APP_RATE_TERM_LIMIT_US) / active_pid_gains.roll.ki) :
                              APP_RATE_INTEGRAL_SAFETY_LIMIT_DPS_S;
        roll_integral_dps_s = App_ClampFloat(roll_integral_dps_s, -integral_limit_roll, integral_limit_roll);

        if (isfinite(pitch_integral_dps_s) == 0)
        {
          pitch_integral_dps_s = 0.0f;
        }
        integral_limit_pitch = (active_pid_gains.pitch.ki > 0.000001f) ?
                               (((float)APP_RATE_TERM_LIMIT_US) / active_pid_gains.pitch.ki) :
                               APP_RATE_INTEGRAL_SAFETY_LIMIT_DPS_S;
        pitch_integral_dps_s = App_ClampFloat(pitch_integral_dps_s, -integral_limit_pitch, integral_limit_pitch);

        if (isfinite(yaw_integral_dps_s) == 0)
        {
          yaw_integral_dps_s = 0.0f;
        }
        integral_limit_yaw = (active_pid_gains.yaw.ki > 0.000001f) ?
                             (((float)APP_RATE_TERM_LIMIT_US) / active_pid_gains.yaw.ki) :
                             APP_RATE_INTEGRAL_SAFETY_LIMIT_DPS_S;
        yaw_integral_dps_s = App_ClampFloat(yaw_integral_dps_s, -integral_limit_yaw, integral_limit_yaw);

        /* D term uses its own low-pass-filtered rate signal so raw gyro noise/spikes don't kick it. */
        dterm_lpf_alpha = App_LpfAlpha(dt_s, APP_DTERM_LPF_HZ);
        dterm_filt_roll_rate_dps += dterm_lpf_alpha * (measured_roll_rate_dps - dterm_filt_roll_rate_dps);
        dterm_filt_pitch_rate_dps += dterm_lpf_alpha * (measured_pitch_rate_dps - dterm_filt_pitch_rate_dps);
        dterm_filt_yaw_rate_dps += dterm_lpf_alpha * (measured_yaw_rate_dps - dterm_filt_yaw_rate_dps);

        /* Same permanent-poisoning risk as the integrals above - these are also
         * running EMA accumulators (x += alpha*(new-x)), so one non-finite
         * input keeps them non-finite forever after with no other recovery path. */
        if (isfinite(dterm_filt_roll_rate_dps) == 0)
        {
          dterm_filt_roll_rate_dps = measured_roll_rate_dps;
        }
        if (isfinite(dterm_filt_pitch_rate_dps) == 0)
        {
          dterm_filt_pitch_rate_dps = measured_pitch_rate_dps;
        }
        if (isfinite(dterm_filt_yaw_rate_dps) == 0)
        {
          dterm_filt_yaw_rate_dps = measured_yaw_rate_dps;
        }

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

        if (isfinite(ff_filt_cmd_roll_rate_dps) == 0)
        {
          ff_filt_cmd_roll_rate_dps = cmd_roll_rate_dps;
        }
        if (isfinite(ff_filt_cmd_pitch_rate_dps) == 0)
        {
          ff_filt_cmd_pitch_rate_dps = cmd_pitch_rate_dps;
        }
        if (isfinite(ff_filt_cmd_yaw_rate_dps) == 0)
        {
          ff_filt_cmd_yaw_rate_dps = cmd_yaw_rate_dps;
        }

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

        /* Diagnostic only - identifies which specific upstream value first went
         * non-finite, once per arm session, without changing control behavior
         * (the isfinite() resets above already keep the integral/filter states
         * themselves safe regardless of whether this fires). Checked in
         * causal order: the two raw rate inputs first (pins it to either the
         * attitude/command side or the gyro/measurement side), then each of
         * the four term contributors. */
        if ((pitch_nonfinite_reported == 0U) && (isfinite(pitch_term_f) == 0))
        {
          pitch_nonfinite_reported = 1U;
          printf("PID_NONFINITE[axis=pitch cmd=%.2f meas=%.2f err=%.2f int=%.2f deriv=%.2f ff=%.2f term_f=%.2f t=%lu]\r\n",
                 (double)cmd_pitch_rate_dps, (double)measured_pitch_rate_dps, (double)pitch_rate_error_dps,
                 (double)pitch_integral_dps_s, (double)pitch_rate_derivative_dps_per_s,
                 (double)pitch_rate_ff_dps_per_s, (double)pitch_term_f, (unsigned long)now_ms);
        }
        if ((roll_nonfinite_reported == 0U) && (isfinite(roll_term_f) == 0))
        {
          roll_nonfinite_reported = 1U;
          printf("PID_NONFINITE[axis=roll cmd=%.2f meas=%.2f err=%.2f int=%.2f deriv=%.2f ff=%.2f term_f=%.2f t=%lu]\r\n",
                 (double)cmd_roll_rate_dps, (double)measured_roll_rate_dps, (double)roll_rate_error_dps,
                 (double)roll_integral_dps_s, (double)roll_rate_derivative_dps_per_s,
                 (double)roll_rate_ff_dps_per_s, (double)roll_term_f, (unsigned long)now_ms);
        }
        if ((yaw_nonfinite_reported == 0U) && (isfinite(yaw_term_f) == 0))
        {
          yaw_nonfinite_reported = 1U;
          printf("PID_NONFINITE[axis=yaw cmd=%.2f meas=%.2f err=%.2f int=%.2f deriv=%.2f ff=%.2f term_f=%.2f t=%lu]\r\n",
                 (double)cmd_yaw_rate_dps, (double)measured_yaw_rate_dps, (double)yaw_rate_error_dps,
                 (double)yaw_integral_dps_s, (double)yaw_rate_derivative_dps_per_s,
                 (double)yaw_rate_ff_dps_per_s, (double)yaw_term_f, (unsigned long)now_ms);
        }

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
        /* Runs UNCONDITIONALLY every iteration, regardless of flight mode or
         * the gate below - same "keep tracking warm, never gate the
         * reference itself" discipline as althold_hover_throttle_us. See
         * vert_ekf.c's top-of-file design comment for what this actually does -
         * VertEkf_Predict()/UpdateBaro()/UpdateRange() are called elsewhere in
         * this same loop and from App_SetRangefinderCm()/App_SetLunaCm(), this
         * is just reading the current fused estimate. */
        althold_fused_alt_m_now = VertEkf_GetHeightM();
        /* One-way latch per arm cycle (fixed 2026-08-30 after a real incident): a
         * plain "revert to 0 the instant height dips below threshold" (the previous
         * behavior) let a single noisy height sample right at the liftoff boundary
         * flicker ground_effect_clear true/false/true THREE TIMES within ~3 seconds
         * of real flight data. Each flip silently swapped the throttle control law
         * between open-loop liftoff-assist and closed-loop climb-rate authority -
         * felt by the pilot as the stick alternately doing nothing and then
         * commanding a fast, uncommanded-feeling climb, and required an emergency
         * mid-flight disarm. Once genuinely clear for this arm cycle, liftoff assist
         * is not needed again until disarm/re-arm - see the arm-reset block above
         * (ground_effect_clear = 0U there) for where the latch actually resets. */
        if (ground_effect_clear == 0U)
        {
          if (althold_fused_alt_m_now < APP_BARO_VZ_DAMP_MIN_ALT_M)
          {
            ground_effect_below_since_ms = now_ms;
          }
          else if ((now_ms - ground_effect_below_since_ms) >= APP_GROUND_EFFECT_CLEAR_DWELL_MS)
          {
            ground_effect_clear = 1U;
          }
        }
        /* External-height-based supplement (2026-08-23, extended 2026-08-25 to
         * cover TF-Luna too via App_GetBestHeightCm()) - can confirm clearing
         * ground effect independently of baro, which is known to swing wildly
         * (up to ~230cm) during exactly this window (see
         * kh7-baro-liftoff-transient memory), which in turn was blocking a
         * comfortable, consistent stick center for the full-authority ALTHOLD
         * redesign below. Only ever ADDS a way to reach ground_effect_clear
         * sooner/more reliably - never blocks the baro-only path above. A
         * missing/stale reading from BOTH sensors (see App_GetBestHeightCm()'s
         * comment) is treated exactly like "still below threshold": it resets
         * this path's own dwell clock rather than either granting clearance or
         * silently freezing a stale clock that could false-trigger the instant
         * a sensor recovers. Also latched one-way per arm cycle, same reasoning
         * as the baro path above. */
        height_cm_now = App_GetBestHeightCm(now_ms);
        if (ground_effect_clear == 0U)
        {
          if ((height_cm_now <= 0.0f) ||
              ((height_cm_now * 0.01f) < APP_BARO_VZ_DAMP_MIN_ALT_M))
          {
            ground_effect_rangefinder_below_since_ms = now_ms;
          }
          else if ((now_ms - ground_effect_rangefinder_below_since_ms) >= APP_GROUND_EFFECT_CLEAR_DWELL_MS)
          {
            ground_effect_clear = 1U;
          }
        }
        althold_trim_us = 0;
        althold_authority_active = 0U;
        althold_liftoff_assist_active = 0U;
        /* Only ever assigned inside the ALTHOLD/NAVBRAKE branch below - reset here
         * unconditionally (2026-08-29) so Telemetry_PrintAltholdState() always reads
         * a well-defined value in RATE/ATTITUDE mode too, not stack garbage. */
        climb_rate_setpoint_mps = 0.0f;
        climb_rate_error_mps = 0.0f;
        if ((flight_mode == APP_FLIGHT_MODE_ALTHOLD) || (flight_mode == APP_FLIGHT_MODE_NAV_POSHOLD))
        {
          /* Settle-latch: re-latches althold_settled_center_us to wherever the
           * stick has genuinely SETTLED, not a fixed value - see its
           * declaration comment for why. Runs unconditionally here (both
           * above and below the ground-effect gate), same discipline as
           * althold_hover_throttle_us. */
          throttle_offset_us = (int32_t)throttle_us - (int32_t)althold_settle_ref_us;
          if ((throttle_offset_us > (int32_t)APP_ALTHOLD_STICK_STABLE_WINDOW_US) ||
              (throttle_offset_us < -(int32_t)APP_ALTHOLD_STICK_STABLE_WINDOW_US))
          {
            althold_settle_ref_us = (uint16_t)throttle_us;
            althold_settle_start_ms = now_ms;
          }
          /* Also require the aircraft to actually BE near-level before
           * committing a relatch, not just the stick being still - see
           * APP_ALTHOLD_RELATCH_MAX_CLIMB_MPS's comment. A real incident: a
           * ~0.8s pause mid-climb (stick genuinely still, but the aircraft
           * still gaining altitude) relatched center right where the pilot
           * happened to pause, silently locking a hold at a lower altitude
           * than intended - reported as the aircraft "just gives up." Also
           * exclude both physical stick extremes entirely - see
           * APP_ALTHOLD_RELATCH_EXCLUDE_MARGIN_US's comment. A second real
           * incident: holding full-down stick to land (perfectly still, by
           * definition, once pinned at the mechanical limit) relatched
           * center to the stick's own position - the SAME full-down stick
           * instantly became "centered" (offset=0), silently converting a
           * commanded max-rate descent into an altitude LOCK. The aircraft
           * never landed - "commanded full down throttle, and the aircraft
           * stayed up and didn't land." */
          else if (((now_ms - althold_settle_start_ms) >= APP_ALTHOLD_THROTTLE_SETTLE_MS) &&
                   (fabsf(VertEkf_GetClimbRateMps()) < APP_ALTHOLD_RELATCH_MAX_CLIMB_MPS) &&
                   (althold_settle_ref_us > (APP_PWM_MIN_US + APP_ALTHOLD_RELATCH_EXCLUDE_MARGIN_US)) &&
                   (althold_settle_ref_us < (APP_PWM_MAX_US - APP_ALTHOLD_RELATCH_EXCLUDE_MARGIN_US)))
          {
            althold_settled_center_us = althold_settle_ref_us;
          }

          throttle_offset_us = (int32_t)throttle_us - (int32_t)althold_settled_center_us;

          if ((baro_healthy_now != 0U) && (ground_effect_clear != 0U))
          {
            /* Full computed-throttle authority - see the big design comment
             * above APP_ALTHOLD_THROTTLE_DEADBAND_US's definition for why this
             * is safe to re-attempt after the 2026-08-21 incident. */
            althold_authority_active = 1U;

            if ((althold_authority_was_active == 0U) &&
                ((throttle_offset_us > (int32_t)APP_ALTHOLD_THROTTLE_DEADBAND_US) ||
                 (throttle_offset_us < -(int32_t)APP_ALTHOLD_THROTTLE_DEADBAND_US)))
            {
              /* Just transitioned from open-loop liftoff assist to full
               * closed-loop authority - seed the hover estimate from what
               * liftoff assist was ACTUALLY outputting the instant before,
               * not its own frozen pre-gate value, so the handoff is
               * continuous instead of snapping. A real flight exposed this:
               * liftoff assist was outputting ~1243us (hover_est=1150 plus a
               * large stick-offset contribution) the sample before the gate
               * opened, then the closed-loop branch used hover_est ALONE
               * (~1150) the sample after - a ~93us drop with no change in
               * what the pilot was doing, regardless of trim.
               *
               * GATED ON OFF-CENTER STICK 2026-08-30 (was unconditional): a
               * real flight showed this firing with the stick merely ~50us
               * off center (well inside the deadband, about to enter the
               * "Centered" branch below anyway) - baking that small, noisy
               * offset into the PERSISTENT hover_throttle_us baseline rather
               * than the transient trim caused a real, felt uncommanded climb
               * to ~1.8m from a ~0.4m target, taking ~13s to recover (see
               * kh7_althold_oscillation_yawhold_todo memory). The original
               * incident this code fixed only ever needs this seed when the
               * stick was GENUINELY off-center during liftoff assist (i.e.
               * about to take the off-center branch below, actively
               * commanding a climb/descend) - if the stick is within the
               * deadband, the Centered branch's own fast trim correction
               * handles any small mismatch without needing a one-time
               * baseline nudge, and that nudge is pure risk (it can silently
               * absorb stick noise) with no offsetting benefit in that case. */
              althold_hover_throttle_us = App_ClampFloat(althold_hover_throttle_us +
                                                          ((float)throttle_offset_us *
                                                           ((float)APP_ALTHOLD_LIFTOFF_ASSIST_MAX_US /
                                                            (float)((int32_t)APP_PWM_MAX_US -
                                                                    (int32_t)APP_PWM_MID_US))),
                                                          (float)APP_ALTHOLD_HOVER_EST_MIN_US,
                                                          (float)APP_ALTHOLD_HOVER_EST_MAX_US);
            }

            if ((throttle_offset_us > -(int32_t)APP_ALTHOLD_THROTTLE_DEADBAND_US) &&
                (throttle_offset_us < (int32_t)APP_ALTHOLD_THROTTLE_DEADBAND_US))
            {
              /* Centered: lock the altitude captured the instant we start
               * holding, not on every centered iteration (would let slow
               * drift continuously redefine "correct"). ALSO require the
               * aircraft's own climb rate to already be near zero
               * (2026-08-30) - reusing APP_ALTHOLD_RELATCH_MAX_CLIMB_MPS,
               * the same threshold/idiom as the stick-center relatch gate
               * above (see its comment). Real flight data from tonight
               * showed every severe hold-oscillation episode had 1.0-1.8 m/s
               * of residual vertical velocity at the exact instant this code
               * (with no velocity check) captured a fresh target - the
               * vehicle then sailed straight past that just-latched target
               * under its own momentum, and the resulting large error is
               * what actually kicked off the multi-cycle oscillations
               * (independent of the separate Ki=0.05 regression tracked in
               * kh7_althold_oscillation_yawhold_todo memory - clean holds all
               * had <0.4-0.5 m/s residual velocity at capture). */
              if ((althold_holding == 0U) &&
                  (fabsf(VertEkf_GetClimbRateMps()) < APP_ALTHOLD_RELATCH_MAX_CLIMB_MPS))
              {
                althold_target_alt_m = althold_fused_alt_m_now;
                althold_integral_us = 0.0f;
                althold_pos_integral_mps = 0.0f;
                althold_holding = 1U;
              }

              if (althold_holding == 0U)
              {
                /* Stick centered but still coasting from real momentum -
                 * don't latch a target yet (would just reproduce the bug
                 * above). Command zero rate so the inner climb-rate loop
                 * arrests the residual velocity; the next centered sample
                 * where velocity has actually settled captures cleanly. */
                climb_rate_setpoint_mps = 0.0f;
              }
              else
              {
                /* Gentle proportional nudge (2026-08-23): stick offset WITHIN
                 * the deadband slowly walks the held target up/down instead of
                 * being completely ignored - a real flight showed a pilot push
                 * further while already holding (still inside the deadband)
                 * expecting some response and getting none, reported as the
                 * aircraft "just gives up." Deliberately a much slower rate
                 * than the full off-center climb/descend authority below
                 * (APP_ALTHOLD_HOLD_NUDGE_MAX_MPS vs APP_ALTHOLD_MAX_CLIMB_MPS)
                 * so there's still a clear, distinct feel between fine-tuning
                 * an existing hold and actively commanding a climb/descend. */
                if ((throttle_offset_us > (int32_t)APP_ALTHOLD_HOLD_NUDGE_DEADZONE_US) ||
                    (throttle_offset_us < -(int32_t)APP_ALTHOLD_HOLD_NUDGE_DEADZONE_US))
                {
                  althold_target_alt_m += ((float)throttle_offset_us /
                                           (float)APP_ALTHOLD_THROTTLE_DEADBAND_US) *
                                          APP_ALTHOLD_HOLD_NUDGE_MAX_MPS * dt_s;
                }
                {
                  float pos_error_m = althold_target_alt_m - althold_fused_alt_m_now;
                  althold_pos_integral_mps = App_ClampFloat(althold_pos_integral_mps +
                                                            (pos_error_m * App_GetAltholdPosKi() * dt_s),
                                                            -APP_ALTHOLD_POS_INTEGRAL_LIMIT_MPS,
                                                            APP_ALTHOLD_POS_INTEGRAL_LIMIT_MPS);
                  climb_rate_setpoint_mps = App_ClampFloat((App_GetAltholdAltHoldKp() * pos_error_m) +
                                                            althold_pos_integral_mps,
                                                            -App_GetAltholdMaxClimbMps(),
                                                            App_GetAltholdMaxClimbMps());
                }
              }
            }
            else
            {
              /* Off-center: pilot commands a climb/descend rate directly,
               * proportional to stick deflection from center (full stick =
               * APP_ALTHOLD_MAX_CLIMB_MPS - same cap the hold P-term above
               * uses, so a climb/descend is never faster than the hold loop
               * can also arrest it). Not holding a position target while
               * off-center - the moment the stick returns to center, a fresh
               * altitude is captured above, i.e. it locks wherever the
               * climb/descend left off, exactly as requested. */
              /* Scaled per-direction by actual remaining stick travel from
               * wherever center settled to that side's physical end, NOT a
               * fixed half-range like the old fixed-1500-center design could
               * assume (1500 was equidistant from both ends by construction;
               * a settled center generally isn't) - otherwise full stick
               * travel toward the nearer end would saturate
               * APP_ALTHOLD_MAX_CLIMB_MPS well before actually reaching it,
               * and the farther end would never reach full rate at all. */
              int32_t climb_rate_scale_us = (throttle_offset_us >= 0) ?
                  ((int32_t)APP_PWM_MAX_US - (int32_t)althold_settled_center_us) :
                  ((int32_t)althold_settled_center_us - (int32_t)APP_PWM_MIN_US);
              if (climb_rate_scale_us < 1)
              {
                climb_rate_scale_us = 1; /* guard - center settled right at an endpoint */
              }
              althold_holding = 0U;
              althold_integral_us = 0.0f;
              althold_pos_integral_mps = 0.0f;
              {
                float max_climb_mps = App_GetAltholdMaxClimbMps();
                climb_rate_setpoint_mps = App_ClampFloat(
                    ((float)throttle_offset_us / (float)climb_rate_scale_us) * max_climb_mps,
                    -max_climb_mps, max_climb_mps);
              }
            }

            climb_rate_error_mps = climb_rate_setpoint_mps - VertEkf_GetClimbRateMps();

            /* Fast/bounded reactive term - same role, same gains, same limit
             * as the original trim design. Protects against a short-term bad
             * reading regardless of what the slow hover estimate below is
             * doing. */
            althold_integral_us = App_ClampFloat(althold_integral_us +
                                                 (climb_rate_error_mps * App_GetAltholdVzKi() * dt_s),
                                                 -(float)APP_ALTHOLD_INTEGRAL_LIMIT_US,
                                                 (float)APP_ALTHOLD_INTEGRAL_LIMIT_US);
            althold_trim_us = App_ClampInt32((int32_t)((climb_rate_error_mps * App_GetAltholdVzKp()) +
                                                        althold_integral_us),
                                             -(int32_t)APP_ALTHOLD_TRIM_LIMIT_US,
                                             (int32_t)APP_ALTHOLD_TRIM_LIMIT_US);

            /* Slow-adapting base throttle - what makes full-authority output
             * possible without raw stick passthrough. Deliberately much
             * slower than the trim above (small Ki, hard-clamped to a narrow
             * absolute range) so it can only ever drift gradually toward
             * this flight's real hover throttle, never jump. */
            althold_hover_throttle_us = App_ClampFloat(althold_hover_throttle_us +
                                                        (climb_rate_error_mps *
                                                         APP_ALTHOLD_HOVER_EST_KI_US_PER_MPS_S * dt_s),
                                                        (float)APP_ALTHOLD_HOVER_EST_MIN_US,
                                                        (float)APP_ALTHOLD_HOVER_EST_MAX_US);
          }
          else
          {
            /* Not yet clear of ground effect, or baro unhealthy: open-loop
             * liftoff assist, NOT raw stick passthrough - see
             * APP_ALTHOLD_LIFTOFF_ASSIST_MAX_US's comment for why. No baro
             * feedback at all here, so althold_hover_throttle_us is
             * deliberately left untouched (stays at its arm-reset seed) until
             * the branch above takes over with real climb-rate feedback. */
            althold_holding = 0U;
            althold_integral_us = 0.0f;
            althold_pos_integral_mps = 0.0f;
            althold_liftoff_assist_active = 1U;
            althold_liftoff_base_us = althold_hover_throttle_us +
                                      ((float)throttle_offset_us *
                                       ((float)APP_ALTHOLD_LIFTOFF_ASSIST_MAX_US /
                                        (float)((int32_t)APP_PWM_MAX_US - (int32_t)APP_PWM_MID_US)));
          }
        }
        else
        {
          althold_holding = 0U;
          althold_integral_us = 0.0f;
          althold_pos_integral_mps = 0.0f;
          /* Not in ALTHOLD/NAVBRAKE - don't let a stale settle-window carry
           * into the next time this mode is selected. */
          althold_settle_ref_us = (uint16_t)throttle_us;
          althold_settle_start_ms = now_ms;
        }
        althold_authority_was_active = althold_authority_active;
        /* Smooth the trim itself (not just the gains feeding it) - noisy baro
         * climb-rate readings were translating directly into jerky per-iteration
         * motor changes on the Z axis. Filters toward 0 the same way when not
         * holding, so re-engaging/disengaging the hold is a smooth ramp too -
         * including across an authority-active/inactive transition, since the
         * base swaps below but the trim itself keeps ramping smoothly through
         * zero regardless. */
        {
          float althold_trim_lpf_alpha = App_LpfAlpha(dt_s, APP_ALTHOLD_TRIM_LPF_HZ);
          althold_trim_filtered_us += althold_trim_lpf_alpha *
                                      ((float)althold_trim_us - althold_trim_filtered_us);
        }
        althold_trim_us = (int32_t)althold_trim_filtered_us;
        /* Base depends on which of the three states above ran this iteration:
         * the slow hover estimate (full authority, gate open), the open-loop
         * liftoff-assist base (gate closed but in ALTHOLD/NAVBRAKE), or plain
         * raw stick throttle (RATE/ATTITUDE - not this mode at all). Whatever
         * the base, the final result is still clamped to
         * [APP_MOTOR_IDLE_US, APP_THROTTLE_MAX_US] below like any other
         * throttle path - full authority here is bounded by the same ceiling
         * full-stick manual flight already has. */
        if (althold_authority_active != 0U)
        {
          althold_throttle_us = (int32_t)althold_hover_throttle_us + althold_trim_us;
        }
        else if (althold_liftoff_assist_active != 0U)
        {
          althold_throttle_us = (int32_t)althold_liftoff_base_us + althold_trim_us;
        }
        else
        {
          althold_throttle_us = (int32_t)throttle_us + althold_trim_us;
        }

        baro_damp_term_us = 0;
        if ((App_GetBaroVzDampGain() != 0.0f) && (baro_healthy_now != 0U) &&
            (ground_effect_clear != 0U))
        {
          baro_damp_term_us = (int32_t)(-App_GetBaroVzDampGain() * VertEkf_GetClimbRateMps());
          baro_damp_term_us = App_ClampInt32(baro_damp_term_us,
                                            -((int32_t)App_GetBaroVzDampLimit()),
                                            (int32_t)App_GetBaroVzDampLimit());
        }
        /* Light dedicated smoothing on this term only - see APP_BARO_VZ_DAMP_LPF_HZ's
         * comment. Filters toward 0 the same way when the gate above is false, same
         * "smooth ramp on engage/disengage" reasoning as the ALTHOLD trim filter. */
        {
          float baro_damp_lpf_alpha = App_LpfAlpha(dt_s, APP_BARO_VZ_DAMP_LPF_HZ);
          baro_damp_term_filtered_us += baro_damp_lpf_alpha *
                                        ((float)baro_damp_term_us - baro_damp_term_filtered_us);
        }
        baro_damp_term_us = (int32_t)baro_damp_term_filtered_us;
        throttle_term = althold_throttle_us + baro_damp_term_us;
        if (throttle_term < (int32_t)APP_MOTOR_IDLE_US)
        {
          throttle_term = APP_MOTOR_IDLE_US;
        }
        /* Tier 2 of the low-battery response (see APP_LOW_BATTERY_CRITICAL_CELL_V
         * above): lower the ceiling instead of the floor, so pitch/roll/yaw
         * authority and the idle floor are completely unaffected - only the
         * ability to sustain a high-power climb/hover is removed. */
        throttle_term = App_ClampInt32(throttle_term,
                                       (int32_t)APP_MOTOR_IDLE_US,
                                       (low_battery_critical_active != 0U) ?
                                         (int32_t)APP_LOW_BATTERY_THROTTLE_CEILING_US :
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
          roll_nonfinite_reported = 0U;
          pitch_nonfinite_reported = 0U;
          yaw_nonfinite_reported = 0U;
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

        g_avg_motor_power_delta_us = (((float)s1_us + (float)s2_us + (float)s3_us + (float)s4_us) * 0.25f) -
                                     (float)APP_MOTOR_IDLE_US;

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
                                      ((Mag_IsHealthy() != 0U) ? APP_SDLOG_FLAG_MAG_HEALTHY : 0U) |
                                      ((mag_nudge_gated != 0U) ? APP_SDLOG_FLAG_MAG_NUDGE_GATED : 0U));
          sdlog_rec.pitch_deg_x10 = (int16_t)(pitch_deg * 10.0f);
          sdlog_rec.roll_deg_x10 = (int16_t)(roll_deg * 10.0f);
          /* target_roll/pitch_deg are only meaningful (set this iteration) in an angle-mode branch. */
          if ((flight_mode == APP_FLIGHT_MODE_ATTITUDE) || (flight_mode == APP_FLIGHT_MODE_ALTHOLD) ||
              (flight_mode == APP_FLIGHT_MODE_NAV_POSHOLD))
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
          sdlog_rec.mag_heading_x10 = (uint16_t)(Mag_GetHeadingDeg(mag_tilt_roll_deg, mag_tilt_pitch_deg, g_avg_motor_power_delta_us) * 10.0f);
          sdlog_rec.mag_field_dev_pct = mag_field_dev_pct;

          sdlog_rec.nav_flags = (uint8_t)((navpos_requested != 0U ? APP_SDLOG_NAV_FLAG_REQUESTED : 0U) |
                                          (navpos_active != 0U ? APP_SDLOG_NAV_FLAG_ACTIVE : 0U) |
                                          (navpos_tilt_limited != 0U ? APP_SDLOG_NAV_FLAG_TILT_LIMITED : 0U) |
                                          (navpos_accel_limited != 0U ? APP_SDLOG_NAV_FLAG_ACCEL_LIMITED : 0U) |
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
          sdlog_rec.nav_desired_vel_n_x100 = (int16_t)(navpos_desired_north_vel_mps * 100.0f);
          sdlog_rec.nav_desired_vel_e_x100 = (int16_t)(navpos_desired_east_vel_mps * 100.0f);
          sdlog_rec.nav_vel_error_n_x100 = (int16_t)(navpos_north_vel_error_mps * 100.0f);
          sdlog_rec.nav_vel_error_e_x100 = (int16_t)(navpos_east_vel_error_mps * 100.0f);
          sdlog_rec.nav_accel_cmd_n_x1000 = (int16_t)(navpos_north_accel_cmd_mps2 * 1000.0f);
          sdlog_rec.nav_accel_cmd_e_x1000 = (int16_t)(navpos_east_accel_cmd_mps2 * 1000.0f);
          sdlog_rec.nav_accel_cmd_fwd_x1000 = (int16_t)(navpos_fwd_accel_cmd_mps2 * 1000.0f);
          sdlog_rec.nav_accel_cmd_right_x1000 = (int16_t)(navpos_right_accel_cmd_mps2 * 1000.0f);
          sdlog_rec.pilot_roll_stick_us = pilot_roll_stick_us;
          sdlog_rec.pilot_pitch_stick_us = pilot_pitch_stick_us;
          sdlog_rec.rangefinder_cm_x10 = (int16_t)(App_GetRangefinderCm(now_ms) * 10.0f);
          sdlog_rec.luna_cm_x10 = (int16_t)(App_GetLunaCm(now_ms) * 10.0f);

          App_BlackboxCapture(&sdlog_rec);
          if ((sdlog_decim_counter++ % APP_SDLOG_DECIMATION) == 0U)
          {
            App_SdLogAppendRecord(&sdlog_rec);
          }

          if ((now_ms - last_sdlog_superblock_sync_ms) >= APP_SDLOG_SUPERBLOCK_SYNC_MS)
          {
            g_sdlog_next_free_block = g_sdlog_flight_next_block;
            App_SdLogSaveSuperblock();
            last_sdlog_superblock_sync_ms = now_ms;
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

        if ((low_battery_cells_known == 0U) && (battery_voltage_v > 1.0f))
        {
          uint8_t cells;
          for (cells = 1U; cells < 13U; cells++)
          {
            if (battery_voltage_v <= (((float)cells) * APP_LOW_BATTERY_MAX_CHARGE_CELL_V))
            {
              break;
            }
          }
          low_battery_cell_count = cells;
          low_battery_cells_known = 1U;
        }
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
    /* `+=` assumed increasing gz_dps increases yaw_deg. Attitude_GetBoardAnglesDeg()
     * computes `yaw = -atan2f(...)` - note the explicit negation - so the true
     * relationship is inverted: increasing gz_dps DECREASES yaw_deg. This is the
     * real-flight-loop counterpart of the same fix already applied at the USB-test
     * call site above (see its comment for the full derivation and live evidence);
     * this site was missed by that earlier edit despite the tool reporting all
     * occurrences replaced - confirmed by the vector-based nudge (2026-08-17)
     * converging smoothly but to the wrong (antipodal) equilibrium with `+=` still
     * here, exactly the signature of inverted feedback on a sin()-shaped error. */
    gz_dps -= mag_yaw_nudge_dps;

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
    g_last_ax_g = ax_g;
    g_last_ay_g = ay_g;
    g_last_az_g = az_g;
    VertEkf_Predict(Attitude_GetVerticalAccelMps2(ax_g, ay_g, az_g), dt_s);
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
      if ((bias_ready_seen_for_zero == 0U) && (g_gyro_bias_ready != 0U))
      {
        bias_ready_seen_for_zero = 1U;
        bias_ready_since_ms = now_ms;
      }

      if ((bias_ready_seen_for_zero != 0U) && (now_ms >= bias_ready_since_ms))
      {
        startup_roll_offset_sum_deg += roll_deg;
        startup_pitch_offset_sum_deg += pitch_deg;
        startup_yaw_offset_sum_deg += yaw_deg;
        startup_zero_avg_sample_count++;
      }

      if ((bias_ready_seen_for_zero != 0U) &&
          ((now_ms - bias_ready_since_ms) >= APP_ATTITUDE_ZERO_AVG_MS) &&
          (startup_zero_avg_sample_count > 0U))
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
    last_known_pitch_deg = pitch_deg;
    last_known_roll_deg = roll_deg;
    last_known_yaw_deg = yaw_deg;

    /* Compute the next iteration's slow yaw drift-correction nudge here (see
     * APP_MAG_YAW_NUDGE_* above) - compares the compass's own rotation-since-ref
     * against the AHRS's rotation-since-boot, so boot heading stays 0 as today. */
    if ((attitude_zero_captured != 0U) && (Mag_IsHealthy() != 0U))
    {
      float mag_field_now_g = sqrtf((Mag_GetXGauss() * Mag_GetXGauss()) +
                                     (Mag_GetYGauss() * Mag_GetYGauss()) +
                                     (Mag_GetZGauss() * Mag_GetZGauss()));

      if (mag_ref_field_g > 0.01f)
      {
        float dev_frac = fabsf(mag_field_now_g - mag_ref_field_g) / mag_ref_field_g;
        float dev_pct_f = dev_frac * 100.0f;

        mag_field_dev_pct = (uint8_t)((dev_pct_f > 255.0f) ? 255.0f : dev_pct_f);
      }

      if (mag_yaw_ref_captured == 0U)
      {
        float heading_now_rad = Mag_GetHeadingDeg(mag_tilt_roll_deg, mag_tilt_pitch_deg, g_avg_motor_power_delta_us) * RAD_PER_DEG;

        if (mag_ref_avg_sample_count == 0U)
        {
          mag_ref_avg_start_ms = now_ms;
        }

        mag_ref_avg_sin_sum += sinf(heading_now_rad);
        mag_ref_avg_cos_sum += cosf(heading_now_rad);
        mag_ref_avg_field_sum_g += mag_field_now_g;
        mag_ref_avg_sample_count++;

        if ((now_ms - mag_ref_avg_start_ms) >= APP_MAG_REF_AVG_MS)
        {
          mag_heading_at_ref_deg = atan2f(mag_ref_avg_sin_sum, mag_ref_avg_cos_sum) * (1.0f / RAD_PER_DEG);
          mag_ref_field_g = mag_ref_avg_field_sum_g / ((float)mag_ref_avg_sample_count);
          mag_yaw_ref_captured = 1U;
          mag_yaw_nudge_dps = 0.0f;
        }
        mag_nudge_gated = 0U;
      }
      else if ((mag_ref_field_g > 0.01f) &&
               (fabsf(mag_field_now_g - mag_ref_field_g) / mag_ref_field_g > APP_MAG_TRUST_MAX_MAG_DEVIATION_FRAC))
      {
        mag_yaw_nudge_dps = 0.0f; /* field strength shifted too much - likely motor/ESC current interference */
        mag_nudge_gated = 1U;
      }
      else
      {
        /* Negated 2026-08-21: bench-verified with real MAG+IMU telemetry that
         * Mag_GetHeadingDeg()'s delta and yaw_deg's delta moved in OPPOSITE
         * directions for the same physical rotation (magDelta +360deg vs
         * yawDelta -257deg over the same turn - rms tracking error 90deg,
         * peak 179.9deg, essentially maximally divergent, not just noisy).
         * This nudge's sign was very likely tuned against the OLD, since-fixed
         * mounting-rotation bug (see mag.c) that made the compass run
         * backwards - now that the compass correctly increases with CW
         * rotation, the nudge's assumed sign is mismatched again. Flipping
         * mag_delta_deg here (not yaw_deg, which NAVBRAKE/telemetry/SD log all
         * also depend on, and not Mag_GetHeadingDeg(), which is bench-verified
         * correct against true rotation direction and magnetic north) is the
         * minimal, localized fix. Re-verify with a fresh bench MAG+IMU capture
         * before trusting this in flight. */
        float mag_delta_deg = -Attitude_WrapAngle180(Mag_GetHeadingDeg(mag_tilt_roll_deg, mag_tilt_pitch_deg, g_avg_motor_power_delta_us) - mag_heading_at_ref_deg);
        /* Cross-product-style error: sin() of the angle difference, not the
         * difference itself - matches a plain error for small angles (sin(x) ~= x
         * near 0) but tapers smoothly to zero at the antipodal point instead of a
         * discontinuous wrap. RAD_PER_DEG in the denominator keeps
         * APP_MAG_YAW_NUDGE_KP_DPS_PER_DEG's small-angle meaning ("dps per degree
         * of error") even though the input to Kp is now a sine, not a degree. */
        float yaw_error_sin = sinf((mag_delta_deg - yaw_deg) * RAD_PER_DEG);
        float mag_yaw_nudge_target_dps = App_ClampFloat(
            yaw_error_sin * (APP_MAG_YAW_NUDGE_KP_DPS_PER_DEG / RAD_PER_DEG),
            -APP_MAG_YAW_NUDGE_MAX_DPS, APP_MAG_YAW_NUDGE_MAX_DPS);
        float mag_yaw_nudge_max_step_dps;

        mag_yaw_nudge_max_step_dps = APP_MAG_YAW_NUDGE_SLEW_DPS_PER_S * dt_s;

        mag_yaw_nudge_dps += App_ClampFloat(mag_yaw_nudge_target_dps - mag_yaw_nudge_dps,
                                             -mag_yaw_nudge_max_step_dps, mag_yaw_nudge_max_step_dps);
        mag_nudge_gated = 0U;
      }
    }
    else
    {
      mag_yaw_nudge_dps = 0.0f;
    }

    /* Stream while disarmed, or while armed with bench telemetry toggled on -
     * but not while an SDLOG DUMP is draining, since this bundle competes with
     * the dump's own block lines for the same UART6 link and was found
     * (2026-08-21) to badly slow down post-crash log recovery. */
    if ((APP_ENABLE_IMU_RUNTIME_TELEMETRY || (g_armed_test_telemetry_enabled != 0U) || (motors_armed == 0U)) &&
        (g_sdlog_dump_active == 0U) &&
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
      Telemetry_PrintVertEkfState(VertEkf_IsHealthy(), VertEkf_GetHeightM(), VertEkf_GetClimbRateMps(),
                                  VertEkf_GetAccelBiasMps2(), VertEkf_GetLidarImpliedHeightM(),
                                  VertEkf_GetSonarImpliedHeightM());
      Telemetry_PrintAltholdState(althold_holding, althold_authority_active, althold_target_alt_m,
                                  althold_fused_alt_m_now, climb_rate_setpoint_mps, climb_rate_error_mps,
                                  althold_trim_us, baro_damp_term_us, althold_hover_throttle_us);
      Telemetry_PrintGpsState(GPS_IsConfigured(), GPS_IsHealthy(), GPS_GetFixType(), GPS_GetNumSatellites(),
                             GPS_GetLatitudeDeg(), GPS_GetLongitudeDeg(), GPS_GetAltitudeM());
      /* Added 2026-09-04 alongside the CFG-NAV5 retry fix - this ack was
       * previously invisible in telemetry, so a lost round-trip (leaving the
       * receiver stuck in factory static-hold, freezing lat/lon) had no
       * symptom short of forensically comparing GPS output against known
       * real motion after the fact. (First attempt at this print landed in
       * the OTHER Telemetry_PrintGpsState call site, which is compiled out
       * behind #if APP_ENABLE_USB_TEST_IMU_TELEMETRY - this one is the call
       * site that's actually live during a real flight.) */
      printf("GPSNAV5[acked=%u]\r\n", (unsigned int)GPS_GetLastNav5Acked());
      Telemetry_PrintMagState(Mag_IsHealthy(), Mag_GetXGauss(), Mag_GetYGauss(), Mag_GetZGauss(),
                             Mag_GetHeadingDeg(mag_tilt_roll_deg, mag_tilt_pitch_deg, g_avg_motor_power_delta_us));
      Telemetry_PrintMagTiltState(mag_tilt_roll_deg, mag_tilt_pitch_deg);
      Telemetry_PrintNavState(nav_state.valid, nav_state.reference_valid, (uint8_t)nav_state.invalid_reason,
                             nav_state.fix_type, nav_state.num_sv, nav_state.h_acc_m,
                             nav_state.age_ms, nav_state.update_period_ms,
                             nav_state.consecutive_valid, nav_state.consecutive_invalid,
                             nav_state.duplicate_count, nav_state.rejected_count, nav_state.dropout_count);
      Telemetry_PrintNavPosVel(nav_state.north_m, nav_state.east_m,
                              nav_state.raw_north_vel_mps, nav_state.raw_east_vel_mps,
                              nav_state.filtered_north_vel_mps, nav_state.filtered_east_vel_mps);
      {
        /* Bench-test diagnostic: while DISARMED, the armed control-law block
         * above never ran, so target_roll/pitch_deg and the navpos_* accel
         * fields are stale/irrelevant - recompute a "would-be" off-center-stick
         * NAV_POSHOLD command here purely for bench validation (sign
         * conventions, yaw rotation, GPS velocity direction) using the SAME
         * named transformation functions as the real controller. This NEVER
         * reaches the motors (motors are already forced to idle/stopped while
         * disarmed). */
        uint8_t bench_requested = navpos_requested;
        uint8_t bench_active = navpos_active;
        uint8_t bench_tilt_limited = navpos_tilt_limited;
        uint8_t bench_accel_limited = navpos_accel_limited;
        float bench_desired_n = navpos_desired_north_vel_mps;
        float bench_desired_e = navpos_desired_east_vel_mps;
        float bench_err_n = navpos_north_vel_error_mps;
        float bench_err_e = navpos_east_vel_error_mps;
        float bench_accel_n = navpos_north_accel_cmd_mps2;
        float bench_accel_e = navpos_east_accel_cmd_mps2;
        float bench_accel_fwd = navpos_fwd_accel_cmd_mps2;
        float bench_accel_right = navpos_right_accel_cmd_mps2;
        float bench_target_roll_deg = target_roll_deg;
        float bench_target_pitch_deg = target_pitch_deg;

        if (motors_armed == 0U)
        {
          /* No APP_PITCH_SIGN flip here - see the matching comment on the armed
           * path above; this bench block must stay in sync with it. */
          float fwd_cmd_mps = App_NavStickOffsetToVelocityMps((int32_t)pitch_us - (int32_t)pitch_center_us,
                                                              APP_NAVPOS_MAX_STICK_VEL_MPS);
          float right_cmd_mps = ((float)APP_ROLL_SIGN) *
                                App_NavStickOffsetToVelocityMps((int32_t)roll_us - (int32_t)roll_center_us,
                                                                APP_NAVPOS_MAX_STICK_VEL_MPS);
          float desired_n;
          float desired_e;

          Nav_RotateBodyToNed(fwd_cmd_mps, right_cmd_mps, yaw_deg, &desired_n, &desired_e);

          bench_err_n = desired_n - nav_state.filtered_north_vel_mps;
          bench_err_e = desired_e - nav_state.filtered_east_vel_mps;
          bench_accel_n = App_ClampFloat(APP_NAVPOS_VELOCITY_KP * bench_err_n,
                                         -APP_NAVPOS_MAX_ACCEL_MPS2, APP_NAVPOS_MAX_ACCEL_MPS2);
          bench_accel_e = App_ClampFloat(APP_NAVPOS_VELOCITY_KP * bench_err_e,
                                         -APP_NAVPOS_MAX_ACCEL_MPS2, APP_NAVPOS_MAX_ACCEL_MPS2);
          Nav_RotateNedToBody(bench_accel_n, bench_accel_e, yaw_deg, &bench_accel_fwd, &bench_accel_right);
          bench_target_pitch_deg = ((float)APP_PITCH_SIGN) * Nav_AccelToAngleDeg(bench_accel_fwd, APP_NAVPOS_MAX_TILT_DEG);
          bench_target_roll_deg = ((float)APP_ROLL_SIGN) * Nav_AccelToAngleDeg(bench_accel_right, APP_NAVPOS_MAX_TILT_DEG);
          bench_desired_n = desired_n;
          bench_desired_e = desired_e;
          bench_tilt_limited = (uint8_t)((fabsf(bench_target_pitch_deg) >= (APP_NAVPOS_MAX_TILT_DEG - 0.01f)) ||
                                         (fabsf(bench_target_roll_deg) >= (APP_NAVPOS_MAX_TILT_DEG - 0.01f)));
          bench_accel_limited = (uint8_t)((fabsf(bench_accel_n) >= (APP_NAVPOS_MAX_ACCEL_MPS2 - 0.001f)) ||
                                          (fabsf(bench_accel_e) >= (APP_NAVPOS_MAX_ACCEL_MPS2 - 0.001f)));
        }

        Telemetry_PrintNavPos(bench_requested, bench_active, bench_tilt_limited, bench_accel_limited,
                               bench_desired_n, bench_desired_e,
                               bench_err_n, bench_err_e,
                               bench_accel_n, bench_accel_e,
                               bench_accel_fwd, bench_accel_right,
                               bench_target_roll_deg, bench_target_pitch_deg,
                               navpos_target_north_m, navpos_target_east_m,
                               navpos_err_north_m, navpos_err_east_m);
         printf("NAVGATE[nav ref att link]=[%u %u %u %u]\r\n",
           (unsigned int)nav_state.valid,
           (unsigned int)nav_state.reference_valid,
           (unsigned int)attitude_zero_captured,
           (unsigned int)receiver_state.link_active);
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
