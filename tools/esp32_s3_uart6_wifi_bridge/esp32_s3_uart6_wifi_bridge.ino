#include <WiFi.h>
#include <math.h>

static const char *WIFI_SSID = "NachoWi-Fi";
static const char *WIFI_PASSWORD = "NrMaintain1!";

/* Must match huart6.Init.BaudRate in Core/Src/main.c on the flight controller.
 * REVERTED to 115200 on 2026-08-22 after 921600 corrupted the UART6 payload
 * (likely a clock-divisor accuracy issue on the STM32 side) - see the comment
 * on huart6.Init.BaudRate in main.c for the full writeup. */
static const uint32_t FC_UART_BAUD = 115200;
static const int FC_UART_RX_PIN = 16;
static const int FC_UART_TX_PIN = 17;
static const uint16_t BRIDGE_PORT = 3333;

/* Sensor mounting/orientation descriptors (2026-08-25) - both the sonar and TF-Luna are
 * mounted facing straight down on this airframe. Deliberately NOT tilt-compensated
 * here: attitude (roll/pitch) is estimated on the FC, so slant-to-vertical projection
 * happens there instead, avoiding a round-trip latency penalty (send raw slant range
 * plus this fixed descriptor, let the FC do the trig against its own up-to-date
 * attitude estimate). Sent once at boot and re-sent alongside the periodic STAT
 * message so a FC that reboots/reconnects after the ESP32 is already running still
 * gets it promptly rather than waiting for a power cycle. axis_offset_deg is a fixed
 * mechanical offset from straight-down for this specific mount (0 = perfectly
 * vertical) - update these two constants if either sensor is ever remounted at an
 * angle; they're deliberately simple scalars, not a full orientation/quaternion
 * system, since neither sensor is mounted anywhere but straight down right now. */
static const char *RANGE_MOUNT_AXIS = "DOWN";
static const float RANGE_MOUNT_OFFSET_DEG = 0.0f;
static const char *LUNA_MOUNT_AXIS = "DOWN";
static const float LUNA_MOUNT_OFFSET_DEG = 0.0f;

/* HC-SR04 ultrasonic rangefinder (2026-08-23) - originally added purely as a
 * ground-truth height reference for calibrating the baro's propwash/ground-effect
 * altitude bias. Reported two ways: over bridge_client_printf(), the same
 * ESP32-originated WiFi side-channel already used for [BRIDGE] CONNECTED/STAT (easy for
 * the ground-station tooling to tell apart from FC-forwarded lines) - THIS copy is
 * subject to WiFi's usual latency/jitter/reconnect drops, so it stays ground-
 * station/logging use only; and, as of 2026-08-23, also over UART6/Serial2 as a plain
 * "RANGE <cm>\r\n" line, a direct wired link to the FC with none of the WiFi copy's
 * reliability caveats (an earlier version of this comment wrongly conflated the two -
 * see App_GetRangefinderCm()'s comment in app.c for the correction and what this copy
 * is now used for on the FC side, including as a genuine control input). Originally
 * added so the FC could log it into its own SD blackbox record on its own time_ms axis
 * (this replaced an earlier approach of aligning two independently-timed capture
 * streams after the fact, which proved too fragile to trust for real analysis). Serial2
 * is a shared, bidirectional command/telemetry link with the
 * FC in both directions, so both new writers onto it (FC-ward RANGE reports here, and the
 * existing GS-ward RANGE reports over bridge_client_printf) are queued and only flushed at
 * a confirmed clean line boundary on their respective direction, exactly like the
 * client-command relay already does - see s_uart_line_boundary and
 * s_client_to_uart_line_boundary below. GPIO4/5 chosen to avoid FC_UART_RX_PIN/
 * FC_UART_TX_PIN (16/17) above and the usual ESP32-S3 strapping/flash/PSRAM/USB-reserved
 * pins. */
static const int RANGE_TRIG_PIN = 4;
static const int RANGE_ECHO_PIN = 5;
static const uint32_t RANGE_TRIGGER_PERIOD_MS = 100;   /* ~10Hz - plenty for a calibration reference */
static const uint32_t RANGE_ECHO_TIMEOUT_US = 35000;   /* ~600cm round-trip + margin; a stale pending
                                                          * reading just gets abandoned and retried next period */
static const float RANGE_US_PER_CM = 58.0f;            /* standard HC-SR04 conversion (speed of sound, round trip) */
/* Cheap insurance against genuine near-field transducer-ringing noise (same element
 * transmits and receives; a too-close echo can arrive before its own ringing settles).
 * NOT the fix for the much larger 43-597cm scatter observed 2026-08-23 - that was
 * traced to off-axis/multipath echoes (something besides a clean, close, perpendicular
 * surface in the ~15deg beam cone), not this near-field effect - see the bootstrap gate
 * below and kh7-rangefinder-setup memory for the real mitigation. */
static const float RANGE_MIN_VALID_CM = 5.0f;
/* Bootstrap validation gate (2026-08-23): the aircraft is always powered on the ground,
 * so true height at ESP32 boot is known to be a few cm at most - use that as a sanity
 * check to help find the sensor's correct near-ground echo among the multipath/off-axis
 * candidates it can otherwise lock onto, rather than trusting it blind from power-up.
 * Until confirmed, only readings plausible for "sitting on the ground"
 * (<=RANGE_BOOTSTRAP_REJECT_ABOVE_CM) are reported at all; several consecutive readings
 * landing convincingly close (<=RANGE_BOOTSTRAP_CONFIRM_BELOW_CM, deliberately a bit
 * looser than the reject line for hysteresis) confirms the sensor has locked onto the
 * real echo, after which it's trusted normally with NO upper bound - it needs to report
 * real, larger distances once airborne. Only resets on ESP32 power-up/reset (static
 * initializers below) - there's no other arm/liftoff signal visible from this board, but
 * that matches "always powered on the ground" since the bridge shares power with the FC. */
static const float RANGE_BOOTSTRAP_REJECT_ABOVE_CM = 5.0f;
static const float RANGE_BOOTSTRAP_CONFIRM_BELOW_CM = 10.0f;
static const uint8_t RANGE_BOOTSTRAP_CONFIRM_COUNT = 5U;
static uint8_t s_range_bootstrap_confirmed = 0U;
static uint8_t s_range_bootstrap_streak = 0U;

/* Written only from the ISR; read/cleared only from loop() with interrupts briefly
 * disabled around the read - keeps the ISR itself minimal (timestamp + flag only).
 * s_echo_fall_us (2026-08-25) is the moment the echo fully returned - the best
 * available estimate of "when was this distance actually true," used as the
 * sonar's per-reading timestamp in the SENSOR packet (see the sensor-conditioning
 * redesign comment further down). Genuinely ISR-latched, so it's immune to
 * loop()/WiFi scheduling jitter, unlike TF-Luna's timestamp below. */
static volatile uint32_t s_echo_rise_us = 0;
static volatile uint32_t s_echo_fall_us = 0;
static volatile uint32_t s_echo_pulse_us = 0;
static volatile uint8_t s_echo_ready = 0;
static uint32_t s_range_trigger_pending_ms = 0;
static uint32_t s_last_range_trigger_ms = 0;

/* A RANGE report used to be sent the instant it was computed, from anywhere in loop() -
 * including possibly mid-relay of a long SDLOG[...] hex line from the FC, which can take
 * several loop() iterations to fully arrive over UART6 (each iteration forwards whatever
 * bytes have arrived so far). A report landing in one of those gaps spliced straight into
 * the middle of an unfinished line, corrupting it - confirmed 2026-08-23 by re-pulling the
 * same un-erased SD data twice and getting a DIFFERENT set of corrupted blocks each time
 * (rules out bad data at rest; it's a transient relay-interleaving bug). Fixed by queueing
 * the report and only flushing it once s_uart_line_boundary confirms the FC's own output
 * is at a clean line break, never mid-line. */
static uint8_t s_uart_line_boundary = 1U;
static float s_range_report_pending_cm = 0.0f;
static uint32_t s_range_report_pending_age_ms = 0U;
static uint8_t s_range_report_pending_to_gs = 0U; /* -> [BRIDGE] RANGE, gated by s_uart_line_boundary */
/* Reported to the FC too (2026-08-23) as "RANGE <cm>" over Serial2, so it lands in the SD
 * log on the FC's own time_ms axis - see App_SetRangefinderCm()'s comment in app.c for why
 * (this replaced an earlier approach of aligning two independently-timed capture streams
 * after the fact, which proved too fragile to trust for real analysis). This is the SAME
 * shared UART6 link ground-station commands travel on in the other direction, so it needs
 * the identical queue-until-line-boundary discipline as the GS-facing report above - just
 * gated on s_client_to_uart_line_boundary (declared near that relay direction) instead.
 *
 * 2026-08-25: the FC-facing copy carries more than just cm now - see the "SENSOR" packet
 * format further down (sensor ID, validity, range, confidence, timestamp). The GS-facing
 * "[BRIDGE] RANGE" copy above is UNCHANGED - that one is for human/GUI monitoring only,
 * out of scope for the sensor-conditioning redesign that added the fields below. */
static uint8_t s_range_report_pending_to_fc = 0U;
static float s_range_report_pending_confidence = 0.0f;
static uint32_t s_range_report_pending_ts_us = 0U;
static uint8_t s_range_report_pending_valid = 0U;

/* TF-Luna LiDAR rangefinder (2026-08-25) - added ALONGSIDE the HC-SR04 above, not
 * replacing it (kept exactly as-is per the request that added this). Better range
 * (0.2-8m vs the sonar's realistic ~4-6m ceiling) and not subject to the sonar's
 * multipath/off-axis scatter, so it's a natural complement once the aircraft is
 * higher than the sonar can trust. Free-runs and streams a 9-byte frame continuously
 * over UART (default 115200 8N1, same baud already used for Serial2/FC) once
 * powered - no trigger pulse needed, unlike the sonar. Wired to a THIRD UART
 * (Serial1) on GPIO6 (ESP32 TX -> Luna RXD) / GPIO7 (ESP32 RX <- Luna TXD),
 * deliberately not reusing GPIO4/5 (sonar) or 16/17 (FC) - see those constants'
 * comments for why those specific pins were already spoken for. TF-Luna's UART I/O
 * is 3.3V-logic-compatible even though it's powered from 5V, so - unlike the
 * sonar's echo line - no voltage divider is needed here. */
static const int LUNA_RX_PIN = 7;
static const int LUNA_TX_PIN = 6;
static const uint32_t LUNA_UART_BAUD = 115200;
HardwareSerial LunaSerial(1);

/* Standard TF-Luna frame: 0x59 0x59 DistL DistH StrengthL StrengthH TempL TempH Checksum
 * (checksum = low byte of the sum of the first 8 bytes). Distance is in cm directly -
 * no conversion factor needed, unlike the sonar's echo-time-to-distance math. */
#define LUNA_FRAME_LEN 9U
#define LUNA_FRAME_HEADER 0x59U
static uint8_t s_luna_frame_buf[LUNA_FRAME_LEN];
static uint8_t s_luna_frame_pos = 0U;
/* Per Benewake's datasheet: strength below LUNA_STRENGTH_FLOOR is noise-level - not a
 * weak echo, no real return at all - still a hard invalid. At/above
 * LUNA_STRENGTH_SATURATED the receiver is swamped (target too close/reflective) and the
 * reported distance is a qualitatively different kind of unreliable than "weak," so
 * that also stays a hard invalid rather than folding into the confidence ramp below.
 *
 * Between those two extremes (2026-08-25, replacing the old hard LUNA_STRENGTH_MIN=100
 * accept/reject cutoff): strength maps to a continuous confidence instead of a
 * pass/fail gate, ramping 0.0 at LUNA_STRENGTH_FLOOR up to 1.0 at
 * LUNA_STRENGTH_CONFIDENT - a weak-but-real return (dark surface, bright sunlight
 * washing out the receiver) gets reported with a low confidence number instead of
 * being silently dropped, so the FC can decide how much to trust it rather than never
 * seeing it at all. LUNA_STRENGTH_CONFIDENT=1000 is a reasonable "clearly good return"
 * value for this sensor family, not a precisely calibrated number - revisit if real
 * flight data shows confidence tracking poorly against known-good/known-bad readings. */
static const uint16_t LUNA_STRENGTH_FLOOR = 20U;
static const uint16_t LUNA_STRENGTH_CONFIDENT = 1000U;
static const uint16_t LUNA_STRENGTH_SATURATED = 65535U;
/* Lowered from the datasheet's rated 20cm minimum to 5cm (2026-08-25) after a bench
 * test, then to 0cm (2026-08-29) after a real liftoff capture showed this floor
 * actively rejecting genuine readings: this mount sits only 0-3cm off the ground at
 * rest (the LEFT-FORWARD arm sits lower than the sonar's arm), and diagnostic
 * telemetry (raw_cm/strength added to the [BRIDGE] LUNA line for exactly this
 * investigation) showed strength consistently 340-900+ - well above
 * LUNA_STRENGTH_FLOOR - throughout that 0-3cm range, with raw_cm climbing smoothly
 * and continuously as the aircraft rose (0,1,2,3cm at rest -> a clean monotonic
 * spread up through 166cm during the climb), not noisy or erratic the way a
 * genuinely-unreliable near-field reading would be. That was costing ~5 real seconds
 * of every flight - including all of liftoff - with zero valid lidar data at all.
 * Strength remains the real quality gate (see LUNA_STRENGTH_FLOOR above); this
 * distance floor is now purely a non-negative guard, not a near-field cutoff. Revisit
 * upward again only if a future unit's low-range telemetry (raw_cm/strength, still
 * in the GS-facing LUNA line) shows different behavior. */
static const float LUNA_MIN_VALID_CM = 0.0f;

static uint8_t s_luna_report_pending_to_gs = 0U;
static uint8_t s_luna_report_pending_to_fc = 0U;
static float s_luna_report_pending_cm = 0.0f;
/* Same additions as the sonar's pending state above, for the same "SENSOR" packet -
 * see that format's comment near the filter refactor further down. */
static float s_luna_report_pending_confidence = 0.0f;
static uint32_t s_luna_report_pending_ts_us = 0U;
static uint8_t s_luna_report_pending_valid = 0U;
/* Diagnostic-only (2026-08-29): raw distance/strength BEFORE the validity gate,
 * unconditionally captured every frame regardless of accept/reject - added after a
 * real liftoff capture showed LUNA reporting cm=0.0 (rejected) for several seconds
 * at the start of every flight with no visibility into WHY (near-field floor?
 * genuinely weak IR return off the floor surface? saturation?). GS-facing only -
 * not part of the SENSOR wire format to the FC, which does not need this. */
static uint16_t s_luna_report_pending_raw_distance_cm = 0U;
static uint16_t s_luna_report_pending_strength = 0U;

static const int RANGE_FILTER_WINDOW = 10;
static const int LUNA_FILTER_WINDOW = 30;
static const float RANGE_FILTER_K = 2.0f;              /* reject beyond k MAD-equivalent-sigma from median */
static const float RANGE_FILTER_MAD_SCALE = 1.4826f;   /* makes MAD comparable to a std dev for normal data */
/* Largest window either sensor uses - sizes the shared scratch arrays in
 * range_filter_apply_ex() below. Bump this if a future sensor needs a bigger window. */
#define RANGE_FILTER_MAX_WINDOW 32

static float s_range_filter_buf[RANGE_FILTER_WINDOW];
static uint8_t s_range_filter_count = 0U;   /* fills 0..WINDOW during ramp-up, then stays WINDOW */
static uint8_t s_range_filter_idx = 0U;     /* next slot to overwrite (circular) */

static float s_luna_filter_buf[LUNA_FILTER_WINDOW];
static uint8_t s_luna_filter_count = 0U;
static uint8_t s_luna_filter_idx = 0U;

/* Tiny insertion sort - windows are small (<=32 elements), no need for anything fancier. */
static float range_filter_median(float *arr, int n)
{
  int i;
  int j;
  float key;

  for (i = 1; i < n; i++)
  {
    key = arr[i];
    j = i - 1;
    while ((j >= 0) && (arr[j] > key))
    {
      arr[j + 1] = arr[j];
      j--;
    }
    arr[j + 1] = key;
  }
  if (n <= 0)
  {
    return 0.0f;
  }
  if ((n % 2) == 1)
  {
    return arr[n / 2];
  }
  return 0.5f * (arr[n / 2 - 1] + arr[n / 2]);
}

static float clampf(float v, float lo, float hi)
{
  if (v < lo) { return lo; }
  if (v > hi) { return hi; }
  return v;
}

/* Median/MAD outlier-rejecting filter (2026-08-23, generalized 2026-08-25 to serve both
 * sensors from one implementation instead of two copies - originally added for the
 * sonar: multipath/off-axis echoes produce large, isolated spikes (offline analysis of
 * real flight captures confirmed most bad readings are single-sample glitches, not
 * sustained runs - see kh7-rangefinder-setup memory). Keeps the last `window` accepted
 * readings for whichever sensor's buf/count/idx are passed in, rejects anything more
 * than RANGE_FILTER_K MAD-equivalent-sigma from the window's median, and returns the
 * median of whatever survives. MAD (not raw std dev) specifically because std dev is
 * dragged around by the very outliers being rejected - a couple of 300cm spikes in a
 * 10-point window inflate std dev enough to mask themselves (2-3sigma looks "normal"
 * once the spike has already widened sigma); MAD's ~50% breakdown point doesn't have
 * that problem. Offline-validated against two real flight captures before implementing
 * here - cut max spike magnitude from 350cm+/285cm+ down to ~100cm on both, tracking
 * the true low/high hold baselines cleanly throughout.
 *
 * out_confidence (2026-08-25, added for the sensor-conditioning redesign): how close
 * THIS sample sits to the window's median relative to the reject threshold - 1.0 at
 * zero deviation, sliding to 0.0 right at the reject boundary, computed from the exact
 * same med/mad/thresh already being calculated here rather than a separate metric. This
 * is the "flag likely weak-echo conditions" signal for the sonar (a reading that
 * technically passed but sits near the edge of what the recent window considers
 * plausible gets a low confidence instead of being silently treated as equally trustworthy
 * to a dead-center one) and doubles as a general per-sample quality score for either
 * sensor. Defaults to 1.0 during the window's ramp-up (thresh==0, nothing to compare
 * against yet - no evidence of a problem, so no reason to report low confidence). */
static float range_filter_apply_ex(float *buf, uint8_t *count, uint8_t *idx, int window,
                                    float new_value, float *out_confidence)
{
  float win[RANGE_FILTER_MAX_WINDOW];
  float deviations[RANGE_FILTER_MAX_WINDOW];
  float survivors[RANGE_FILTER_MAX_WINDOW];
  float med;
  float mad;
  float thresh;
  int n_survivors = 0;
  int i;

  buf[*idx] = new_value;
  *idx = (uint8_t)((*idx + 1) % window);
  if (*count < (uint8_t)window)
  {
    (*count)++;
  }

  /* Order doesn't matter for median/MAD, so reading the circular buffer 0..count-1
   * (rather than tracking chronological order) is fine either during ramp-up or once
   * it's full and wrapping. */
  for (i = 0; i < *count; i++)
  {
    win[i] = buf[i];
    deviations[i] = win[i];
  }
  med = range_filter_median(deviations, *count);

  for (i = 0; i < *count; i++)
  {
    deviations[i] = fabsf(win[i] - med);
  }
  mad = range_filter_median(deviations, *count);
  thresh = RANGE_FILTER_K * RANGE_FILTER_MAD_SCALE * mad;

  if (out_confidence != NULL)
  {
    *out_confidence = (thresh > 0.0f) ? clampf(1.0f - (fabsf(new_value - med) / thresh), 0.0f, 1.0f)
                                       : 1.0f;
  }

  for (i = 0; i < *count; i++)
  {
    if ((thresh <= 0.0f) || (fabsf(win[i] - med) <= thresh))
    {
      survivors[n_survivors] = win[i];
      n_survivors++;
    }
  }
  if (n_survivors == 0)
  {
    return med;
  }
  return range_filter_median(survivors, n_survivors);
}

/* Byte-at-a-time frame sync: TF-Luna doesn't frame-align itself on power-up, so this
 * hunts for two consecutive 0x59 header bytes before trusting anything after them -
 * same idea as any UART protocol that can start listening mid-stream. A checksum
 * mismatch just drops back to hunting for the next header instead of trying to
 * resync byte-by-byte within a corrupt-looking frame. */
static void luna_service()
{
  while (LunaSerial.available() > 0)
  {
    uint8_t b = (uint8_t)LunaSerial.read();

    if (s_luna_frame_pos < 2U)
    {
      if (b == LUNA_FRAME_HEADER)
      {
        s_luna_frame_buf[s_luna_frame_pos++] = b;
      }
      else
      {
        s_luna_frame_pos = 0U;
      }
      continue;
    }

    s_luna_frame_buf[s_luna_frame_pos++] = b;
    if (s_luna_frame_pos < LUNA_FRAME_LEN)
    {
      continue;
    }

    /* Full frame collected - validate and reset the hunt regardless of outcome. */
    s_luna_frame_pos = 0U;
    {
      uint8_t checksum = 0U;
      int i;
      for (i = 0; i < (int)(LUNA_FRAME_LEN - 1U); i++)
      {
        checksum = (uint8_t)(checksum + s_luna_frame_buf[i]);
      }
      if (checksum != s_luna_frame_buf[LUNA_FRAME_LEN - 1U])
      {
        continue;
      }

      {
        /* Closest available approximation to a receive timestamp: micros() captured
         * right as the frame's last (checksum) byte is consumed. Not a true
         * hardware-latched receive time the way the sonar's ISR timestamp is - this is
         * pure polled loop() code, so it inherits whatever scheduling delay loop() saw
         * getting here (WiFi/lwIP task activity included). If that delay ever needs to
         * be characterized precisely, this is the place to add real measurement rather
         * than assume either way. */
        uint32_t ts_us = micros();
        uint16_t distance_cm = (uint16_t)(s_luna_frame_buf[2] | ((uint16_t)s_luna_frame_buf[3] << 8));
        uint16_t strength = (uint16_t)(s_luna_frame_buf[4] | ((uint16_t)s_luna_frame_buf[5] << 8));
        /* Same "always report a line, cm=0.0 on rejection" convention as the sonar -
         * see range_service()'s comment for why (tells "actively rejecting" apart
         * from "sensor/link is dead" from the far end without extra bookkeeping). */
        float report_cm = 0.0f;
        float report_confidence = 0.0f;
        uint8_t report_valid = 0U;

        if ((strength >= LUNA_STRENGTH_FLOOR) && (strength < LUNA_STRENGTH_SATURATED) &&
            ((float)distance_cm >= LUNA_MIN_VALID_CM))
        {
          /* Only feed already-plausible readings into the outlier filter - keeps
           * this a clean, separate concern from the per-frame validation above,
           * same idiom as the sonar's near-field-floor-then-filter ordering. */
          float temporal_confidence = 1.0f;
          float filtered_cm = range_filter_apply_ex(s_luna_filter_buf, &s_luna_filter_count,
                                                     &s_luna_filter_idx, LUNA_FILTER_WINDOW,
                                                     (float)distance_cm, &temporal_confidence);
          /* Strength-based confidence (2026-08-25, replaces the old hard
           * LUNA_STRENGTH_MIN cutoff - see that constant's comment) combined with the
           * filter's temporal agreement confidence via MIN - either a weak echo OR a
           * temporally-inconsistent one should pull the reported confidence down, not
           * just one or the other. */
          float strength_confidence = clampf(((float)strength - (float)LUNA_STRENGTH_FLOOR) /
                                              ((float)LUNA_STRENGTH_CONFIDENT - (float)LUNA_STRENGTH_FLOOR),
                                              0.0f, 1.0f);
          report_cm = filtered_cm;
          report_confidence = (strength_confidence < temporal_confidence) ? strength_confidence
                                                                           : temporal_confidence;
          report_valid = 1U;
        }

        /* Queue only - same s_uart_line_boundary discipline as the sonar's RANGE
         * report, for the identical reason (shared UART6 link to the FC). */
        s_luna_report_pending_cm = report_cm;
        s_luna_report_pending_confidence = report_confidence;
        s_luna_report_pending_ts_us = ts_us;
        s_luna_report_pending_valid = report_valid;
        s_luna_report_pending_raw_distance_cm = distance_cm;
        s_luna_report_pending_strength = strength;
        s_luna_report_pending_to_gs = 1U;
        s_luna_report_pending_to_fc = 1U;
      }
    }
  }
}

static void IRAM_ATTR range_echo_isr()
{
  if (digitalRead(RANGE_ECHO_PIN) == HIGH)
  {
    s_echo_rise_us = micros();
  }
  else
  {
    s_echo_fall_us = micros();
    s_echo_pulse_us = s_echo_fall_us - s_echo_rise_us;
    s_echo_ready = 1;
  }
}

/* Multiple simultaneous TCP clients (2026-08-23) - the GUI and any ad-hoc capture
 * script (SD log pulls, live BARO/RANGE captures, etc.) used to fight over a single
 * connection slot: whichever connected first silently starved every later one (accepted
 * at the TCP/OS level, since that happens autonomously, but never serviced by this
 * sketch's loop() - looked identical to a hung/dead board from the far end, cost a lot
 * of debugging time before the single-client limit was found). FC UART data now
 * broadcasts to every connected client; commands FROM any client relay to the FC -
 * concurrent senders can still interleave commands into a corrupted line at the FC's
 * parser, same risk as any shared serial console, just no longer silently dropped. */
static const int MAX_CLIENTS = 3;
WiFiServer server(BRIDGE_PORT);
WiFiClient clients[MAX_CLIENTS];

static uint32_t last_status_ms = 0;
static uint32_t uart_to_tcp_bytes = 0;
static uint32_t tcp_to_uart_bytes = 0;

/* CONNECTED/STAT used to call bridge_client_printf() directly, unlike RANGE/LUNA which
 * were already queued - meaning they could still splice into the middle of an
 * in-progress FC-relayed line on the TCP stream to GS clients, the same corruption
 * class the RANGE/LUNA queuing was built to prevent, just not closed off for these two
 * (2026-08-25: this was flagged as a known ~2.3% residual corruption risk earlier and
 * deferred - closing it now). Same single-slot "overwrite is fine, this is live status
 * not a log" pattern as the sensor reports - gated on the same s_uart_line_boundary. */
static uint8_t s_connected_pending = 0U;
static char s_connected_pending_msg[96];
static uint8_t s_stat_pending = 0U;
static char s_stat_pending_msg[96];

/* SENSOR_CFG mounting-descriptor queue state (2026-08-25) - see RANGE_MOUNT_AXIS's
 * comment for what this sends and why. Queued/flushed the same way as the SENSOR
 * reports below, gated on s_client_to_uart_line_boundary since these are FC-facing. */
static uint8_t s_range_cfg_pending_to_fc = 0U;
static uint8_t s_luna_cfg_pending_to_fc = 0U;

/* Prevents two different clients' commands from interleaving into one corrupted line
 * at the FC's parser (2026-08-25) - a real incident: an ad-hoc diagnostic connection
 * collided with a real SD-log pull command on this exact link this same night. -1 means
 * the link is free for any client to claim; otherwise only that client's bytes may be
 * forwarded until its line ends in '\n'. s_client_uart_owner_last_ms guards against a
 * client that disconnects or goes quiet mid-command permanently blocking everyone else -
 * an ownership scheme needs an escape hatch or it's just a different deadlock. */
static int s_client_uart_owner = -1;
static uint32_t s_client_uart_owner_last_ms = 0U;
#define CLIENT_UART_OWNER_TIMEOUT_MS 1000U

static void bridge_client_printf(const char *fmt, ...)
{
  char buf[192];
  va_list args;

  va_start(args, fmt);
  (void)vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  for (int i = 0; i < MAX_CLIENTS; i++)
  {
    if (clients[i] && clients[i].connected())
    {
      clients[i].print(buf);
    }
  }
}

/* Onboard addressable RGB LED (WS2812) - GPIO48 on ESP32-S3-DevKitC-1 (the exact board
 * this project targets per platformio.ini's board=esp32-s3-devkitc-1, before that file
 * was removed as stale - see git history). Was lit white by the factory/bootloader
 * firmware; goes dark once this sketch's own code starts running and never touches it.
 * Added 2026-08-23 as a visible "is everything actually working" indicator: off while
 * disconnected/(re)connecting, bright green once WiFi is up. rgbLedWrite() is the
 * arduino-esp32 core's built-in single-pixel helper (core 3.x+) - no external NeoPixel
 * library needed. */
static const int STATUS_LED_PIN = 48;

static void connect_wifi()
{
  rgbLedWrite(STATUS_LED_PIN, 0, 0, 0);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.println("[ESP32] Connecting to WiFi...");

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  rgbLedWrite(STATUS_LED_PIN, 0, 255, 0);
  Serial.println("\n[ESP32] WiFi connected!");
  Serial.print("[ESP32] IP: ");
  Serial.println(WiFi.localIP());
}

/* Byte-by-byte relaying (one client.write()/Serial2.write() call, plus one
 * Serial.printf() debug line, PER BYTE) was the actual bottleneck for SD log
 * pulls - measured ~8.8KB/s, right at the UART's raw baud ceiling, because each
 * printf() call (formatting + a blocking USB-serial write) took far longer than
 * one UART bit-time, so debug logging was throttling the relay to barely UART
 * speed regardless of how fast the far ends could go. Found/fixed 2026-08-22.
 * Draining each side into a local buffer and doing ONE write() per loop pass
 * removes that bottleneck; the buffer size just needs to comfortably exceed
 * what one loop() iteration can accumulate. */
static uint8_t s_uart_to_tcp_buf[512];
static uint8_t s_tcp_to_uart_buf[512];

/* Mirrors s_uart_line_boundary but for the opposite direction (client -> Serial2/FC) -
 * tracks whether the last byte relayed to the FC ended a line, so the FC-facing RANGE
 * report (s_range_report_pending_to_fc) can be safely interleaved without splitting a
 * ground-station command mid-line. Starts true since nothing has been sent yet. */
static uint8_t s_client_to_uart_line_boundary = 1U;

void setup()
{
  Serial.begin(115200);
  Serial2.setRxBufferSize(2048);
  Serial2.begin(FC_UART_BAUD, SERIAL_8N1, FC_UART_RX_PIN, FC_UART_TX_PIN);

  pinMode(RANGE_TRIG_PIN, OUTPUT);
  digitalWrite(RANGE_TRIG_PIN, LOW);
  pinMode(RANGE_ECHO_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(RANGE_ECHO_PIN), range_echo_isr, CHANGE);

  LunaSerial.begin(LUNA_UART_BAUD, SERIAL_8N1, LUNA_RX_PIN, LUNA_TX_PIN);

  /* Send the mounting descriptor once at boot - see RANGE_MOUNT_AXIS's comment. Also
   * re-sent every ~3s alongside STAT (see loop()) for a FC that reboots/reconnects later. */
  s_range_cfg_pending_to_fc = 1U;
  s_luna_cfg_pending_to_fc = 1U;

  connect_wifi();
  server.begin();
  server.setNoDelay(true);
}

/* Fires a new trigger pulse every RANGE_TRIGGER_PERIOD_MS, independent of whether the
 * previous one ever got an echo back (a missed/timed-out reading just skips a cycle
 * rather than blocking anything - see the ISR above for how a reading actually completes).
 * The 10us trigger pulse itself is the only blocking part, same as any HC-SR04 driver -
 * negligible next to the relay loop's timing. */
static void range_service()
{
  uint32_t now_ms = millis();

  if ((now_ms - s_last_range_trigger_ms) >= RANGE_TRIGGER_PERIOD_MS)
  {
    s_last_range_trigger_ms = now_ms;
    s_echo_ready = 0;
    digitalWrite(RANGE_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(RANGE_TRIG_PIN, LOW);
    s_range_trigger_pending_ms = now_ms;
  }

  if (s_echo_ready != 0)
  {
    uint32_t pulse_us;
    uint32_t fall_us;

    noInterrupts();
    pulse_us = s_echo_pulse_us;
    fall_us = s_echo_fall_us;
    s_echo_ready = 0;
    interrupts();

    /* Always report exactly one line per completed echo cycle - cm=0.0 on any
     * rejection path (echo timeout, near-field floor, unconfirmed-bootstrap-too-far)
     * instead of silently doing nothing, so the far end can tell "actively rejecting
     * noise" apart from "sensor/link is dead" just by watching whether cm keeps
     * updating at all. Added 2026-08-23 after exactly that ambiguity cost real
     * debugging time - see kh7-rangefinder-setup memory. valid/confidence (2026-08-25)
     * follow the same idea, just split into two separate signals instead of one -
     * see range_filter_apply_ex()'s comment for what confidence means, and
     * luna_service() for why a low-but-nonzero confidence is NOT the same as invalid. */
    float report_cm = 0.0f;
    float report_confidence = 0.0f;
    uint8_t report_valid = 0U;

    if (pulse_us < RANGE_ECHO_TIMEOUT_US)
    {
      float distance_cm = (float)pulse_us / RANGE_US_PER_CM;
      /* Diagnostic capture 2026-08-23 confirmed the still-wild readings are NOT a
       * near-field issue (0/85 raw samples were even below 5cm - the scatter ran
       * 43-597cm) - this is off-axis/multipath (something besides a clean, close,
       * perpendicular surface in the ~15deg beam cone), not the transducer-ringing dead
       * zone this filter targets. Kept anyway as cheap insurance against genuine
       * near-field noise, but it is NOT the fix for the multipath scatter - see
       * kh7-rangefinder-setup memory. */
      if (distance_cm >= RANGE_MIN_VALID_CM)
      {
        /* Only feed already-past-the-near-field-floor candidates into the multipath
         * filter - keeps this a clean, separate concern from the bootstrap gate below,
         * which then operates on the filtered value instead of the raw one. */
        float confidence = 1.0f;
        float filtered_cm = range_filter_apply_ex(s_range_filter_buf, &s_range_filter_count,
                                                   &s_range_filter_idx, RANGE_FILTER_WINDOW,
                                                   distance_cm, &confidence);

        if (s_range_bootstrap_confirmed == 0U)
        {
          if (filtered_cm <= RANGE_BOOTSTRAP_CONFIRM_BELOW_CM)
          {
            s_range_bootstrap_streak++;
            if (s_range_bootstrap_streak >= RANGE_BOOTSTRAP_CONFIRM_COUNT)
            {
              s_range_bootstrap_confirmed = 1U;
            }
          }
          else
          {
            s_range_bootstrap_streak = 0U;
          }
        }

        if ((s_range_bootstrap_confirmed != 0U) || (filtered_cm <= RANGE_BOOTSTRAP_REJECT_ABOVE_CM))
        {
          report_cm = filtered_cm;
          report_confidence = confidence;
          report_valid = 1U;
        }
      }
    }

    /* Queue only - see s_uart_line_boundary's comment for why this can't send directly
     * from here. A newer reading overwriting a not-yet-flushed older one is fine; this
     * is a live sensor value, not a log that needs every sample delivered. */
    s_range_report_pending_cm = report_cm;
    s_range_report_pending_age_ms = millis() - s_range_trigger_pending_ms;
    s_range_report_pending_confidence = report_confidence;
    s_range_report_pending_ts_us = fall_us;
    s_range_report_pending_valid = report_valid;
    s_range_report_pending_to_gs = 1U;
    s_range_report_pending_to_fc = 1U;
  }
}

void loop()
{
  range_service();
  luna_service();

  if (WiFi.status() != WL_CONNECTED)
  {
    connect_wifi();
  }

  {
    WiFiClient next = server.available();
    if (next)
    {
      int slot = -1;

      for (int i = 0; i < MAX_CLIENTS; i++)
      {
        if (!clients[i] || !clients[i].connected())
        {
          slot = i;
          break;
        }
      }

      if (slot >= 0)
      {
        clients[slot] = next;
        clients[slot].setNoDelay(true);
        Serial.printf("[ESP32] TCP Client connected in slot %d!\r\n", slot);
        /* Queued now instead of sent directly - see s_connected_pending's declaration
         * comment for why. A second connection arriving before this one flushes just
         * overwrites the pending message (same "live status, not a log" tradeoff RANGE/
         * LUNA already make) - acceptable since this is an informational notice, not
         * something any client's own operation depends on receiving. */
        (void)snprintf(s_connected_pending_msg, sizeof(s_connected_pending_msg),
                       "[BRIDGE] CONNECTED ip=%s uart_rx=%d uart_tx=%d slot=%d\r\n",
                       WiFi.localIP().toString().c_str(),
                       FC_UART_RX_PIN,
                       FC_UART_TX_PIN,
                       slot);
        s_connected_pending = 1U;
      }
      else
      {
        Serial.println("[ESP32] TCP Client rejected - all slots full");
        next.stop();
      }
    }
  }

  /* Drain Serial2 unconditionally (not gated on any client being connected) so its
   * hardware RX buffer can't fill up/overflow while nobody's listening - broadcasting
   * is then just a no-op per-client loop below if n==0 or nothing's connected. */
  {
    int n = 0;
    while (Serial2.available() > 0 && n < (int)sizeof(s_uart_to_tcp_buf))
    {
      s_uart_to_tcp_buf[n++] = (uint8_t)Serial2.read();
    }
    if (n > 0)
    {
      for (int i = 0; i < MAX_CLIENTS; i++)
      {
        if (clients[i] && clients[i].connected())
        {
          clients[i].write(s_uart_to_tcp_buf, n);
        }
      }
      uart_to_tcp_bytes += (uint32_t)n;
      s_uart_line_boundary = (s_uart_to_tcp_buf[n - 1] == (uint8_t)'\n') ? 1U : 0U;
    }
  }

  /* Only safe to interleave a RANGE report once the FC's own output is confirmed at a
   * clean line break - see s_uart_line_boundary's declaration comment. */
  if ((s_range_report_pending_to_gs != 0U) && (s_uart_line_boundary != 0U))
  {
    s_range_report_pending_to_gs = 0U;
    bridge_client_printf("[BRIDGE] RANGE cm=%.1f age_ms=%lu\r\n",
                         (double)s_range_report_pending_cm,
                         (unsigned long)s_range_report_pending_age_ms);
  }

  /* Same queue-until-line-boundary discipline as the sonar's RANGE report above -
   * see LUNA_RX_PIN's declaration comment for why this sensor exists alongside it. */
  if ((s_luna_report_pending_to_gs != 0U) && (s_uart_line_boundary != 0U))
  {
    s_luna_report_pending_to_gs = 0U;
    bridge_client_printf("[BRIDGE] LUNA cm=%.1f raw_cm=%u strength=%u\r\n",
                         (double)s_luna_report_pending_cm,
                         (unsigned int)s_luna_report_pending_raw_distance_cm,
                         (unsigned int)s_luna_report_pending_strength);
  }

  /* Same discipline for CONNECTED/STAT - see s_connected_pending's declaration comment
   * for why these were a real, if smaller, gap until now. */
  if ((s_connected_pending != 0U) && (s_uart_line_boundary != 0U))
  {
    s_connected_pending = 0U;
    bridge_client_printf("%s", s_connected_pending_msg);
  }
  if ((s_stat_pending != 0U) && (s_uart_line_boundary != 0U))
  {
    s_stat_pending = 0U;
    bridge_client_printf("%s", s_stat_pending_msg);
  }

  {
    uint8_t wrote_to_uart_this_iter = 0U;
    uint8_t last_byte_to_uart = 0U;

    /* Release a stuck/abandoned mid-line claim - see s_client_uart_owner's declaration
     * comment. Checked every iteration, not just when a write happens, so a client that
     * silently vanishes mid-command doesn't leave everyone else blocked until the next
     * byte happens to arrive from someone. */
    if (s_client_uart_owner >= 0)
    {
      bool owner_gone = !(clients[s_client_uart_owner] && clients[s_client_uart_owner].connected());
      bool owner_stale = (millis() - s_client_uart_owner_last_ms) > CLIENT_UART_OWNER_TIMEOUT_MS;
      if (owner_gone || owner_stale)
      {
        s_client_uart_owner = -1;
      }
    }

    for (int i = 0; i < MAX_CLIENTS; i++)
    {
      if (!(clients[i] && clients[i].connected()))
      {
        continue;
      }
      /* Only the current line-owner (or, if the link is currently free, whichever
       * client happens to have data first this iteration) may write to Serial2 this
       * pass - see s_client_uart_owner's declaration comment for the incident that
       * motivated this. */
      if ((s_client_uart_owner >= 0) && (s_client_uart_owner != i))
      {
        continue;
      }
      if (clients[i].available() <= 0)
      {
        continue;
      }

      int m = 0;
      while (clients[i].available() > 0 && m < (int)sizeof(s_tcp_to_uart_buf))
      {
        s_tcp_to_uart_buf[m++] = (uint8_t)clients[i].read();
      }
      Serial2.write(s_tcp_to_uart_buf, m);
      tcp_to_uart_bytes += (uint32_t)m;
      wrote_to_uart_this_iter = 1U;
      last_byte_to_uart = s_tcp_to_uart_buf[m - 1];
      s_client_uart_owner = i;
      s_client_uart_owner_last_ms = millis();
      break; /* only one client serviced per iteration - see the comment above */
    }
    if (wrote_to_uart_this_iter != 0U)
    {
      s_client_to_uart_line_boundary = (last_byte_to_uart == (uint8_t)'\n') ? 1U : 0U;
      if (s_client_to_uart_line_boundary != 0U)
      {
        s_client_uart_owner = -1; /* line released - anyone can claim it next */
      }
    }
  }

  /* Symmetric to the GS-facing flush above, for the same reason - see
   * s_range_report_pending_to_fc's declaration comment. Format:
   * "SENSOR <id> <valid> <range_cm> <confidence> <ts_us>\r\n" - <range_cm> is the raw
   * SLANT range, deliberately not tilt-compensated (see RANGE_MOUNT_AXIS's comment). */
  if ((s_range_report_pending_to_fc != 0U) && (s_client_to_uart_line_boundary != 0U))
  {
    s_range_report_pending_to_fc = 0U;
    Serial2.print("SENSOR SONAR ");
    Serial2.print(s_range_report_pending_valid);
    Serial2.print(' ');
    Serial2.print(s_range_report_pending_cm, 1);
    Serial2.print(' ');
    Serial2.print(s_range_report_pending_confidence, 2);
    Serial2.print(' ');
    Serial2.print(s_range_report_pending_ts_us);
    Serial2.print("\r\n");
  }

  /* Same queue-until-line-boundary discipline as the sonar's FC-facing SENSOR flush
   * above. See App_SetLunaCm()'s comment in app.c for what the FC does with this. */
  if ((s_luna_report_pending_to_fc != 0U) && (s_client_to_uart_line_boundary != 0U))
  {
    s_luna_report_pending_to_fc = 0U;
    Serial2.print("SENSOR LUNA ");
    Serial2.print(s_luna_report_pending_valid);
    Serial2.print(' ');
    Serial2.print(s_luna_report_pending_cm, 1);
    Serial2.print(' ');
    Serial2.print(s_luna_report_pending_confidence, 2);
    Serial2.print(' ');
    Serial2.print(s_luna_report_pending_ts_us);
    Serial2.print("\r\n");
  }

  /* Mounting/orientation descriptor flush - see RANGE_MOUNT_AXIS's comment and
   * s_range_cfg_pending_to_fc's declaration. Format:
   * "SENSOR_CFG <id> <axis> <offset_deg>\r\n". */
  if ((s_range_cfg_pending_to_fc != 0U) && (s_client_to_uart_line_boundary != 0U))
  {
    s_range_cfg_pending_to_fc = 0U;
    Serial2.print("SENSOR_CFG SONAR ");
    Serial2.print(RANGE_MOUNT_AXIS);
    Serial2.print(' ');
    Serial2.print(RANGE_MOUNT_OFFSET_DEG, 1);
    Serial2.print("\r\n");
  }
  if ((s_luna_cfg_pending_to_fc != 0U) && (s_client_to_uart_line_boundary != 0U))
  {
    s_luna_cfg_pending_to_fc = 0U;
    Serial2.print("SENSOR_CFG LUNA ");
    Serial2.print(LUNA_MOUNT_AXIS);
    Serial2.print(' ');
    Serial2.print(LUNA_MOUNT_OFFSET_DEG, 1);
    Serial2.print("\r\n");
  }

  if ((millis() - last_status_ms) > 3000)
  {
    last_status_ms = millis();
    int connected_count = 0;
    for (int i = 0; i < MAX_CLIENTS; i++)
    {
      if (clients[i] && clients[i].connected())
      {
        connected_count++;
      }
    }
    Serial.printf("[ESP32] STAT wifi=%d clients=%d/%d u2t=%lu t2u=%lu\r\n",
                  (int)WiFi.status(),
                  connected_count,
                  MAX_CLIENTS,
                  (unsigned long)uart_to_tcp_bytes,
                  (unsigned long)tcp_to_uart_bytes);
    /* Queued now instead of sent directly - see s_stat_pending's declaration comment.
     * A STAT line still pending from 3s ago being overwritten by this one is fine, same
     * live-status tradeoff as everything else on this link. */
    (void)snprintf(s_stat_pending_msg, sizeof(s_stat_pending_msg),
                   "[BRIDGE] STAT wifi=%d clients=%d/%d u2t=%lu t2u=%lu\r\n",
                   (int)WiFi.status(),
                   connected_count,
                   MAX_CLIENTS,
                   (unsigned long)uart_to_tcp_bytes,
                   (unsigned long)tcp_to_uart_bytes);
    s_stat_pending = 1U;

    /* Re-send the mounting descriptor alongside STAT - see s_range_cfg_pending_to_fc's
     * declaration comment for why (a FC that reboots/reconnects after the ESP32 is
     * already running still gets it promptly instead of waiting for a power cycle). */
    s_range_cfg_pending_to_fc = 1U;
    s_luna_cfg_pending_to_fc = 1U;
  }
}
