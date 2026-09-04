#include "receiver.h"
#include "communications.h"
#include "gps.h"
#include "app.h"
#include "motors.h"

#include "main.h"

#include <string.h>
#include <stdio.h>

#define CRSF_PAYLOAD_MAX_SIZE          62U
#define CRSF_FRAME_MAX_SIZE            (CRSF_PAYLOAD_MAX_SIZE + 2U)
#define CRSF_FRAME_TYPE_RC_CHANNELS    0x16U
#define CRSF_RC_PAYLOAD_SIZE           22U
#define CRSF_LINK_TIMEOUT_MS           250U
#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8U
/* Battery sensor telemetry TX (FC -> RX -> transmitter), so the radio's own
 * (usually voice-alarm-capable) telemetry display gets battery voltage
 * instead of relying solely on the aircraft's own onboard piezo, which is
 * easy to miss over prop noise/distance. Standard CRSF frame per
 * github.com/tbs-fpv/tbs-crsf-spec: payload is voltage(u16 BE, 0.1V),
 * current(u16 BE, 0.1A), capacity_used(u24 BE, mAh), remaining(u8, percent) -
 * verified 2026-08-21 against Betaflight's crsfFrameBatterySensor(). This FC
 * has no current/capacity sensing, so those fields are always sent as 0;
 * voltage is the field radio-side alarms actually key off. huart4 is
 * configured UART_MODE_TX_RX (full duplex, separate TX/RX wiring assumed,
 * not single-wire half-duplex) - if the RX's telemetry pad isn't physically
 * wired to this MCU's UART4_TX pin, this is a harmless no-op. */
#define CRSF_FRAMETYPE_BATTERY_SENSOR      0x08U
#define CRSF_BATTERY_SENSOR_PAYLOAD_SIZE   8U
#define CRSF_FRAME_LENGTH_TYPE_CRC         2U /* type + crc bytes, added to payload size for the length field */

/* GPS/Attitude/Vario/Flight-mode telemetry TX, matching iNavFlight/inav's
 * src/main/telemetry/crsf.c payload formats exactly (verified 2026-08-21
 * against that source) so EdgeTX's standard iNav telemetry LUA script reads
 * this FC the same way it reads a real iNav flight controller, even though
 * this isn't iNav - the wire format is just the shared CRSF spec, iNav's
 * script doesn't care what firmware produced it. */
#define CRSF_FRAMETYPE_GPS                 0x02U
#define CRSF_GPS_PAYLOAD_SIZE              15U /* lat i32, lon i32, speed u16, heading u16, alt u16, sats u8 */
#define CRSF_FRAMETYPE_VARIO               0x07U
#define CRSF_VARIO_PAYLOAD_SIZE            2U  /* vertical speed i16, cm/s */
#define CRSF_FRAMETYPE_ATTITUDE            0x1EU
#define CRSF_ATTITUDE_PAYLOAD_SIZE         6U  /* pitch/roll/yaw i16 each, rad*10000 */
#define CRSF_FRAMETYPE_FLIGHT_MODE         0x21U
#define CRSF_DEG_TO_RAD                    0.0174532925f

/* Highest-priority safety net: the instant a CRSF frame shows the arm switch in
 * the disarm position, force motors to idle right here in the UART4 RX ISR -
 * completely independent of whether App_Update()'s main loop is still running.
 * This is REDUNDANT with app.c's own arm-switch handling (still the source of
 * truth for normal operation/telemetry/logging) and only matters if the main
 * loop is stalled and never reaches its own check. NOTE: this cannot help if
 * the CPU is fully deadlocked inside a HIGHER-priority interrupt - UART4_IRQn
 * runs at priority 4, below OTG_FS_IRQn's priority 0, so a stuck USB ISR would
 * still block this from ever running. Constants below intentionally duplicate
 * app.c's APP_CH_ARM_INDEX/APP_ARM_THRESHOLD_US/CRSF-to-us scaling so this
 * safety path has zero dependency on app.c - keep in sync if those ever change. */
#define RECEIVER_ARM_CHANNEL_INDEX     4U
#define RECEIVER_ARM_RAW_THRESHOLD     992U /* raw CRSF value corresponding to ~1500us */
/* Consecutive-low-frame requirement added 2026-08-30 after 3 real in-flight
 * incidents (uncommanded motor cutoff and fall from ~0.8m, twice on one arm
 * switch and once after remapping the arm function to a physically DIFFERENT
 * switch - ruling out a single bad switch) all traced to this exact check:
 * it fired on a single CRC-valid CRSF frame with no debounce of its own,
 * unlike app.c's own arm-switch handling (60ms sustained-low requirement).
 * A lone corrupted-but-CRC-passing frame (rare, but not impossible over a
 * long session) or a one-off decode edge case is enough to trip an
 * undebounced single-frame check; requiring a few CONSECUTIVE low frames
 * rejects that while still being far faster than app.c's 60ms and, critically,
 * still fully independent of it - preserves the "protects against a fully
 * hung main loop" purpose this check exists for. */
#define RECEIVER_ARM_CONSECUTIVE_REQUIRED  3U

/* Per-channel plausibility gate added 2026-08-30 after a real, twice-repeated in-flight
 * incident: raw RC channels (roll/pitch/throttle, not just arm) showed large, sudden
 * excursions - throttle jumping toward its max then instantly to 988 (minimum), roll/pitch
 * swinging hard - with the pilot confirming no corresponding stick input. Root cause
 * theory: Receiver_HandleByte()'s own comment already documents that a dropped/corrupted
 * byte can desync this parser, and each resync attempt is checked by only an 8-bit CRC
 * (a 1-in-256 chance of a garbage frame coincidentally passing) - during a brief link
 * hiccup with several resync attempts in quick succession, hitting that coincidence on a
 * nonsense frame is not implausible, and a single such frame can hand app.c's control loop
 * wildly wrong roll/pitch/throttle/arm values simultaneously. The earlier
 * RECEIVER_ARM_CONSECUTIVE_REQUIRED fix only guards the dedicated arm-channel safety net;
 * it does nothing for roll/pitch/throttle/yaw, which app.c consumes directly with no
 * plausibility check of their own. This gate protects the CONTINUOUS analog channels
 * (roll/pitch/throttle/yaw, indices 0-3): a freshly decoded frame is only committed to
 * g_receiver_state.channels if each of those channels' change from the last ACCEPTED
 * frame is within a value no real stick could physically produce in one frame period - a
 * corrupted frame that fails this is dropped entirely (holding the last good values)
 * rather than partially or fully applied. Not gated on frame rate (unknown/variable here)
 * - deliberately generous so genuine fast stick movement across a real link's
 * frame-to-frame period is never rejected; sized to still catch a near-full-range
 * single-frame swing like the one that caused this incident.
 *
 * DELIBERATELY EXCLUDES switch/discrete channels (arm, flight mode, telarm, indices 4+,
 * see RECEIVER_ANALOG_CHANNEL_COUNT below) - BUG CAUGHT THE SAME NIGHT: the first version
 * of this gate checked ALL 16 channels, including arm, and the aircraft then refused to
 * arm at all (twice). Root cause: unlike an analog stick, a physical switch is DESIGNED
 * to snap instantly between its extreme positions in a single frame - that is not a
 * corruption signature, it's completely normal, and treating it as implausible silently
 * dropped every frame containing the pilot's real arm command. The arm channel already
 * has its own, differently-shaped defense for this exact corruption class
 * (RECEIVER_ARM_CONSECUTIVE_REQUIRED - a few consecutive low readings, not a magnitude
 * check) which remains in effect regardless of this gate. */
#define RECEIVER_MAX_CHANNEL_DELTA_RAW      700U
#define RECEIVER_ANALOG_CHANNEL_COUNT       4U /* roll, pitch, throttle, yaw - indices 0-3 */

typedef struct
{
  uint8_t frame[CRSF_FRAME_MAX_SIZE + 2U];
  uint8_t index;
  uint8_t expected_size;
} receiver_parser_t;

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart6;

static volatile receiver_state_t g_receiver_state;
static receiver_parser_t g_parser;
static uint8_t g_rx_byte;
static uint8_t g_uart6_bridge_byte;

static uint8_t Receiver_Crc8(const uint8_t *data, uint8_t length)
{
  uint8_t crc = 0U;
  uint8_t i;

  while (length-- > 0U)
  {
    crc ^= *data++;
    for (i = 0U; i < 8U; i++)
    {
      if ((crc & 0x80U) != 0U)
      {
        crc = (uint8_t)((crc << 1) ^ 0xD5U);
      }
      else
      {
        crc <<= 1;
      }
    }
  }

  return crc;
}

void Receiver_SendBatteryTelemetry(float voltage_v)
{
  uint8_t frame[3U + CRSF_BATTERY_SENSOR_PAYLOAD_SIZE + 1U]; /* sync+length+type + payload + crc */
  uint16_t voltage_decivolts;

  if (voltage_v < 0.0f)
  {
    voltage_v = 0.0f;
  }
  else if (voltage_v > 6553.5f)
  {
    voltage_v = 6553.5f;
  }
  voltage_decivolts = (uint16_t)((voltage_v * 10.0f) + 0.5f);

  frame[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER;
  frame[1] = (uint8_t)(CRSF_BATTERY_SENSOR_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC);
  frame[2] = CRSF_FRAMETYPE_BATTERY_SENSOR;
  frame[3] = (uint8_t)(voltage_decivolts >> 8);
  frame[4] = (uint8_t)(voltage_decivolts & 0xFFU);
  frame[5] = 0U; /* current hi - no current sensing on this board */
  frame[6] = 0U; /* current lo */
  frame[7] = 0U; /* capacity used [23:16] mAh - no capacity tracking */
  frame[8] = 0U; /* capacity used [15:8] */
  frame[9] = 0U; /* capacity used [7:0] */
  frame[10] = 0U; /* remaining percent - not computed */
  frame[11] = Receiver_Crc8(&frame[2], (uint8_t)(CRSF_BATTERY_SENSOR_PAYLOAD_SIZE + 1U));

  (void)HAL_UART_Transmit(&huart4, frame, sizeof(frame), 10U);
}

void Receiver_SendGpsTelemetry(float lat_deg, float lon_deg, float alt_m,
                               float ground_speed_mps, float heading_deg, uint8_t num_satellites)
{
  uint8_t frame[3U + CRSF_GPS_PAYLOAD_SIZE + 1U];
  int32_t lat_i = (int32_t)((double)lat_deg * 10000000.0);
  int32_t lon_i = (int32_t)((double)lon_deg * 10000000.0);
  uint16_t speed_kmh_x10 = (uint16_t)((ground_speed_mps * 36.0f) + 0.5f); /* m/s -> km/h*10 */
  uint16_t heading_x100;
  int32_t alt_encoded = (int32_t)alt_m + 1000; /* iNav's -1000m offset encoding */

  if (heading_deg < 0.0f)
  {
    heading_deg += 360.0f;
  }
  heading_x100 = (uint16_t)((heading_deg * 100.0f) + 0.5f);

  if (alt_encoded < 0) { alt_encoded = 0; }
  if (alt_encoded > 65535) { alt_encoded = 65535; }

  frame[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER;
  frame[1] = (uint8_t)(CRSF_GPS_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC);
  frame[2] = CRSF_FRAMETYPE_GPS;
  frame[3] = (uint8_t)((uint32_t)lat_i >> 24);
  frame[4] = (uint8_t)((uint32_t)lat_i >> 16);
  frame[5] = (uint8_t)((uint32_t)lat_i >> 8);
  frame[6] = (uint8_t)((uint32_t)lat_i);
  frame[7] = (uint8_t)((uint32_t)lon_i >> 24);
  frame[8] = (uint8_t)((uint32_t)lon_i >> 16);
  frame[9] = (uint8_t)((uint32_t)lon_i >> 8);
  frame[10] = (uint8_t)((uint32_t)lon_i);
  frame[11] = (uint8_t)(speed_kmh_x10 >> 8);
  frame[12] = (uint8_t)(speed_kmh_x10 & 0xFFU);
  frame[13] = (uint8_t)(heading_x100 >> 8);
  frame[14] = (uint8_t)(heading_x100 & 0xFFU);
  frame[15] = (uint8_t)((uint16_t)alt_encoded >> 8);
  frame[16] = (uint8_t)((uint16_t)alt_encoded & 0xFFU);
  frame[17] = num_satellites;
  frame[18] = Receiver_Crc8(&frame[2], (uint8_t)(CRSF_GPS_PAYLOAD_SIZE + 1U));

  (void)HAL_UART_Transmit(&huart4, frame, sizeof(frame), 10U);
}

void Receiver_SendVarioTelemetry(float climb_rate_mps)
{
  uint8_t frame[3U + CRSF_VARIO_PAYLOAD_SIZE + 1U];
  int16_t vspeed_cms = (int16_t)(climb_rate_mps * 100.0f);

  frame[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER;
  frame[1] = (uint8_t)(CRSF_VARIO_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC);
  frame[2] = CRSF_FRAMETYPE_VARIO;
  frame[3] = (uint8_t)(((uint16_t)vspeed_cms) >> 8);
  frame[4] = (uint8_t)(((uint16_t)vspeed_cms) & 0xFFU);
  frame[5] = Receiver_Crc8(&frame[2], (uint8_t)(CRSF_VARIO_PAYLOAD_SIZE + 1U));

  (void)HAL_UART_Transmit(&huart4, frame, sizeof(frame), 10U);
}

void Receiver_SendAttitudeTelemetry(float pitch_deg, float roll_deg, float yaw_deg)
{
  uint8_t frame[3U + CRSF_ATTITUDE_PAYLOAD_SIZE + 1U];
  int16_t pitch_i = (int16_t)(pitch_deg * CRSF_DEG_TO_RAD * 10000.0f);
  int16_t roll_i = (int16_t)(roll_deg * CRSF_DEG_TO_RAD * 10000.0f);
  int16_t yaw_i = (int16_t)(yaw_deg * CRSF_DEG_TO_RAD * 10000.0f);

  frame[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER;
  frame[1] = (uint8_t)(CRSF_ATTITUDE_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC);
  frame[2] = CRSF_FRAMETYPE_ATTITUDE;
  frame[3] = (uint8_t)(((uint16_t)pitch_i) >> 8);
  frame[4] = (uint8_t)(((uint16_t)pitch_i) & 0xFFU);
  frame[5] = (uint8_t)(((uint16_t)roll_i) >> 8);
  frame[6] = (uint8_t)(((uint16_t)roll_i) & 0xFFU);
  frame[7] = (uint8_t)(((uint16_t)yaw_i) >> 8);
  frame[8] = (uint8_t)(((uint16_t)yaw_i) & 0xFFU);
  frame[9] = Receiver_Crc8(&frame[2], (uint8_t)(CRSF_ATTITUDE_PAYLOAD_SIZE + 1U));

  (void)HAL_UART_Transmit(&huart4, frame, sizeof(frame), 10U);
}

void Receiver_SendFlightModeTelemetry(const char *mode_name)
{
  uint8_t frame[3U + 16U + 1U]; /* generous cap for a short mode string + null + crc */
  uint8_t payload_len;
  uint8_t i;

  for (i = 0U; (mode_name[i] != '\0') && (i < 15U); i++)
  {
    frame[3U + i] = (uint8_t)mode_name[i];
  }
  frame[3U + i] = 0U; /* null terminator */
  payload_len = (uint8_t)(i + 1U);

  frame[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER;
  frame[1] = (uint8_t)(payload_len + CRSF_FRAME_LENGTH_TYPE_CRC);
  frame[2] = CRSF_FRAMETYPE_FLIGHT_MODE;
  frame[3U + payload_len] = Receiver_Crc8(&frame[2], (uint8_t)(payload_len + 1U));

  (void)HAL_UART_Transmit(&huart4, frame, (uint16_t)(3U + payload_len + 1U), 10U);
}

static void Receiver_ResetParser(void)
{
  g_parser.index = 0U;
  g_parser.expected_size = 0U;
}

/* Decodes into a caller-provided buffer, NOT directly into g_receiver_state.channels -
 * see RECEIVER_MAX_CHANNEL_DELTA_RAW's comment: the caller must plausibility-check this
 * decoded frame against the last ACCEPTED one before committing it, so a frame that fails
 * that check never partially or fully overwrites the live channel state. */
static void Receiver_DecodeChannels(const uint8_t *payload, uint16_t *out_channels)
{
  out_channels[0] = (uint16_t)((payload[0] | (payload[1] << 8U)) & 0x07FFU);
  out_channels[1] = (uint16_t)(((payload[1] >> 3U) | (payload[2] << 5U)) & 0x07FFU);
  out_channels[2] = (uint16_t)(((payload[2] >> 6U) | (payload[3] << 2U) | (payload[4] << 10U)) & 0x07FFU);
  out_channels[3] = (uint16_t)(((payload[4] >> 1U) | (payload[5] << 7U)) & 0x07FFU);
  out_channels[4] = (uint16_t)(((payload[5] >> 4U) | (payload[6] << 4U)) & 0x07FFU);
  out_channels[5] = (uint16_t)(((payload[6] >> 7U) | (payload[7] << 1U) | (payload[8] << 9U)) & 0x07FFU);
  out_channels[6] = (uint16_t)(((payload[8] >> 2U) | (payload[9] << 6U)) & 0x07FFU);
  out_channels[7] = (uint16_t)(((payload[9] >> 5U) | (payload[10] << 3U)) & 0x07FFU);
  out_channels[8] = (uint16_t)((payload[11] | (payload[12] << 8U)) & 0x07FFU);
  out_channels[9] = (uint16_t)(((payload[12] >> 3U) | (payload[13] << 5U)) & 0x07FFU);
  out_channels[10] = (uint16_t)(((payload[13] >> 6U) | (payload[14] << 2U) | (payload[15] << 10U)) & 0x07FFU);
  out_channels[11] = (uint16_t)(((payload[15] >> 1U) | (payload[16] << 7U)) & 0x07FFU);
  out_channels[12] = (uint16_t)(((payload[16] >> 4U) | (payload[17] << 4U)) & 0x07FFU);
  out_channels[13] = (uint16_t)(((payload[17] >> 7U) | (payload[18] << 1U) | (payload[19] << 9U)) & 0x07FFU);
  out_channels[14] = (uint16_t)(((payload[19] >> 2U) | (payload[20] << 6U)) & 0x07FFU);
  out_channels[15] = (uint16_t)(((payload[20] >> 5U) | (payload[21] << 3U)) & 0x07FFU);
}

static void Receiver_ProcessFrame(void)
{
  uint8_t frame_size;
  uint8_t frame_type;
  uint8_t payload_size;
  uint8_t crc_expected;
  uint8_t crc_computed;
  /* See RECEIVER_ARM_CONSECUTIVE_REQUIRED's comment - runs in the UART4 RX
   * ISR only, so no locking needed for this single-writer counter. */
  static uint8_t arm_low_consecutive_count = 0U;

  frame_size = g_parser.frame[1];
  if ((frame_size < 2U) || (frame_size > CRSF_FRAME_MAX_SIZE))
  {
    return;
  }

  frame_type = g_parser.frame[2];
  payload_size = (uint8_t)(frame_size - 2U);
  crc_expected = g_parser.frame[frame_size + 1U];
  crc_computed = Receiver_Crc8(&g_parser.frame[2], (uint8_t)(frame_size - 1U));

  if (crc_expected != crc_computed)
  {
    g_receiver_state.crc_error_count++;
    return;
  }

  if ((frame_type == CRSF_FRAME_TYPE_RC_CHANNELS) && (payload_size == CRSF_RC_PAYLOAD_SIZE))
  {
    uint16_t decoded_channels[RECEIVER_CHANNEL_COUNT];
    uint8_t plausible = 1U;
    uint8_t ch;

    Receiver_DecodeChannels(&g_parser.frame[3], decoded_channels);

    /* See RECEIVER_MAX_CHANNEL_DELTA_RAW's comment. Skip the check entirely on the
     * very first frame ever accepted (frame_count==0) - there is no prior real
     * frame to compare against, and g_receiver_state.channels still holds
     * Receiver_Init()'s 992-for-everything seed, which a genuine first frame
     * (e.g. throttle at its real low endpoint) could legitimately differ from
     * by more than the threshold. */
    if (g_receiver_state.frame_count != 0U)
    {
      for (ch = 0U; ch < RECEIVER_ANALOG_CHANNEL_COUNT; ch++)
      {
        int32_t delta = (int32_t)decoded_channels[ch] - (int32_t)g_receiver_state.channels[ch];
        if (delta < 0) { delta = -delta; }
        if (delta > (int32_t)RECEIVER_MAX_CHANNEL_DELTA_RAW)
        {
          plausible = 0U;
          break;
        }
      }
    }

    if (plausible == 0U)
    {
      g_receiver_state.implausible_frame_count++;
      return;
    }

    for (ch = 0U; ch < RECEIVER_CHANNEL_COUNT; ch++)
    {
      g_receiver_state.channels[ch] = decoded_channels[ch];
    }
    g_receiver_state.link_active = 1U;
    g_receiver_state.frame_received = 1U;
    g_receiver_state.frame_count++;
    g_receiver_state.last_frame_ms = HAL_GetTick();

    if (g_receiver_state.channels[RECEIVER_ARM_CHANNEL_INDEX] < RECEIVER_ARM_RAW_THRESHOLD)
    {
      arm_low_consecutive_count++;
      if (arm_low_consecutive_count >= RECEIVER_ARM_CONSECUTIVE_REQUIRED)
      {
        Motors_ForceIdleRegistersOnly();
      }
    }
    else
    {
      arm_low_consecutive_count = 0U;
    }
  }
}

static void Receiver_HandleByte(uint8_t byte)
{
  if (g_parser.index == 0U)
  {
    /* A dropped/corrupted byte anywhere upstream can desync this parser -
     * without checking for the real frame-start address here, a single lost
     * byte can leave it misaligned for multiple subsequent frames (each one
     * only self-corrects by chance + a CRC failure), showing up as a
     * "spotty" link that isn't actually an RF problem. Discard anything that
     * isn't a real frame start and keep scanning for it. */
    if (byte != CRSF_ADDRESS_FLIGHT_CONTROLLER)
    {
      return;
    }
    g_parser.frame[g_parser.index++] = byte;
    return;
  }

  if (g_parser.index == 1U)
  {
    if ((byte < 2U) || (byte > CRSF_FRAME_MAX_SIZE))
    {
      g_receiver_state.sync_error_count++;
      Receiver_ResetParser();
      return;
    }

    g_parser.frame[g_parser.index++] = byte;
    g_parser.expected_size = (uint8_t)(byte + 2U);
    return;
  }

  g_parser.frame[g_parser.index++] = byte;

  if (g_parser.index >= g_parser.expected_size)
  {
    Receiver_ProcessFrame();
    Receiver_ResetParser();
  }
}

void Receiver_Init(void)
{
  uint8_t channel_index;
  HAL_StatusTypeDef status4, status6;
  char msg[128];

  memset((void *)&g_receiver_state, 0, sizeof(g_receiver_state));
  for (channel_index = 0U; channel_index < RECEIVER_CHANNEL_COUNT; channel_index++)
  {
    g_receiver_state.channels[channel_index] = 992U;
  }

  Receiver_ResetParser();
  HAL_NVIC_SetPriority(UART4_IRQn, 4U, 0U);
  printf("[Receiver_Init] Starting UART4/UART6 RX interrupts\r\n");
  App_AppendBootLog("[Receiver_Init] Starting UART4/UART6 RX interrupts\r\n");
  
  status4 = HAL_UART_Receive_IT(&huart4, &g_rx_byte, 1U);
  snprintf(msg, sizeof(msg), "[Receiver_Init] UART4 Receive_IT status = %d (0=OK)\r\n", (int)status4);
  printf("%s", msg);
  App_AppendBootLog(msg);
  
  status6 = HAL_UART_Receive_IT(&huart6, &g_uart6_bridge_byte, 1U);
  snprintf(msg, sizeof(msg), "[Receiver_Init] UART6 Receive_IT status = %d (0=OK)\r\n", (int)status6);
  printf("%s", msg);
  App_AppendBootLog(msg);
}

void Receiver_Update(uint32_t now_ms)
{
  __disable_irq();
  now_ms = HAL_GetTick();
  if ((g_receiver_state.link_active != 0U) &&
      ((now_ms - g_receiver_state.last_frame_ms) > CRSF_LINK_TIMEOUT_MS))
  {
    g_receiver_state.link_active = 0U;
  }
  __enable_irq();
}

void Receiver_GetState(receiver_state_t *state)
{
  if (state == NULL)
  {
    return;
  }

  __disable_irq();
  memcpy(state, (const void *)&g_receiver_state, sizeof(*state));
  g_receiver_state.frame_received = 0U;
  __enable_irq();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART4)
  {
    Receiver_HandleByte(g_rx_byte);
    (void)HAL_UART_Receive_IT(&huart4, &g_rx_byte, 1U);
  }
  else if (huart->Instance == USART6)
  {
    Communications_HandleUart6Byte(g_uart6_bridge_byte);
    (void)HAL_UART_Receive_IT(huart, &g_uart6_bridge_byte, 1U);
  }
  else if (huart->Instance == USART3)
  {
    GPS_UartRxCpltCallback();
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART4)
  {
    uint32_t error_code = huart->ErrorCode;

    if ((error_code & HAL_UART_ERROR_ORE) != 0U)
    {
      g_receiver_state.overrun_error_count++;
    }
    if ((error_code & HAL_UART_ERROR_FE) != 0U)
    {
      g_receiver_state.framing_error_count++;
    }
    if ((error_code & HAL_UART_ERROR_NE) != 0U)
    {
      g_receiver_state.noise_error_count++;
    }
    Receiver_ResetParser();
    (void)HAL_UART_AbortReceive(huart);
    (void)HAL_UART_Receive_IT(&huart4, &g_rx_byte, 1U);
  }
  else if (huart->Instance == USART6)
  {
    (void)HAL_UART_AbortReceive(huart);
    (void)HAL_UART_Receive_IT(huart, &g_uart6_bridge_byte, 1U);
  }
  else if (huart->Instance == USART3)
  {
    GPS_UartErrorCallback();
  }
}
