#include "gps.h"

#include <stdio.h>
#include <string.h>

#define GPS_UART_TX_TIMEOUT_MS   50U
#define GPS_ACK_TIMEOUT_MS       300U
#define GPS_FIX_STALE_MS         2000U
#define GPS_COMMS_STALE_MS       3000U
#define GPS_MAX_PAYLOAD          96U

#define GPS_UBX_SYNC1            0xB5U
#define GPS_UBX_SYNC2            0x62U

#define GPS_UBX_CLASS_NAV        0x01U
#define GPS_UBX_CLASS_ACK        0x05U
#define GPS_UBX_CLASS_CFG        0x06U
#define GPS_UBX_ID_NAV_PVT       0x07U
#define GPS_UBX_ID_ACK_NAK       0x00U
#define GPS_UBX_ID_ACK_ACK       0x01U
#define GPS_UBX_ID_CFG_PRT       0x00U
#define GPS_UBX_ID_CFG_MSG       0x01U
#define GPS_UBX_ID_CFG_RATE      0x08U
#define GPS_UBX_ID_CFG_NAV5      0x24U
#define GPS_UBX_ID_CFG_CFG       0x09U
#define GPS_UBX_ID_CFG_RST       0x04U
/* Raise the nav solution rate from the module's 1Hz default - the velocity-brake
 * controller needs fresher GPS velocity than 1Hz to be useful. 5Hz (200ms) is well
 * within the SAM-M10Q's supported single-GNSS nav rate. */
#define GPS_NAV_RATE_HZ          5U
#define GPS_NAV_MEAS_RATE_MS     (1000U / GPS_NAV_RATE_HZ)

typedef enum
{
  GPS_PARSE_SYNC1 = 0,
  GPS_PARSE_SYNC2,
  GPS_PARSE_CLASS,
  GPS_PARSE_ID,
  GPS_PARSE_LEN1,
  GPS_PARSE_LEN2,
  GPS_PARSE_PAYLOAD,
  GPS_PARSE_CK_A,
  GPS_PARSE_CK_B
} gps_parse_state_t;

extern UART_HandleTypeDef huart3;

static uint8_t g_gps_rx_byte;

static gps_parse_state_t g_parse_state = GPS_PARSE_SYNC1;
static uint8_t g_msg_class;
static uint8_t g_msg_id;
static uint16_t g_payload_len;
static uint16_t g_payload_idx;
static uint8_t g_payload[GPS_MAX_PAYLOAD];
static uint8_t g_ck_a;
static uint8_t g_ck_b;
static uint8_t g_ck_a_calc;
static uint8_t g_ck_b_calc;

static volatile uint8_t g_last_ack_valid = 0U;
static volatile uint8_t g_last_ack_ok = 0U;
static volatile uint8_t g_last_ack_class = 0xFFU;
static volatile uint8_t g_last_ack_id = 0xFFU;

static uint8_t g_configured = 0U;
static volatile uint8_t g_last_prt_acked = 0U;
static volatile uint8_t g_last_msg_acked = 0U;
static volatile uint8_t g_last_rate_acked = 0U;
static volatile uint8_t g_last_nav5_acked = 0U;

static volatile uint8_t g_fix_type = 0U;
static volatile uint8_t g_num_sv = 0U;
static volatile float g_lat_deg = 0.0f;
static volatile float g_lon_deg = 0.0f;
static volatile float g_alt_m = 0.0f;
static volatile float g_alt_ellipsoid_m = 0.0f;
static volatile float g_h_acc_m = 0.0f;
static volatile float g_v_acc_m = 0.0f;
static volatile float g_s_acc_mps = 0.0f;
static volatile float g_ground_speed_mps = 0.0f;
static volatile float g_course_deg = 0.0f;
static volatile float g_head_acc_deg = 0.0f;
static volatile float g_pdop = 99.99f; /* worst-case-looking default until a real PVT is decoded */
static volatile uint8_t g_gnss_fix_ok = 0U;
static volatile float g_vel_n_mps = 0.0f;
static volatile float g_vel_e_mps = 0.0f;
static volatile float g_vel_d_mps = 0.0f;
static volatile uint32_t g_last_itow_ms = 0U;
static volatile uint32_t g_last_pvt_host_ms = 0U;
static volatile uint32_t g_pvt_update_count = 0U;
static volatile uint32_t g_last_fix_ms = 0U;
static volatile uint8_t g_have_fix_ever = 0U;
static volatile uint32_t g_last_rx_ms = 0U;
static volatile uint8_t g_have_rx_ever = 0U;

static void GPS_ResetParser(void)
{
  g_parse_state = GPS_PARSE_SYNC1;
}

static void GPS_DecodePvt(const uint8_t *payload)
{
  int32_t lon_raw;
  int32_t lat_raw;
  int32_t height_ellipsoid_mm;
  int32_t height_msl_mm;
  uint32_t itow_ms;
  uint32_t h_acc_mm;
  uint32_t v_acc_mm;
  int32_t vel_n_mm_s;
  int32_t vel_e_mm_s;
  int32_t vel_d_mm_s;
  int32_t gspeed_mm_s;
  int32_t head_mot_raw;
  uint32_t s_acc_mm_s;
  uint32_t head_acc_raw;
  uint16_t pdop_raw;

  itow_ms = (uint32_t)payload[0] | ((uint32_t)payload[1] << 8) |
            ((uint32_t)payload[2] << 16) | ((uint32_t)payload[3] << 24);

  lon_raw = (int32_t)((uint32_t)payload[24] | ((uint32_t)payload[25] << 8) |
                       ((uint32_t)payload[26] << 16) | ((uint32_t)payload[27] << 24));
  lat_raw = (int32_t)((uint32_t)payload[28] | ((uint32_t)payload[29] << 8) |
                       ((uint32_t)payload[30] << 16) | ((uint32_t)payload[31] << 24));
  /* UBX-NAV-PVT payload layout has TWO height fields back to back: height-above-
   * ellipsoid at offset 32, then height-above-mean-sea-level (hMSL) at offset 36 -
   * see GPS_GetAltitudeM()'s comment for why this project's "altitude" getter has
   * always been the hMSL one (offset 36), not this ellipsoid one. Both are cheap to
   * keep now that this function already walks past both fields. */
  height_ellipsoid_mm = (int32_t)((uint32_t)payload[32] | ((uint32_t)payload[33] << 8) |
                                  ((uint32_t)payload[34] << 16) | ((uint32_t)payload[35] << 24));
  height_msl_mm = (int32_t)((uint32_t)payload[36] | ((uint32_t)payload[37] << 8) |
                            ((uint32_t)payload[38] << 16) | ((uint32_t)payload[39] << 24));
  h_acc_mm = (uint32_t)payload[40] | ((uint32_t)payload[41] << 8) |
             ((uint32_t)payload[42] << 16) | ((uint32_t)payload[43] << 24);
  v_acc_mm = (uint32_t)payload[44] | ((uint32_t)payload[45] << 8) |
             ((uint32_t)payload[46] << 16) | ((uint32_t)payload[47] << 24);
  vel_n_mm_s = (int32_t)((uint32_t)payload[48] | ((uint32_t)payload[49] << 8) |
                         ((uint32_t)payload[50] << 16) | ((uint32_t)payload[51] << 24));
  vel_e_mm_s = (int32_t)((uint32_t)payload[52] | ((uint32_t)payload[53] << 8) |
                         ((uint32_t)payload[54] << 16) | ((uint32_t)payload[55] << 24));
  vel_d_mm_s = (int32_t)((uint32_t)payload[56] | ((uint32_t)payload[57] << 8) |
                         ((uint32_t)payload[58] << 16) | ((uint32_t)payload[59] << 24));
  gspeed_mm_s = (int32_t)((uint32_t)payload[60] | ((uint32_t)payload[61] << 8) |
                          ((uint32_t)payload[62] << 16) | ((uint32_t)payload[63] << 24));
  head_mot_raw = (int32_t)((uint32_t)payload[64] | ((uint32_t)payload[65] << 8) |
                           ((uint32_t)payload[66] << 16) | ((uint32_t)payload[67] << 24));
  s_acc_mm_s = (uint32_t)payload[68] | ((uint32_t)payload[69] << 8) |
               ((uint32_t)payload[70] << 16) | ((uint32_t)payload[71] << 24);
  /* headAcc (offset 72, U4, same 1e-5 deg scaling as headMot) - accuracy estimate for
   * the course-over-ground value above. Was never parsed before, so headMot had no
   * paired accuracy to gate trust on, unlike every other value/accuracy pair this
   * driver exposes (hAcc, vAcc, sAcc). */
  head_acc_raw = (uint32_t)payload[72] | ((uint32_t)payload[73] << 8) |
                 ((uint32_t)payload[74] << 16) | ((uint32_t)payload[75] << 24);
  /* pDOP (offset 76, U2, scale 0.01) - position dilution of precision, a pure
   * satellite-geometry quality metric independent of the receiver's own hAcc error
   * estimate. Standard practice to gate on both together since hAcc can be
   * overconfident in poor geometry. Was never parsed before. */
  pdop_raw = (uint16_t)((uint16_t)payload[76] | ((uint16_t)payload[77] << 8));

  g_fix_type = payload[20];
  /* flags (offset 21), bit 0 = gnssFixOK - the receiver's own authoritative "is this
   * fix actually good" confidence flag, distinct from (and more reliable than)
   * fixType alone: fixType can report a 3D fix during a marginal/transient solution
   * that the receiver itself doesn't yet trust. Was never checked anywhere before. */
  g_gnss_fix_ok = (uint8_t)(payload[21] & 0x01U);
  g_num_sv = payload[23];
  g_lat_deg = (float)lat_raw * 1.0e-7f;
  g_lon_deg = (float)lon_raw * 1.0e-7f;
  g_alt_ellipsoid_m = (float)height_ellipsoid_mm * 0.001f;
  g_alt_m = (float)height_msl_mm * 0.001f;
  g_h_acc_m = (float)h_acc_mm * 0.001f;
  g_v_acc_m = (float)v_acc_mm * 0.001f;
  g_s_acc_mps = (float)s_acc_mm_s * 0.001f;
  g_vel_n_mps = (float)vel_n_mm_s * 0.001f;
  g_vel_e_mps = (float)vel_e_mm_s * 0.001f;
  g_vel_d_mps = (float)vel_d_mm_s * 0.001f;
  g_ground_speed_mps = (float)gspeed_mm_s * 0.001f;
  /* headMot is degrees * 1e-5, 0..360, already clockwise-from-north (compass convention). */
  g_course_deg = (float)head_mot_raw * 1.0e-5f;
  g_head_acc_deg = (float)head_acc_raw * 1.0e-5f;
  g_pdop = (float)pdop_raw * 0.01f;
  g_last_itow_ms = itow_ms;
  g_last_pvt_host_ms = HAL_GetTick();
  g_pvt_update_count++;
  g_last_fix_ms = HAL_GetTick();
  g_have_fix_ever = 1U;
}

static void GPS_ProcessFrame(void)
{
  g_last_rx_ms = HAL_GetTick();
  g_have_rx_ever = 1U;

  if ((g_msg_class == GPS_UBX_CLASS_ACK) && (g_payload_len == 2U))
  {
    g_last_ack_class = g_payload[0];
    g_last_ack_id = g_payload[1];
    g_last_ack_ok = (g_msg_id == GPS_UBX_ID_ACK_ACK) ? 1U : 0U;
    g_last_ack_valid = 1U;
  }
  else if ((g_msg_class == GPS_UBX_CLASS_NAV) && (g_msg_id == GPS_UBX_ID_NAV_PVT) &&
           (g_payload_len >= 40U))
  {
    GPS_DecodePvt(g_payload);
  }
}

/* Fletcher-8 UBX frame parser, fed one byte at a time from either the RX ISR
 * (continuous NAV-PVT stream) or the blocking ack-wait poll during GPS_Init(). */
static void GPS_HandleByte(uint8_t byte)
{
  switch (g_parse_state)
  {
    case GPS_PARSE_SYNC1:
      if (byte == GPS_UBX_SYNC1)
      {
        g_parse_state = GPS_PARSE_SYNC2;
      }
      break;

    case GPS_PARSE_SYNC2:
      g_parse_state = (byte == GPS_UBX_SYNC2) ? GPS_PARSE_CLASS : GPS_PARSE_SYNC1;
      break;

    case GPS_PARSE_CLASS:
      g_msg_class = byte;
      g_ck_a_calc = byte;
      g_ck_b_calc = byte;
      g_parse_state = GPS_PARSE_ID;
      break;

    case GPS_PARSE_ID:
      g_msg_id = byte;
      g_ck_a_calc = (uint8_t)(g_ck_a_calc + byte);
      g_ck_b_calc = (uint8_t)(g_ck_b_calc + g_ck_a_calc);
      g_parse_state = GPS_PARSE_LEN1;
      break;

    case GPS_PARSE_LEN1:
      g_payload_len = byte;
      g_ck_a_calc = (uint8_t)(g_ck_a_calc + byte);
      g_ck_b_calc = (uint8_t)(g_ck_b_calc + g_ck_a_calc);
      g_parse_state = GPS_PARSE_LEN2;
      break;

    case GPS_PARSE_LEN2:
      g_payload_len |= (uint16_t)((uint16_t)byte << 8);
      g_ck_a_calc = (uint8_t)(g_ck_a_calc + byte);
      g_ck_b_calc = (uint8_t)(g_ck_b_calc + g_ck_a_calc);
      g_payload_idx = 0U;
      if (g_payload_len > sizeof(g_payload))
      {
        GPS_ResetParser();
      }
      else
      {
        g_parse_state = (g_payload_len == 0U) ? GPS_PARSE_CK_A : GPS_PARSE_PAYLOAD;
      }
      break;

    case GPS_PARSE_PAYLOAD:
      g_payload[g_payload_idx++] = byte;
      g_ck_a_calc = (uint8_t)(g_ck_a_calc + byte);
      g_ck_b_calc = (uint8_t)(g_ck_b_calc + g_ck_a_calc);
      if (g_payload_idx >= g_payload_len)
      {
        g_parse_state = GPS_PARSE_CK_A;
      }
      break;

    case GPS_PARSE_CK_A:
      g_ck_a = byte;
      g_parse_state = GPS_PARSE_CK_B;
      break;

    case GPS_PARSE_CK_B:
    default:
      g_ck_b = byte;
      if ((g_ck_a == g_ck_a_calc) && (g_ck_b == g_ck_b_calc))
      {
        GPS_ProcessFrame();
      }
      GPS_ResetParser();
      break;
  }
}

static void GPS_SendUbx(uint8_t msg_class, uint8_t msg_id, const uint8_t *payload, uint16_t len)
{
  uint8_t header[6];
  uint8_t ck_a = 0U;
  uint8_t ck_b = 0U;
  uint16_t i;

  header[0] = GPS_UBX_SYNC1;
  header[1] = GPS_UBX_SYNC2;
  header[2] = msg_class;
  header[3] = msg_id;
  header[4] = (uint8_t)(len & 0xFFU);
  header[5] = (uint8_t)((len >> 8) & 0xFFU);

  for (i = 2U; i < 6U; i++)
  {
    ck_a = (uint8_t)(ck_a + header[i]);
    ck_b = (uint8_t)(ck_b + ck_a);
  }
  for (i = 0U; i < len; i++)
  {
    ck_a = (uint8_t)(ck_a + payload[i]);
    ck_b = (uint8_t)(ck_b + ck_a);
  }

  (void)HAL_UART_Transmit(&huart3, header, sizeof(header), GPS_UART_TX_TIMEOUT_MS);
  if (len > 0U)
  {
    (void)HAL_UART_Transmit(&huart3, (uint8_t *)payload, len, GPS_UART_TX_TIMEOUT_MS);
  }
  (void)HAL_UART_Transmit(&huart3, &ck_a, 1U, GPS_UART_TX_TIMEOUT_MS);
  (void)HAL_UART_Transmit(&huart3, &ck_b, 1U, GPS_UART_TX_TIMEOUT_MS);
}

/* Blocking poll for UBX-ACK-ACK/NAK matching msg_class/msg_id - only used during
 * the one-time GPS_Init() handshake, before RX-ISR mode is armed. */
static uint8_t GPS_WaitAck(uint8_t msg_class, uint8_t msg_id, uint32_t timeout_ms)
{
  uint32_t start_ms = HAL_GetTick();
  uint8_t byte;

  g_last_ack_valid = 0U;
  GPS_ResetParser();

  while ((HAL_GetTick() - start_ms) < timeout_ms)
  {
    if (HAL_UART_Receive(&huart3, &byte, 1U, 20U) == HAL_OK)
    {
      GPS_HandleByte(byte);
      if ((g_last_ack_valid != 0U) && (g_last_ack_class == msg_class) && (g_last_ack_id == msg_id))
      {
        return g_last_ack_ok;
      }
    }
  }

  return 0U;
}

HAL_StatusTypeDef GPS_Init(void)
{
  uint8_t cfg_prt_payload[20];
  uint8_t cfg_msg_payload[8];
  uint8_t cfg_rate_payload[6];
  uint8_t cfg_nav5_payload[36];
  uint8_t cfg_cfg_payload[13];
  uint8_t prt_acked;
  uint8_t msg_acked;
  uint8_t rate_acked;
  uint8_t nav5_acked;

  g_configured = 0U;
  g_have_fix_ever = 0U;
  g_fix_type = 0U;
  g_num_sv = 0U;

  /* A prior failed attempt leaves HAL_UART_Receive_IT() armed (RxState stays
   * BUSY_RX) - without aborting it first, GPS_WaitAck()'s blocking
   * HAL_UART_Receive() below returns HAL_BUSY immediately every time and never
   * actually reads a byte, so retries could never succeed even once the module
   * is ready. Must abort before re-arming. */
  (void)HAL_UART_AbortReceive(&huart3);
  GPS_ResetParser();

  /* UBX-CFG-PRT: UART1 (module-internal), 8N1, baud unchanged (115200), UBX+NMEA
   * in, UBX-only out. */
  memset(cfg_prt_payload, 0, sizeof(cfg_prt_payload));
  cfg_prt_payload[0] = 1U;                 /* portID = UART1 */
  cfg_prt_payload[4] = 0xD0U;              /* mode = 0x000008D0 (8N1), LE */
  cfg_prt_payload[5] = 0x08U;
  cfg_prt_payload[8] = 0x00U;               /* baudRate = 115200, LE */
  cfg_prt_payload[9] = 0xC2U;
  cfg_prt_payload[10] = 0x01U;
  cfg_prt_payload[12] = 0x03U;               /* inProtoMask = UBX+NMEA */
  cfg_prt_payload[14] = 0x01U;               /* outProtoMask = UBX only */
  GPS_SendUbx(GPS_UBX_CLASS_CFG, GPS_UBX_ID_CFG_PRT, cfg_prt_payload, sizeof(cfg_prt_payload));
  prt_acked = GPS_WaitAck(GPS_UBX_CLASS_CFG, GPS_UBX_ID_CFG_PRT, GPS_ACK_TIMEOUT_MS);

  /* UBX-CFG-MSG: enable NAV-PVT at 1 message/fix on UART1. */
  memset(cfg_msg_payload, 0, sizeof(cfg_msg_payload));
  cfg_msg_payload[0] = GPS_UBX_CLASS_NAV;
  cfg_msg_payload[1] = GPS_UBX_ID_NAV_PVT;
  cfg_msg_payload[3] = 1U;                   /* rate[UART1] = 1 */
  GPS_SendUbx(GPS_UBX_CLASS_CFG, GPS_UBX_ID_CFG_MSG, cfg_msg_payload, sizeof(cfg_msg_payload));
  msg_acked = GPS_WaitAck(GPS_UBX_CLASS_CFG, GPS_UBX_ID_CFG_MSG, GPS_ACK_TIMEOUT_MS);

  /* UBX-CFG-RATE: raise the nav solution rate (module default is 1Hz) so the
   * velocity-brake controller sees fresher velocity updates. navRate=1 means
   * one nav solution per measRate period; timeRef=1 selects GPS time. */
  memset(cfg_rate_payload, 0, sizeof(cfg_rate_payload));
  cfg_rate_payload[0] = (uint8_t)(GPS_NAV_MEAS_RATE_MS & 0xFFU);
  cfg_rate_payload[1] = (uint8_t)((GPS_NAV_MEAS_RATE_MS >> 8) & 0xFFU);
  cfg_rate_payload[2] = 1U; /* navRate */
  cfg_rate_payload[4] = 1U; /* timeRef = GPS time */
  GPS_SendUbx(GPS_UBX_CLASS_CFG, GPS_UBX_ID_CFG_RATE, cfg_rate_payload, sizeof(cfg_rate_payload));
  rate_acked = GPS_WaitAck(GPS_UBX_CLASS_CFG, GPS_UBX_ID_CFG_RATE, GPS_ACK_TIMEOUT_MS);

  /* UBX-CFG-NAV5: set dynamic platform model to Airborne <1g (found 2026-09-04,
   * from analyzing an entire prior flight-test session's captured telemetry - see
   * kh7_gps_data_and_poshold_design memory). This was never configured before,
   * so the module ran its factory-default dynamic model (Portable on u-blox
   * chips), which applies "static hold" position clamping at low speed to
   * suppress jitter for pedestrian-style use - it freezes the reported lat/lon
   * bit-for-bit while the receiver judges itself stationary/slow. Cross-
   * referencing every captured flight this session found GPS position frozen
   * for 51.6% of total flight time with ZERO correlation to motor PWM (ruling
   * out motor-current interference), clustered right at liftoff/low-speed - the
   * exact signature of static hold, and exactly poshold's own operating
   * envelope, meaning it could report a perfectly-tracking hold while the
   * aircraft was actually free to drift with the position feed blind to it.
   * Only the dyn field is masked (bit0) - everything else (fixMode, static-hold
   * threshold/distance, etc.) is left at the receiver's own defaults, since
   * dynModel=6 already disables static hold outright for this platform class. */
  /* Retried up to 3x (2026-09-04): a single lost ACK on this send used to
   * leave the receiver silently stuck in factory static-hold for the WHOLE
   * flight session, with no other symptom until someone forensically compared
   * GPS lat/lon against known real motion - the exact bug this command exists
   * to fix, reappearing invisibly whenever the one UBX round-trip happened to
   * drop. This module already has a known history of occasionally not ACKing
   * a config message it otherwise applies fine (see CFG-PRT/CFG-RATE), so one
   * attempt was never good enough for a setting this consequential. */
  memset(cfg_nav5_payload, 0, sizeof(cfg_nav5_payload));
  cfg_nav5_payload[0] = 0x01U;    /* mask bit0 = apply dyn (dynModel) only */
  cfg_nav5_payload[1] = 0x00U;
  cfg_nav5_payload[2] = 6U;       /* dynModel = 6 (Airborne <1g) */
  nav5_acked = 0U;
  for (uint8_t nav5_attempt = 0U; (nav5_attempt < 3U) && (nav5_acked == 0U); nav5_attempt++)
  {
    GPS_SendUbx(GPS_UBX_CLASS_CFG, GPS_UBX_ID_CFG_NAV5, cfg_nav5_payload, sizeof(cfg_nav5_payload));
    nav5_acked = GPS_WaitAck(GPS_UBX_CLASS_CFG, GPS_UBX_ID_CFG_NAV5, GPS_ACK_TIMEOUT_MS);
  }

  /* UBX-CFG-CFG: save to BBR (no Flash/EEPROM on this module). Best-effort,
   * doesn't gate g_configured since the module works fine without it applied. */
  memset(cfg_cfg_payload, 0, sizeof(cfg_cfg_payload));
  cfg_cfg_payload[4] = 0xFFU;
  cfg_cfg_payload[5] = 0xFFU;
  cfg_cfg_payload[6] = 0xFFU;
  cfg_cfg_payload[7] = 0xFFU;
  cfg_cfg_payload[12] = 0x01U;                /* deviceMask = BBR only */
  GPS_SendUbx(GPS_UBX_CLASS_CFG, GPS_UBX_ID_CFG_CFG, cfg_cfg_payload, sizeof(cfg_cfg_payload));
  (void)GPS_WaitAck(GPS_UBX_CLASS_CFG, GPS_UBX_ID_CFG_CFG, GPS_ACK_TIMEOUT_MS);

  GPS_ResetParser();
  (void)HAL_UART_Receive_IT(&huart3, &g_gps_rx_byte, 1U);

  g_last_prt_acked = prt_acked;
  g_last_msg_acked = msg_acked;
  g_last_rate_acked = rate_acked;
  g_last_nav5_acked = nav5_acked;
  /* CFG-RATE failing is not fatal (module still works at its default 1Hz) - don't
   * gate g_configured on it, just report it via GPS_GetLastRateAcked() for diagnostics.
   * CFG-PRT given the same treatment 2026-08-21 after the HGLRC M100 Pro (a newer
   * u-blox-M10-class module, replacing the SoloGood M10-180C this payload was
   * originally written against) never ACKed it while still correctly ACKing
   * CFG-MSG on the same link - a firmware quirk on this specific module, not a
   * comms failure (round-trip UBX ACK/NACK demonstrably works via CFG-MSG, and
   * "SD RBLOCK"/GPS_ScanBaud() confirms real UBX frames at 115200, this module's
   * documented default - so the port is very likely already exactly what CFG-PRT
   * was asking for, which is plausibly why it saw no state change to ACK). Still
   * sent every init for modules that DO need/honor it; just no longer required
   * for g_configured, matching the CFG-RATE precedent above. CFG-NAV5 (dynamic
   * model) gets the identical treatment for the identical reason - it's a real
   * improvement when it lands, but the module still flies (just with static-
   * hold's low-speed position clamp back in play) if this specific module ever
   * declines to ACK it the way CFG-PRT already does. */
  g_configured = (msg_acked != 0U);
  return (g_configured != 0U) ? HAL_OK : HAL_ERROR;
}

#define GPS_SCAN_CANDIDATE_COUNT 6U
static const uint32_t GPS_SCAN_BAUD_CANDIDATES[GPS_SCAN_CANDIDATE_COUNT] =
{
  4800U, 9600U, 19200U, 38400U, 57600U, 115200U
};

/* Broadcasts a UBX-CFG-CFG "clear all sections to firmware default, load
 * defaults into active config" + UBX-CFG-RST "cold start" at every candidate
 * baud in turn, since we don't know (and this may be called precisely
 * because we can't determine) which baud the module is currently listening
 * at. No way to confirm the module actually received any of this without a
 * working RX path - follow up with GPS_ScanBaud()/"GPS SCAN" afterward and
 * judge success by whether valid frames start showing up. This module has no
 * Flash/EEPROM (deviceMask targets BBR only), and BBR itself is very likely
 * not battery-backed on a board this size, so a plain power-cycle achieves
 * the same reset - this exists for the case comms can be established but a
 * power-cycle isn't convenient/possible right now. */
void GPS_FactoryReset(void)
{
  uint8_t cfg_cfg_payload[13];
  uint8_t cfg_rst_payload[4];
  uint16_t i;

  (void)HAL_UART_AbortReceive(&huart3);

  memset(cfg_cfg_payload, 0, sizeof(cfg_cfg_payload));
  cfg_cfg_payload[0] = 0xFFU;                 /* clearMask = all sections, LE */
  cfg_cfg_payload[1] = 0xFFU;
  cfg_cfg_payload[2] = 0xFFU;
  cfg_cfg_payload[3] = 0xFFU;
  cfg_cfg_payload[8] = 0xFFU;                 /* loadMask = all sections, LE */
  cfg_cfg_payload[9] = 0xFFU;
  cfg_cfg_payload[10] = 0xFFU;
  cfg_cfg_payload[11] = 0xFFU;
  cfg_cfg_payload[12] = 0x01U;                /* deviceMask = BBR only */

  memset(cfg_rst_payload, 0, sizeof(cfg_rst_payload));
  cfg_rst_payload[0] = 0xFFU;                 /* navBbrMask = 0xFFFF, cold start, LE */
  cfg_rst_payload[1] = 0xFFU;
  cfg_rst_payload[2] = 0x01U;                 /* resetMode = controlled software reset */

  for (i = 0U; i < GPS_SCAN_CANDIDATE_COUNT; i++)
  {
    uint32_t baud = GPS_SCAN_BAUD_CANDIDATES[i];

    huart3.Init.BaudRate = baud;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
      printf("GPS_FACTORY_RESET[baud=%lu ERROR reinit_failed]\r\n", (unsigned long)baud);
      continue;
    }
    GPS_SendUbx(GPS_UBX_CLASS_CFG, GPS_UBX_ID_CFG_CFG, cfg_cfg_payload, sizeof(cfg_cfg_payload));
    HAL_Delay(50U);
    GPS_SendUbx(GPS_UBX_CLASS_CFG, GPS_UBX_ID_CFG_RST, cfg_rst_payload, sizeof(cfg_rst_payload));
    HAL_Delay(50U);
    printf("GPS_FACTORY_RESET[baud=%lu sent]\r\n", (unsigned long)baud);
  }

  huart3.Init.BaudRate = 115200U;
  (void)HAL_UART_Init(&huart3);
  g_configured = 0U;
  GPS_ResetParser();
  (void)HAL_UART_Receive_IT(&huart3, &g_gps_rx_byte, 1U);
  printf("GPS_FACTORY_RESET[DONE re-run GPS SCAN to check for a response]\r\n");
}

#define GPS_SCAN_WINDOW_MS  500U
#define GPS_SCAN_BUF_SIZE   300U

/* Independent of the live UBX parser/state (GPS_HandleByte/GPS_ProcessFrame)
 * so probing at the wrong baud can never corrupt real GPS_Get*() state -
 * bytes received during a scan are only ever looked at here, never fed to
 * the real parser. */
static uint8_t GPS_ScanLooksLikeUbxFrame(const uint8_t *buf, uint16_t count, uint16_t start_idx)
{
  uint16_t payload_len;
  uint16_t frame_end;
  uint8_t ck_a = 0U;
  uint8_t ck_b = 0U;
  uint16_t k;

  if ((uint32_t)start_idx + 8U > (uint32_t)count)
  {
    return 0U;
  }
  if (buf[start_idx + 1U] != GPS_UBX_SYNC2)
  {
    return 0U;
  }
  payload_len = (uint16_t)((uint16_t)buf[start_idx + 4U] | ((uint16_t)buf[start_idx + 5U] << 8));
  if (payload_len > GPS_MAX_PAYLOAD)
  {
    return 0U;
  }
  frame_end = (uint16_t)(start_idx + 6U + payload_len + 2U);
  if (frame_end > count)
  {
    return 0U;
  }
  for (k = (uint16_t)(start_idx + 2U); k < (uint16_t)(start_idx + 6U + payload_len); k++)
  {
    ck_a = (uint8_t)(ck_a + buf[k]);
    ck_b = (uint8_t)(ck_b + ck_a);
  }
  return (uint8_t)((ck_a == buf[start_idx + 6U + payload_len]) &&
                    (ck_b == buf[start_idx + 7U + payload_len]));
}

/* Looks for '$'...<hex><hex>\r\n within a short window - a checksummed NMEA
 * sentence, which a fresh/never-configured module may emit by default even
 * before GPS_Init() has told it to switch to UBX-only output. */
static uint8_t GPS_ScanLooksLikeNmeaSentence(const uint8_t *buf, uint16_t count, uint16_t start_idx)
{
  uint16_t j;
  uint16_t max_j = (uint16_t)(start_idx + 90U);

  if (max_j > count)
  {
    max_j = count;
  }
  for (j = (uint16_t)(start_idx + 1U); (uint32_t)(j + 4U) < (uint32_t)max_j; j++)
  {
    if (buf[j] == '*')
    {
      uint8_t c1 = buf[j + 1U];
      uint8_t c2 = buf[j + 2U];
      uint8_t is_hex1 = (uint8_t)(((c1 >= '0') && (c1 <= '9')) || ((c1 >= 'A') && (c1 <= 'F')));
      uint8_t is_hex2 = (uint8_t)(((c2 >= '0') && (c2 <= '9')) || ((c2 >= 'A') && (c2 <= 'F')));
      return (uint8_t)(is_hex1 && is_hex2 && (buf[j + 3U] == '\r') && (buf[j + 4U] == '\n'));
    }
    if ((buf[j] < 0x20U) || (buf[j] > 0x7EU))
    {
      return 0U;
    }
  }
  return 0U;
}

/* Manual bench diagnostic (see "GPS SCAN" command) - tries each candidate
 * baud in turn, listens for a short window, and reports which one(s) produced
 * a valid checksummed frame. Leaves huart3 configured at the best candidate
 * found (or 115200, this module's factory default, if none matched) and
 * leaves g_configured=0 so the normal GPS_Init() retry path re-attempts the
 * real handshake at whatever baud this settles on. Blocks for up to
 * GPS_SCAN_CANDIDATE_COUNT*GPS_SCAN_WINDOW_MS (~3s) - caller must only invoke
 * this while disarmed. */
void GPS_ScanBaud(void)
{
  static uint8_t buf[GPS_SCAN_BUF_SIZE];
  uint32_t found_baud = 0U;
  uint16_t i;

  (void)HAL_UART_AbortReceive(&huart3);

  for (i = 0U; i < GPS_SCAN_CANDIDATE_COUNT; i++)
  {
    uint32_t baud = GPS_SCAN_BAUD_CANDIDATES[i];
    uint32_t start_ms;
    uint16_t count = 0U;
    uint16_t ubx_frames = 0U;
    uint16_t nmea_sentences = 0U;
    uint16_t j;

    huart3.Init.BaudRate = baud;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
      printf("GPS_SCAN[baud=%lu ERROR reinit_failed]\r\n", (unsigned long)baud);
      continue;
    }

    start_ms = HAL_GetTick();
    while ((HAL_GetTick() - start_ms) < GPS_SCAN_WINDOW_MS)
    {
      uint8_t byte;
      if (HAL_UART_Receive(&huart3, &byte, 1U, 20U) == HAL_OK)
      {
        if (count < GPS_SCAN_BUF_SIZE)
        {
          buf[count] = byte;
          count++;
        }
      }
    }

    for (j = 0U; j < count; j++)
    {
      if ((buf[j] == GPS_UBX_SYNC1) && (GPS_ScanLooksLikeUbxFrame(buf, count, j) != 0U))
      {
        ubx_frames++;
      }
      if ((buf[j] == '$') && (GPS_ScanLooksLikeNmeaSentence(buf, count, j) != 0U))
      {
        nmea_sentences++;
      }
    }

    printf("GPS_SCAN[baud=%lu bytes=%u ubx_frames=%u nmea_sentences=%u]\r\n",
           (unsigned long)baud, (unsigned int)count, (unsigned int)ubx_frames,
           (unsigned int)nmea_sentences);

    if (((ubx_frames > 0U) || (nmea_sentences > 0U)) && (found_baud == 0U))
    {
      found_baud = baud;
    }
  }

  huart3.Init.BaudRate = (found_baud != 0U) ? found_baud : 115200U;
  (void)HAL_UART_Init(&huart3);
  if (found_baud != 0U)
  {
    printf("GPS_SCAN[DONE best_baud=%lu]\r\n", (unsigned long)found_baud);
  }
  else
  {
    printf("GPS_SCAN[DONE no_valid_baud_found fell_back=115200]\r\n");
  }

  g_configured = 0U;
  GPS_ResetParser();
  (void)HAL_UART_Receive_IT(&huart3, &g_gps_rx_byte, 1U);
}

uint8_t GPS_IsConfigured(void)
{
  return g_configured;
}

uint8_t GPS_GetLastPrtAcked(void)
{
  return g_last_prt_acked;
}

uint8_t GPS_GetLastMsgAcked(void)
{
  return g_last_msg_acked;
}

uint8_t GPS_GetLastRateAcked(void)
{
  return g_last_rate_acked;
}

uint8_t GPS_GetLastNav5Acked(void)
{
  return g_last_nav5_acked;
}

uint8_t GPS_IsHealthy(void)
{
  uint32_t age_ms;

  if ((g_configured == 0U) || (g_have_rx_ever == 0U))
  {
    return 0U;
  }
  age_ms = HAL_GetTick() - g_last_rx_ms;
  return (uint8_t)(age_ms < GPS_COMMS_STALE_MS);
}

uint8_t GPS_HasFix(void)
{
  return (uint8_t)((g_have_fix_ever != 0U) && (g_fix_type >= 2U) &&
                    (GPS_GetLastFixAgeMs() < GPS_FIX_STALE_MS));
}

uint8_t GPS_GetFixType(void)
{
  return g_fix_type;
}

uint8_t GPS_GetNumSatellites(void)
{
  return g_num_sv;
}

float GPS_GetLatitudeDeg(void)
{
  return g_lat_deg;
}

float GPS_GetLongitudeDeg(void)
{
  return g_lon_deg;
}

float GPS_GetAltitudeM(void)
{
  return g_alt_m;
}

float GPS_GetAltitudeEllipsoidM(void)
{
  return g_alt_ellipsoid_m;
}

uint8_t GPS_GetGnssFixOk(void)
{
  return g_gnss_fix_ok;
}

float GPS_GetPdop(void)
{
  return g_pdop;
}

float GPS_GetHeadingAccuracyDeg(void)
{
  return g_head_acc_deg;
}

float GPS_GetHorizontalAccuracyM(void)
{
  return g_h_acc_m;
}

float GPS_GetVerticalAccuracyM(void)
{
  return g_v_acc_m;
}

float GPS_GetSpeedAccuracyMps(void)
{
  return g_s_acc_mps;
}

float GPS_GetGroundSpeedMps(void)
{
  return g_ground_speed_mps;
}

float GPS_GetCourseDeg(void)
{
  return g_course_deg;
}

float GPS_GetVelNorthMps(void)
{
  return g_vel_n_mps;
}

float GPS_GetVelEastMps(void)
{
  return g_vel_e_mps;
}

float GPS_GetVelDownMps(void)
{
  return g_vel_d_mps;
}

uint32_t GPS_GetLastITowMs(void)
{
  return g_last_itow_ms;
}

uint32_t GPS_GetLastPvtHostMs(void)
{
  return g_last_pvt_host_ms;
}

uint32_t GPS_GetPvtUpdateCount(void)
{
  return g_pvt_update_count;
}

uint32_t GPS_GetLastFixAgeMs(void)
{
  if (g_have_fix_ever == 0U)
  {
    return 0xFFFFFFFFU;
  }
  return HAL_GetTick() - g_last_fix_ms;
}

void GPS_UartRxCpltCallback(void)
{
  GPS_HandleByte(g_gps_rx_byte);
  (void)HAL_UART_Receive_IT(&huart3, &g_gps_rx_byte, 1U);
}

void GPS_UartErrorCallback(void)
{
  (void)HAL_UART_AbortReceive(&huart3);
  GPS_ResetParser();
  (void)HAL_UART_Receive_IT(&huart3, &g_gps_rx_byte, 1U);
}
