#include "gps.h"

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
#define GPS_UBX_ID_CFG_CFG       0x09U

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

static volatile uint8_t g_fix_type = 0U;
static volatile uint8_t g_num_sv = 0U;
static volatile float g_lat_deg = 0.0f;
static volatile float g_lon_deg = 0.0f;
static volatile float g_alt_m = 0.0f;
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
  int32_t height_mm;

  lon_raw = (int32_t)((uint32_t)payload[24] | ((uint32_t)payload[25] << 8) |
                       ((uint32_t)payload[26] << 16) | ((uint32_t)payload[27] << 24));
  lat_raw = (int32_t)((uint32_t)payload[28] | ((uint32_t)payload[29] << 8) |
                       ((uint32_t)payload[30] << 16) | ((uint32_t)payload[31] << 24));
  height_mm = (int32_t)((uint32_t)payload[36] | ((uint32_t)payload[37] << 8) |
                        ((uint32_t)payload[38] << 16) | ((uint32_t)payload[39] << 24));

  g_fix_type = payload[20];
  g_num_sv = payload[23];
  g_lat_deg = (float)lat_raw * 1.0e-7f;
  g_lon_deg = (float)lon_raw * 1.0e-7f;
  g_alt_m = (float)height_mm * 0.001f;
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
  uint8_t cfg_cfg_payload[13];
  uint8_t prt_acked;
  uint8_t msg_acked;

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

  /* UBX-CFG-PRT: UART1 (module-internal), 8N1, baud unchanged (9600), UBX+NMEA
   * in, UBX-only out. */
  memset(cfg_prt_payload, 0, sizeof(cfg_prt_payload));
  cfg_prt_payload[0] = 1U;                 /* portID = UART1 */
  cfg_prt_payload[4] = 0xD0U;              /* mode = 0x000008D0 (8N1), LE */
  cfg_prt_payload[5] = 0x08U;
  cfg_prt_payload[8] = 0x80U;               /* baudRate = 9600, LE */
  cfg_prt_payload[9] = 0x25U;
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
  g_configured = (uint8_t)((prt_acked != 0U) && (msg_acked != 0U));
  return (g_configured != 0U) ? HAL_OK : HAL_ERROR;
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
