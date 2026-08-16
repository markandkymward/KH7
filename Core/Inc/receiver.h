#ifndef RECEIVER_H
#define RECEIVER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RECEIVER_CHANNEL_COUNT 16U

typedef struct
{
  uint16_t channels[RECEIVER_CHANNEL_COUNT];
  uint8_t link_active;
  uint8_t frame_received;
  uint32_t frame_count;
  uint32_t last_frame_ms;
  uint32_t crc_error_count;
  uint32_t sync_error_count;
  uint32_t overrun_error_count;
  uint32_t framing_error_count;
  uint32_t noise_error_count;
} receiver_state_t;

void Receiver_Init(void);
void Receiver_Update(uint32_t now_ms);
void Receiver_GetState(receiver_state_t *state);

#ifdef __cplusplus
}
#endif

#endif
