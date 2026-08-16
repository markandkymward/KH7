#include "motors.h"

#include "main.h"

#define MOTOR_PWM_MIN_US 1000U
#define MOTOR_PWM_MAX_US 2000U
#define MOTOR_TEST_ARM_DELAY_MS 3000U
#define MOTOR_TEST_STEP_MS 2000U
#define MOTOR_TEST_PULSE_US 1300U

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

static uint8_t g_motor_output_enabled = 0U;
static uint8_t g_motor_pwm_started = 0U;
static uint8_t g_motor_ch1_started = 0U;
static uint8_t g_motor_ch2_started = 0U;
static uint8_t g_motor_ch3_started = 0U;
static uint8_t g_motor_ch4_started = 0U;

static void Motors_ForceEnablePwmChannels(void)
{
  __HAL_TIM_ENABLE(&htim2);
  __HAL_TIM_ENABLE(&htim3);

  htim2.Instance->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC3E);
  htim3.Instance->CCER |= (TIM_CCER_CC3E | TIM_CCER_CC4E);
}

static void Motors_ForceDisablePwmChannels(void)
{
  htim2.Instance->CCER &= (uint32_t)~(TIM_CCER_CC2E | TIM_CCER_CC3E);
  htim3.Instance->CCER &= (uint32_t)~(TIM_CCER_CC3E | TIM_CCER_CC4E);
}

static void Motors_TryStartChannel(TIM_HandleTypeDef *htim,
                                   uint32_t channel,
                                   uint8_t *started_flag)
{
  if (*started_flag != 0U)
  {
    return;
  }

  if (HAL_TIM_PWM_Start(htim, channel) == HAL_OK)
  {
    *started_flag = 1U;
  }
}

static void Motors_StartPwmOutputs(void)
{
  Motors_TryStartChannel(&htim3, TIM_CHANNEL_3, &g_motor_ch1_started);
  Motors_TryStartChannel(&htim3, TIM_CHANNEL_4, &g_motor_ch2_started);
  Motors_TryStartChannel(&htim2, TIM_CHANNEL_2, &g_motor_ch3_started);
  Motors_TryStartChannel(&htim2, TIM_CHANNEL_3, &g_motor_ch4_started);

  Motors_ForceEnablePwmChannels();

  g_motor_ch1_started = 1U;
  g_motor_ch2_started = 1U;
  g_motor_ch3_started = 1U;
  g_motor_ch4_started = 1U;

  g_motor_pwm_started = 1U;
}

static void Motors_StopPwmOutputs(void)
{
  if (g_motor_pwm_started == 0U)
  {
    return;
  }

  (void)HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
  (void)HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
  (void)HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
  (void)HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);

  Motors_ForceDisablePwmChannels();

  g_motor_ch1_started = 0U;
  g_motor_ch2_started = 0U;
  g_motor_ch3_started = 0U;
  g_motor_ch4_started = 0U;
  g_motor_pwm_started = 0U;
}

static uint32_t Motors_ClampPulseUs(uint16_t pulse_us)
{
  if (pulse_us < MOTOR_PWM_MIN_US)
  {
    return MOTOR_PWM_MIN_US;
  }

  if (pulse_us > MOTOR_PWM_MAX_US)
  {
    return MOTOR_PWM_MAX_US;
  }

  return pulse_us;
}

void Motors_Init(void)
{
  g_motor_output_enabled = 0U;
  g_motor_pwm_started = 0U;

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0U);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0U);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0U);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0U);

  Motors_StopPwmOutputs();
}

void Motors_SetOutputEnabled(uint8_t enabled)
{
  g_motor_output_enabled = enabled;

  if (g_motor_output_enabled != 0U)
  {
    Motors_StartPwmOutputs();
  }
  else
  {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0U);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0U);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0U);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0U);
    Motors_StopPwmOutputs();
  }
}

void Motors_WriteUs(uint16_t s1_us,
                    uint16_t s2_us,
                    uint16_t s3_us,
                    uint16_t s4_us)
{
  if (g_motor_pwm_started == 0U)
  {
    Motors_StartPwmOutputs();
  }

  if (g_motor_output_enabled == 0U)
  {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0U);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0U);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0U);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0U);
    return;
  }

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, Motors_ClampPulseUs(s1_us));
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, Motors_ClampPulseUs(s2_us));
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, Motors_ClampPulseUs(s3_us));
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, Motors_ClampPulseUs(s4_us));
}

uint8_t Motors_RunTestPattern(uint32_t now_ms)
{
  uint32_t step_index;

  if (g_motor_output_enabled == 0U)
  {
    Motors_StopAll();
    return 0U;
  }

  if (now_ms < MOTOR_TEST_ARM_DELAY_MS)
  {
    Motors_StopAll();
    return 0U;
  }

  step_index = ((now_ms - MOTOR_TEST_ARM_DELAY_MS) / MOTOR_TEST_STEP_MS) % 5U;

  switch (step_index)
  {
    case 0U:
      Motors_WriteUs(MOTOR_TEST_PULSE_US, MOTOR_PWM_MIN_US, MOTOR_PWM_MIN_US, MOTOR_PWM_MIN_US);
      return 1U;
    case 1U:
      Motors_WriteUs(MOTOR_PWM_MIN_US, MOTOR_TEST_PULSE_US, MOTOR_PWM_MIN_US, MOTOR_PWM_MIN_US);
      return 2U;
    case 2U:
      Motors_WriteUs(MOTOR_PWM_MIN_US, MOTOR_PWM_MIN_US, MOTOR_TEST_PULSE_US, MOTOR_PWM_MIN_US);
      return 3U;
    case 3U:
      Motors_WriteUs(MOTOR_PWM_MIN_US, MOTOR_PWM_MIN_US, MOTOR_PWM_MIN_US, MOTOR_TEST_PULSE_US);
      return 4U;
    default:
      Motors_StopAll();
      return 0U;
  }
}

void Motors_StopAll(void)
{
  Motors_WriteUs(MOTOR_PWM_MIN_US,
                 MOTOR_PWM_MIN_US,
                 MOTOR_PWM_MIN_US,
                 MOTOR_PWM_MIN_US);
}

/* Reentrant-safe subset of Motors_StopAll(): ONLY writes the timer compare
 * registers, touching none of the g_motor_pwm_started/g_motor_ch*_started
 * lazy-init state machine or HAL_TIM_PWM_Start()/Stop() calls. Safe to call
 * from an interrupt that can preempt the main loop mid-way through that
 * state machine (see receiver.c's ISR-level arm-switch safety net) - the
 * plain Motors_StopAll()/Motors_WriteUs() path is NOT safe for that since it
 * can leave the timer's channel-enable bits/HAL state corrupted if the ISR
 * lands mid-sequence. */
void Motors_ForceIdleRegistersOnly(void)
{
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, MOTOR_PWM_MIN_US);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, MOTOR_PWM_MIN_US);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, MOTOR_PWM_MIN_US);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, MOTOR_PWM_MIN_US);
}
