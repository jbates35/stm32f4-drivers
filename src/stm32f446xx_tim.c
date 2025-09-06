/**
  Timer Module
  Made by Jimmy Bates - ~November 2024
**/

#include "stm32f446xx_tim.h"

#include <stdio.h>

#include "stm32f446xx.h"

#define TIMERS {TIM1, TIM2, TIM3, TIM4, TIM5, TIM6, TIM7, TIM8, TIM9, TIM10, TIM11, TIM12, TIM13, TIM14}
#define TIMERS_RCC_REGS                                                                                     \
  {&RCC->APB2ENR, &RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB1ENR, \
   &RCC->APB2ENR, &RCC->APB2ENR, &RCC->APB2ENR, &RCC->APB2ENR, &RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB1ENR}
#define TIMERS_RCC_POS                                                                                    \
  {                                                                                                       \
      RCC_APB2ENR_TIM1EN_Pos,  RCC_APB1ENR_TIM2EN_Pos,  RCC_APB1ENR_TIM3EN_Pos,  RCC_APB1ENR_TIM4EN_Pos,  \
      RCC_APB1ENR_TIM5EN_Pos,  RCC_APB1ENR_TIM6EN_Pos,  RCC_APB1ENR_TIM7EN_Pos,  RCC_APB2ENR_TIM8EN_Pos,  \
      RCC_APB2ENR_TIM9EN_Pos,  RCC_APB2ENR_TIM10EN_Pos, RCC_APB2ENR_TIM11EN_Pos, RCC_APB1ENR_TIM12EN_Pos, \
      RCC_APB1ENR_TIM13EN_Pos, RCC_APB1ENR_TIM14EN_Pos,                                                   \
  }

#define TIMER_CCR_REGS(timer) {&timer->CCR1, &timer->CCR2, &timer->CCR3, &timer->CCR4}
#define TIMER_CCMR_REGS(timer) {&timer->CCMR1, &timer->CCMR1, &timer->CCMR2, &timer->CCMR2}

#define SIZEOF(arr) ((int)sizeof(arr) / sizeof(arr[0]))
#define SIZEOFP(arr) ((int)sizeof(arr) / sizeof(uint32_t))  // Memory size of stm32f4

#define CHANNEL_CONFIGS(cfg) {&cfg->channel_1, &cfg->channel_2, &cfg->channel_3, &cfg->channel_4}

/**
 * @brief  Controls the peripheral clock for a given timer.
 *
 * This function enables or disables the peripheral clock for the specified timer.
 *
 * @param  base_addr: Base address of the timer.
 * @param  en_state: Enable state (1 to enable, 0 to disable).
 *
 * @retval 0 on success, -1 on failure (e.g., if base_addr is NULL or out of range).
 */
int timer_peri_clock_control(const TIM_TypeDef *base_addr, const uint8_t en_state) {
  if (base_addr == NULL) return -1;

  // Grab the index of the current timer
  const TIM_TypeDef *timers_arr[] = TIMERS;
  int i = 0;
  for (; i < SIZEOFP(timers_arr); i++) {
    if (timers_arr[i] == base_addr) break;
  }

  // Do nothing if the index is out of range
  if (i >= SIZEOFP(timers_arr)) return -1;

  // Turn on either a register in APB1 or APB2 according to the timer addr
  const uint32_t timer_rcc_pos_arr[] = TIMERS_RCC_POS;
  volatile uint32_t *timer_rcc_reg_arr[] = TIMERS_RCC_REGS;
  *timer_rcc_reg_arr[i] |= (1 << timer_rcc_pos_arr[i]);

  return 0;
}

/**
 * @brief  Initializes the timer with the specified configuration.
 *
 * This function sets up the timer based on the provided configuration in the TimerHandle_t structure.
 * It configures the clock divider, prescaler, auto-reload register, direction, and channel-specific settings.
 *
 * @param  timer_handle: Pointer to the TimerHandle_t structure containing the timer base address and configuration.
 *
 * @retval 0 on success, -1 if the timer_handle is NULL.
 */
int timer_init(const TimerHandle_t *timer_handle) {
  if (timer_handle == NULL || timer_handle->p_base_addr == NULL) return -1;

  // Break the addr and config into separate variables for nicer looking code
  TIM_TypeDef *timer = (timer_handle->p_base_addr);
  const TimerConfig_t *cfg = &(timer_handle->cfg);

  // Get the bits that match the clock divider enum
  uint8_t clock_divider = 0b00;
  if (cfg->clock_divider == TIMER_CLOCK_DIVIDE_2)
    clock_divider = 0b01;
  else if (cfg->clock_divider == TIMER_CLOCK_DIVIDE_4)
    clock_divider = 0b10;

  // Set the frequency of the timer
  timer->CR1 &= ~(0b11 << TIM_CR1_CKD_Pos);
  timer->CR1 |= (clock_divider << TIM_CR1_CKD_Pos);
  timer->PSC = cfg->prescaler;
  timer->ARR = cfg->arr;

  // Set direction of the timer
  if (cfg->direction == TIMER_DIR_UP)
    timer->CR1 &= ~(1 << TIM_CR1_DIR_Pos);
  else if (cfg->direction == TIMER_DIR_DOWN)
    timer->CR1 |= (1 << TIM_CR1_DIR_Pos);

  // Enable the Trigger
  uint8_t trig_assign = cfg->trigger_assignment;
  if (trig_assign > 0b111) trig_assign = 0;
  timer->CR2 |= (trig_assign << TIM_CR2_MMS_Pos);

  // For easier indexing of addresses
  volatile uint32_t *ccr_reg[] = TIMER_CCR_REGS(timer);
  volatile uint32_t *ccmr_reg[] = TIMER_CCMR_REGS(timer);
  const TimerChannelConfig_t *channel_cfg[] = CHANNEL_CONFIGS(cfg);

  // Configure channel specific attributes
  for (int i = 0; i < cfg->channel_count && i < 4; i++) {
    // Reset the bits for output mode so it can be set by the following
    *ccmr_reg[i] &= ~(0b111 << (TIM_CCMR1_OC1M_Pos + (i * 8) % 16));

    // Configure the output mode accordingly
    TimerChanMode_t channel_mode = channel_cfg[i]->channel_mode;
    if (channel_mode == TIMER_CHANNEL_MODE_COMPARE) {
      // Ensure the timer is set to output capture mode
      *ccmr_reg[i] &= ~(0b11 << ((TIM_CCMR1_CC1S_Pos + i * 8) % 16));

      ///// NOTE: Going to assume we want the gpio to toggle if in output mode
      ///// But we might want a separate set of enums to dictate what we want here
      *ccmr_reg[i] &= ~(0b111 << ((TIM_CCMR1_OC1M_Pos + i * 8) % 16));
      *ccmr_reg[i] |= (0b011 << ((TIM_CCMR1_OC1M_Pos + i * 8) % 16));

      ///// NOTE: If pre-load is enabled, it should go here

    } else if (channel_mode == TIMER_CHANNEL_MODE_CAPTURE) {
      // Change the timer to input capture mode
      *ccmr_reg[i] &= ~(0b11 << ((TIM_CCMR1_CC1S_Pos + i * 8) % 16));
      *ccmr_reg[i] |= (0b01 << ((TIM_CCMR1_CC1S_Pos + i * 8) % 16));

      // Program the input filter duration
      TimerCaptureFilter_t filter_len = channel_cfg[i]->capture_input_filter;
      *ccmr_reg[i] &= ~(0b1111 << ((TIM_CCMR1_IC1F_Pos + i * 8) % 16));
      uint8_t filter_val = 0b0000;
      if (filter_len == TIMER_CAPTURE_FILTER_FAST)
        filter_val = 0b0010;
      else if (filter_len == TIMER_CAPTURE_FILTER_MEDIUM)
        filter_val = 0b1001;
      else if (filter_len == TIMER_CAPTURE_FILTER_SLOW)
        filter_val = 0b1111;
      *ccmr_reg[i] |= (filter_val << ((TIM_CCMR1_IC1F_Pos + i * 8) % 16));

      // Select the active edge - 101 is both edges, 001 is falling edge, 000 is rising edge
      TimerCaptureEdgeSel_t edge_sel = channel_cfg[i]->capture_edge;
      timer->CCER &= ~(0b111 << (TIM_CCER_CC1P_Pos + i * 4));
      uint8_t edge_val = 0b000;
      if (edge_sel == TIMER_CAPTURE_FALLING_EDGE)
        edge_val = 0b001;
      else if (edge_sel == TIMER_CAPTURE_BOTH_EDGE)
        edge_val = 0b101;
      timer->CCER |= (edge_val << (TIM_CCER_CC1P_Pos + i * 4));

      ///// NOTE: This is where the capture prescaler would go in, if we should be putting it in

    } else if (channel_mode == TIMER_CHANNEL_MODE_PWM_HI) {
      *ccmr_reg[i] |= ((0b110 << (TIM_CCMR1_OC1M_Pos + i * 8) % 16));
    } else if (channel_mode == TIMER_CHANNEL_MODE_PWM_LO) {
      *ccmr_reg[i] |= ((0b111 << (TIM_CCMR1_OC1M_Pos + i * 8) % 16));
    }

    // Enable or disable the forwarding of the pin information to the GPIO pin
    TimerEnable_t gpio_en = channel_cfg[i]->gpio_en;
    if (gpio_en == TIMER_ENABLE)
      timer->CCER |= (1 << (TIM_CCER_CC1E_Pos + (4 * i)));
    else
      timer->CCER &= ~(1 << (TIM_CCER_CC1E_Pos + (4 * i)));

    // Load the times when interrupts happen
    *ccr_reg[i] = channel_cfg[i]->ccr;

    // Set interrupt if required
    timer->DIER &= ~(1 << (TIM_DIER_CC1IE_Pos + i));
    if (channel_cfg[i]->interrupt_en == TIMER_ENABLE) timer->DIER |= (1 << (TIM_DIER_CC1IE_Pos + i));
  }

  // Force all values to be forced into their shadow registers
  timer->EGR |= (0b1 << TIM_EGR_UG_Pos);

  // Lastly, enable the timer
  if (cfg->start_enabled == TIMER_ENABLE) timer->CR1 |= (1 << TIM_CR1_CEN_Pos);

  return 0;
}

/**
 * @brief  Sets the PWM value for a specified timer channel.
 *
 * This function sets the PWM value for a given timer channel by writing
 * the value to the appropriate Capture/Compare Register (CCR).
 *
 * @param  timer   Pointer to the TIM_TypeDef structure that contains
 *                 the configuration information for the specified timer.
 * @param  channel The timer channel to set the PWM value for (1 to 4).
 * @param  pwm_val The PWM value to set.
 *
 * @retval int     Returns 0 on success, -1 if the timer pointer is NULL,
 *                 -2 if the channel is invalid.
 */
int timer_set_pwm(TIM_TypeDef *timer, const uint8_t channel, uint16_t pwm_val) {
  if (timer == NULL) return -1;  // Error: null pointer

  // Grab the correct CCR register so we can load the pwm value in
  volatile uint32_t *ccr_reg[] = TIMER_CCR_REGS(timer);

  // Ensure the value of channel is valid
  if (channel == 0 || channel > SIZEOF(ccr_reg)) return -2;  // Error: invalid channel

  if (pwm_val > timer->ARR) pwm_val = timer->ARR;

  // Set PWM register
  *ccr_reg[channel - 1] = pwm_val;
  return 0;
}

/**
 * @brief  Sets the PWM duty cycle percentage for a specified timer channel.
 *
 * This function calculates the PWM value based on the percentage provided
 * and sets it for the specified timer channel.
 *
 * @param  timer   Pointer to the TIM_TypeDef structure that contains
 *                 the configuration information for the specified timer.
 * @param  channel The timer channel to set the PWM value for.
 * @param  pct     The desired PWM duty cycle percentage (0.0 to 100.0).
 *
 * @retval int     Returns 0 on success, -1 if the timer pointer is NULL.
 */
int timer_set_pwm_percent(TIM_TypeDef *timer, const uint8_t channel, const float pct) {
  if (timer == NULL) return -1;

  // Grab the pwm value from the max value the timer will count up to
  uint16_t pwm_val = pct * timer->ARR;

  // Set PWM
  return timer_set_pwm(timer, channel, pwm_val);
}

/**
 * @brief Get the current timer ticks for a specific channel.
 *
 * @param timer Pointer to the TIM_TypeDef structure.
 * @param channel Timer channel number.
 * @return uint16_t Current timer ticks.
 */
// TODO: Make this function (and a couple others) static inline and defined in the .h file
uint16_t timer_get_current_ticks(const TIM_TypeDef *timer, const uint8_t channel) {
  if (timer == NULL) return -1;  // Error: null pointer

  // Grab the correct CCR register so we can load the pwm value in
  const volatile uint32_t *ccr_reg[] = TIMER_CCR_REGS(timer);
  return *ccr_reg[channel - 1];
}

/**
 * @brief Get the period ticks of the timer.
 *
 * @param timer Pointer to the TIM_TypeDef structure.
 * @return uint16_t Timer period ticks.
 */
uint16_t timer_get_period_ticks(const TIM_TypeDef *timer) { return timer->ARR; }

/**
 * @brief Configure the timer IRQ interrupt.
 *
 * @param irq_number IRQ number.
 * @param en_state Enable or disable state.
 */
void timer_irq_interrupt_config(const uint8_t irq_number, const uint8_t en_state) {
  // Enables or disables NVIC
  // Programs ISER if enable, programs ICER if disable
  if (en_state == TIMER_ENABLE)
    NVIC->ISER[irq_number / 32] |= (1 << (irq_number % 32));
  else
    NVIC->ICER[irq_number / 32] |= (1 << (irq_number % 32));
}

/**
 * @brief Configure the priority of the timer IRQ.
 *
 * @param irq_number IRQ number.
 * @param irq_priority IRQ priority.
 */
void timer_irq_priority_config(const uint8_t irq_number, const uint8_t irq_priority) {
  uint8_t qshift = (irq_number * 8) % 32;  // Will result in bits 0 - 240*8
  uint8_t qindex = (irq_number * 8) / 32;

  NVIC->IP[qindex] &= ~(0xFF << qshift);
  NVIC->IP[qindex] |= (irq_priority << qshift << 4) & (0xF0 << qshift);
}

/**
 * @brief Handle the timer IRQ.
 *
 * @param timer Pointer to the TIM_TypeDef structure.
 * @param channel Timer channel number.
 * @return int IRQ handling status.1 to indicate that there was an interrupt on this channel
 */
int timer_irq_handling(TIM_TypeDef *timer, const uint8_t channel) {
  if (timer == NULL) return 0;  // Error: null pointer

  // Ensure the value of channel is valid
  if (channel == 0 || channel > 4) return 0;  // Error: invalid channel

  if (timer->SR & (1 << channel)) {
    timer->SR &= ~(1 << channel);
    return 1;
  }

  return 0;
}
