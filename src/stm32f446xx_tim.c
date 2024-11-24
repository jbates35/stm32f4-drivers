#include "stm32f446xx_tim.h"

#include <stdio.h>

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

int timer_init(const TimerHandle_t *timer_handle) {
  if (timer_handle == NULL) return -1;

  // Break the addr and config into separate variables for nicer looking code
  TIM_TypeDef *timer = (timer_handle->p_base_addr);
  const TimerConfig_t *cfg = &(timer_handle->cfg);

  // Set the frequency of the timer
  timer->CR1 &= ~(0b11 << TIM_CR1_CKD_Pos);
  timer->CR1 |= ((cfg->clock_divider & 0b11) << TIM_CR1_CKD_Pos);
  timer->PSC = cfg->prescaler;
  timer->ARR = cfg->arr;

  // Set direction of the timer
  if (cfg->direction == TIMER_DIR_UP)
    timer->CR1 &= ~(1 << TIM_CR1_DIR);
  else if (cfg->direction == TIMER_DIR_DOWN)
    timer->CR1 |= (1 << TIM_CR1_DIR);

  // For easier indexing of addresses
  volatile uint32_t *ccr_reg[] = TIMER_CCR_REGS(timer);
  volatile uint32_t *ccmr_reg[] = TIMER_CCMR_REGS(timer);
  const TimerChannelConfig_t *channel_cfg[] = CHANNEL_CONFIGS(cfg);

  // Configure channel specific attributes
  for (int i = 0; i < cfg->channel_count && i < 4; i++) {
    // Reset the bits for output mode so it can be set by the following
    *ccmr_reg[i] &= ~(0b111 << (TIM_CCMR1_OC1M_Pos + (i * 8) % 16));

    // Configure the output mode accordingly
    TimChanMode_t channel_mode = channel_cfg[i]->channel_mode;
    if (channel_mode == TIMER_CHANNEL_MODE_COMPARE) {
    } else if (channel_mode == TIMER_CHANNEL_MODE_CAPTURE) {
    } else if (channel_mode == TIMER_CHANNEL_MODE_PWM_HI) {
      *ccmr_reg[i] |= (0b110 << (TIM_CCMR1_OC1M_Pos + (i * 8) % 16));
    } else if (channel_mode == TIMER_CHANNEL_MODE_PWM_LO) {
      *ccmr_reg[i] |= (0b111 << (TIM_CCMR1_OC1M_Pos + (i * 8) % 16));
    }

    // Enable or disable the forwarding of the pin information to the GPIO pin
    TimGPIOEn_t gpio_en = channel_cfg[i]->gpio_en;
    if (gpio_en == TIMER_GPIO_ENABLE)
      timer->CCER |= (1 << (TIM_CCER_CC1E_Pos + (4 * i)));
    else
      timer->CCER &= ~(1 << (TIM_CCER_CC1E_Pos + (4 * i)));

    // Load the times when interrupts happen
    *ccr_reg[i] = channel_cfg[i]->ccr;

    // Set interrupt if required
    timer->DIER &= ~(1 << (TIM_DIER_CC1IE_Pos + i));
    if (channel_cfg[i]->interrupt_en == TIMER_INTERRUPT_ENABLE) timer->DIER |= (1 << (TIM_DIER_CC1IE_Pos + i));
  }

  // Lastly, enable the timer
  timer->CR1 |= (1 << TIM_CR1_CEN_Pos);

  return 0;
}

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

int timer_set_pwm_percent(TIM_TypeDef *timer, const uint8_t channel, const float pct) {
  if (timer == NULL) return -1;

  // Grab the pwm value from the max value the timer will count up to
  uint16_t pwm_val = pct * timer->ARR;

  // Set PWM
  return timer_set_pwm(timer, channel, pwm_val);
}

void timer_irq_interrupt_config(const uint8_t irq_number, const uint8_t en_state) {
  // Enables or disables NVIC
  // Programs ISER if enable, programs ICER if disable
  if (en_state == TIMER_IRQ_ENABLE)
    NVIC->ISER[irq_number / 32] |= (1 << (irq_number % 32));
  else
    NVIC->ICER[irq_number / 32] |= (1 << (irq_number % 32));
}

void timer_irq_priority_config(const uint8_t irq_number, const uint8_t irq_priority) {
  uint8_t qshift = (irq_number * 8) % 32;  // Will result in bits 0 - 240*8
  uint8_t qindex = (irq_number * 8) / 32;

  NVIC->IP[qindex] &= ~(0xFF << qshift);
  NVIC->IP[qindex] |= (irq_priority << qshift << 4) & (0xF0 << qshift);
}

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
