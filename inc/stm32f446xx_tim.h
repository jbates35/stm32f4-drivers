#ifndef INC_STM34F446XX_TIMER_H_
#define INC_STM34F446XX_TIMER_H_

#include <stdint.h>

#include "stm32f446xx.h"

typedef enum {
  TIMER_CHANNEL_MODE_COMPARE = 0,
  TIMER_CHANNEL_MODE_CAPTURE,
  TIMER_CHANNEL_MODE_PWM_HI,
  TIMER_CHANNEL_MODE_PWM_LO
} TimerChanMode_t;
typedef enum { TIMER_CLOCK_DIVIDE_1 = 0, TIMER_CLOCK_DIVIDE_2, TIMER_CLOCK_DIVIDE_4 } TimerClockDivider_t;
typedef enum { TIMER_INTERRUPT_DISABLE = 0, TIMER_INTERRUPT_ENABLE } TimerInterruptEn_t;
typedef enum { TIMER_IRQ_DISABLE = 0, TIMER_IRQ_ENABLE } TimIRQ_t;
typedef enum { TIMER_PERI_CLOCK_DISABLE = 0, TIMER_PERI_CLOCK_ENABLE } TimPeriEn_t;
typedef enum { TIMER_GPIO_DISABLE = 0, TIMER_GPIO_ENABLE } TimerGPIOEn_t;
typedef enum { TIMER_DIR_UP = 0, TIMER_DIR_DOWN } TimerDir_t;
typedef enum {
  TIMER_CAPTURE_FILTER_NONE = 0,
  TIMER_CAPTURE_FILTER_SLOW,
  TIMER_CAPTURE_FILTER_MEDIUM,
  TIMER_CAPTURE_FILTER_FAST
} TimerCaptureFilter_t;
typedef enum {
  TIMER_CAPTURE_RISING_EDGE = 0,
  TIMER_CAPTURE_FALLING_EDGE,
  TIMER_CAPTURE_BOTH_EDGE,
} TimerCaptureEdgeSel_t;

/**
  * Channel specific configuration struct
  * @gpio_en: Whether the associated GPIO should be set up to follow the timer. Necessary for PWM and Input capture
  * @interrupt_en: Whether the interrupt is enabled or not
  * @channel_mode: Whether the channel should be set to compare, input capture, or pwm
  * @input_filter: Allow for either no filtering, slow medium or fast filtering  NOTE: Not implemented yet
  * @ccr: The ccr vals to be set (when on output or pwm mode), which dictate when the interrupts happen
  * NOTE: NOT meant to be used outside of the TimerConfig_t struct
**/
typedef struct {
  TimerGPIOEn_t gpio_en;
  TimerInterruptEn_t interrupt_en;
  TimerChanMode_t channel_mode;
  TimerCaptureFilter_t capture_input_filter;
  TimerCaptureEdgeSel_t capture_edge;
  uint16_t ccr;
} TimerChannelConfig_t;

/**
  * Timer configuration struct
  * @arr: The auto reset reload register for the timer
  * @prescaler: Pre-scalar to run the timer through (actual_freq = base_freq/(psc+1))
  * @clock_divider: Before pre-scaling, this value will either divide the clock /1->00, /2->01, /4->10, 11 not allowed
  * @direction: Whether up counting or down counting
  * @channel_count: The number of channels to init (TIM1-5, and 8 have 4 timers, others have 2 timers)
  * @channel_1: Timer channel 1 specific configuration
  * @channel_2: Timer channel 2 specific configuration
  * @channel_3: Timer channel 3 specific configuration
  * @channel_4: Timer channel 4 specific configuration
  *
  * NOTE: If using with nucleo f446 board, the base clock will be 16MHz
**/
typedef struct {
  uint16_t arr;
  uint16_t prescaler;
  TimerClockDivider_t clock_divider;
  TimerDir_t direction;
  uint8_t channel_count;
  TimerChannelConfig_t channel_1;
  TimerChannelConfig_t channel_2;
  TimerChannelConfig_t channel_3;
  TimerChannelConfig_t channel_4;
} TimerConfig_t;

/**
  * Timer handler struct
  * @p_base_addr: Address of the timer (Accessed via TIM1, TIM2, ..., TIM14)
  * @cfg: Configuration struct for the timer
  * NOTE: This API only allows for 16-bit values to be compared to in output mode
**/
typedef struct {
  TIM_TypeDef *p_base_addr;
  TimerConfig_t cfg;
} TimerHandle_t;

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
int timer_peri_clock_control(const TIM_TypeDef *base_addr, const uint8_t en_state);

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
int timer_init(const TimerHandle_t *timer_handle);

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
int timer_set_pwm(TIM_TypeDef *timer, const uint8_t channel, uint16_t pwm_val);

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
int timer_set_pwm_percent(TIM_TypeDef *timer, const uint8_t channel, const float pct);

/**
 * @brief Get the current timer ticks for a specific channel.
 * 
 * @param timer Pointer to the TIM_TypeDef structure.
 * @param channel Timer channel number.
 * @return uint16_t Current timer ticks.
 */
uint16_t timer_get_current_ticks(const TIM_TypeDef *timer, const uint8_t channel);

/**
 * @brief Get the period ticks of the timer.
 * 
 * @param timer Pointer to the TIM_TypeDef structure.
 * @return uint16_t Timer period ticks.
 */
uint16_t timer_get_period_ticks(const TIM_TypeDef *timer);

/**
 * @brief Configure the timer IRQ interrupt.
 * 
 * @param irq_number IRQ number.
 * @param en_state Enable or disable state.
 */
void timer_irq_interrupt_config(const uint8_t irq_number, const uint8_t en_state);

/**
 * @brief Configure the priority of the timer IRQ.
 * 
 * @param irq_number IRQ number.
 * @param irq_priority IRQ priority.
 */
void timer_irq_priority_config(const uint8_t irq_number, const uint8_t irq_priority);

/**
 * @brief Handle the timer IRQ.
 * 
 * @param timer Pointer to the TIM_TypeDef structure.
 * @param channel Timer channel number.
 * @return int IRQ handling status.1 to indicate that there was an interrupt on this channel
 */
int timer_irq_handling(TIM_TypeDef *timer, const uint8_t channel);

#endif
