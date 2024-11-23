#include <stdint.h>

#include "stm32f446xx.h"

/**
  * Timer configuration struct
  * @gpio_en: Whether the associated GPIO should be set up to follow the timer. Necessary for PWM and Input capture
  * @interrupt_en: Whether the interrupt is enabled or not
  * @ccr_vals: The ccr vals to be set, which dictate when the interrupts happen
  * @timer_mode: Capture or compare mode, or PWM
  * @arr_val: The auto reset reload register for the timer
  * @prescaler: Pre-scalar to run the timer through (actual_freq = base_freq/(psc+1))
  * @channel_count: The number of channels to init (TIM1-5, and 8 have 4 timers, others have 2 timers)
  *
  * NOTE: If using with nucleo f446 board, the base clock will be 16MHz
**/

typedef struct {
  uint8_t gpio_en[4];
  uint8_t interrupt_en[4];
  uint8_t channel_mode[4];
  uint16_t ccr[4];
  uint16_t arr;
  uint16_t prescaler;
  uint8_t channel_count;
  uint8_t dir;
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

enum {
  TIMER_CHANNEL_MODE_COMPARE = 0,
  TIMER_CHANNEL_MODE_CAPTURE = 1,
  TIMER_CHANNEL_MODE_PWM_HI = 2,
  TIMER_CHANNEL_MODE_PWM_LO = 3
};
enum { TIMER_INTERRUPT_DISABLE = 0, TIMER_INTERRUPT_ENABLE = 1 };
enum { TIMER_IRQ_DISABLE = 0, TIMER_IRQ_ENABLE = 1 };
enum { TIMER_PERI_CLOCK_DISABLE = 0, TIMER_PERI_CLOCK_ENABLE = 1 };
// enum { TIMER_CHANNEL_1 = 0, TIMER_CHANNEL_2 = 1, TIMER_CHANNEL_3 = 2, TIMER_CHANNEL_4 = 3 };
enum { TIMER_GPIO_DISABLE = 0, TIMER_GPIO_ENABLE = 1 };
enum { TIMER_DIR_UP = 0, TIMER_DIR_DOWN = 1 };

int timer_peri_clock_control(const TIM_TypeDef *base_addr, const uint8_t en_state);

int timer_init(const TimerHandle_t *timer_handle);
int timer_set_pwm(TIM_TypeDef *timer, const uint8_t channel, uint16_t pwm_val);
int timer_set_pwm_percent(TIM_TypeDef *timer, const uint8_t channel, const float pct);
void timer_irq_interrupt_config(const uint8_t irq_number, const uint8_t en_state);
void timer_irq_priority_config(const uint8_t irq_number, const uint8_t irq_priority);
int timer_irq_handling(TIM_TypeDef *timer, const uint8_t channel);
