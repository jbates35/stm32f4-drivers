#include <stdint.h>

#include "stm32f446xx.h"

/**
  * Channel specific configuration struct
  * @gpio_en: Whether the associated GPIO should be set up to follow the timer. Necessary for PWM and Input capture
  * @interrupt_en: Whether the interrupt is enabled or not
  * @channel_mode: Whether the channel should be set to compare, input capture, or pwm
  * @ccr: The ccr vals to be set, which dictate when the interrupts happen
  * NOTE: NOT meant to be used outside of the TimerConfig_t struct
**/
typedef struct {
  uint8_t gpio_en;
  uint8_t interrupt_en;
  uint8_t channel_mode;
  uint16_t ccr;
} TimerConfigChannel_t;

/**
  * Timer configuration struct
  * @arr: The auto reset reload register for the timer
  * @prescaler: Pre-scalar to run the timer through (actual_freq = base_freq/(psc+1))
  * @dir: Whether up counting or down counting
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
  uint8_t dir;
  uint8_t channel_count;
  TimerConfigChannel_t channel_1;
  TimerConfigChannel_t channel_2;
  TimerConfigChannel_t channel_3;
  TimerConfigChannel_t channel_4;
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
enum { TIMER_GPIO_DISABLE = 0, TIMER_GPIO_ENABLE = 1 };
enum { TIMER_DIR_UP = 0, TIMER_DIR_DOWN = 1 };

int timer_peri_clock_control(const TIM_TypeDef *base_addr, const uint8_t en_state);

int timer_init(const TimerHandle_t *timer_handle);
int timer_set_pwm(TIM_TypeDef *timer, const uint8_t channel, uint16_t pwm_val);
int timer_set_pwm_percent(TIM_TypeDef *timer, const uint8_t channel, const float pct);
void timer_irq_interrupt_config(const uint8_t irq_number, const uint8_t en_state);
void timer_irq_priority_config(const uint8_t irq_number, const uint8_t irq_priority);
int timer_irq_handling(TIM_TypeDef *timer, const uint8_t channel);
