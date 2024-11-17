/*
 * STM32H723xx_gpio.h
 *
 *  Created on: Oct. 1, 2023
 *      Author: jbates
 */

#ifndef INC_STM32H723XX_GPIO_H_
#define INC_STM32H723XX_GPIO_H_

#include "stm32f446xx.h"

#define GPIO_CLOCK_ENABLE 1
#define GPIO_CLOCK_DISABLE 0

#define GPIO_INT_ENABLE 1
#define GPIO_INT_DISABLE 0

/**
 * GPIO pin configuration structure
 * @GPIO_pin_number: the pin associated with the particular port (i.e. 5 if PE5)
 * @GPIO_pin_mode: the mode of the pin (i.e. input, output, etc.)
 * @GPIO_pin_speed: the speed of the pin (i.e. low, medium, high, etc.)
 * @GPIO_pin_pupd_control: the pull up/pull down configuration of the pin
 * @GPIO_pin_out_type: the output type of the pin (i.e. push-pull or open drain)
 * @GPIO_pin_alt_func_mode: the alternate function mode of the pin
 */
typedef struct {
  uint8_t GPIO_pin_number;
  uint8_t GPIO_pin_mode;
  uint8_t GPIO_pin_speed;
  uint8_t GPIO_pin_pupd_control;
  uint8_t GPIO_pin_out_type;
  uint8_t GPIO_pin_alt_func_mode;
} GPIO_PinConfig_t;

typedef struct {
  GPIO_TypeDef *p_GPIO_x;            // Holds the base address of the GPIO port which the pin belongs
  GPIO_PinConfig_t GPIO_pin_config;  // Holds the GPIO pin configuration settings
} GPIO_Handle_t;

/*
 * GPIO pin possible modes
 */
typedef enum {
  GPIO_MODE_IN = 0,
  GPIO_MODE_OUT = 1,
  GPIO_MODE_ALTFN = 2,
  GPIO_MODE_ANALOG = 3,
  GPIO_MODE_IT_FT = 4,
  GPIO_MODE_IT_RT = 5,
  GPIO_MODE_IT_RFT = 6
} GPIO_MODE_BIT;

/*
 * GPIO pin possible output types
 */
typedef enum { GPIO_OP_TYPE_PUSHPULL = 0, GPIO_OP_TYPE_OPENDRAIN = 1 } GPIO_OP_TYPE_BIT;

/*
 * GPIO pin possible output speeds
 */
typedef enum {
  GPIO_SPEED_LOW = 0,
  GPIO_SPEED_MEDIUM = 1,
  GPIO_SPEED_HIGH = 2,
  GPIO_SPEED_VERY_HIGH = 3
} GPIO_SPEED_BIT;

/*
 * GPIO pin pull up and pull down configuration macros
 */
typedef enum { GPIO_PUPDR_NONE = 0, GPIO_PUPDR_PULLUP = 1, GPIO_PUPDR_PULLDOWN = 2 } GPIO_PUPDR_BIT;
/*
 * Peripheral clock setup
 */
void GPIO_peri_clock_control(const GPIO_TypeDef *p_GPIO_x, const uint8_t en_state);

/*
 * Init and de-init of GPIO
 */
void GPIO_init(const GPIO_Handle_t *p_GPIO_handle);
void GPIO_deinit(const GPIO_TypeDef *p_GPIO_x);

/*
 * Data read and write
 */
uint8_t GPIO_get_input(const GPIO_TypeDef *p_GPIO_x, const uint8_t pin);
uint16_t GPIO_get_input_port(const GPIO_TypeDef *p_GPIO_x);
void GPIO_set_output(GPIO_TypeDef *p_GPIO_x, uint8_t pin, uint8_t val);
void GPIO_set_output_port(GPIO_TypeDef *p_GPIO_x, uint16_t val);
void GPIO_toggle_output(GPIO_TypeDef *p_GPIO_x, uint8_t pin);

/*
 * IRQ configuration and IRQ handling
 */
void GPIO_irq_interrupt_config(uint8_t irq_number, uint8_t en_state);
void GPIO_irq_priority_config(uint8_t irq_number, uint8_t irq_priority);
int GPIO_irq_handling(uint8_t pin);

#endif /* INC_STM32H723XX_GPIO_H_ */
