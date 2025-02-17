/*
 * STM32H723xx_gpio.h
 *
 *  Created on: Oct. 1, 2023
 *      Author: jbates
 */

#ifndef INC_STM32F446XX_GPIO_H_
#define INC_STM32F446XX_GPIO_H_

#include "stm32f446xx.h"

#define GPIO_CLOCK_ENABLE 1
#define GPIO_CLOCK_DISABLE 0

#define GPIO_INT_ENABLE 1
#define GPIO_INT_DISABLE 0

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
} GPIOModeBit_t;

/*
 * GPIO pin possible output types
 */
typedef enum { GPIO_OP_TYPE_PUSHPULL = 0, GPIO_OP_TYPE_OPENDRAIN = 1 } GPIOOpTypeBit_t;

/*
 * GPIO pin possible output speeds
 */
typedef enum {
  GPIO_SPEED_LOW = 0,
  GPIO_SPEED_MEDIUM = 1,
  GPIO_SPEED_HIGH = 2,
  GPIO_SPEED_VERY_HIGH = 3
} GPIOSpeedBit_t;

/*
 * GPIO pin pull up and pull down configuration macros
 */
typedef enum { GPIO_PUPDR_NONE = 0, GPIO_PUPDR_PULLUP = 1, GPIO_PUPDR_PULLDOWN = 2 } GPIOPuPdRBit_t;

/**
* GPIO peripheral clock enable/disable
*/
typedef enum { GPIO_PERI_CLOCK_DISABLE = 0, GPIO_PERI_CLOCK_ENABLE } GPIOPeriClockEnable_t;

/**
 * @brief GPIO pin configuration structure
 * @param pin_number: the pin associated with the particular port (i.e. 5 if PE5)
 * @param mode: the mode of the pin (i.e. input, output, etc.)
 * @param speed: the speed of the pin (i.e. low, medium, high, etc.)
 * @param float_resistor: the pull up/pull down configuration of the pin so the input is driven a certain way when floating
 * @param output_type: the output type of the pin (i.e. push-pull or open drain)
 * @param alt_func_num: the alternate function mode of the pin
 */
typedef struct {
  uint8_t pin_number;
  GPIOModeBit_t mode;
  GPIOSpeedBit_t speed;
  GPIOPuPdRBit_t float_resistor;
  GPIOOpTypeBit_t output_type;
  uint8_t alt_func_num;
} GPIOConfig_t;

typedef struct {
  GPIO_TypeDef *p_GPIO_addr;  // Holds the base address of the GPIO port which the pin belongs
  GPIOConfig_t cfg;           // Holds the GPIO pin configuration settings
} GPIOHandle_t;

/*
 * Peripheral clock setup
 */
void GPIO_peri_clock_control(const GPIO_TypeDef *p_GPIO_x, const GPIOPeriClockEnable_t en_state);

/*
 * Init and de-init of GPIO
 */
void GPIO_init(const GPIOHandle_t *p_GPIO_handle);
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
