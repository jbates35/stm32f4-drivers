#include "stm32f446xx_gpio.h"

#include <stdio.h>

#define ENABLE 1
#define DISABLE 0

#define GPIOS {GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH}
#define GPIO_SIZE(arr) (int)sizeof(arr) / sizeof(GPIO_TypeDef *)

#define SYSCFG_ENABLE() (RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN)
/*
 * Peripheral clock setup for GPIO
 *
 * @param p_GPIO_x Pointer to base address for gpio struct
 * @param en_state 1 for enable, 0 for disable
 */
void GPIO_peri_clock_control(const GPIO_TypeDef *p_GPIO_x, const uint8_t en_state) {
  if (p_GPIO_x == NULL) return;

  static GPIO_TypeDef *const GPIO_base_addrs[8] = GPIOS;

  for (int i = 0; i < GPIO_SIZE(GPIO_base_addrs); i++) {
    if (GPIO_base_addrs[i] != p_GPIO_x) continue;

    if (en_state == ENABLE)
      RCC->AHB1ENR |= (1 << i);
    else
      RCC->AHB1ENR &= ~(1 << i);
  }
}

/*
 * Initialize a GPIO pin
+* @param p_GPIO_handle The handle type as defined in the gpio.h file which describes both the base address and the required configuration
 */
void GPIO_init(const GPIO_Handle_t *p_GPIO_handle) {
  if (p_GPIO_handle == NULL) return;

  // Create pointers to the GPIO port and the pin configuration for easier
  // access/readability
  GPIO_TypeDef *gpiox = p_GPIO_handle->p_GPIO_x;
  const GPIO_PinConfig_t *cfg = &(p_GPIO_handle->GPIO_pin_config);

  // For easy bit-shifting, dshift is 2*pin number, whereas sshift is just
  // pin_number
  const uint8_t qshift = (4 * cfg->GPIO_pin_number);
  const uint8_t dshift = 2 * cfg->GPIO_pin_number;
  const uint8_t sshift = cfg->GPIO_pin_number;

  // Set mode
  if (cfg->GPIO_pin_mode <= GPIO_MODE_ANALOG) {
    // Esssentially, digital out, analog in, etc. Everything up and until interrupts
    gpiox->MODER &= ~(0x3 << dshift);
    gpiox->MODER |= (cfg->GPIO_pin_mode << dshift);
  } else {
    // Enable SysClk
    SYSCFG_ENABLE();

    /********* INTERRUPT LOGIC STARTS HERE **********/
    // Turn on correct EXTI
    static GPIO_TypeDef *const GPIO_base_addrs[8] = GPIOS;
    for (int i = 0; i < GPIO_SIZE(GPIO_base_addrs); i++) {
      if (GPIO_base_addrs[i] != gpiox) continue;

      // Banks are grouped into groups of 4, one per pin (i.e. PA0, PB0, .. PH0 are multiplexed onto EXTI0)
      uint8_t exti_index = cfg->GPIO_pin_number / 4;
      SYSCFG->EXTICR[exti_index] |= (i << (qshift % 16));
    }

    // Configure correct edge

    // Unmask bit in EXTI

    // Make pin input
  }
  // Set output speed - clear bits to 00 and then set
  gpiox->OSPEEDR &= ~(0x3 << dshift);
  gpiox->OSPEEDR |= (cfg->GPIO_pin_speed << dshift);

  // Set output type - clear bits to 0 first and then set
  gpiox->OTYPER &= ~(0x1 << sshift);
  gpiox->OTYPER |= (cfg->GPIO_pin_out_type << sshift);

  // Set pullup/pulldown resistor - clear bits to 00 and then set
  gpiox->PUPDR &= ~(0x3 << dshift);
  gpiox->PUPDR |= (cfg->GPIO_pin_pupd_control << dshift);

  // Configure alt functionality - clear bits to 0000 and then set
  const uint8_t alt_no = cfg->GPIO_pin_number / 8;
  const uint8_t alt_shift = (cfg->GPIO_pin_number * 4) % 32;
  gpiox->AFR[alt_no] &= ~(0xF << alt_shift);
  gpiox->AFR[alt_no] |= cfg->GPIO_pin_alt_func_mode << alt_shift;
}

/**
  * @brief Reset the entire GPIO port
  * 
  * @param p_GPIO_x The base address of the GPIO port to reset
  */
void GPIO_deinit(const GPIO_TypeDef *p_GPIO_x) {
  if (p_GPIO_x == NULL) return;

  const GPIO_TypeDef *GPIO_base_addrs[8] = GPIOS;
  for (int i = 0; i < GPIO_SIZE(GPIO_base_addrs); i++) {
    if (GPIO_base_addrs[i] != p_GPIO_x) continue;

    // Turn reset register on and off
    RCC->AHB1RSTR |= (1 << i);
    RCC->AHB1RSTR &= ~(1 << i);
  }
}

/**
 * @brief Reads the value of the given pin
 *
 * @param p_GPIO_x GPIO base address (and overlaid struct)
 * @param pin Pin to be read
 * @return uint8_t Value of the input associated with the pin
 */
uint8_t GPIO_read_from_input_pin(const GPIO_TypeDef *p_GPIO_x, uint8_t pin) {
  if (p_GPIO_x == NULL) return 0;

  return (p_GPIO_x->IDR >> pin) & 0x1;
}

/**
 * @brief Read the entire value of the GPIO port
 *
 * @param p_GPIO_x GPIO base address (and overlaid struct)
 * @return uint16_t Word containing the value of the GPIO port
 */
uint16_t GPIO_read_from_input_port(const GPIO_TypeDef *p_GPIO_x) {
  if (p_GPIO_x == NULL) return 0;

  return (uint16_t)p_GPIO_x->IDR;
}

/**
 * @brief Value to be written to the given pin, when configured as output
 *
 * @param p_GPIO_x GPIO base address (and overlaid struct)
 * @param pin Pin which will be read
 * @param val Output value - 1 for high, 0 for low
 */
void GPIO_write_to_output_pin(GPIO_TypeDef *p_GPIO_x, uint8_t pin, uint8_t val) {
  if (p_GPIO_x == NULL) return;

  if (val)
    // Reset pin from  1 to 0
    p_GPIO_x->BSRR |= (1 << pin << 16);
  else
    // Set pin from 0 to 1
    p_GPIO_x->BSRR |= (1 << pin);
}

/**
 * @brief Value to write to entire GPIO port, when configured as output
 * Try not to use this. This is not race-condition protected
 * TODO: Maybe re-write this function so it is atomic. That will require some logic as it needs to have each bit parsed and then be mapped to the BSRR reg accordingly
 *
 * @param p_GPIO_x GPIO base address (and overlaid struct)
 * @param val Word containing the value to be written to the GPIO port
 */
void GPIO_write_to_output_port(GPIO_TypeDef *p_GPIO_x, uint16_t val) {
  if (p_GPIO_x == NULL) return;

  p_GPIO_x->ODR = val;
}

/**
 * @brief Toggle the output value of the given pin
 *
 * @param p_GPIO_x GPIO base address (and overlaid struct)
 * @param pin Pin to be toggled
 */
void GPIO_toggle_output_pin(GPIO_TypeDef *p_GPIO_x, uint8_t pin) {
  if (p_GPIO_x == NULL) return;

  uint8_t val = (p_GPIO_x->ODR) & (1 << pin);

  if (val)
    // Reset pin from  1 to 0
    p_GPIO_x->BSRR |= (1 << pin << 16);
  else
    // Set pin from 0 to 1
    p_GPIO_x->BSRR |= (1 << pin);
}
