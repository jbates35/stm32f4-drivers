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
void GPIO_peri_clock_control(GPIO_TypeDef *p_GPIO_x, uint8_t en_state) {
  if (p_GPIO_x == NULL) return;

  static GPIO_TypeDef *const gpio_base_addresses[8] = GPIOS;

  for (int i = 0; i < GPIO_SIZE(gpio_base_addresses); i++) {
    if (gpio_base_addresses[i] != p_GPIO_x) continue;

    if (en_state == ENABLE)
      RCC->AHB1ENR |= (1 << i);
    else
      RCC->AHB1ENR &= ~(1 << i);
  }
}

/*
 * Init and de-init of GPIO
 */
void GPIO_init(GPIO_Handle_t *p_GPIO_handle) {
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
void GPIO_deinit(GPIO_TypeDef *p_GPIO_x) {}

/**
 * @brief Read the entire value of the GPIO port
 *
 * @param p_GPIO_x GPIO port information
 * @return uint16_t Word containing the value of the GPIO port
 */
uint16_t GPIO_read_from_input_port(const GPIO_TypeDef *p_GPIO_x) {
  if (p_GPIO_x == NULL) return 0;

  return (uint16_t)p_GPIO_x->IDR;
}

/**
 * @brief Value to be written to the given pin, when configured as output
 *
 * @param p_GPIO_x GPIO port information
 * @param pin Pin which will be read
 * @param val Output value - 1 for high, 0 for low
 */
void GPIO_write_to_output_pin(GPIO_TypeDef *p_GPIO_x, uint8_t pin, uint8_t val) {
  if (p_GPIO_x == NULL) return;

  if (val == 0)
    p_GPIO_x->ODR &= ~(1 << pin);
  else
    p_GPIO_x->ODR |= (1 << pin);
}

/**
 * @brief Value to write to entire GPIO port, when configured as output
 *
 * @param p_GPIO_x GPIO port information
 * @param val Word containing the value to be written to the GPIO port
 */
void GPIO_write_to_output_port(GPIO_TypeDef *p_GPIO_x, uint16_t val) {
  if (p_GPIO_x == NULL) return;

  p_GPIO_x->ODR = val;
}

/**
 * @brief Toggle the output value of the given pin
 *
 * @param p_GPIO_x GPIO port information
 * @param pin Pin to be toggled
 */
void GPIO_toggle_output_pin(GPIO_TypeDef *p_GPIO_x, uint8_t pin) {
  if (p_GPIO_x == NULL) return;

  p_GPIO_x->ODR ^= (1 << pin);
}
