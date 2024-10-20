#include "stm32f446xx_gpio.h"

#include <stdio.h>

#define ENABLE 1
#define DISABLE 0

/*
 * Peripheral clock setup for GPIO
 *
 * @param p_GPIO_x Pointer to base address for gpio struct
 * @param en_state 1 for enable, 0 for disable
 */
void GPIO_peri_clock_control(GPIO_TypeDef *p_GPIO_x, uint8_t en_state) {
  if (p_GPIO_x == NULL) return;

  static GPIO_TypeDef *const gpio_base_addresses[8] = {GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH};

  for (int i = 0; i < sizeof(gpio_base_addresses) / sizeof(GPIO_TypeDef *); i++) {
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

  // Set mode
  if (cfg->GPIO_pin_mode <= GPIO_MODE_ANALOG) {
    // Esssentially, digital out, analog in, etc. Everything up and until interrupts
  } else {
    // Enable SysClk

    /********* INTERRUPT LOGIC STARTS HERE **********/
    // Turn on correct EXTI

    // Configure correct edge

    // Unmask bit in EXTI

    // Make pin input
  }
  // Set output speed - clear bits to 00 and then set

  // Set output type - clear bits to 0 first and then set

  // Set pullup/pulldown resistor - clear bits to 00 and then set

  // Configure alt functionality - clear bits to 0000 and then set
}
void GPIO_deinit(GPIO_TypeDef *p_GPIO_x);
