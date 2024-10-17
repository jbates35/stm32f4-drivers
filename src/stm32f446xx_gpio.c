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
void GPIO_init(GPIO_Handle_t *p_GPIO_handle);
void GPIO_deinit(GPIO_TypeDef *p_GPIO_x);
