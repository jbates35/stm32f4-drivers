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
 * @param p_GPIO_addr Pointer to base address for gpio struct
 * @param en_state 1 for enable, 0 for disable
 */
void GPIO_peri_clock_control(const GPIO_TypeDef *p_GPIO_addr, const uint8_t en_state) {
  if (p_GPIO_addr == NULL) return;

  static GPIO_TypeDef *const GPIO_base_addrs[8] = GPIOS;

  for (int i = 0; i < GPIO_SIZE(GPIO_base_addrs); i++) {
    if (GPIO_base_addrs[i] != p_GPIO_addr) continue;

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
  GPIO_TypeDef *gpiox = p_GPIO_handle->p_GPIO_addr;
  const GPIO_PinConfig_t *cfg = &(p_GPIO_handle->GPIO_pin_config);

  // For easy bit-shifting, dshift is 2*pin number, whereas sshift is just
  // pin_number
  const uint8_t qshift = 4 * cfg->GPIO_pin_number;
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
    if (cfg->GPIO_pin_mode == GPIO_MODE_IT_FT) {
      EXTI->FTSR |= (1 << sshift);
      EXTI->RTSR &= ~(1 << sshift);
    } else if (cfg->GPIO_pin_mode == GPIO_MODE_IT_RT) {
      EXTI->FTSR &= ~(1 << sshift);
      EXTI->RTSR |= (1 << sshift);
    } else {
      EXTI->FTSR |= (1 << sshift);
      EXTI->RTSR |= (1 << sshift);
    }

    // Unmask bit in EXTI
    EXTI->IMR |= (1 << sshift);

    // Make pin input
    gpiox->MODER &= ~(0x3 << dshift);
    gpiox->MODER |= (GPIO_MODE_IN << dshift);
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
  * @param p_GPIO_addr The base address of the GPIO port to reset
  */
void GPIO_deinit(const GPIO_TypeDef *p_GPIO_addr) {
  if (p_GPIO_addr == NULL) return;

  const GPIO_TypeDef *GPIO_base_addrs[8] = GPIOS;
  for (int i = 0; i < GPIO_SIZE(GPIO_base_addrs); i++) {
    if (GPIO_base_addrs[i] != p_GPIO_addr) continue;

    // Turn reset register on and off
    RCC->AHB1RSTR |= (1 << i);
    RCC->AHB1RSTR &= ~(1 << i);
  }
}

/**
 * @brief Reads the value of the given pin
 *
 * @param p_GPIO_addr GPIO base address (and overlaid struct)
 * @param pin Pin to be read
 * @return uint8_t Value of the input associated with the pin
 */
uint8_t GPIO_get_input(const GPIO_TypeDef *p_GPIO_addr, uint8_t pin) {
  if (p_GPIO_addr == NULL) return 0;

  return (p_GPIO_addr->IDR >> pin) & 0x1;
}

/**
 * @brief Read the entire value of the GPIO port
 *
 * @param p_GPIO_addr GPIO base address (and overlaid struct)
 * @return uint16_t Word containing the value of the GPIO port
 */
uint16_t GPIO_get_input_port(const GPIO_TypeDef *p_GPIO_addr) {
  if (p_GPIO_addr == NULL) return 0;

  return (uint16_t)p_GPIO_addr->IDR;
}

/**
 * @brief Value to be written to the given pin, when configured as output
 *
 * @param p_GPIO_addr GPIO base address (and overlaid struct)
 * @param pin Pin which will be read
 * @param val Output value - 1 for high, 0 for low
 */
void GPIO_set_output(GPIO_TypeDef *p_GPIO_addr, uint8_t pin, uint8_t val) {
  if (p_GPIO_addr == NULL) return;

  if (!val)
    // Reset pin from  1 to 0
    p_GPIO_addr->BSRR |= (1 << pin << 16);
  else
    // Set pin from 0 to 1
    p_GPIO_addr->BSRR |= (1 << pin);
}

/**
 * @brief Value to write to entire GPIO port, when configured as output
 * // TODO: Test this function
 *
 * @param p_GPIO_addr GPIO base address (and overlaid struct)
 * @param val Word containing the value to be written to the GPIO port
 */
void GPIO_set_output_port(GPIO_TypeDef *p_GPIO_addr, uint16_t val) {
  if (p_GPIO_addr == NULL) return;

  // Set logic
  uint32_t set_byte = (uint32_t)0xFFFF & val;
  p_GPIO_addr->BSRR |= set_byte;

  // Reset logic
  uint32_t reset_byte = (uint32_t)0xFFFF & ~val;
  p_GPIO_addr->BSRR |= set_byte << 16;
}

/**
 * @brief Toggle the output value of the given pin
 *
 * @param p_GPIO_addr GPIO base address (and overlaid struct)
 * @param pin Pin to be toggled
 */
void GPIO_toggle_output(GPIO_TypeDef *p_GPIO_addr, uint8_t pin) {
  if (p_GPIO_addr == NULL) return;

  uint8_t val = (p_GPIO_addr->ODR) & (1 << pin);

  if (val)
    // Reset pin from  1 to 0
    p_GPIO_addr->BSRR |= (1 << pin << 16);
  else
    // Set pin from 0 to 1
    p_GPIO_addr->BSRR |= (1 << pin);
}

/**
 * @brief Configure the IRQ for the given pin
 *
 * @param irq_number IRQ number to be configured
 * @param en_state State of the IRQ - 1 for enable, 0 for disable
 */
void GPIO_irq_interrupt_config(uint8_t irq_number, uint8_t en_state) {
  // Enables or disables NVIC
  // Programs ISER if enable, programs ICER if disable
  if (en_state)
    NVIC->ISER[irq_number / 32] |= (1 << (irq_number % 32));
  else
    NVIC->ICER[irq_number / 32] |= (1 << (irq_number % 32));
}

/**
 * @brief Configure the IRQ priority
 *
 * @param irq_number IRQ number which priority should be changed
 * @param irq_priority Priority of the IRQ
 */
void GPIO_irq_priority_config(uint8_t irq_number, uint8_t irq_priority) {
  uint8_t qshift = (irq_number * 8) % 32;  // Will result in bits 0 - 240*8
  uint8_t qindex = (irq_number * 8) / 32;

  NVIC->IP[qindex] &= ~(0xFF << qshift);
  NVIC->IP[qindex] |= (irq_priority << qshift << 4) & (0xF0 << qshift);
}

/**
 * @brief Set the IRQ handling for the given pin
 *
 * @param pin Pin to have IRQ handling set
 *
 * @return 1 if there was an ISR flag. 0 if there wasn't.
 */
int GPIO_irq_handling(uint8_t pin) {
  // Clear ISR flag
  if (EXTI->PR & (1 << pin)) {
    EXTI->PR |= (1 << pin);
    return 1;
  }
  return 0;
}
