#include "stm32f446xx.h"
#include "stm32f446xx_usart.h"

#define USARTS {USART1, USART2, USART3, UART4, UART5, USART6}
#define USART_RCC_REGS {&RCC->APB2ENR, &RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB2ENR}
#define USART_RSTR_REGS {&RCC->APB2RSTR, &RCC->APB1RSTR, &RCC->APB1RSTR, &RCC->APB1RSTR, &RCC->APB1RSTR, &RCC->APB2RSTR}
#define USART_RCC_POS {4, 17, 18, 19, 20, 5}
#define SIZEOF(arr) ((int)sizeof(arr) / sizeof(arr[0]))
#define SIZEOFP(arr) ((int)sizeof(arr) / sizeof(uint32_t))  // Memory size of stm32f4

static inline int get_usart_index(const USART_TypeDef* addr) {
  const volatile USART_TypeDef* usart_addrs[] = USARTS;
  int usart_index = -1;
  for (int i = 0; i < SIZEOFP(usart_addrs); i++) {
    if (addr == usart_addrs[i]) {
      usart_index = i;
      break;
    }
  }
  return usart_index;
}

static inline uint8_t get_status(const USART_TypeDef* usart_reg, const uint32_t mask) {
  if (usart_reg->SR & mask) return 1;
  return 0;
}

static inline int is_usart_instance(const USART_TypeDef* usart_reg) {
  return (usart_reg == USART1 || usart_reg == USART2 || usart_reg == USART3 || usart_reg == UART4 ||
          usart_reg == UART5 || usart_reg == USART6);
}

USARTStatus_t usart_setup_interrupt(USART_TypeDef* usart_reg, const USARTInterruptConfig_t* setup_info) {}
USARTStatus_t usart_reset_interrupt(const USART_TypeDef* usart_reg) {}
USARTStatus_t usart_start_interrupt(USART_TypeDef* usart_reg) {}
USARTInterruptStatus_t usart_irq_word_handling(USART_TypeDef* usart_reg) {}

USARTIRQType_t usart_irq_handling(const USART_TypeDef* usart_reg) {
  // Normal transmissions
  if (get_status(usart_reg, USART_SR_RXNE)) return USART_IRQ_TYPE_RXNE;
  if (get_status(usart_reg, USART_SR_TXE)) return USART_IRQ_TYPE_TXE;
  if (get_status(usart_reg, USART_SR_TC)) return USART_IRQ_TYPE_TRANSMISSION_COMPLETE;

  // Errors
  if (get_status(usart_reg, USART_SR_FE)) return USART_IRQ_TYPE_FRAMING_ERROR;
  if (get_status(usart_reg, USART_SR_IDLE)) return USART_IRQ_TYPE_IDLE;
  if (get_status(usart_reg, USART_SR_LBD)) return USART_IRQ_TYPE_LINE_BREAK_DETECTED;
  if (get_status(usart_reg, USART_SR_ORE)) return USART_IRQ_TYPE_OVERRUN_ERROR;
  if (get_status(usart_reg, USART_SR_PE)) return USART_IRQ_TYPE_PARITY_ERROR;
  if (get_status(usart_reg, USART_SR_NE)) return USART_IRQ_TYPE_NOISE_DETECTED;

  return USART_IRQ_TYPE_NONE;
}
