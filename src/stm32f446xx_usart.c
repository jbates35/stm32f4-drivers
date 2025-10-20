#include "stm32f446xx_usart.h"

#include "stm32f446xx.h"

#define USARTS {USART1, USART2, USART3, UART4, UART5, USART6}
#define USART_RCC_REGS {&RCC->APB2ENR, &RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB2ENR}
#define USART_RSTR_REGS {&RCC->APB2RSTR, &RCC->APB1RSTR, &RCC->APB1RSTR, &RCC->APB1RSTR, &RCC->APB1RSTR, &RCC->APB2RSTR}
#define USART_RCC_POS {4, 17, 18, 19, 20, 5}
#define SIZEOF(arr) ((int)sizeof(arr) / sizeof(arr[0]))
#define SIZEOFP(arr) ((int)sizeof(arr) / sizeof(uint32_t))  // Memory size of stm32f4

static inline int get_usart_index(const USART_TypeDef *addr) {
  const volatile USART_TypeDef *usart_addrs[] = USARTS;
  int usart_index = -1;
  for (int i = 0; i < SIZEOFP(usart_addrs); i++) {
    if (addr == usart_addrs[i]) {
      usart_index = i;
      break;
    }
  }
  return usart_index;
}

static inline uint8_t get_status(const USART_TypeDef *usart_reg, const uint32_t mask) {
  if (usart_reg->SR & mask) return 1;
  return 0;
}

static inline int is_i2c_instance(const USART_TypeDef *usart_reg) {
  return (usart_reg == USART1 || usart_reg == USART2 || usart_reg == USART3 || usart_reg == UART4 ||
          usart_reg == UART5 || usart_reg == USART6);
}

USARTStatus_t usart_peri_clock_control(const USART_TypeDef *usart_reg, const USARTEnable_t en_state) {
  int index = get_usart_index(usart_reg);
  if (index < 0) return USART_STATUS_INVALID_ADDR;

  volatile uint32_t *usart_reg_arr[] = USART_RCC_REGS;
  const unsigned int usart_rcc_pos[] = USART_RCC_POS;

  if (en_state == USART_DISABLE)
    *usart_reg_arr[index] &= ~(1 << usart_rcc_pos[index]);
  else
    *usart_reg_arr[index] |= (1 << usart_rcc_pos[index]);

  return USART_STATUS_OK;
}

USARTStatus_t usart_init(const USARTHandle_t *usart_handle) {
  USART_TypeDef *addr = usart_handle->addr;

  int index = get_usart_index(addr);
  if (index < 0) return USART_STATUS_INVALID_ADDR;

  const USARTConfig_t *cfg = &usart_handle->cfg;

  uint8_t freq_mhz = cfg->peri_clock_freq_hz / (int)1E6;

  uint32_t cr1_word = 0, cr2_word = 0, cr3_word = 0, brr_word = 0;

  // baud rate calculation
  // BR = Fck / ( 8 * (2 - OVER8) * USARTDIV)
  // Therefore, USARTDIV = Fck / ( 8 * (2 - OVER8) * BR)
  uint32_t freq = cfg->peri_clock_freq_hz;
  uint32_t divisor = 16 * cfg->baud_rate;
  uint32_t usart_div = freq / divisor;
  uint32_t remainder = freq % divisor;
  uint32_t usart_frac = 16 * remainder / divisor;
  brr_word = (usart_div << 4) | (usart_frac & 0xF);

  // Select word length - 8 data + n stop, or 9 data + n stop
  if (cfg->word_length == USART_WORD_LENGTH_9_BIT_DATA) cr1_word |= USART_CR1_M;
  if (cfg->en_on_start == USART_ENABLE) cr1_word |= USART_CR1_UE;

  // Parity bits
  if (cfg->parity_type == USART_PARITY_ODD)
    cr1_word |= (USART_CR1_PCE | USART_CR1_PS);
  else if (cfg->parity_type == USART_PARITY_EVEN)
    cr1_word |= USART_CR1_PCE;

  // Directionality
  if (cfg->mode == USART_MODE_BIDIRECTIONAL || cfg->mode == USART_MODE_TX_ONLY) cr1_word |= USART_CR1_TE;
  if (cfg->mode == USART_MODE_BIDIRECTIONAL || cfg->mode == USART_MODE_RX_ONLY) cr1_word |= USART_CR1_RE;

  // Synchronous mode
  if (cfg->synchronous == USART_SYNCHRONOUS) cr2_word |= USART_CR2_CLKEN;

  // Stop bit count
  if (cfg->stop_bit_count == USART_STOP_BITS_TWO) cr2_word |= USART_CR2_STOP_1;

  // Flow control
  if (cfg->hw_flow_control == USART_HW_FLOW_CTS) cr3_word |= USART_CR3_CTSE;
  if (cfg->hw_flow_control == USART_HW_FLOW_RTS) cr3_word |= USART_CR3_RTSE;

  addr->CR3 = cr3_word;
  addr->CR2 = cr2_word;
  addr->CR1 = cr1_word;
  addr->BRR = brr_word;

  return USART_STATUS_OK;
}

USARTStatus_t usart_deinit(const USART_TypeDef *usart_reg) {
  int index = get_usart_index(usart_reg);
  if (index < 0) return USART_STATUS_INVALID_ADDR;

  volatile uint32_t *usart_rstr_arr[] = USART_RSTR_REGS;
  const unsigned int usart_rcc_pos[] = USART_RCC_POS;

  *usart_rstr_arr[index] |= (1 << usart_rcc_pos[index]);
  __NOP();
  __NOP();
  *usart_rstr_arr[index] &= ~(1 << usart_rcc_pos[index]);

  return USART_STATUS_OK;
}

USARTStatus_t usart_enable(const USART_TypeDef *usart_reg) { return USART_STATUS_OK; }
USARTStatus_t usart_disable(const USART_TypeDef *usart_reg) { return USART_STATUS_OK; }
USARTStatus_t usart_tx_byte_blocking(const USART_TypeDef *usart_reg, uint16_t tx_buff) { return USART_STATUS_OK; }
uint16_t usart_rx_byte_blocking(const USART_TypeDef *usart_reg) { return 0; }
USARTStatus_t usart_tx_word_blocking(const USART_TypeDef *usart_reg, void *tx_buff, uint16_t len) {
  return USART_STATUS_OK;
}
USARTStatus_t usart_rx_word_blocking(const USART_TypeDef *usart_reg, void *rx_buff, uint16_t len) {
  return USART_STATUS_OK;
}
