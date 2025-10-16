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

  USARTConfig_t *cfg = &usart_handle->cfg;

  uint8_t freq_mhz = cfg->peri_clock_freq_hz / (int)1E6;
  // if (freq_mhz < 2 || freq_mhz > 50) return I2C_STATUS_INVALID_PERI_FREQUENCY;

  uint32_t cr1_word, cr2_word, cr3_word = 0;

  // Select word length - 8 data + n stop, or 9 data + n stop
  uint8_t word_length = (cfg->word_length == USART_WORD_LENGTH_9_BIT_DATA) ? 1 : 0;
  cr1_word |= (word_length << USART_CR1_M_Pos);

  if (cfg->parity_type == USART_PARITY_ODD) {
    cr1_word |= (USART_CR1_PCE | USART_CR1_PS);
  } else if (cfg->parity_type == USART_PARITY_EVEN) {
    cr1_word |= USART_CR1_PCE;
  }

  uint8_t mode = (cfg->mode == USART_MODE_USART) ? 1 : 0;
  cr2_word |= (mode << USART_CR2_CLKEN_Pos);

  uint8_t stop_bit_count = (cfg->stop_bit_count == USART_STOP_BITS_TWO) ? 2 : 0;
  cr2_word |= (stop_bit_count << USART_CR2_STOP_Pos);

  addr->CR1 = cr1_word;
  addr->CR2 = cr2_word;
  addr->CR3 = cr3_word;

  return USART_STATUS_OK;
}
USARTStatus_t usart_deinit(const USART_TypeDef *usart_reg) { return USART_STATUS_OK; }
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
