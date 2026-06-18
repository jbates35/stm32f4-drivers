#include "stm32f446xx_usart.h"

#include "stm32f446xx.h"

#define USARTS {USART1, USART2, USART3, UART4, UART5, USART6}
#define USART_RCC_REGS {&RCC->APB2ENR, &RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB2ENR}
#define USART_RSTR_REGS {&RCC->APB2RSTR, &RCC->APB1RSTR, &RCC->APB1RSTR, &RCC->APB1RSTR, &RCC->APB1RSTR, &RCC->APB2RSTR}
#define USART_RCC_POS {4, 17, 18, 19, 20, 5}
#define SIZEOF(arr) ((int)sizeof(arr) / sizeof(arr[0]))
#define SIZEOFP(arr) ((int)sizeof(arr) / sizeof(uint32_t))  // Memory size of stm32f4

static inline int get_usart_index(const USART_TypeDef* addr) {
  const volatile USART_TypeDef* usart_addrs[] = USARTS;
  int usart_index = -1;
  for (size_t i = 0; i < SIZEOFP(usart_addrs); i++) {
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

/**
 * @brief Turns the USART peripheral clock on/off
 *
 * @param usart_reg Base address of UART/USART peripheral
 * @param en_state Whether to enable or disable the peripheral clock
 *
 * @return USARTStatus_t Return status of the function
 */
USARTStatus_t usart_peri_clock_control(const USART_TypeDef* usart_reg, const USARTEnable_t en_state) {
  int index = get_usart_index(usart_reg);
  if (index < 0) return USART_STATUS_INVALID_ADDR;

  volatile uint32_t* usart_reg_arr[] = USART_RCC_REGS;
  const unsigned int usart_rcc_pos[] = USART_RCC_POS;

  if (en_state == USART_DISABLE)
    *usart_reg_arr[index] &= ~(1 << usart_rcc_pos[index]);
  else
    *usart_reg_arr[index] |= (1 << usart_rcc_pos[index]);

  return USART_STATUS_OK;
}

/**
 * @brief Initializes the usart peripheral with the configuration struct in the usart_handle
 *
 * @param usart_handle Handle containing the base address of the usart perpiheral, plus the configuration struct
 *
 * @return USARTStatus_t Return status of the function
 */
USARTStatus_t usart_init(const USARTHandle_t* usart_handle) {
  USART_TypeDef* addr = usart_handle->addr;

  int index = get_usart_index(addr);
  if (index < 0) return USART_STATUS_INVALID_ADDR;

  const USARTConfig_t* cfg = &usart_handle->cfg;

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

  // Select word length - 8 data + n stop, or 9 data + n stop (only if parity is selected)
  if (cfg->parity_type != USART_PARITY_NONE && cfg->word_length == USART_WORD_LENGTH_9_BIT_DATA)
    cr1_word |= USART_CR1_M;

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

  // DMA
  if (cfg->tx_dma_en == USART_ENABLE) cr3_word |= USART_CR3_DMAT;
  if (cfg->rx_dma_en == USART_ENABLE) cr3_word |= USART_CR3_DMAR;

  addr->CR3 = cr3_word;
  addr->CR2 = cr2_word;
  addr->CR1 = cr1_word;
  addr->BRR = brr_word;

  return USART_STATUS_OK;
}

/**
 * @brief Resets the USART peripheral to its default state and disables its clock.
 *
 * @param usart_reg Base address of the USART/UART peripheral
 *
 * @return USARTStatus_t Return status of the function
 */
USARTStatus_t usart_deinit(const USART_TypeDef* usart_reg) {
  int index = get_usart_index(usart_reg);
  if (index < 0) return USART_STATUS_INVALID_ADDR;

  volatile uint32_t* usart_rstr_arr[] = USART_RSTR_REGS;
  const unsigned int usart_rcc_pos[] = USART_RCC_POS;

  *usart_rstr_arr[index] |= (1 << usart_rcc_pos[index]);
  __NOP();
  __NOP();
  *usart_rstr_arr[index] &= ~(1 << usart_rcc_pos[index]);

  return USART_STATUS_OK;
}

/**
 * @brief Enables the USART peripheral (sets the UE bit in CR1).
 *
 * @param usart_reg Base address of the USART/UART peripheral
 *
 * @return USARTStatus_t Return status of the function
 */
USARTStatus_t usart_enable(USART_TypeDef* usart_reg) {
  if (!is_usart_instance(usart_reg)) return USART_STATUS_INVALID_ADDR;
  usart_reg->CR1 |= USART_CR1_UE;
  return USART_STATUS_OK;
}

/**
 * @brief Disables the USART peripheral (clears the UE bit in CR1).
 *
 * @param usart_reg Base address of the USART/UART peripheral
 *
 * @return USARTStatus_t Return status of the function
 */
USARTStatus_t usart_disable(USART_TypeDef* usart_reg) {
  if (!is_usart_instance(usart_reg)) return USART_STATUS_INVALID_ADDR;
  usart_reg->CR1 &= ~USART_CR1_UE;
  return USART_STATUS_OK;
}

/**
 * @brief Transmits a single byte over USART, blocking until the data register is empty.
 *
 * @param usart_reg Base address of the USART/UART peripheral
 * @param tx_byte   Byte to transmit
 */
void usart_tx_byte_blocking(USART_TypeDef* usart_reg, uint8_t tx_byte) {
  // While the TX Buffer is not empty...
  while (!get_status(usart_reg, USART_SR_TXE));
  usart_reg->DR = tx_byte;
}

/**
 * @brief Receives a single byte over USART, blocking until data is available.
 *
 * @param usart_reg Base address of the USART/UART peripheral
 *
 * @return uint8_t The received byte
 */
uint8_t usart_rx_byte_blocking(const USART_TypeDef* usart_reg) {
  // While the TX Buffer is not empty...
  while (!get_status(usart_reg, USART_SR_RXNE));
  return (uint8_t)usart_reg->DR;
}

/**
 * @brief Transmits a buffer of bytes over USART, blocking until all bytes are sent.
 *
 * @param usart_reg Base address of the USART/UART peripheral
 * @param tx_buff   Pointer to the transmit buffer
 * @param len       Number of bytes to transmit
 *
 * @return USARTStatus_t Return status of the function
 */
USARTStatus_t usart_tx_word_blocking(USART_TypeDef* usart_reg, void* tx_buff, uint16_t len) {
  if (!is_usart_instance(usart_reg)) return USART_STATUS_INVALID_ADDR;

  for (int i = 0; i < len; i++) {
    uint8_t data = ((uint8_t*)tx_buff)[i] & 0xFF;
    usart_tx_byte_blocking(usart_reg, data);
  }

  return USART_STATUS_OK;
}

/**
 * @brief Receives a buffer of bytes over USART, blocking until all bytes are received.
 *
 * @param usart_reg Base address of the USART/UART peripheral
 * @param rx_buff   Pointer to the receive buffer
 * @param len       Number of bytes to receive
 *
 * @return USARTStatus_t Return status of the function
 */
USARTStatus_t usart_rx_word_blocking(const USART_TypeDef* usart_reg, void* rx_buff, uint16_t len) {
  if (!is_usart_instance(usart_reg)) return USART_STATUS_INVALID_ADDR;

  uint8_t parity = (usart_reg->CR1 & USART_CR1_PCE);
  uint8_t data_nine_bits = (usart_reg->CR1 & USART_CR1_M);

  uint8_t mask = 0xFF;
  if (!parity && !data_nine_bits) mask = 0x7F;

  for (int i = 0; i < len; i++) {
    uint16_t data = usart_rx_byte_blocking(usart_reg);
    ((uint8_t*)rx_buff)[i] = data & mask;
  }

  return USART_STATUS_OK;
}
