#include "stm32f446xx_spi.h"

#include <string.h>

#include "stm32f446xx.h"

#define SPIS_NUM 4
#define SPIS {SPI1, SPI2, SPI3, SPI4}
#define SPIS_RCC_REGS {&RCC->APB2ENR, &RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB2ENR}
#define SPIS_RCC_POS {RCC_APB2ENR_SPI1EN_Pos, RCC_APB1ENR_SPI2EN_Pos, RCC_APB1ENR_SPI3EN_Pos, RCC_APB2ENR_SPI4EN}
#define SIZEOF(arr) ((int)sizeof(arr) / sizeof(arr[0]))
#define SIZEOFP(arr) ((int)sizeof(arr) / sizeof(uint32_t))  // Memory size of stm32f4

// For keeping track especially when it comes to interrupt instanstiations
typedef struct {
  volatile char *buffer;
  volatile int eles_left;
  volatile int len;
  volatile SPIInterruptStatus_t is_busy;
  volatile void (*fnc_ptr)(void);
} SPIInterruptBuffer_t;

typedef struct {
  SPIInterruptBuffer_t tx;
  SPIInterruptBuffer_t rx;
} SPIInterruptInfo_t;
SPIInterruptInfo_t spi_interrupt_info[SPIS_NUM] = {0};

int handle_spi_tx_int(SPI_TypeDef *spi_reg);
int handle_spi_rx_int(const SPI_TypeDef *spi_reg);

/**
 * @brief  Helper function to get the correct SPI Config
 *
 * @param spi_base_addr The SPI peripheral, base address.
 *
 * @return SPIConfig_t  Returns the :q

 */
static inline SPIInterruptBuffer_t *get_spi_interrupt_info(const SPI_TypeDef *spi_base_addr,
                                                           const SPIInterruptType_t type) {
  const volatile SPI_TypeDef *spi_addrs[SPIS_NUM] = SPIS;
  for (int i = 0; i < SPIS_NUM; i++) {
    if (spi_addrs[i] == spi_base_addr) {
      if (type == SPI_INTERRUPT_TYPE_TX)
        return &spi_interrupt_info[i].tx;
      else
        return &spi_interrupt_info[i].rx;
    }
  }
  return NULL;
}

int spi_peri_clock_control(const SPI_TypeDef *spi_reg, const SPIPeriClockEnable_t en_state) {
  if (spi_reg == NULL) return -1;  // Error: null pointer

  const SPI_TypeDef *spis_arr[] = SPIS;
  int i = 0;

  for (; i < SIZEOFP(spis_arr); i++) {
    if (spis_arr[i] == spi_reg) break;
  }

  if (i >= SIZEOFP(spis_arr)) return -1;

  if (en_state) {
    volatile uint32_t *spi_regs[] = SPIS_RCC_REGS;
    const unsigned int spi_rcc_pos[] = SPIS_RCC_POS;
    *spi_regs[i] |= (1 << spi_rcc_pos[i]);
  }

  return 0;
}

int spi_init(const SPIHandle_t *spi_handle) {
  if (spi_handle == NULL || spi_handle->addr == NULL) return -1;  // NUll pointers

  SPI_TypeDef *spi_reg = spi_handle->addr;
  const SPIConfig_t *cfg = &(spi_handle->cfg);

  spi_reg->CR1 = 0;
  spi_reg->CR2 = 0;

  // Is SPI in master mode or slave mode?
  uint8_t master_mode = cfg->device_mode == SPI_DEVICE_MODE_MASTER ? 1 : 0;
  spi_reg->CR1 |= (master_mode << SPI_CR1_MSTR_Pos);

  // Enable software slave management, or not
  if (cfg->ssm == SPI_SSM_ENABLE) {
    spi_reg->CR1 |= (1 << SPI_CR1_SSM_Pos);
    spi_reg->CR1 |= (1 << SPI_CR1_SSI_Pos);
  } else {
    spi_reg->CR1 |= (1 << SPI_CR2_SSOE_Pos);
  }

  // Handle the different modes
  switch (cfg->bus_config) {
    case SPI_BUS_CONFIG_HALF_DUPLEX:
      spi_reg->CR1 |= (1 << SPI_CR1_BIDIMODE_Pos);
      break;
    case SPI_BUS_CONFIG_SIMPLEX_RX_ONLY:
      spi_reg->CR1 |= (1 << SPI_CR1_RXONLY_Pos);
      break;
    default:
      break;
  }

  // Baud rate
  spi_reg->CR1 |= ((cfg->baud_divisor & 0b111) << SPI_CR1_BR_Pos);

  // Data frame mode
  uint8_t data_frame = cfg->dff ? 1 : 0;
  spi_reg->CR1 |= (data_frame << SPI_CR1_DFF_Pos);

  // Capture mode
  if (cfg->capture_mode == SPI_CAPTURE_MODE_RISING) {
    spi_reg->CR1 |= (1 << SPI_CR1_CPOL_Pos);
    spi_reg->CR1 |= (1 << SPI_CR1_CPHA_Pos);
  }

  // Enable interrupts
  if (cfg->interrupt_setup.en == SPI_INTERRUPT_ENABLE) {
    if (cfg->interrupt_setup.type == SPI_INTERRUPT_TYPE_TX)
      spi_reg->CR2 |= (1 << SPI_CR2_TXEIE_Pos);
    else if (cfg->interrupt_setup.type == SPI_INTERRUPT_TYPE_RX)
      spi_reg->CR2 |= (1 << SPI_CR2_RXNEIE_Pos);
  }

  // Enable DMA
  if (cfg->dma_setup.tx == SPI_DMA_ENABLE) spi_reg->CR2 |= (1 << SPI_CR2_TXDMAEN_Pos);
  if (cfg->dma_setup.rx == SPI_DMA_ENABLE) spi_reg->CR2 |= (1 << SPI_CR2_RXDMAEN_Pos);

  // Enable SPI
  spi_reg->CR1 |= (1 << SPI_CR1_SPE_Pos);

  return 0;
}

int spi_deinit(const SPI_TypeDef *p_spi_addr) { return 0; }

int spi_tx_byte(SPI_TypeDef *spi_port, const uint16_t tx_byte) {
  if (spi_port == NULL) return -1;

  // While the TX Buffer is not empty...
  while (!(spi_port->SR & (1 << SPI_SR_TXE_Pos)));
  spi_port->DR = tx_byte;

  // Clear flag
  spi_port->SR &= ~(1 << SPI_SR_TXE_Pos);

  return 0;
}

int spi_tx_word(SPI_TypeDef *spi_reg, const void *tx_buffer, int len) {
  return spi_full_duplex_transfer(spi_reg, (void *)tx_buffer, (void *)tx_buffer, len);
}

uint16_t spi_rx_byte(SPI_TypeDef *spi_reg) {
  if (spi_reg == NULL) return 0;

  // Wait until the rx is empty
  while (!(spi_reg->SR & (1 << SPI_SR_RXNE_Pos)));
  uint16_t rx_word = spi_reg->DR;

  spi_reg->SR &= ~(1 << SPI_SR_RXNE_Pos);
  return rx_word;
}

int spi_rx_word(SPI_TypeDef *spi_reg, uint8_t *rx_buffer, int len) {
  return spi_full_duplex_transfer(spi_reg, rx_buffer, rx_buffer, len);
}

int spi_full_duplex_transfer(SPI_TypeDef *spi_reg, void *tx_buffer, void *rx_buffer, int len) {
  if (spi_reg == NULL) return -1;

  // Get the amount of bytes per frame - Should be 1 bytes, or 2 bytes (dff=1)
  uint8_t dff_bytes = ((spi_reg->CR1 >> SPI_CR1_DFF_Pos) & 0b1) + 1;

  while (len > 0) {
    // Prepare the tx part of the transmission
    uint16_t tx_word = 0;
    if (dff_bytes == 1)
      tx_word = *((uint8_t *)tx_buffer);
    else {
      tx_word = *((uint16_t *)tx_buffer);
      if (len == 1) tx_word &= 0xFF00;
    }

    // Send byte
    spi_tx_byte(spi_reg, tx_word);

    // Prepare the rx part of the transmission and receive bytes
    uint16_t rx_byte = spi_rx_byte(spi_reg);

    if (dff_bytes == 1 || len == 1)
      *((uint8_t *)rx_buffer) = rx_byte & 0xFF;
    else
      *((uint16_t *)rx_buffer) = rx_byte;

    // Move buffer along to the next available frame
    tx_buffer = (uint8_t *)tx_buffer + dff_bytes;
    rx_buffer = (uint8_t *)rx_buffer + dff_bytes;
    len -= dff_bytes;
  }

  return 0;
}

int spi_enable_interrupt(SPI_TypeDef *spi_reg, SPIInterruptType_t type, SPIInterruptEn_t en) {
  if (spi_reg == NULL) return -1;

  uint8_t type_offset = 0;
  if (type == SPI_INTERRUPT_TYPE_RX)
    type_offset = SPI_CR2_RXNEIE_Pos;
  else if (type == SPI_INTERRUPT_TYPE_TX)
    type_offset = SPI_CR2_TXEIE_Pos;
  else
    return -2;

  if (en == SPI_INTERRUPT_ENABLE) {
    spi_reg->CR2 |= (1 << type_offset);
  } else if (en == SPI_INTERRUPT_DISABLE) {
    spi_reg->CR2 &= ~(1 << type_offset);
  } else
    return -3;

  return 0;
}

int spi_setup_interrupt(const SPI_TypeDef *spi_reg, const SPIInterruptType_t type, char *buffer, const int len) {
  if (spi_reg == NULL) return -1;

  SPIInterruptBuffer_t *buf_info = get_spi_interrupt_info(spi_reg, type);
  if (buf_info->is_busy == SPI_INTERRUPT_BUSY) return -2;

  buf_info->buffer = buffer;
  buf_info->len = len;
  buf_info->eles_left = len;

  return 0;
}

int spi_irq_handling(const SPI_TypeDef *spi_reg) { return 0; }

int set_spi_tx_interrupt_word(const SPI_TypeDef *spi_reg, const char *tx_word, const int len) {
  if (spi_reg == NULL) return -1;

  SPIInterruptInfo_t *int_info = get_spi_interrupt_info(spi_reg);
  if (int_info->tx.is_busy == SPI_INTERRUPT_BUSY) return -2;

  memcpy(&int_info->tx.buffer, tx_word, len);
  int_info->tx.len = len;

  return 0;
}

int set_spi_rx_interrupt_word(const SPI_TypeDef *spi_reg, volatile char *rx_word) {
  if (spi_reg == NULL) return -1;

  SPIInterruptInfo_t *int_info = get_spi_interrupt_info(spi_reg);
  if (int_info->rx.is_busy == SPI_INTERRUPT_BUSY) return -2;

  int_info->rx.buffer = rx_word;

  return 0;
}

int set_spi_rx_interrupt_length(const SPI_TypeDef *spi_reg, const int len) {
  if (spi_reg == NULL) return -1;

  SPIInterruptInfo_t *int_info = get_spi_interrupt_info(spi_reg);
  if (int_info->rx.is_busy == SPI_INTERRUPT_BUSY) return -2;

  int_info->rx.len = len;

  return 0;
}

SPIInterruptStatus_t get_spi_interrupt_tx_status(const SPI_TypeDef *spi_reg) {
  if (spi_reg == NULL) return 1;

  SPIInterruptInfo_t *int_info = get_spi_interrupt_info(spi_reg);
  return int_info->tx.is_busy;
}

SPIInterruptStatus_t get_spi_interrupt_rx_status(const SPI_TypeDef *spi_reg) {
  if (spi_reg == NULL) return 1;

  SPIInterruptInfo_t *int_info = get_spi_interrupt_info(spi_reg);
  return int_info->rx.is_busy;
}

int set_spi_interrupt_tx_callback(const SPI_TypeDef *spi_reg, volatile void (*fnc_ptr)(void)) {
  if (spi_reg == NULL) return -1;

  SPIInterruptInfo_t *int_info = get_spi_interrupt_info(spi_reg);
  if (int_info->tx.is_busy == SPI_INTERRUPT_BUSY) return -2;
  int_info->tx.fnc_ptr = fnc_ptr;

  return 0;
}

int set_spi_interrupt_rx_callback(const SPI_TypeDef *spi_reg, volatile void (*fnc_ptr)(void)) {
  if (spi_reg == NULL) return -1;

  SPIInterruptInfo_t *int_info = get_spi_interrupt_info(spi_reg);
  if (int_info->rx.is_busy == SPI_INTERRUPT_BUSY) return -2;
  int_info->rx.fnc_ptr = fnc_ptr;

  return 0;
}

int handle_spi_tx_int(SPI_TypeDef *spi_reg) {
  if (spi_reg == NULL) return -1;

  SPIInterruptInfo_t *int_info = get_spi_interrupt_info(spi_reg);

  int_info->tx.is_busy = SPI_INTERRUPT_BUSY;

  // Get the amount of bytes per frame - Should be 1 bytes, or 2 bytes (dff=1)
  uint8_t dff_bytes = ((spi_reg->CR1 >> SPI_CR1_DFF_Pos) & 0b1) + 1;
  uint8_t tx_byte;

  if (dff_bytes == 1)
    tx_byte = *((uint8_t *)int_info->tx.buffer);
  else {
    tx_byte = *((uint16_t *)int_info->tx.buffer);
    if (int_info->tx.eles_left == 1) tx_byte &= 0xFF00;
  }

  spi_reg->DR = tx_byte;

  int_info->tx.buffer = (char *)int_info->tx.buffer + dff_bytes;
  int_info->tx.eles_left -= dff_bytes;

  uint8_t ret_val = 1;
  if (int_info->tx.eles_left <= 0) {
    int_info->tx.is_busy = SPI_INTERRUPT_READY;
    int_info->tx.eles_left = int_info->tx.len;
    ret_val = 0;
  }
  return ret_val;
}

int handle_spi_rx_int(const SPI_TypeDef *spi_reg) {
  if (spi_reg == NULL) return -1;
  SPIInterruptInfo_t *int_info = get_spi_interrupt_info(spi_reg);

  // Get the amount of bytes per frame - Should be 1 bytes, or 2 bytes (dff=1)
  uint8_t dff_bytes = ((spi_reg->CR1 >> SPI_CR1_DFF_Pos) & 0b1) + 1;

  // Prepare the rx part of the transmission and receive bytes
  uint16_t rx_byte = spi_reg->DR;

  if (dff_bytes == 1 || int_info->rx.eles_left == 1)
    *((uint8_t *)int_info->rx.buffer) = rx_byte & 0xFF;
  else
    *((uint16_t *)int_info->rx.buffer) = rx_byte;

  // Move buffer along to the next available frame
  int_info->rx.buffer = (char *)int_info->rx.buffer + dff_bytes;
  int_info->rx.eles_left -= dff_bytes;

  uint8_t ret_val = 1;
  if (int_info->rx.eles_left <= 0) {
    int_info->rx.is_busy = SPI_INTERRUPT_READY;
    int_info->rx.eles_left = int_info->rx.len;
    ret_val = 0;
  }
  return ret_val;
}
