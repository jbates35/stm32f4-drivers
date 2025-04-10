#include "stm32f446xx_spi.h"

#include <stdio.h>

#include "stm32f446xx.h"

#define SPIS {SPI1, SPI2, SPI3, SPI4}
#define SPIS_RCC_REGS {&RCC->APB2ENR, &RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB2ENR}
#define SPIS_RCC_POS {RCC_APB2ENR_SPI1EN_Pos, RCC_APB1ENR_SPI2EN_Pos, RCC_APB1ENR_SPI3EN_Pos, RCC_APB2ENR_SPI4EN}
#define SIZEOF(arr) ((int)sizeof(arr) / sizeof(arr[0]))
#define SIZEOFP(arr) ((int)sizeof(arr) / sizeof(uint32_t))  // Memory size of stm32f4

SPIConfig_t spi1_config = {0}, spi2_config = {0}, spi3_config = {0};

int spi_peri_clock_control(const SPI_TypeDef *p_spi_addr, const SPIPeriClockEnable_t en_state) {
  if (p_spi_addr == NULL) return -1;  // Error: null pointer

  const SPI_TypeDef *spis_arr[] = SPIS;
  int i = 0;

  for (; i < SIZEOFP(spis_arr); i++) {
    if (spis_arr[i] == p_spi_addr) break;
  }

  if (i >= SIZEOFP(spis_arr)) return -1;

  if (en_state) {
    volatile uint32_t *spi_regs[] = SPIS_RCC_REGS;
    const unsigned int spi_rcc_pos[] = SPIS_RCC_POS;
    *spi_regs[i] |= (1 << spi_rcc_pos[i]);
  }

  return 0;
}

int spi_init(const SPIHandle_t *p_spi_handle) {
  if (p_spi_handle == NULL || p_spi_handle->p_spi_addr == NULL) return -1;  // NUll pointers

  SPI_TypeDef *spi = p_spi_handle->p_spi_addr;
  const SPIConfig_t *cfg = &(p_spi_handle->cfg);

  spi->CR1 = 0;
  spi->CR2 = 0;

  // Is SPI in master mode or slave mode?
  uint8_t master_mode = cfg->device_mode == SPI_DEVICE_MODE_MASTER ? 1 : 0;
  spi->CR1 |= (master_mode << SPI_CR1_MSTR_Pos);

  // Enable software slave management, or not
  if (cfg->ssm == SPI_SSM_ENABLE) {
    spi->CR1 |= (1 << SPI_CR1_SSM_Pos);
    spi->CR1 |= (1 << SPI_CR1_SSI_Pos);
  } else {
    spi->CR1 |= (1 << SPI_CR2_SSOE_Pos);
  }

  // Handle the different modes
  switch (cfg->bus_config) {
    case SPI_BUS_CONFIG_HALF_DUPLEX:
      spi->CR1 |= (1 << SPI_CR1_BIDIMODE_Pos);
      break;
    case SPI_BUS_CONFIG_SIMPLEX_RX_ONLY:
      spi->CR1 |= (1 << SPI_CR1_RXONLY_Pos);
      break;
    default:
      break;
  }

  // Baud rate
  spi->CR1 |= ((cfg->baud_divisor & 0b111) >> SPI_CR1_BR_Pos);

  // Data frame mode
  uint8_t data_frame = cfg->dff ? 1 : 0;
  spi->CR1 |= (data_frame << SPI_CR1_DFF_Pos);

  // Capture mode
  if (cfg->capture_mode == SPI_CAPTURE_MODE_RISING) {
    spi->CR1 |= (1 << SPI_CR1_CPOL_Pos);
    spi->CR1 |= (1 << SPI_CR1_CPHA_Pos);
  }

  // Enable interrupts
  if (cfg->interrupt_setup.en == SPI_INTERRUPT_ENABLE) {
    if (cfg->interrupt_setup.type == SPI_INTERRUPT_TYPE_TX)
      spi->CR2 |= (1 << SPI_CR2_TXEIE_Pos);
    else if (cfg->interrupt_setup.type == SPI_INTERRUPT_TYPE_RX)
      spi->CR2 |= (1 << SPI_CR2_RXNEIE_Pos);
  }

  // Enable DMA

  // Enable SPI
  spi->CR1 |= (1 << SPI_CR1_SPE_Pos);

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

int spi_tx_word(SPI_TypeDef *spi_port, const void *tx_buffer, int len) {
  return spi_full_duplex_transfer(spi_port, (void *)tx_buffer, (void *)tx_buffer, len);
}

uint16_t spi_rx_byte(SPI_TypeDef *spi_port) {
  if (spi_port == NULL) return 0;

  // Wait until the rx is empty
  while (!(spi_port->SR & (1 << SPI_SR_RXNE_Pos)));
  uint16_t rx_word = spi_port->DR;

  spi_port->SR &= ~(1 << SPI_SR_RXNE_Pos);
  return rx_word;
}

int spi_rx_word(SPI_TypeDef *spi_port, uint8_t *rx_buffer, int len) {
  return spi_full_duplex_transfer(spi_port, rx_buffer, rx_buffer, len);
}

int spi_full_duplex_transfer(SPI_TypeDef *spi_port, void *tx_buffer, void *rx_buffer, int len) {
  if (spi_port == NULL) return -1;

  // Get the amount of bytes per frame - Should be 1 bytes, or 2 bytes (dff=1)
  uint8_t dff_bytes = ((spi_port->CR1 >> SPI_CR1_DFF_Pos) & 0b1) + 1;

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
    spi_tx_byte(spi_port, tx_word);

    // Prepare the rx part of the transmission and receive bytes
    uint16_t rx_byte = spi_rx_byte(spi_port);

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

int spi_send_data(const SPI_TypeDef *p_spi_addr, uint8_t *p_tx_buffer, const uint32_t len) { return 0; }

int spi_receive_data(const SPI_TypeDef *p_spi_addr, uint8_t *p_rx_buffer, const uint32_t len) { return 0; }

int spi_irq_handling(const SPI_TypeDef *p_spi_addr) { return 0; }
