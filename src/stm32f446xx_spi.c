#include "stm32f446xx_spi.h"

#include <stdio.h>

#include "stm32f446xx.h"

#define SPIS {SPI1, SPI2, SPI3, SPI4}
#define SPIS_RCC_REGS {&RCC->APB2ENR, &RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB2ENR}
#define SPIS_RCC_POS {RCC_APB2ENR_SPI1EN_Pos, RCC_APB1ENR_SPI2EN_Pos, RCC_APB1ENR_SPI3EN_Pos, RCC_APB2ENR_SPI4EN}
#define SIZEOF(arr) ((int)sizeof(arr) / sizeof(arr[0]))
#define SIZEOFP(arr) ((int)sizeof(arr) / sizeof(uint32_t))  // Memory size of stm32f4

SPIConfig_t spi1_config = {0}, spi2_config = {0}, spi3_config = {0};

int spi_peri_clock_control(const SPI_TypeDef *p_spi_addr, const uint8_t en_state) {
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

  // Enable SPI
  spi->CR1 |= (1 << SPI_CR1_SPE_Pos);

  return 0;
}

int spi_deinit(const SPI_TypeDef *p_spi_addr) { return 0; }

int spi_send_data(const SPI_TypeDef *p_spi_addr, uint8_t *p_tx_buffer, const uint32_t len) { return 0; }

int spi_receive_data(const SPI_TypeDef *p_spi_addr, uint8_t *p_rx_buffer, const uint32_t len) { return 0; }

int spi_irq_handling(const SPI_TypeDef *p_spi_addr) { return 0; }
