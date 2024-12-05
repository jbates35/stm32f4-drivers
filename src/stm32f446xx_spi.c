#include "stm32f446xx_spi.h"

#include <stdio.h>

#include "stm32f446xx.h"

#define SPIS {SPI1, SPI2, SPI3, SPI4}
#define SPIS_RCC_REGS {&RCC->APB2ENR, &RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB2ENR}
#define SPIS_RCC_POS {RCC_APB2ENR_SPI1EN_Pos, RCC_APB1ENR_SPI2EN_Pos, RCC_APB1ENR_SPI3EN_Pos, RCC_APB2ENR_SPI4EN}
#define SIZEOF(arr) ((int)sizeof(arr) / sizeof(arr[0]))
#define SIZEOFP(arr) ((int)sizeof(arr) / sizeof(uint32_t))  // Memory size of stm32f4

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

int spi_init(const SPI_Handle_t *p_spi_handle) { return 0; }

int spi_deinit(const SPI_TypeDef *p_spi_addr) { return 0; }

int spi_send_data(const SPI_TypeDef *p_spi_addr, uint8_t *p_tx_buffer, const uint32_t len) { return 0; }

int spi_receive_data(const SPI_TypeDef *p_spi_addr, uint8_t *p_rx_buffer, const uint32_t len) { return 0; }

int spi_irq_handling(const SPI_TypeDef *p_spi_addr) { return 0; }
