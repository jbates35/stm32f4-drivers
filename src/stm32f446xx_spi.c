#include "stm32f446xx_spi.h"

int spi_peri_clock_en(const SPI_TypeDef *p_spi_addr, const uint8_t en) { return 0; }

int spi_init(const SPI_Handle_t *p_spi_handle) { return 0; }

int spi_deinit(const SPI_TypeDef *p_spi_addr) { return 0; }

int spi_send_data(const SPI_TypeDef *p_spi_addr, uint8_t *p_tx_buffer, const uint32_t len) { return 0; }

int spi_receive_data(const SPI_TypeDef *p_spi_addr, uint8_t *p_rx_buffer, const uint32_t len) { return 0; }

int spi_irq_handling(const SPI_TypeDef *p_spi_addr) { return 0; }
