#ifndef INC_STM34F446XX_SPI_H_
#define INC_STM34F446XX_SPI_H_

#include <stdint.h>

#include "stm32f446xx.h"

typedef struct {
  uint8_t device_mode;
  uint8_t bus_config;
  uint8_t sclk_speed;
  uint8_t dff;
  uint8_t cpol;
  uint8_t cpha;
  uint8_t ssm;
} SPI_Config_t;

typedef struct {
  SPI_TypeDef *p_spi_addr;
  SPI_Config_t cfg;
} SPI_Handle_t;

int spi_peri_clock_control(const SPI_TypeDef *p_spi_addr, const uint8_t en);

int spi_init(const SPI_Handle_t *p_spi_handle);

int spi_deinit(const SPI_TypeDef *p_spi_addr);

int spi_send_data(const SPI_TypeDef *p_spi_addr, uint8_t *p_tx_buffer, const uint32_t len);

int spi_receive_data(const SPI_TypeDef *p_spi_addr, uint8_t *p_rx_buffer, const uint32_t len);

int spi_irq_handling(const SPI_TypeDef *p_spi_addr);

#endif
