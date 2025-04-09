#ifndef INC_STM34F446XX_SPI_H_
#define INC_STM34F446XX_SPI_H_

#include <stdint.h>

#include "stm32f446xx.h"

// SPI Master vs slave mode select:
typedef enum { SPI_DEVICE_MODE_SLAVE = 0, SPI_DEVICE_MODE_MASTER } SPIDeviceMode_t;

// Bus configuration (simplex duplex etc)
typedef enum {
  SPI_BUS_CONFIG_FULL_DUPLEX = 0,
  SPI_BUS_CONFIG_SIMPLEX_TX_ONLY,
  SPI_BUS_CONFIG_SIMPLEX_RX_ONLY,
  SPI_BUS_CONFIG_HALF_DUPLEX
} SPIBusConfig_t;

// Data frame format (i.e. 4-bit frames, 8-bit frames, etc)
typedef enum {
  SPI_DFF_8_BIT = 0,
  SPI_DFF_16_BIT,
} SPIDff_t;

// Capture on rising edge while data stable, or falling edge
typedef enum { SPI_CAPTURE_MODE_RISING = 0, SPI_CAPTURE_MODE_FALLING } SPICaptureMode_t;

// Software slave management (enable or disable)
typedef enum { SPI_SSM_DISABLE = 0, SPI_SSM_ENABLE } SPISsm_t;

// Baud divisor (Dictates the speed of the spi bus)
typedef enum {
  SPI_BAUD_DIVISOR_2 = 0,
  SPI_BAUD_DIVISOR_4,
  SPI_BAUD_DIVISOR_8,
  SPI_BAUD_DIVISOR_16,
  SPI_BAUD_DIVISOR_32,
  SPI_BAUD_DIVISOR_64,
  SPI_BAUD_DIVISOR_128,
  SPI_BAUD_DIVISOR_256
} SPIBaudDivisor_t;

/**
 * @brief SPI configuration setup which is used to initiailize the SPI
 *
 * DeviceMode = SPI master, spi slave, etc.
 * BusConfig = Full duplex, half duplex, simplex
 * DFF - Data Frame Format (8bit data vs 16bit data)
 * CPHA - clock phase
 * CPOL - clock polarity
 * SSM - slave select management, software vs hardware
 * Speed - SPI clock speed based on divisors
 * */
typedef struct {
  SPIDeviceMode_t device_mode;
  SPIBusConfig_t bus_config;
  SPIDff_t dff;
  SPICaptureMode_t capture_mode;
  SPISsm_t ssm;
  SPIBaudDivisor_t baud_divisor;
} SPIConfig_t;

/**
 * @brief Overall handler which is used in the init
 * @param p_spi_addr The SPI peripheral being used (SPI1, SPI2, ... SPI4)
 * @param cfg The configuration struct used to dictate how the SPI bus should be
 *set up
 **/
typedef struct {
  SPI_TypeDef *p_spi_addr;
  SPIConfig_t cfg;
} SPIHandle_t;

int spi_peri_clock_control(const SPI_TypeDef *p_spi_addr, const uint8_t en);

int spi_init(const SPIHandle_t *p_spi_handle);

int spi_deinit(const SPI_TypeDef *p_spi_addr);

int spi_send_data(const SPI_TypeDef *p_spi_addr, uint8_t *p_tx_buffer, const uint32_t len);

int spi_receive_data(const SPI_TypeDef *p_spi_addr, uint8_t *p_rx_buffer, const uint32_t len);

int spi_irq_handling(const SPI_TypeDef *p_spi_addr);

#endif
