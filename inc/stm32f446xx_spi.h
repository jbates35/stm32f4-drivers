#ifndef INC_STM34F446XX_SPI_H_
#define INC_STM34F446XX_SPI_H_

#include <stdint.h>

#include "stm32f446xx.h"

// SPI Master vs slave mode select:
typedef enum { SPI_DEVICE_MODE_SLAVE = 0, SPI_DEVICE_MODE_MASTER } SpiDeviceMode_t;

// Bus configuration (simplex duplex etc)
typedef enum {
  SPI_BUS_CONFIG_FULL_DUPLEX = 0,
  SPI_BUS_CONFIG_SIMPLEX_TX_ONLY,
  SPI_BUS_CONFIG_SIMPLEX_RX_ONLY,
  SPI_BUS_CONFIG_HALF_DUPLEX
} SpiBusConfig_t;

// Data frame format (i.e. 4-bit frames, 8-bit frames, etc)
typedef enum {
  SPI_DFF_4_BIT = 0x3,
  SPI_DFF_8_BIT = 0x7,
  SPI_DFF_16_BIT = 0xf,
  SPI_DFF_24_BIT = 0x17,
  SPI_DFF_32_BIT = 0x1f
} SpiDff_t;

// Clock phase angle (i.e. if cpol is active high, first edge = rising edge, second edge = falling edge)
typedef enum { SPI_CPHA_CAPTURE_FIRST_EDGE = 0, SPI_CPHA_CAPTURE_SECOND_EDGE } SpiCpha_t;

// Clock polarity (does it start with a low or a high? Used with clock phase to dictate rising edge of acctive edge)
typedef enum { SPI_CPOL_CAPTURE_ACTIVE_HIGH = 0, SPI_CPOL_CAPTURE_ACTIVE_LOW } SpiCpol_t;

// Software slave management (enable or disable)
typedef enum { SPI_SSM_DISABLE = 0, SPI_SSM_ENABLE } SpiSsm_t;

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
} SpiBaudDivisor_t;

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
  SpiDeviceMode_t device_mode;
  SpiBusConfig_t bus_config;
  SpiDff_t dff;
  SpiCpol_t cpol;
  SpiCpha_t cpha;
  SpiSsm_t ssm;
  SpiBaudDivisor_t baud_divisor;
} SPI_Config_t;

/**
  * @brief Overall handler which is used in the init
  * @param p_spi_addr The SPI peripheral being used (SPI1, SPI2, ... SPI4)
  * @param cfg The configuration struct used to dictate how the SPI bus should be set up
**/
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
