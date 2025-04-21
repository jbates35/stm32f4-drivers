#ifndef INC_STM34F446XX_SPI_H_
#define INC_STM34F446XX_SPI_H_

#include <stdint.h>

#include "stm32f446xx.h"

typedef enum { SPI_PERI_CLOCK_DISABLE = 0, SPI_PERI_CLOCK_ENABLE } SPIPeriClockEnable_t;
typedef enum { SPI_DEVICE_MODE_SLAVE = 0, SPI_DEVICE_MODE_MASTER } SPIDeviceMode_t;
typedef enum {
  SPI_BUS_CONFIG_FULL_DUPLEX = 0,
  SPI_BUS_CONFIG_SIMPLEX_TX_ONLY,
  SPI_BUS_CONFIG_SIMPLEX_RX_ONLY,
  SPI_BUS_CONFIG_HALF_DUPLEX
} SPIBusConfig_t;
typedef enum { SPI_DFF_8_BIT = 0, SPI_DFF_16_BIT } SPIDff_t;
typedef enum { SPI_CAPTURE_MODE_RISING = 0, SPI_CAPTURE_MODE_FALLING } SPICaptureMode_t;
typedef enum { SPI_SSM_DISABLE = 0, SPI_SSM_ENABLE } SPISsm_t;
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
typedef enum { SPI_INTERRUPT_TYPE_TX = 0, SPI_INTERRUPT_TYPE_RX } SPIInterruptType_t;
typedef enum {
  SPI_INTERRUPT_READY = 0,
  SPI_INTERRUPT_BUSY,
  SPI_INTERRUPT_DONE,
  SPI_INTERRUPT_INVALID
} SPIInterruptStatus_t;
typedef enum { SPI_DISABLE = 0, SPI_ENABLE } SPIEnable_t;

typedef struct {
  SPIEnable_t en;
  SPIInterruptType_t type;
  uint16_t length;
} SPIInterruptSetup_t;

typedef struct {
  SPIEnable_t tx;
  SPIEnable_t rx;
} SPIDMASetup_t;

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
  SPIInterruptSetup_t interrupt_setup;
  SPIDMASetup_t dma_setup;
  SPIBaudDivisor_t baud_divisor;
  SPIEnable_t enable_on_init;
} SPIConfig_t;

/**
 * @brief Overall handler which is used in the init
 * @param p_spi_addr The SPI peripheral being used (SPI1, SPI2, ... SPI4)
 * @param cfg The configuration struct used to dictate how the SPI bus should be
 *set up
 **/
typedef struct {
  SPI_TypeDef *addr;
  SPIConfig_t cfg;
} SPIHandle_t;

// SPI Inits
int spi_peri_clock_control(const SPI_TypeDef *spi_reg, const SPIPeriClockEnable_t en_state);
int spi_init(const SPIHandle_t *spi_handle);
int spi_deinit(const SPI_TypeDef *spi_reg);

// Blocking SPI calls
int spi_tx_byte(SPI_TypeDef *spi_reg, const uint16_t tx_byte);
int spi_tx_word(SPI_TypeDef *spi_reg, const void *tx_buffer, int len);
uint16_t spi_rx_byte(SPI_TypeDef *spi_reg);
int spi_rx_word(SPI_TypeDef *spi_reg, uint8_t *rx_buffer, int len);
int spi_full_duplex_transfer(SPI_TypeDef *spi_reg, void *tx_buffer, void *rx_buffer, int len);

// Interrupt related SPI calls
int spi_enable_interrupt(SPI_TypeDef *spi_reg, SPIInterruptType_t type, SPIEnable_t en);
int spi_setup_interrupt(const SPI_TypeDef *spi_reg, const SPIInterruptType_t type, char *buffer, const int len);
SPIInterruptStatus_t spi_get_interrupt_status(const SPI_TypeDef *spi_reg, const SPIInterruptType_t type);
int spi_set_interrupt_callback(const SPI_TypeDef *spi_reg, const SPIInterruptType_t type, void (*fnc_ptr)(void));
int spi_irq_handling(SPI_TypeDef *spi_reg);
int spi_start_interrupt_transfer(SPI_TypeDef *spi_reg);
#endif
