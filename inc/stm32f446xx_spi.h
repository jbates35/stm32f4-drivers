#ifndef INC_STM34F446XX_SPI_H_
#define INC_STM34F446XX_SPI_H_

#include <stdint.h>

#include "stm32f446xx.h"

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
typedef enum { SPI_INTERRUPT_TYPE_NONE = 0, SPI_INTERRUPT_TYPE_TX, SPI_INTERRUPT_TYPE_RX } SPIInterruptType_t;
typedef enum {
  SPI_INTERRUPT_READY = 0,
  SPI_INTERRUPT_BUSY,
  SPI_INTERRUPT_DONE,
  SPI_INTERRUPT_INVALID
} SPIInterruptStatus_t;
typedef enum { SPI_INTERRUPT_NONCIRCULAR = 0, SPI_INTERRUPT_CIRCULAR } SPIInterruptCircular_t;
typedef enum { SPI_DISABLE = 0, SPI_ENABLE } SPIEnable_t;

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

/**
 * @brief Controls the peripheral clock for the specified SPI instance.
 *
 * This function enables or disables the peripheral clock for a given SPI instance
 * based on the provided enable/disable state. It validates the SPI instance and
 * updates the corresponding RCC register to control the clock.
 *
 * @param spi_reg Pointer to the SPI peripheral base address.
 * @param en_state Enable/disable state for the peripheral clock. Use 1 to enable
 *            and 0 to disable.
 *
 * @return int Returns 0 on success, or -1 if the input pointer is NULL or the SPI
 *         instance is invalid.
 *
 * @note Ensure that the SPI instance is valid and corresponds to one of the supported
 *       SPI peripherals before calling this function.
 */
int spi_peri_clock_control(const SPI_TypeDef *spi_reg, const SPIEnable_t en_state);

/**
 * @brief Initializes the SPI peripheral with the specified configuration.
 *
 * This function configures the SPI peripheral based on the settings provided
 * in the SPIHandle_t structure. It supports master/slave mode, software/hardware
 * slave management, different bus configurations, baud rate settings, data frame
 * formats, clock polarity/phase, and DMA enablement. Optionally, the SPI can be
 * enabled immediately after initialization.
 *
 * @param spi_handle Pointer to the SPI handle structure containing the
 *            base address of the SPI peripheral and its configuration.
 *
 * @return int Returns 0 on successful initialization, or -1 if the input
 *         pointer is NULL or invalid.
 *
 * @note Ensure that the SPIHandle_t structure is properly initialized before
 *       calling this function. The SPI peripheral must be disabled before
 *       reconfiguring it.
 */
int spi_init(const SPIHandle_t *spi_handle);

/**
 * @brief NOT IMPLEMENTED YET
 */
int spi_deinit(const SPI_TypeDef *spi_reg);

/**
 * @brief Transmits a single byte over SPI.
 *
 * This function sends a single byte of data through the specified SPI port.
 * It waits until the TX buffer is empty before transmitting the byte.
 *
 * @param spi_port Pointer to the SPI peripheral base address.
 * @param tx_byte The byte to be transmitted.
 *
 * @return int Returns 0 on success, or -1 if the SPI port is NULL.
 */
int spi_tx_byte(SPI_TypeDef *spi_reg, const uint16_t tx_byte);

/**
 * @brief Transmits a buffer of data over SPI in full-duplex mode.
 *
 * This function sends a buffer of data through the specified SPI port
 * while simultaneously receiving data. It uses the full-duplex transfer
 * function internally.
 *
 * @param spi_reg Pointer to the SPI peripheral base address.
 * @param tx_buffer Pointer to the buffer containing data to transmit.
 * @param len Length of the data to transmit.
 *
 * @return int Returns 0 on success, or -1 if the SPI port is NULL.
 */
int spi_tx_word(SPI_TypeDef *spi_reg, const void *tx_buffer, int len);

/**
 * @brief Receives a single byte over SPI.
 *
 * This function waits until the RX buffer is not empty and then reads
 * a single byte of data from the specified SPI port.
 *
 * @param spi_reg Pointer to the SPI peripheral base address.
 *
 * @return uint16_t Returns the received byte, or 0 if the SPI port is NULL.
 */
uint16_t spi_rx_byte(SPI_TypeDef *spi_reg);

/**
 * @brief Receives a buffer of data over SPI in full-duplex mode.
 *
 * This function receives a buffer of data through the specified SPI port
 * while simultaneously transmitting data. It uses the full-duplex transfer
 * function internally.
 *
 * @param spi_reg Pointer to the SPI peripheral base address.
 * @param rx_buffer Pointer to the buffer to store received data.
 * @param len Length of the data to receive.
 *
 * @return int Returns 0 on success, or -1 if the SPI port is NULL.
 */
int spi_rx_word(SPI_TypeDef *spi_reg, uint8_t *rx_buffer, int len);

/**
 * @brief Performs a full-duplex SPI transfer.
 *
 * This function transmits and receives data simultaneously over SPI.
 * It handles both 8-bit and 16-bit data frames based on the DFF setting.
 *
 * @param spi_reg Pointer to the SPI peripheral base address.
 * @param tx_buffer Pointer to the buffer containing data to transmit.
 * @param rx_buffer Pointer to the buffer to store received data.
 * @param len Length of the data to transfer.
 *
 * @return int Returns 0 on success, or -1 if the SPI port is NULL.
 */
int spi_full_duplex_transfer(SPI_TypeDef *spi_reg, void *tx_buffer, void *rx_buffer, int len);

/**
 * @brief Sets up an SPI interrupt for transmission or reception.
 *
 * This function configures the SPI interrupt for either TX or RX operations.
 * It initializes the interrupt buffer and enables the interrupt.
 *
 * @param spi_reg Pointer to the SPI peripheral base address.
 * @param type Type of interrupt (TX or RX).
 * @param buffer Pointer to the buffer for interrupt data.
 * @param len Length of the buffer.
 *
 * @return int Returns 0 on success, -1 if the SPI port is NULL, or -2 if the interrupt is busy.
 */
int spi_setup_interrupt(const SPI_TypeDef *spi_reg, const SPIInterruptType_t type, char *buffer, const int len);

/**
 * @brief Configures circular mode for SPI interrupts.
 *
 * This function enables or disables circular mode for SPI interrupts.
 * In circular mode, the interrupt buffers are reused after completion.
 *
 * @param spi_reg Pointer to the SPI peripheral base address.
 * @param circular_en Circular mode enable/disable setting.
 *
 * @return int Returns 0 on success, -1 if the SPI port is NULL, or -2 if the interrupt info is NULL.
 */
int spi_set_circular_interrupt(const SPI_TypeDef *spi_reg, const SPIInterruptCircular_t circular_en);

/**
 * @brief Retrieves the current status of the SPI interrupt.
 *
 * This function returns the current status of the SPI interrupt, such as
 * whether it is ready, busy, or completed.
 *
 * @param spi_reg Pointer to the SPI peripheral base address.
 *
 * @return SPIInterruptStatus_t Returns the interrupt status, or SPI_INTERRUPT_INVALID if the SPI port is NULL.
 */
SPIInterruptStatus_t spi_get_interrupt_status(const SPI_TypeDef *spi_reg);

/**
 * @brief Handles the SPI interrupt and determines its type.
 *
 * This function processes the SPI interrupt and identifies whether it is
 * a TX or RX interrupt.
 *
 * @param spi_reg Pointer to the SPI peripheral base address.
 *
 * @return SPIInterruptType_t Returns the type of interrupt (TX, RX, or NONE).
 */
SPIInterruptType_t spi_irq_handling(const SPI_TypeDef *spi_reg);

/**
 * @brief Handles SPI interrupt word transfers.
 *
 * This function processes TX and RX interrupts for word transfers. It
 * manages the interrupt buffers and invokes the callback upon completion.
 *
 * @param spi_reg Pointer to the SPI peripheral base address.
 *
 * @return int Returns 1 if the transfer is complete, 0 otherwise, or -1 if the SPI port is NULL.
 */
int spi_irq_word_handling(SPI_TypeDef *spi_reg);

/**
 * @brief Sets the callback function for SPI interrupts.
 *
 * This function assigns a user-defined callback function to be invoked
 * when the SPI interrupt completes.
 *
 * @param spi_reg Pointer to the SPI peripheral base address.
 * @param fnc_ptr Pointer to the callback function.
 *
 * @return int Returns 0 on success, or -1 if the SPI port or interrupt info is NULL.
 */
int spi_set_interrupt_callback(const SPI_TypeDef *spi_reg, void (*fnc_ptr)(void));

/**
 * @brief Starts an SPI interrupt word transfer.
 *
 * This function initializes the SPI interrupt for word transfers and
 * enables the necessary interrupt flags.
 *
 * @param spi_reg Pointer to the SPI peripheral base address.
 *
 * @return int Returns 1 if the transfer is started, 0 otherwise, or -1 if the SPI port is NULL.
 */
int spi_start_int_word_transfer(SPI_TypeDef *spi_reg);

/**
 * @brief Enables SPI interrupt transfers for TX and/or RX.
 *
 * This function enables the SPI interrupt for TX and/or RX operations
 * based on the provided enable settings.
 *
 * @param spi_reg Pointer to the SPI peripheral base address.
 * @param tx Enable/disable setting for TX interrupt.
 * @param rx Enable/disable setting for RX interrupt.
 *
 * @return int Returns 1 if the interrupt is enabled, 0 otherwise, or -1 if the SPI port is NULL.
 */
int spi_enable_int_transfer(SPI_TypeDef *spi_reg, SPIEnable_t tx, SPIEnable_t rx);

/**
 * @brief Disables SPI interrupt transfers.
 *
 * This function disables the SPI interrupt for both TX and RX operations
 * and resets the interrupt status.
 *
 * @param spi_reg Pointer to the SPI peripheral base address.
 *
 * @return int Returns 1 if the interrupt is disabled, 0 otherwise, or -1 if the SPI port is NULL.
 */
int spi_disable_int_transfer(SPI_TypeDef *spi_reg);

#endif
