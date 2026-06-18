#ifndef INC_STM34F446XX_I2C_H_
#define INC_STM34F446XX_I2C_H_

#include <stdint.h>
#include <stdio.h>

#include "stm32f446xx.h"

// NOTE: I2C Resistor calculation: https://www.ti.com/lit/an/slva689/slva689.pdf?ts=1750740074661
// For some other electrical characteristic information,
// check page 700 of the reference manual for the stm32f446re

typedef enum {
  I2C_STATUS_OK = 0,
  I2C_STATUS_I2C_ADDR_INVALID = -1,
  I2C_STATUS_INVALID_PERI_FREQUENCY = -2,
  I2C_STATUS_INVALID_CCR_CCR_VAL = -3,
  I2C_STATUS_ACK_FAIL = -4,
  I2C_STATUS_INVALID_I2C_INT_TYPE = -5,
  I2C_STATUS_INTERRUPT_BUSY = -6
} I2CStatus_t;
typedef enum { I2C_NO_STOP = 0, I2C_STOP } I2CStop_t;
typedef enum { I2C_DEVICE_MODE_SLAVE = 0, I2C_DEVICE_MODE_MASTER } I2CDeviceMode_t;
typedef enum { I2C_SCL_MODE_SPEED_SM = 0, I2C_SCL_MODE_SPEED_FM } I2CSclMode_t;
typedef enum { I2C_FM_DUTY_CYCLE_2 = 0, I2C_FM_DUTY_CYCLE_16_9 } I2CFMDutyCycle_t;
typedef enum { I2C_TXRX_DIR_SEND = 0, I2C_TXRX_DIR_RECEIVE } I2CTxRxDirection_t;
typedef enum {
  I2C_IRQ_TYPE_NONE = 0,
  I2C_IRQ_TYPE_STARTED,
  I2C_IRQ_TYPE_ADDR_SENT,
  I2C_IRQ_TYPE_TXE,
  I2C_IRQ_TYPE_RXNE,
  I2C_IRQ_TYPE_BTF,
  I2C_IRQ_TYPE_ERROR_ARBLOST,
  I2C_IRQ_TYPE_ERROR_BUS,
  I2C_IRQ_TYPE_ERROR_ACKFAIL,
  I2C_IRQ_TYPE_ERROR_OVERRUN,
  I2C_IRQ_TYPE_ERROR_TIMEOUT
} I2CIRQType_t;
typedef enum {
  I2C_INTERRUPT_STATUS_READY = 0,
  I2C_INTERRUPT_STATUS_BUSY,
  I2C_INTERRUPT_STATUS_DONE,
  I2C_INTERRUPT_STATUS_WAITING,
  I2C_INTERRUPT_STATUS_ERROR,
  I2C_INTERRUPT_STATUS_INVALID_ADDR
} I2CInterruptStatus_t;
typedef enum { I2C_INTERRUPT_NON_CIRCULAR = 0, I2C_INTERRUPT_CIRCULAR } I2CInterruptCircular_t;
typedef enum { I2C_DISABLE = 0, I2C_ENABLE } I2CEnable_t;

/**
 * @brief I2C slave device setup configuration.
 *
 * Contains the slave device address and clock stretching enable/disable setting.
 */
typedef struct {
  uint8_t device_address;    /**< 7-bit slave device address */
  I2CEnable_t clock_stretch; /**< Enable or disable clock stretching */
} I2CSlaveSetup_t;
/**
 * @brief I2C data buffer descriptor.
 *
 * Holds a pointer to a data buffer and its length for transmit or receive operations.
 */
typedef struct {
  void *buff;  /**< Pointer to data buffer */
  int32_t len; /**< Length of buffer in bytes */
} I2CBuffer_t;

/**
 * @brief I2C interrupt-driven transaction configuration.
 *
 * Describes TX/RX buffers, slave address, circular mode, and callback for interrupt-based transfers.
 */
typedef struct {
  I2CBuffer_t tx;                  /**< Transmit buffer */
  I2CBuffer_t rx;                  /**< Receive buffer */
  uint8_t address;                 /**< 7-bit slave address */
  I2CInterruptCircular_t circular; /**< Enable or disable circular mode */
  void (*callback)(void);          /**< Callback function on completion */
} I2CInterruptConfig_t;

/**
 * @brief I2C DMA interrupt-driven transaction configuration.
 *
 * Describes TX/RX buffers, slave address, circular mode, DMA streams, and callbacks for DMA-based transfers.
 */
typedef struct {
  I2CBuffer_t tx;                                                      /**< Transmit buffer */
  I2CBuffer_t rx;                                                      /**< Receive buffer */
  uint8_t address;                                                     /**< 7-bit slave address */
  I2CInterruptCircular_t circular;                                     /**< Enable or disable circular mode */
  DMA_Stream_TypeDef *tx_stream;                                       /**< DMA stream for TX */
  DMA_Stream_TypeDef *rx_stream;                                       /**< DMA stream for RX */
  void (*dma_set_buffer_cb)(DMA_Stream_TypeDef *, volatile void *ptr); /**< Callback to set DMA buffer */
  void (*dma_start_transfer_cb)(DMA_Stream_TypeDef *, uint32_t);       /**< Callback to start DMA transfer */
  void (*callback)(void);                                              /**< Callback function on completion */
} I2CDMAConfig_t;

/**
 * @brief I2C peripheral configuration.
 *
 * Contains all configuration parameters for initializing an I2C peripheral.
 */
typedef struct {
  uint32_t peri_clock_freq_hz;    /**< Peripheral clock frequency in Hz */
  I2CDeviceMode_t device_mode;    /**< Master or slave mode */
  I2CSlaveSetup_t slave_setup;    /**< Slave device setup */
  I2CSclMode_t scl_mode;          /**< SCL clock mode (standard/fast) */
  I2CFMDutyCycle_t fm_duty_cycle; /**< Fast mode duty cycle */
  I2CEnable_t interrupt_enable;   /**< Enable or disable interrupts */
  I2CEnable_t dma_enable;         /**< Enable or disable DMA */
  I2CEnable_t enable_on_init;     /**< Enable peripheral on initialization */
} I2CConfig_t;

/**
 * @brief I2C peripheral handle.
 *
 * Holds the peripheral base address and its configuration.
 */
typedef struct {
  I2C_TypeDef *addr; /**< Pointer to I2C peripheral base address */
  I2CConfig_t cfg;   /**< I2C configuration structure */
} I2CHandle_t;

/**
 * @brief Turn the I2C peripheral clock on or off
 *
 * @param i2c_reg Base address of the I2C peripheral
 * @param en Enable or disable
 *
 * @return I2CStatus_t Return status of the function
 */
I2CStatus_t i2c_peri_clock_control(const I2C_TypeDef *i2c_reg, const I2CEnable_t en);

/**
 * @brief Initializes the i2c peripheral with the configuration struct in the i2c handle
 *
 * @param i2c_handle Handle containing the base address of the i2c perpiheral, plus the configuration struct
 *
 * @return I2CStatus_t Return status of the function

 */
I2CStatus_t i2c_init(I2CHandle_t *i2c_handle);

/**
 * @brief Deinitializes the specified I2C peripheral by resetting its registers
 *
 * @param i2c_reg Pointer to the I2C peripheral base address
 *
 * @return I2CStatus_t Return status of the function
 */
I2CStatus_t i2c_deinit(const I2C_TypeDef *i2c_reg);

/**
 * @brief Enables the specified I2C peripheral
 *
 * @param i2c_reg Pointer to the I2C peripheral base address
 *
 * @return I2CStatus_t Return status of the function
 */
I2CStatus_t i2c_enable(I2C_TypeDef *i2c_reg);

/**
 * @brief Sends data as I2C master to a slave device (blocking)
 *
 * @param i2c_reg     Pointer to the I2C peripheral base address
 * @param tx_buffer   Pointer to the data buffer to send
 * @param len         Number of bytes to send
 * @param slave_addr  7-bit slave address
 * @param stop_at_end Whether to send STOP condition at the end (I2C_STOP/I2C_NO_STOP)
 *
 * @return I2CStatus_t Return status of the function
 */
I2CStatus_t i2c_master_send(I2C_TypeDef *i2c_reg, void *tx_buffer, int32_t len, uint8_t slave_addr,
                            I2CStop_t stop_at_end);

/**
 * @brief Receives data as I2C master from a slave device (blocking)
 *
 * @param i2c_reg    Pointer to the I2C peripheral base address
 * @param rx_buffer  Pointer to the buffer to store received data
 * @param len        Number of bytes to receive
 * @param slave_addr 7-bit slave address
 *
 * @return I2CStatus_t Return status of the function
 */
I2CStatus_t i2c_master_receive(I2C_TypeDef *i2c_reg, void *rx_buffer, int32_t len, uint8_t slave_addr);

/**
 * @brief Sets up the DMA interrupt configuration for an I2C peripheral.
 *
 * Initializes the interrupt info structure for the given I2C peripheral with the provided
 * DMA transmit and receive buffer information, circular mode, callback, and DMA streams.
 *
 * @param i2c_reg Pointer to the I2C peripheral base address.
 * @param setup_info Pointer to the I2CDMAConfig_t structure containing DMA setup parameters.
 * @return I2CStatus_t Status of the setup operation.
 */
I2CStatus_t i2c_setup_interrupt(I2C_TypeDef *i2c_reg, const I2CInterruptConfig_t *setup_info);

/**
 * @brief Resets the interrupt info structure for the specified I2C peripheral.
 *
 * Resets the TX and RX buffer state and sets the interrupt status to ready.
 *
 * @param i2c_reg Pointer to the I2C peripheral base address.
 * @return I2CStatus_t Status of the reset operation.
 */
I2CStatus_t i2c_reset_interrupt(const I2C_TypeDef *i2c_reg);

/**
 * @brief Starts an I2C interrupt-driven transaction.
 *
 * Sets the interrupt info status to busy and generates a START condition.
 *
 * @param i2c_reg Pointer to the I2C peripheral base address.
 * @return I2CStatus_t Status of the start operation.
 */
I2CStatus_t i2c_start_interrupt(I2C_TypeDef *i2c_reg);

/**
 * @brief Handles I2C word-level interrupt events for transmit and receive.
 *
 * Processes TX and RX events based on the current IRQ reason, manages buffer state,
 * and invokes the callback when the transaction is complete or circular mode is enabled.
 *
 * @param i2c_reg Pointer to the I2C peripheral base address.
 * @return I2CInterruptStatus_t The current status of the interrupt transaction.
 */
I2CInterruptStatus_t i2c_irq_word_handling(I2C_TypeDef *i2c_reg);

/**
 * @brief Determines the current I2C event interrupt type.
 *
 * Checks the I2C status registers to identify the current event (start, address sent, RXNE, TXE, BTF).
 *
 * @param i2c_reg Pointer to the I2C peripheral base address.
 * @return I2CIRQType_t The identified IRQ event type.
 */
I2CIRQType_t i2c_irq_event_handling(const I2C_TypeDef *i2c_reg);

/**
 * @brief Handles I2C error interrupts and clears error flags.
 *
 * Checks for arbitration lost, bus error, acknowledge failure, timeout, and overrun errors.
 * Clears the corresponding error flag and returns the error type.
 *
 * @param i2c_reg Pointer to the I2C peripheral base address.
 * @return I2CIRQType_t The identified IRQ error type, or NONE if no error.
 */
I2CIRQType_t i2c_irq_error_handling(I2C_TypeDef *i2c_reg);

/**
 * @brief Starts an I2C DMA interrupt-driven transaction.
 *
 * Sets the interrupt info status to busy and generates a START condition for DMA-based transfers.
 *
 * @param i2c_reg Pointer to the I2C peripheral base address.
 * @return I2CStatus_t Status of the start operation.
 */
I2CStatus_t i2c_start_interrupt_dma(I2C_TypeDef *i2c_reg);

/**
 * @brief Sets up the DMA interrupt configuration for an I2C peripheral.
 *
 * Initializes the interrupt info structure for the given I2C peripheral with the provided
 * DMA transmit and receive buffer information, circular mode, callback, and DMA streams.
 *
 * @param i2c_reg Pointer to the I2C peripheral base address.
 * @param setup_info Pointer to the I2CDMAConfig_t structure containing DMA setup parameters.
 * @return I2CStatus_t Status of the setup operation.
 */
I2CStatus_t i2c_setup_interrupt_dma(const I2C_TypeDef *i2c_reg, const I2CDMAConfig_t *setup_info);

/**
 * @brief Handles the start phase of an I2C DMA interrupt transaction.
 *
 * Sends the slave address and initializes DMA transfer for TX or RX, depending on the buffer state.
 *
 * @param i2c_reg Pointer to the I2C peripheral base address.
 * @return I2CInterruptStatus_t The current status of the interrupt transaction.
 */
I2CInterruptStatus_t i2c_dma_irq_handling_start(I2C_TypeDef *i2c_reg);  // Goes in I2C EV

/**
 * @brief Handles the end phase of an I2C DMA interrupt transaction.
 *
 * Finalizes the DMA transfer for TX or RX, manages repeated start if needed, and invokes the callback
 * when the transaction is complete or circular mode is enabled.
 *
 * @param i2c_reg Pointer to the I2C peripheral base address.
 * @param dir Direction of the transfer (I2C_TXRX_DIR_SEND or I2C_TXRX_DIR_RECEIVE).
 * @return I2CInterruptStatus_t The current status of the interrupt transaction.
 */
I2CInterruptStatus_t i2c_dma_irq_handling_end(I2C_TypeDef *i2c_reg, I2CTxRxDirection_t dir);  // Goes into DMA EOT

// NOT COMPLETED:
// Any of this in slave mode. If I need them, I'll write them as I go

#endif
