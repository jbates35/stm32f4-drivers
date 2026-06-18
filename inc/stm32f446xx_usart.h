
#ifndef INC_STM34F446XX_USART_H_
#define INC_STM34F446XX_USART_H_

// TODO: Remove 9 bit data mode, but enable it when parity is used

#include <stdint.h>
#include <stdlib.h>

#include "stm32f446xx.h"

typedef enum { USART_STATUS_OK = 0, USART_STATUS_INVALID_ADDR = -1, USART_STATUS_INTERRUPT_BUSY = -2 } USARTStatus_t;
typedef enum { USART_MODE_TX_ONLY, USART_MODE_RX_ONLY, USART_MODE_BIDIRECTIONAL } USARTMode_t;
typedef enum {
  USART_BAUD_RATE_150 = 150,
  USART_BAUD_RATE_300 = 300,
  USART_BAUD_RATE_1200 = 1200,
  USART_BAUD_RATE_2400 = 2400,
  USART_BAUD_RATE_4800 = 4800,
  USART_BAUD_RATE_9600 = 9600,
  USART_BAUD_RATE_19200 = 19200,
  USART_BAUD_RATE_38400 = 38400,
  USART_BAUD_RATE_57600 = 57600,
  USART_BAUD_RATE_115200 = 115200,
  USART_BAUD_RATE_230400 = 230400,
  USART_BAUD_RATE_460800 = 460800,
  USART_BAUD_RATE_921600 = 921600
} USARTBaudRate_t;
typedef enum {
  USART_IRQ_TYPE_NONE,
  USART_IRQ_TYPE_TXE,
  USART_IRQ_TYPE_RXNE,
  USART_IRQ_TYPE_TRANSMISSION_COMPLETE,
  USART_IRQ_TYPE_IDLE,
  USART_IRQ_TYPE_LINE_BREAK_DETECTED,
  USART_IRQ_TYPE_OVERRUN_ERROR,
  USART_IRQ_TYPE_NOISE_DETECTED,
  USART_IRQ_TYPE_FRAMING_ERROR,
  USART_IRQ_TYPE_PARITY_ERROR
} USARTIRQType_t;
typedef enum {
  USART_INTERRUPT_STATUS_READY = 0,
  USART_INTERRUPT_STATUS_BUSY,
  USART_INTERRUPT_STATUS_DONE,
  USART_INTERRUPT_STATUS_WAITING,
  USART_INTERRUPT_STATUS_ERROR,
  USART_INTERRUPT_STATUS_INVALID_ADDR
} USARTInterruptStatus_t;
typedef enum { USART_STOP_BITS_ONE, USART_STOP_BITS_TWO } USARTStopBitCount_t;
typedef enum { USART_WORD_LENGTH_8_BIT_DATA, USART_WORD_LENGTH_9_BIT_DATA } USARTWordLength_t;
typedef enum { USART_ASYNCHRONOUS, USART_SYNCHRONOUS } USARTSynchronous_t;
typedef enum { USART_PARITY_NONE, USART_PARITY_EVEN, USART_PARITY_ODD } USARTPartityType_t;
typedef enum { USART_HW_FLOW_NONE, USART_HW_FLOW_CTS, USART_HW_FLOW_RTS } USARTHWFlowControl_t;
typedef enum { USART_ACCEPT_ESCAPE_CHARS, USART_IGNORE_ESCAPE_CHARS } USARTIgnoreEscapeChars_t;
typedef enum { USART_DISABLE = 0, USART_ENABLE } USARTEnable_t;

/**
 * @brief USART buffer configuration.
 *
 * Contains various flags, the buffer pointer and its length, and a callback function for interrupt-based transfers.
 */
typedef struct {
  USARTEnable_t en;                           /** Required to set up the interrupt */
  void* buff;                                 /** Pointer to data buffer */
  int32_t len;                                /** Length of buffer in bytes */
  USARTEnable_t tx_circular_en;               /** TX Only - Enable or disable circular mode */
  USARTEnable_t rx_length_byte_en;            /** RX Only - let the first byte be the length of the expected array */
  USARTIgnoreEscapeChars_t rx_ignore_escapes; /** RX only - ignore '\0' - This should probably not be used at the same
                                                 time as rx_length_byte_en .. */
  void (*callback)(void);                     /** Callback function on completion */
} USARTBuffer_t;

/**
 * @brief USART interrupt configuration.
 *
 * Groups the TX and RX buffer configurations together with interrupt enable flags
 * for transmission-complete, idle-line, and error events.
 */
typedef struct {
  USARTBuffer_t tx;                  /**< Transmit buffer configuration */
  USARTBuffer_t rx;                  /**< Receive buffer configuration */
  USARTEnable_t tx_complete_en;      /**< Enable transmission-complete interrupt */
  USARTEnable_t idle_en;             /**< Enable idle-line detection interrupt */
  USARTEnable_t error_interrupts_en; /**< Enable error interrupts (overrun, noise, framing, parity) */
} USARTInterruptConfig_t;

/**
 * @brief USART peripheral configuration.
 *
 * Specifies all settings required to initialise a USART/UART peripheral.
 */
typedef struct {
  uint32_t peri_clock_freq_hz;          /**< Peripheral (APB) clock frequency in Hz, used for baud-rate calculation */
  USARTMode_t mode;                     /**< TX-only, RX-only, or bidirectional mode */
  USARTBaudRate_t baud_rate;            /**< Desired baud rate */
  USARTStopBitCount_t stop_bit_count;   /**< Number of stop bits (1 or 2) */
  USARTWordLength_t word_length;        /**< Data frame width (8 or 9 bits) */
  USARTPartityType_t parity_type;       /**< Parity: none, even, or odd */
  USARTSynchronous_t synchronous;       /**< Asynchronous or synchronous (clock-enabled) mode */
  USARTHWFlowControl_t hw_flow_control; /**< Hardware flow control: none, CTS, or RTS */
  USARTEnable_t tx_dma_en;              /**< Enable DMA for transmit */
  USARTEnable_t rx_dma_en;              /**< Enable DMA for receive */
  USARTEnable_t en_on_start;            /**< Enable the peripheral immediately after usart_init() */
} USARTConfig_t;

/**
 * @brief USART peripheral handle.
 *
 * Bundles the peripheral base address with its configuration so a single
 * pointer can be passed to all driver functions.
 */
typedef struct {
  USART_TypeDef* addr; /**< Base address of the USART/UART peripheral */
  USARTConfig_t cfg;   /**< Peripheral configuration */
} USARTHandle_t;

// Control functions
/**
 * @brief Turns the USART peripheral clock on/off
 *
 * @param usart_reg Base address of UART/USART peripheral
 * @param en_state Whether to enable or disable the peripheral clock
 *
 * @return USARTStatus_t Return status of the function
 */
USARTStatus_t usart_peri_clock_control(const USART_TypeDef* usart_reg, const USARTEnable_t en_state);

/**
 * @brief Initializes the usart peripheral with the configuration struct in the usart_handle
 *
 * @param usart_handle Handle containing the base address of the usart perpiheral, plus the configuration struct
 *
 * @return USARTStatus_t Return status of the function
 */
USARTStatus_t usart_init(const USARTHandle_t* usart_handle);

/**
 * @brief Resets the USART peripheral to its default state and disables its clock.
 *
 * @param usart_reg Base address of the USART/UART peripheral
 *
 * @return USARTStatus_t Return status of the function
 */
USARTStatus_t usart_deinit(const USART_TypeDef* usart_reg);

/**
 * @brief Enables the USART peripheral (sets the UE bit in CR1).
 *
 * @param usart_reg Base address of the USART/UART peripheral
 *
 * @return USARTStatus_t Return status of the function
 */
USARTStatus_t usart_enable(USART_TypeDef* usart_reg);

/**
 * @brief Disables the USART peripheral (clears the UE bit in CR1).
 *
 * @param usart_reg Base address of the USART/UART peripheral
 *
 * @return USARTStatus_t Return status of the function
 */
USARTStatus_t usart_disable(USART_TypeDef* usart_reg);

// Blocking tx/rx
/**
 * @brief Transmits a single byte over USART, blocking until the data register is empty.
 *
 * @param usart_reg Base address of the USART/UART peripheral
 * @param tx_byte   Byte to transmit
 */
void usart_tx_byte_blocking(USART_TypeDef* usart_reg, uint8_t tx_byte);

/**
 * @brief Receives a single byte over USART, blocking until data is available.
 *
 * @param usart_reg Base address of the USART/UART peripheral
 *
 * @return uint8_t The received byte
 */
uint8_t usart_rx_byte_blocking(const USART_TypeDef* usart_reg);

/**
 * @brief Transmits a buffer of bytes over USART, blocking until all bytes are sent.
 *
 * @param usart_reg Base address of the USART/UART peripheral
 * @param tx_buff   Pointer to the transmit buffer
 * @param len       Number of bytes to transmit
 *
 * @return USARTStatus_t Return status of the function
 */
USARTStatus_t usart_tx_word_blocking(USART_TypeDef* usart_reg, void* tx_buff, uint16_t len);

/**
 * @brief Receives a buffer of bytes over USART, blocking until all bytes are received.
 *
 * @param usart_reg Base address of the USART/UART peripheral
 * @param rx_buff   Pointer to the receive buffer
 * @param len       Number of bytes to receive
 *
 * @return USARTStatus_t Return status of the function
 */
USARTStatus_t usart_rx_word_blocking(const USART_TypeDef* usart_reg, void* rx_buff, uint16_t len);

// Interrupt based tx/rx
/**
 * @brief Configures TX and RX interrupts for the given USART peripheral.
 *
 * Stores the buffer descriptors and enables the interrupt sources specified in
 * setup_info. Must be called before starting an interrupt-driven transfer.
 *
 * @param usart_reg  Base address of the USART/UART peripheral
 * @param setup_info Pointer to the interrupt configuration struct
 *
 * @return USARTStatus_t Return status of the function
 */
USARTStatus_t usart_setup_interrupt(USART_TypeDef* usart_reg, const USARTInterruptConfig_t* setup_info);

/**
 * @brief Disables the TX interrupt and resets the transmit state to idle.
 *
 * @param usart_reg Base address of the USART/UART peripheral
 *
 * @return USARTStatus_t Return status of the function
 */
USARTStatus_t usart_reset_tx_interrupt(USART_TypeDef* usart_reg);

/**
 * @brief Disables the RX interrupt and resets the receive state to idle.
 *
 * @param usart_reg Base address of the USART/UART peripheral
 *
 * @return USARTStatus_t Return status of the function
 */
USARTStatus_t usart_reset_rx_interrupt(USART_TypeDef* usart_reg);

/**
 * @brief Enables the TXE interrupt to begin an interrupt-driven transmission.
 *
 * The TX buffer must have been set up via usart_setup_tx() or usart_setup_interrupt()
 * before calling this function.
 *
 * @param usart_reg Base address of the USART/UART peripheral
 *
 * @return USARTStatus_t Return status of the function
 */
USARTStatus_t usart_start_tx_interrupt(USART_TypeDef* usart_reg);

/**
 * @brief Updates the TX buffer descriptor without reconfiguring other interrupt settings.
 *
 * @param usart_reg Base address of the USART/UART peripheral
 * @param tx        Pointer to the transmit buffer configuration
 *
 * @return USARTStatus_t Return status of the function
 */
USARTStatus_t usart_setup_tx(USART_TypeDef* usart_reg, const USARTBuffer_t* tx);

/**
 * @brief Updates the RX buffer descriptor without reconfiguring other interrupt settings.
 *
 * @param usart_reg Base address of the USART/UART peripheral
 * @param rx        Pointer to the receive buffer configuration
 *
 * @return USARTStatus_t Return status of the function
 */
USARTStatus_t usart_setup_rx(USART_TypeDef* usart_reg, const USARTBuffer_t* rx);

/**
 * @brief Services a TXE interrupt — writes the next byte from the TX buffer to DR.
 *
 * Should be called from the USARTx_IRQHandler when usart_irq_handling() returns
 * USART_IRQ_TYPE_TXE. Automatically disables the TXE interrupt and invokes the
 * callback when the buffer is exhausted.
 *
 * @param usart_reg Base address of the USART/UART peripheral
 *
 * @return USARTInterruptStatus_t Current TX interrupt status after handling
 */
USARTInterruptStatus_t usart_irq_tx_word_handling(USART_TypeDef* usart_reg);

/**
 * @brief Services an RXNE interrupt — reads the next byte from DR into the RX buffer.
 *
 * Should be called from the USARTx_IRQHandler when usart_irq_handling() returns
 * USART_IRQ_TYPE_RXNE. Invokes the callback when the expected number of bytes
 * has been received.
 *
 * @param usart_reg Base address of the USART/UART peripheral
 *
 * @return USARTInterruptStatus_t Current RX interrupt status after handling
 */
USARTInterruptStatus_t usart_irq_rx_word_handling(USART_TypeDef* usart_reg);

/**
 * @brief Identifies which interrupt event has occurred on the USART peripheral.
 *
 * Reads the status register and returns the highest-priority active IRQ type.
 * Intended to be the first call inside a USARTx_IRQHandler.
 *
 * @param usart_reg Base address of the USART/UART peripheral
 *
 * @return USARTIRQType_t The type of interrupt that fired
 */
USARTIRQType_t usart_irq_handling(const USART_TypeDef* usart_reg);

// Interrupt helpers
/**
 * @brief Returns the current status of the TX interrupt state machine.
 *
 * @param usart_reg Base address of the USART/UART peripheral
 *
 * @return USARTInterruptStatus_t Current TX interrupt status
 */
USARTInterruptStatus_t usart_get_tx_interrupt_status(USART_TypeDef* usart_reg);

/**
 * @brief Returns the current status of the RX interrupt state machine.
 *
 * @param usart_reg Base address of the USART/UART peripheral
 *
 * @return USARTInterruptStatus_t Current RX interrupt status
 */
USARTInterruptStatus_t usart_get_rx_interrupt_status(USART_TypeDef* usart_reg);

/**
 * @brief Returns the number of bytes received in the last interrupt-driven transfer.
 *
 * Useful when the RX length is determined dynamically (e.g. via the length-byte feature).
 *
 * @param usart_reg Base address of the USART/UART peripheral
 *
 * @return uint16_t Number of bytes received
 */
uint16_t usart_get_rx_interrupt_length(USART_TypeDef* usart_reg);

/**
 * @brief Overrides the expected TX transfer length at runtime.
 *
 * Can be used to adjust the number of bytes to transmit after the buffer has
 * already been set up.
 *
 * @param usart_reg Base address of the USART/UART peripheral
 * @param len       New transfer length in bytes
 */
void usart_set_tx_interrupt_length(USART_TypeDef* usart_reg, uint16_t len);

#endif
