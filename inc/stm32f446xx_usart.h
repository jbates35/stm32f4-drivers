
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

typedef struct {
  USARTBuffer_t tx; /**< Transmit buffer */
  USARTBuffer_t rx; /**< Receive buffer */
  USARTEnable_t tx_complete_en;
  USARTEnable_t idle_en;
  USARTEnable_t error_interrupts_en;
} USARTInterruptConfig_t;

typedef struct {
  uint32_t peri_clock_freq_hz;
  USARTMode_t mode;
  USARTBaudRate_t baud_rate;
  USARTStopBitCount_t stop_bit_count;
  USARTWordLength_t word_length;
  USARTPartityType_t parity_type;
  USARTSynchronous_t synchronous;
  USARTHWFlowControl_t hw_flow_control;
  USARTEnable_t en_on_start;
} USARTConfig_t;

typedef struct {
  USART_TypeDef* addr;
  USARTConfig_t cfg;
} USARTHandle_t;

// Control functions
USARTStatus_t usart_peri_clock_control(const USART_TypeDef* usart_reg, const USARTEnable_t en_state);
USARTStatus_t usart_init(const USARTHandle_t* usart_handle);
USARTStatus_t usart_deinit(const USART_TypeDef* usart_reg);
USARTStatus_t usart_enable(USART_TypeDef* usart_reg);
USARTStatus_t usart_disable(USART_TypeDef* usart_reg);

// Blockin tx/rx
void usart_tx_byte_blocking(USART_TypeDef* usart_reg, uint8_t tx_byte);
uint8_t usart_rx_byte_blocking(const USART_TypeDef* usart_reg);
USARTStatus_t usart_tx_word_blocking(USART_TypeDef* usart_reg, void* tx_buff, uint16_t len);
USARTStatus_t usart_rx_word_blocking(const USART_TypeDef* usart_reg, void* rx_buff, uint16_t len);

// Interrupt based tx/rx
USARTStatus_t usart_setup_interrupt(USART_TypeDef* usart_reg, const USARTInterruptConfig_t* setup_info);
USARTStatus_t usart_reset_tx_interrupt(USART_TypeDef* usart_reg);
USARTStatus_t usart_reset_rx_interrupt(USART_TypeDef* usart_reg);
USARTStatus_t usart_start_tx_interrupt(USART_TypeDef* usart_reg);
USARTStatus_t usart_setup_tx(USART_TypeDef* usart_reg, const USARTBuffer_t* tx);
USARTStatus_t usart_setup_rx(USART_TypeDef* usart_reg, const USARTBuffer_t* rx);
USARTInterruptStatus_t usart_irq_tx_word_handling(USART_TypeDef* usart_reg);
USARTInterruptStatus_t usart_irq_rx_word_handling(USART_TypeDef* usart_reg);
USARTIRQType_t usart_irq_handling(const USART_TypeDef* usart_reg);

// Interrupt helpers
USARTInterruptStatus_t usart_get_tx_interrupt_status(USART_TypeDef* usart_reg);
USARTInterruptStatus_t usart_get_rx_interrupt_status(USART_TypeDef* usart_reg);
uint16_t usart_get_rx_interrupt_length(USART_TypeDef* usart_reg);
void usart_set_tx_interrupt_length(USART_TypeDef* usart_reg, uint16_t len);

// DMA based tx/rx

#endif
