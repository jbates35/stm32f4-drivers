
#ifndef INC_STM34F446XX_USART_H_
#define INC_STM34F446XX_USART_H_

#include <stdint.h>
#include <stdlib.h>

#include "stm32f446xx.h"

typedef enum { USART_STATUS_OK = 0, USART_STATUS_INVALID_ADDR = -1 } USARTStatus_t;
typedef enum { USART_MODE_UART, USART_MODE_USART } USARTMode_t;
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
typedef enum { USART_STOP_BITS_ONE, USART_STOP_BITS_TWO } USARTStopBitCount_t;
typedef enum { USART_WORD_LENGTH_8_BIT_DATA, USART_WORD_LENGTH_9_BIT_DATA } USARTWordLength_t;
typedef enum { USART_PARITY_NONE, USART_PARITY_EVEN, USART_PARITY_ODD } USARTPartityType_t;
typedef enum { not_sure_yet } USARTHWFlowControl_t;
typedef enum { USART_DISABLE = 0, USART_ENABLE } USARTEnable_t;

typedef struct {
  uint32_t peri_clock_freq_hz;
  USARTMode_t mode;
  USARTBaudRate_t baud_rate;
  USARTStopBitCount_t stop_bit_count;
  USARTWordLength_t word_length;
  USARTPartityType_t parity_type;
  USARTHWFlowControl_t hw_flow_control;
  USARTEnable_t en_on_start;
} USARTConfig_t;

typedef struct {
  USART_TypeDef *addr;
  USARTConfig_t cfg;
} USARTHandle_t;

USARTStatus_t usart_peri_clock_control(const USART_TypeDef *usart_reg, const USARTEnable_t en_state);
USARTStatus_t usart_init(const USARTHandle_t *usart_handle);
USARTStatus_t usart_deinit(const USART_TypeDef *usart_reg);
USARTStatus_t usart_enable(const USART_TypeDef *usart_reg);
USARTStatus_t usart_disable(const USART_TypeDef *usart_reg);
USARTStatus_t usart_tx_byte_blocking(const USART_TypeDef *usart_reg, uint16_t tx_buff);
uint16_t usart_rx_byte_blocking(const USART_TypeDef *usart_reg);
USARTStatus_t usart_tx_word_blocking(const USART_TypeDef *usart_reg, void *tx_buff, uint16_t len);
USARTStatus_t usart_rx_word_blocking(const USART_TypeDef *usart_reg, void *rx_buff, uint16_t len);

#endif
