
#ifndef INC_STM34F446XX_USART_H_
#define INC_STM34F446XX_USART_H_

#include <stdint.h>
#include <stdlib.h>

#include "stm32f446xx.h"

typedef enum { USART_MODE_UART = 0, USART_MODE_USART } USARTMode_t;
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

typedef struct {
  uint32_t peri_clock_freq_hz;
  USARTMode_t mode;
  uint8_t baud_rate;
  uint8_t stop_bit_count;
  uint8_t word_length;
  uint8_t parity_type;
  uint8_t hw_flow_control;
} USARTConfig_t;

#endif
