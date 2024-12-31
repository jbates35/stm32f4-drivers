/*
 * STM32H723xx_gpio.h
 *
 *  Created on: Dec. 29, 2024
 *      Author: jbates
 */

#ifndef INC_STM32F446XX_DMA_H_
#define INC_STM32F446XX_DMA_H_

#include "stm32f446xx.h"

typedef enum { DMA_IO_TYPE_PERIPHERAL = 0, DMA_IO_TYPE_MEMORY } DMAIOType;
typedef enum { DMA_DATA_SIZE_8_BIT = 0, DMA_DATA_SIZE_16_BIT = 1, DMA_DATA_SIZE_24_BIT = 2 } DMADataSize;
typedef enum { DMA_PRIORITY_LOW = 0, DMA_PRIORITY_MEDIUM, DMA_PRIORITY_HIGH, DMA_PRIORITY_MAX } DMAPriority;
typedef enum { DMA_IO_ARR_STATIC = 0, DMA_IO_ARR_INCREMENT } DMAArrIncrement;
typedef enum { DMA_BUFFER_FINITE = 0, DMA_BUFFER_CIRCULAR } DMACircBuffer;

typedef struct {
  uint32_t addr;
  DMAIOType type;
  DMAArrIncrement inc;
} IOHandle_t;

typedef struct {
  IOHandle_t in;
  IOHandle_t out;
  DMADataSize mem_data_size;
  DMADataSize peri_data_size;
  uint8_t channel;
  DMAPriority priority;
  DMACircBuffer circ_buffer;
} DMAConfig_t;

typedef struct {
  DMAConfig_t cfg;
  DMA_Stream_TypeDef *dma_stream;
} DMAHandle_t;

#endif
