/*
 * STM32H723xx_gpio.h
 *
 *  Created on: Dec. 29, 2024
 *      Author: jbates
 */

#ifndef INC_STM32F446XX_DMA_H_
#define INC_STM32F446XX_DMA_H_

#include <stdint.h>

#include "stm32f446xx.h"

typedef enum { DMA_IO_TYPE_PERIPHERAL = 0, DMA_IO_TYPE_MEMORY } DMAIOType_t;
typedef enum { DMA_DATA_SIZE_8_BIT = 0, DMA_DATA_SIZE_16_BIT = 1, DMA_DATA_SIZE_32_BIT = 2 } DMADataSize_t;
typedef enum { DMA_PRIORITY_LOW = 0, DMA_PRIORITY_MEDIUM, DMA_PRIORITY_HIGH, DMA_PRIORITY_MAX } DMAPriority_t;
typedef enum { DMA_IO_ARR_STATIC = 0, DMA_IO_ARR_INCREMENT } DMAArrIncrement_t;
typedef enum { DMA_BUFFER_FINITE = 0, DMA_BUFFER_CIRCULAR } DMACircBuffer_t;
typedef enum { DMA_PERIPH_NO_FLOW_CONTROL = 0, DMA_PERIPH_FLOW_CONTROL = 1 } DMAFlowControl_t;
typedef enum { DMA_INTERRUPT_DISABLE = 0, DMA_INTERRUPT_ENABLE } DMAInterruptEn_t;
typedef enum { DMA_PERI_CLOCK_DISABL = 0, DMA_PERI_CLOCK_ENABLE } DMAPeriClockEn_t;
typedef enum {
  DMA_INTERRUPT_TYPE_FIFO_ERROR = 0,
  DMA_INTERRUPT_TYPE_DIRECT_MODE_ERROR = 2,
  DMA_INTERRUPT_TYPE_TRANSFER_ERROR = 3,
  DMA_INTERRUPT_TYPE_HALF_TRANSFER_COMPLETE = 4,
  DMA_INTERRUPT_TYPE_FULL_TRANSFER_COMPLETE = 5
} DMAInterruptType_t;

typedef struct {
  uint32_t addr;
  DMAIOType_t type;
  DMAArrIncrement_t inc;
} IOHandle_t;

typedef struct {
  DMAInterruptEn_t direct_mode_error;
  DMAInterruptEn_t transfer_error;
  DMAInterruptEn_t half_transfer;
  DMAInterruptEn_t full_transfer;
} DMAAllInterruptsEn_t;

typedef struct {
  IOHandle_t in;
  IOHandle_t out;
  DMAAllInterruptsEn_t interrupt_en;
  DMADataSize_t mem_data_size;
  DMADataSize_t peri_data_size;
  uint8_t channel;
  DMAPriority_t priority;
  DMAFlowControl_t flow_control;
  DMACircBuffer_t circ_buffer;
  uint16_t dma_elements;
} DMAConfig_t;

typedef struct {
  DMAConfig_t cfg;
  DMA_Stream_TypeDef *p_stream_addr;
} DMAHandle_t;

int dma_peri_clock_control(const DMA_TypeDef *base_addr, const DMAPeriClockEn_t en_state);

int dma_stream_init(const DMAHandle_t *dma_handle);

int dma_irq_handling(const DMA_TypeDef *base_addr, uint8_t stream_num, DMAInterruptType_t interrupt_type);

#endif
