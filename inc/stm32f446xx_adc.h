/*
 * STM32H723xx_adc.h
 *
 *  Created on: Jan. 19, 2024
 *      Author: jbates
 */

#ifndef INC_STM32F446XX_ADC_H_
#define INC_STM32F446XX_ADC_H_

#include <stdint.h>

#include "stm32f446xx.h"

typedef enum { ADC_DMA_DISABLE = 0, ADC_DMA_ENABLE } ADCDMAEn_t;
typedef enum { ADC_INTERRUPT_DISABLE = 0, ADC_INTERRUPT_ENABLE } ADCInterruptEn_t;

typedef struct {
  ADCDMAEn_t en;
  DMA_Stream_TypeDef *stream;
  uint8_t channel;
} ADCDMACfg_t;

typedef struct {
  ADCDMACfg_t dma_cfg;
} ADCConfig_t;

typedef struct {
  ADCConfig_t cfg;
  ADC_TypeDef *addr;
} ADCHandle_t;

int adc_peri_clock_control(const ADC_TypeDef *base_addr, const uint8_t en_state);

int adc_stream_init(const ADCHandle_t *adc_handle);

#endif
