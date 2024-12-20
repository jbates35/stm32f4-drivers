
/*
 * STM32H723xx_gpio.h
 *
 *  Created on: Dec. 19, 2023
 *      Author: jbates
 */

#ifndef INC_STM32F446XX_GPIO_H_
#define INC_STM32F446XX_GPIO_H_

#include "stm32f446xx.h"

typedef enum { ADC_MODE_SINGLE = 0, ADC_MODE_MULTI, ADC_MODE_SINGLE_CONTINUOUS, ADC_MODE_MULTI_CONTINUOUS } ADCMode_t;
typedef enum {
  ADC_RESOLUTION_12_BIT = 0,
  ADC_RESOLUTION_10_BIT = 1,
  ADC_RESOLUTION_8_BIT = 2,
  ADC_RESOLUTION_6_BIT = 3
} ADCResolution_t;
typedef enum { ADC_INTERRUPT_DIS = 0, ADC_INTERRUPT_EN } ADCInterruptEn_t;

typedef struct {
  uint8_t channel;
  uint8_t sample_time;
} ADCChannelConfig_t;

typedef struct {
  ADCMode_t mode;
  ADCResolution_t resolution;
  ADCInterruptEn_t interrupt_en;
  uint8_t channel_count;
  ADCChannelConfig_t channel_cfg[16];
} ADCConfig_t;

// NOTE:
// Might want to add low threshold/high threshold voltage watch
// Could also add injected channels

typedef struct {
  ADC_TypeDef *p_adc_addr;
  ADCConfig_t cfg;
} ADCHandle_t;

#endif
