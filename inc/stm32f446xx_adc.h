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

typedef enum {
  ADC_RESOLUTION_12_BIT = 0,
  ADC_RESOLUTION_10_BIT,
  ADC_RESOLUTION_8_BIT,
  ADC_RESOLUTION_6_BIT
} ADCResolution_t;
typedef enum { ADC_INTERRUPT_DISABLE = 0, ADC_INTERRUPT_ENABLE } ADCInterruptEn_t;
typedef enum { ADC_INTERRUPT_EOC_SELECT_SINGLE = 0, ADC_INTERRUPT_EOC_SELECT_GROUP } ADCIntEOCSelect_t;
typedef enum { ADC_TRIGGER_MODE_MANUAL = 0, ADC_TRIGGER_MODE_CONTINUOUS, ADC_TRIGGER_MODE_EXT } ADCTriggerMode_t;
typedef enum { ADC_CHANNEL_DISABLE = 0, ADC_CHANNEL_ENABLE } ADCScanEn_t;
typedef enum { ADC_DUAL_MODE_DISABLE = 0, ADC_DUAL_MODE_ENABLE } ADCDualModeEn_t;
typedef enum { ADC_DATA_CONFIG_SEQUENTIAL = 0, ADC_DATA_CONFIG_GROUPED } ADCDataConfig_t;
typedef enum { ADC_DMA_DISABLE = 0, ADC_DMA_ENABLE } ADCDMAEn_t;
typedef enum { ADC_CHANNEL_SPEED_LOW = 0, ADC_CHANNEL_SPEED_MEDIUM, ADC_CHANNEL_SPEED_HIGH } ADCChannelSpeed_t;
typedef enum { ADC_CHANNEL_NON_REVERSED = 0, ADC_CHANNEL_REVERSED } ADCChannelReversed_t;
typedef enum { ADC_DMA_DATA_WIDTH_16_BIT = 0, ADC_DMA_DATA_WIDTH_32_BIT } ADCDMADataWidth_t;

typedef struct {
  uint8_t channel;
  ADCChannelSpeed_t speed;
} ADCChannel_t;

typedef struct {
  ADCScanEn_t en;
  uint8_t sequence_length;
  ADCChannel_t sequence[16];
  ADCChannelReversed_t reversed;
} ADCScanConfig_t;

typedef struct {
  ADCTriggerMode_t mode;
  uint8_t timer_sel;
  uint8_t ccr_sel;
} ADCTriggerConfig_t;

typedef struct {
  ADCDMAEn_t en;
  DMA_Stream_TypeDef *stream;
  uint8_t dma_channel;
  uint32_t data_reg_addr;
  ADCDMADataWidth_t data_width;
} ADCDMAConfig_t;

typedef struct {
  ADCDualModeEn_t en;
  ADCDataConfig_t data_cfg;
} ADCDualConfig_t;

typedef struct {
  ADCResolution_t resolution;
  ADCInterruptEn_t interrupt_en;
  ADCIntEOCSelect_t interrupt_eoc_sel;
  ADCScanConfig_t main_seq_chan_cfg;
  ADCScanConfig_t main_inj_chan_cfg;
  ADCScanConfig_t slave_seq_chan_cfg;
  ADCScanConfig_t slave_inj_chan_cfg;
  ADCTriggerConfig_t trigger_cfg;
  ADCDualConfig_t dual_cfg;
  ADCDMAConfig_t dma_cfg;
} ADCConfig_t;

typedef struct {
  ADCConfig_t cfg;
  ADC_TypeDef *addr;
} ADCHandle_t;

int adc_peri_clock_control(const ADC_TypeDef *base_addr, const uint8_t en_state);

int adc_stream_init(const ADCHandle_t *adc_handle);

float convert_adc_to_temperature(uint16_t adc_val, uint8_t adc_bit_width);

#endif
