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

typedef enum { ADC_PERI_CLOCK_DISABLE = 0, ADC_PERI_CLOCK_ENABLE } ADCPeriClockEn_t;
typedef enum {
  ADC_RESOLUTION_12_BIT = 0,
  ADC_RESOLUTION_10_BIT,
  ADC_RESOLUTION_8_BIT,
  ADC_RESOLUTION_6_BIT
} ADCResolution_t;
typedef enum { ADC_INTERRUPT_DISABLE = 0, ADC_INTERRUPT_ENABLE } ADCInterruptEn_t;
typedef enum { ADC_INTERRUPT_EOC_SELECT_GROUP = 0, ADC_INTERRUPT_EOC_SELECT_SINGLE } ADCEOCSelect_t;
typedef enum {
  ADC_TRIGGER_MODE_MANUAL = 0,
  ADC_TRIGGER_MODE_CONTINUOUS,
  ADC_TRIGGER_MODE_TIM,
  ADC_TRIGGER_MODE_EXT11
} ADCTriggerMode_t;
typedef enum {
  ADC_TRIGGER_TIM1_CH1 = 0,
  ADC_TRIGGER_TIM1_CH2,
  ADC_TRIGGER_TIM1_CH3,
  ADC_TRIGGER_TIM2_CH2,
  ADC_TRIGGER_TIM2_CH3,
  ADC_TRIGGER_TIM2_CH4,
  ADC_TRIGGER_TIM2_TRGO,
  ADC_TRIGGER_TIM3_CH1,
  ADC_TRIGGER_TIM3_TRGO,
  ADC_TRIGGER_TIM4_CH4,
  ADC_TRIGGER_TIM5_CH1,
  ADC_TRIGGER_TIM5_CH2,
  ADC_TRIGGER_TIM5_CH3,
  ADC_TRIGGER_TIM8_CH1,
  ADC_TRIGGER_TIM8_TRGO
} ADCTriggerTimSel_t;
typedef enum { ADC_TRIGGER_EDGE_RISING = 1, ADC_TRIGGER_EDGE_FALLING, ADC_TRIGGER_EDGE_BOTH } ADCTriggerEdgeSel_t;
typedef enum {
  ADC_TRIGGER_CHANNEL_TYPE_NORMAL = 0,
  ADC_TRIGGER_CHANNEL_TYPE_INJECTED,
  ADC_TRIGGER_CHANNEL_TYPE_BOTH
} ADCTriggerChanSel_t;
typedef enum { ADC_SCAN_DISABLE = 0, ADC_SCAN_ENABLE } ADCScanEn_t;
typedef enum { ADC_INJ_AUTOSTART_OFF = 0, ADC_INJ_AUTOSTART_ON } ADCInjAutostart_t;
typedef enum { ADC_DUAL_MODE_DISABLE = 0, ADC_DUAL_MODE_ENABLE } ADCDualModeEn_t;
typedef enum { ADC_DATA_CONFIG_GROUPED = 0, ADC_DATA_CONFIG_SEQUENTIAL } ADCDataConfig_t;
typedef enum { ADC_DMA_DISABLE = 0, ADC_DMA_ENABLE } ADCDMAEn_t;
typedef enum { ADC_CHANNEL_SPEED_LOW = 0, ADC_CHANNEL_SPEED_MEDIUM, ADC_CHANNEL_SPEED_HIGH } ADCChannelSpeed_t;
typedef enum { ADC_CHANNEL_NON_REVERSED = 0, ADC_CHANNEL_REVERSED } ADCChannelReversed_t;
typedef enum { ADC_DMA_DATA_WIDTH_16_BIT = 0, ADC_DMA_DATA_WIDTH_32_BIT } ADCDMADataWidth_t;
typedef enum { ADC_TEMPORBAT_DISABLE = 0, ADC_TEMPORBAT_TEMPERATURE, ADC_TEMPORBAT_BATTERY } ADCTempOrBatEn_t;
typedef enum { ADC_NON_BLOCKING = 0, ADC_BLOCKING } ADCBlocking_t;
typedef enum { ADC_INTERRUPT_TYPE_AWD = 0, ADC_INTERRUPT_TYPE_EOC, ADC_INTERRUPT_TYPE_JEOC } ADCInterruptType_t;

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
  ADCTriggerTimSel_t timer_sel;
  ADCTriggerEdgeSel_t edge_sel;
  ADCTriggerChanSel_t channel_type_sel;
} ADCTriggerConfig_t;

typedef struct {
  ADCDualModeEn_t en;
  ADCDataConfig_t data_cfg;
} ADCDualConfig_t;

typedef struct {
  ADCResolution_t resolution;          // Done
  ADCInterruptEn_t interrupt_en;       // Done
  ADCEOCSelect_t eoc_sel;              // Done
  ADCScanConfig_t main_seq_chan_cfg;   // Done
  ADCScanConfig_t main_inj_chan_cfg;   // Done
  ADCScanConfig_t slave_seq_chan_cfg;  // Done
  ADCScanConfig_t slave_inj_chan_cfg;  // Done
  ADCTriggerConfig_t trigger_cfg;      // Not done
  ADCInjAutostart_t inj_autostart;     // Done
  ADCDualConfig_t dual_cfg;            // Done
  ADCTempOrBatEn_t temp_or_bat_en;
} ADCConfig_t;

typedef struct {
  ADCConfig_t cfg;
  ADC_TypeDef *addr;
} ADCHandle_t;

int adc_peri_clock_control(const ADC_TypeDef *base_addr, const ADCPeriClockEn_t en_state);

int adc_init(const ADCHandle_t *adc_handle);
uint16_t adc_single_sample(ADC_TypeDef *adc_reg, const uint8_t channel, const ADCChannelSpeed_t channel_speed,
                           const ADCBlocking_t blocking);

// For ADC Scan sample, handle the data with an interrupt, as blocking with EOC is disabled when DMA is involved
uint8_t adc_scan_sample(ADC_TypeDef *adc_reg);

// For ADC Inj scan sample, you can use blocking no problem
uint8_t adc_inj_scan_sample(ADC_TypeDef *adc_reg, const ADCBlocking_t blocking);

uint16_t adc_get_inj_data(ADC_TypeDef *adc_reg, const uint8_t channel);

uint8_t adc_irq_handling(ADC_TypeDef *adc_reg, const ADCInterruptType_t interrupt_type);

float convert_adc_to_temperature(uint16_t adc_val, ADCResolution_t resolution);

#endif
