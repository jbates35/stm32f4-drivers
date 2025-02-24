/*
 * STM32H723xx_adc.h
 *
 *  Created on: Jan. 19, 2024
 *      Author: jbates
 */

#ifndef INC_STM32F446XX_ADC_H_
#define INC_STM32F446XX_ADC_H_

#include <stdint.h>
#include <stdio.h>

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
  ADC_TRIGGER_MODE_EXTI11
} ADCTriggerMode_t;
typedef enum {
  ADC_TRIGGER_JT1TR = 1,
  ADC_TRIGGER_JT2TR = 3,
  ADC_TRIGGER_T2TR = 6,
  ADC_TRIGGER_T3TR = 8,
  ADC_TRIGGER_JT4TR = 9,
  ADC_TRIGGER_JT5TR = 11,
  ADC_TRIGGER_T8TR = 14
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
typedef enum {
  ADC_INTERRUPT_TYPE_AWD = 0,
  ADC_INTERRUPT_TYPE_EOC,
  ADC_INTERRUPT_TYPE_JEOC,
  ADC_INTERRUPT_TYPE_JSTART,
  ADC_INTERRUPT_TYPE_START
} ADCInterruptType_t;

/**
 * @brief  ADC Channel configuration structure definition
 * 
 * This structure is used to configure a specific ADC channel and its speed.
 * 
 * @param channel  Specifies the ADC channel will be sampled.
 * @param speed    Specifies the speed of the ADC channel conversion.
 */
typedef struct {
  uint8_t channel;
  ADCChannelSpeed_t speed;
} ADCChannel_t;

/**
 * @brief  ADC Scan Configuration structure definition
 * 
 * This structure is used to configure the ADC scan mode.
 * 
 * @param en                Specifies whether scan mode is enabled.
 * @param sequence_length   Specifies the length of the scan sequence.
 * @param sequence          Specifies the channels and their respective speeds in the scan sequence.
 */
typedef struct {
  ADCScanEn_t en;
  uint8_t sequence_length;
  ADCChannel_t sequence[16];
} ADCScanConfig_t;

/**
 * @brief  ADC Trigger Configuration structure definition
 * 
 * This structure is used to configure the ADC trigger mode.
 * 
 * @param mode              Specifies the trigger mode (manual, continuous, timer, exti11).
 * @param timer_sel         Specifies the timer selection for the trigger, if timer mode is chosen.
 * @param edge_sel          Specifies the edge selection for the trigger, if exti11 or timer modes are chosen.
 * @param channel_type_sel  Specifies the channel type selection for the trigger (normal channel or injected channel or both).
 */
typedef struct {
  ADCTriggerMode_t mode;
  ADCTriggerTimSel_t timer_sel;
  ADCTriggerEdgeSel_t edge_sel;
  ADCTriggerChanSel_t channel_type_sel;
} ADCTriggerConfig_t;

/**
 * @brief  ADC Dual Mode Configuration structure definition
 * 
 * This structure is used to configure the ADC dual mode.
 * 
 * @param en        Specifies whether dual mode is enabled.
 * @param data_cfg  Specifies the data configuration for dual mode (grouped or sequential).
 */
typedef struct {
  ADCDualModeEn_t en;
  ADCDataConfig_t data_cfg;
} ADCDualConfig_t;

/**
 * @brief  ADC Configuration structure definition
 * 
 * This is the main structure used to configure the ADC.
 * 
 * @param resolution          Specifies the resolution of the ADC (6-bit to 12-bit)
 * @param interrupt_en        Specifies whether interrupts are enabled.
 * @param eoc_sel             Tells the chip when to declare EOC, whether after a single or grouped conversion.
 * @param main_seq_chan_cfg   Specifies the main sequence channel configuration.
 * @param main_inj_chan_cfg   Specifies the main injected channel configuration.
 * @param slave_seq_chan_cfg  Specifies the slave sequence channel configuration.
 * @param slave_inj_chan_cfg  Specifies the slave injected channel configuration.
 * @param trigger_cfg         Specifies the way the ADC module is triggered.
 * @param inj_autostart       Specifies whether injected channels auto-start when SWSTART is used.
 * @param dual_cfg            Specifies the dual mode configuration.
 * @param temp_or_bat_en      Specifies whether temperature or battery sensor is enabled.
 */
typedef struct {
  ADCResolution_t resolution;
  ADCInterruptEn_t interrupt_en;
  ADCEOCSelect_t eoc_sel;
  ADCScanConfig_t main_seq_chan_cfg;
  ADCScanConfig_t main_inj_chan_cfg;
  ADCScanConfig_t slave_seq_chan_cfg;
  ADCScanConfig_t slave_inj_chan_cfg;
  ADCTriggerConfig_t trigger_cfg;
  ADCInjAutostart_t inj_autostart;
  ADCDualConfig_t dual_cfg;
  ADCTempOrBatEn_t temp_or_bat_en;
} ADCConfig_t;

/**
 * @brief  ADC Handle structure definition
 * 
 * This structure is used to handle the ADC configuration and address.
 * 
 * @param cfg   Specifies the ADC configuration.
 * @param addr  Specifies the base address of the ADC peripheral (ADC1, ADC2, ADC3)
 */
typedef struct {
  ADCConfig_t cfg;
  ADC_TypeDef *addr;
} ADCHandle_t;

/**
 * @brief  Controls the ADC peripheral clock.
 * 
 * This function enables or disables the clock for the specified ADC peripheral.
 * 
 * @param base_addr  Pointer to the base address of the ADC peripheral.
 * @param en_state   Specifies whether to enable or disable the clock.
 * 
 * @return int  Returns 0 on success, or a negative error code on failure.
 */
int adc_peri_clock_control(const ADC_TypeDef *base_addr, const ADCPeriClockEn_t en_state);

/**
 * @brief  Initializes the ADC peripheral.
 * 
 * This function configures the specified ADC peripheral according to the provided handle.
 * 
 * @param adc_handle  Pointer to an ADC handle structure that contains the configuration information for the specified ADC.
 * 
 * @return int  Returns 0 on success, or a negative error code on failure.
 */
int adc_init(const ADCHandle_t *adc_handle);

/**
 * @brief  Performs a single ADC sample.
 * 
 * This function performs a single sample on the specified ADC channel.
 * If the operation is blocking, it waits for the conversion to complete.
 * Should not be used when ADC is configured in scan or dual mode.
 * 
 * @param adc_reg       Pointer to the ADC register structure.
 * @param channel       Specifies the ADC channel to sample.
 * @param channel_speed Specifies the speed of the ADC channel conversion.
 * @param blocking      Specifies whether the function should block until the conversion is complete.
 * 
 * @return uint16_t  Returns the ADC conversion result.
 */
uint16_t adc_single_sample(ADC_TypeDef *adc_reg, const uint8_t channel, const ADCChannelSpeed_t channel_speed,
                           const ADCBlocking_t blocking);

/**
 * @brief  Scans and samples the ADC.
 * 
 * This function initiates a scan and sample operation on the specified ADC.
 * 
 * @param adc_reg  Pointer to the ADC register structure.
 * 
 * @return int  Returns 0 on success, or -1 if the ADC register pointer is NULL.
 */
inline static int adc_scan_sample(ADC_TypeDef *adc_reg) {
  if (adc_reg == NULL) return -1;
  adc_reg->CR2 |= (1 << ADC_CR2_SWSTART_Pos);
  return 0;
}

/**
 * @brief  Initiates an injected scan and sample operation on the specified ADC.
 * 
 * This function starts an injected scan and sample operation on the specified ADC.
 * If the operation is blocking, it waits for the conversion to complete.
 * 
 * @param adc_reg  Pointer to the ADC register structure.
 * @param blocking Specifies whether the function should block until the conversion is complete.
 * 
 * @return int  Returns 0 on success, or -1 if the ADC register pointer is NULL.
 */
int adc_inj_scan_sample(ADC_TypeDef *adc_reg, const ADCBlocking_t blocking);

/**
 * @brief  Initiates a dual sample operation on ADC1.
 * 
 * This function starts a dual sample operation on ADC1 by setting the SWSTART bit.
 */
inline static void adc_dual_sample() { ADC1->CR2 |= (1 << ADC_CR2_SWSTART_Pos); }

/**
 * @brief  Retrieves the injected conversion data from the specified ADC channel.
 * 
 * This function returns the data from the injected conversion register for the specified ADC channel.
 * 
 * @param adc_reg  Pointer to the ADC register structure.
 * @param channel  Specifies the ADC channel to retrieve the data from.
 * 
 * @return uint16_t  Returns the injected conversion data for the specified channel,
 *                   or 0xFFFF if the adc_reg pointer is NULL.
 */
inline static uint16_t adc_get_inj_data(ADC_TypeDef *adc_reg, const uint8_t channel) {
  if (adc_reg == NULL) return 0xFFFF;

  // Change channel from 1-4 to 0-3 for array, and protect against values over the number of channels allowed
  uint8_t tmp_channel = ((unsigned int)(channel - 1)) > 3 ? 3 : channel - 1;

  volatile uint32_t *inj_regs[] = {&adc_reg->JDR1, &adc_reg->JDR2, &adc_reg->JDR3, &adc_reg->JDR4};
  return *inj_regs[tmp_channel];
}

/**
 * @brief  Handles ADC interrupts for the specified interrupt type.
 * 
 * This function checks if the specified interrupt type has occurred for the given ADC,
 * clears the interrupt flag if it has, and returns a status indicating whether the interrupt was handled.
 * 
 * @param adc_reg        Pointer to the ADC register structure.
 * @param interrupt_type Specifies the type of interrupt to handle.
 * 
 * @return uint8_t  Returns 1 if the interrupt was handled, or 0 if the interrupt was not set or adc_reg is NULL.
 */
inline static uint8_t adc_irq_handling(ADC_TypeDef *adc_reg, const ADCInterruptType_t interrupt_type) {
  if (adc_reg == NULL) return 0;
  if (adc_reg->SR & 1 << interrupt_type) {
    adc_reg->SR &= ~(1 << interrupt_type);
    return 1;
  }
  return 0;
}

float convert_adc_to_temperature(uint16_t adc_val, ADCResolution_t resolution);

#endif
