#include "stm32f446xx_adc.h"

#include <stdio.h>

#include "stm32f446xx.h"
#include "stm32f446xx_dma.h"

#define ADCS {ADC1, ADC2, ADC3}
#define ADC_RCC_POS {RCC_APB2ENR_ADC1EN_Pos, RCC_APB2ENR_ADC2EN_Pos, RCC_APB2ENR_ADC3EN_Pos}

#define SIZEOF(arr) ((int)sizeof(arr) / sizeof(arr[0]))
#define SIZEOFP(arr) ((int)sizeof(arr) / sizeof(uint32_t))  // Memory size of stm32f4

void init_normal_scan_channels(ADC_TypeDef *adc, ADCChannel_t *sequence, uint8_t channel_count);
void init_inj_scan_channels(ADC_TypeDef *adc, uint8_t *channels, uint8_t *speeds, uint8_t channel_count);

uint8_t convert_channel_speed(ADCChannelSpeed_t speed);

int adc_peri_clock_control(const ADC_TypeDef *base_addr, const uint8_t en_state) {
  // Avoid null pointer instantiations
  if (base_addr == NULL) return -1;

  ADC_TypeDef *adc_regs[] = ADCS;

  // Get the ADC register index so we can enable
  int adc_reg_cnt = SIZEOFP(adc_regs);
  int i = 0;
  for (; i < adc_reg_cnt; i++) {
    if (adc_regs[i] == base_addr) break;
  }

  // If invalid ADC was supplied, return
  if (i == adc_reg_cnt) return -1;

  // Now enable or disable the rcc reg
  const uint8_t adc_rcc_pos[] = ADC_RCC_POS;

  if (en_state)
    RCC->APB2ENR |= (1 << adc_rcc_pos[i]);
  else
    RCC->APB2RSTR |= (1 << adc_rcc_pos[i]);

  return 0;
}

int adc_stream_init(const ADCHandle_t *adc_handle) {
  if (adc_handle == NULL || adc_handle->addr == NULL) return -1;

  ADC_TypeDef *adc = adc_handle->addr;
  const ADCConfig_t *cfg = &adc_handle->cfg;

  // Clear ADC configs
  adc->CR1 = 0;
  adc->CR2 = 0;

  // Resolution of ADC
  uint8_t res = cfg->resolution <= 0b11 ? cfg->resolution : 0b11;
  adc->CR1 |= (res >> ADC_CR1_RES_Pos);

  // Interrupt enable (right now, only for EOC)
  uint8_t int_en = cfg->interrupt_en ? 1 : 0;
  adc->CR1 |= (int_en << ADC_CR1_EOCIE_Pos);

  // Whether EOC happens after the entire sequence of conversions or after every single individual conversion
  uint8_t eoc_mode = cfg->interrupt_eoc_sel ? 1 : 0;
  adc->CR2 |= (eoc_mode << ADC_CR2_EOCS_Pos);

  // Check if scan mode
  if (cfg->main_seq_chan_cfg.en) {
    // Set up ADC in scan mode
    adc->CR1 |= (1 << ADC_CR1_SCAN_Pos);

    int channel_count = cfg->main_seq_chan_cfg.channel_count;
    if (channel_count < 16) channel_count = 16;

    adc->SQR1 |= (channel_count << ADC_SQR1_L_Pos);
    volatile uint32_t *sqrs[] = {&adc->SQR3, &adc->SQR2, &adc->SQR1};
    volatile uint32_t *smprs[] = {&adc->SMPR1, &adc->SMPR2};
    for (int i = 0; i < channel_count; i++) {
      uint8_t channel = cfg->main_seq_chan_cfg.sequence[i];
      uint8_t speed = cfg->main_seq_chan_cfg.speeds[i];

      uint8_t sqr_reg = i / 6;
      uint8_t sqr_pos = (i % 6) * 5;
      *sqrs[sqr_reg] |= (channel << sqr_pos);

      uint8_t smpr_reg = channel / 10;
      uint8_t smpr_pos = (channel % 10) * 3;
      *smprs[smpr_reg] |= (speed << smpr_pos);
    }
  }
  if (cfg->main_inj_chan_cfg.en) {
    // Set up ADC in scan mode
    adc->CR1 |= (1 << ADC_CR1_SCAN_Pos);
  }

  return 0;
}

void init_normal_scan_channels(ADC_TypeDef *adc, ADCChannel_t *sequence, uint8_t channel_count) {
  if (channel_count == 0) return;

  // Set up ADC non-injected scan mode
  adc->CR1 |= (1 << ADC_CR1_SCAN_Pos);
  adc->SQR1 |= ((channel_count - 1) << ADC_SQR1_L_Pos);

  volatile uint32_t *sqrs[] = {&adc->SQR3, &adc->SQR2, &adc->SQR1};
  volatile uint32_t *smprs[] = {&adc->SMPR1, &adc->SMPR2};
  for (int i = 0; i < channel_count; i++) {
    uint8_t channel = sequence[i].channel;

    uint8_t sqr_reg = i / 6;
    uint8_t sqr_pos = (i % 6) * 5;
    *sqrs[sqr_reg] |= (channel << sqr_pos);

    uint8_t smpr_reg = channel / 10;
    uint8_t smpr_pos = (channel % 10) * 3;

    uint8_t speed = convert_channel_speed(sequence[i].speed);
    *smprs[smpr_reg] |= (speed << smpr_pos);
  }
}

uint8_t convert_channel_speed(ADCChannelSpeed_t speed) {
  switch (speed) {
    case ADC_CHANNEL_SPEED_HIGH:
      return 0b000;
    case ADC_CHANNEL_SPEED_MEDIUM:
      return 0b100;
    case ADC_CHANNEL_SPEED_LOW:
      return 0b111;
  }
}

float convert_adc_to_temperature(uint16_t adc_val, uint8_t adc_bit_width) {
  // Use the bits per ADC sample to calculate the resolution
  uint16_t adc_res = 1;
  for (int i = 0; i < adc_bit_width; i++) adc_res *= 2;

  // Find temperature in celsius
  float v_sense = adc_val * 3.3 / adc_res;
  return 400 * (v_sense - 0.76) + 25;
}

// NOTE:
// Missing:
// EXTI11 trigger
// Watchdog stuff
