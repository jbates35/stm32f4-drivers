#include "stm32f446xx_adc.h"

#include <stdio.h>

#include "stm32f446xx.h"

#define ADCS {ADC1, ADC2, ADC3}
#define ADC_RCC_POS {RCC_APB2ENR_ADC1EN_Pos, RCC_APB2ENR_ADC2EN_Pos, RCC_APB2ENR_ADC3EN_Pos}

#define SIZEOF(arr) ((int)sizeof(arr) / sizeof(arr[0]))
#define SIZEOFP(arr) ((int)sizeof(arr) / sizeof(uint32_t))  // Memory size of stm32f4

void init_normal_scan_channels(ADC_TypeDef *adc, const ADCChannel_t *sequence, const uint8_t channel_count);
void init_injected_scan_channels(ADC_TypeDef *adc, const ADCChannel_t *sequence, const uint8_t channel_count);

uint8_t adc_base_scan_sample(ADC_TypeDef *adc_reg, uint8_t start_bit_pos, uint8_t eoc_bit_pos,
                             const ADCBlocking_t blocking);

uint8_t convert_channel_speed(ADCChannelSpeed_t speed);

int adc_peri_clock_control(const ADC_TypeDef *base_addr, const ADCPeriClockEn_t en_state) {
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

  if (en_state == ADC_PERI_CLOCK_ENABLE)
    RCC->APB2ENR |= (1 << adc_rcc_pos[i]);
  else
    RCC->APB2RSTR |= (1 << adc_rcc_pos[i]);

  return 0;
}

int adc_init(const ADCHandle_t *adc_handle) {
  // Null pointer handling
  if (adc_handle == NULL || adc_handle->addr == NULL) return -1;

  ADC_TypeDef *adc_reg = adc_handle->addr;
  const ADCConfig_t *cfg = &adc_handle->cfg;

  // If ADC is set in dual mode, the ADC master is ADC 1
  if (cfg->dual_cfg.en && adc_reg != ADC1) return -2;

  // Clear ADC configs
  adc_reg->CR1 = 0;
  adc_reg->CR2 = 0;

  // Resolution of ADC
  uint8_t res = cfg->resolution <= 0b11 ? cfg->resolution : 0b11;
  adc_reg->CR1 |= (res >> ADC_CR1_RES_Pos);

  // Interrupt enable (right now, only for EOC)
  uint8_t int_en = cfg->interrupt_en ? 1 : 0;
  adc_reg->CR1 |= (int_en << ADC_CR1_EOCIE_Pos);

  // Whether EOC happens after the entire sequence of conversions or after every single individual conversion
  uint8_t eoc_sel = cfg->eoc_sel ? 1 : 0;
  adc_reg->CR2 |= (eoc_sel << ADC_CR2_EOCS_Pos);

  // Configure trigger mode
  if ((cfg->trigger_cfg.mode) == ADC_TRIGGER_MODE_CONTINUOUS) {
    adc_reg->CR2 |= (1 << ADC_CR2_CONT_Pos);
  } else if ((cfg->trigger_cfg.mode) != ADC_TRIGGER_MODE_MANUAL) {
    // Take care of enabling external trigger and edge select
    uint8_t edge_sel = cfg->trigger_cfg.edge_sel;
    if (edge_sel > 3) edge_sel = 3;

    // Take care of what actually triggers the ext trig
    uint8_t trigger_ext_sel = 0;
    if ((cfg->trigger_cfg.mode) == ADC_TRIGGER_MODE_TIM)
      trigger_ext_sel = cfg->trigger_cfg.timer_sel;
    else if ((cfg->trigger_cfg.mode) == ADC_TRIGGER_MODE_EXTI11)
      trigger_ext_sel = 0b1111;
    if (trigger_ext_sel > 0b1111) trigger_ext_sel = 0b1111;

    // Store bits from the two options
    ADCTriggerChanSel_t chan_type_sel = cfg->trigger_cfg.channel_type_sel;
    if (chan_type_sel == ADC_TRIGGER_CHANNEL_TYPE_NORMAL || chan_type_sel == ADC_TRIGGER_CHANNEL_TYPE_BOTH) {
      adc_reg->CR2 |= (edge_sel << ADC_CR2_EXTEN_Pos);
      adc_reg->CR2 |= (trigger_ext_sel << ADC_CR2_EXTSEL_Pos);
    }
    if (chan_type_sel == ADC_TRIGGER_CHANNEL_TYPE_INJECTED || chan_type_sel == ADC_TRIGGER_CHANNEL_TYPE_BOTH) {
      adc_reg->CR2 |= (edge_sel << ADC_CR2_JEXTEN_Pos);
      adc_reg->CR2 |= (trigger_ext_sel << ADC_CR2_JEXTSEL_Pos);
    }
  }

  // Configure temp and battery peripherals (only ADC1 can sample these though)
  if (adc_reg == ADC1) {
    uint8_t temp_en = (cfg->temp_or_bat_en == ADC_TEMPORBAT_TEMPERATURE);
    ADC->CCR |= (temp_en << ADC_CCR_TSVREFE_Pos);

    uint8_t bat_en = (cfg->temp_or_bat_en == ADC_TEMPORBAT_BATTERY);
    ADC->CCR |= (bat_en << ADC_CCR_VBATE_Pos);
  }

  // Configure whether injected is autostarted after normal channels are sampled
  uint8_t inj_autostart = cfg->inj_autostart ? 1 : 0;
  adc_reg->CR1 |= (inj_autostart << ADC_CR1_JAUTO_Pos);

  // Configure scan mode (non-injected)
  if (cfg->main_seq_chan_cfg.en) {
    int channel_count = cfg->main_seq_chan_cfg.sequence_length;
    if (channel_count > 16) channel_count = 16;
    init_normal_scan_channels(adc_reg, cfg->main_seq_chan_cfg.sequence, channel_count);
  }

  // Configure scan mode (injected)
  if (cfg->main_inj_chan_cfg.en) {
    int channel_count = cfg->main_inj_chan_cfg.sequence_length;
    if (channel_count > 4) channel_count = 4;
    init_injected_scan_channels(adc_reg, cfg->main_inj_chan_cfg.sequence, channel_count);
  }

  // Take care of dual mode
  if (cfg->dual_cfg.en) {
    // Configure scan mode (non-injected)
    if (cfg->slave_seq_chan_cfg.en) {
      int channel_count = cfg->slave_seq_chan_cfg.sequence_length;
      if (channel_count > 16) channel_count = 16;
      init_normal_scan_channels(ADC2, cfg->slave_seq_chan_cfg.sequence, channel_count);
    }

    // Configure scan mode (injected)
    if (cfg->slave_inj_chan_cfg.en) {
      int channel_count = cfg->slave_inj_chan_cfg.sequence_length;
      if (channel_count > 4) channel_count = 4;
      init_injected_scan_channels(ADC2, cfg->slave_inj_chan_cfg.sequence, channel_count);
    }

    ADC->CCR = 0;
    ADC->CCR |= (1 << ADC_CCR_DDS_Pos);
    ADC->CCR |= (0b00111 << ADC_CCR_MULTI_Pos);

    uint8_t data_dma_mode = 0;
    if (cfg->dual_cfg.data_cfg == ADC_DATA_CONFIG_SEQUENTIAL)
      data_dma_mode = 0b01;
    else if (cfg->dual_cfg.data_cfg == ADC_DATA_CONFIG_GROUPED)
      data_dma_mode = 0b10;
    ADC->CCR |= (data_dma_mode << ADC_CCR_DMA_Pos);

    // Turn ADC2 on before we exit the dual config section
    ADC2->CR2 |= (1 << ADC_CR2_ADON_Pos);
  }

  adc_reg->CR2 |= (1 << ADC_CR2_ADON_Pos);

  return 0;
}

void init_normal_scan_channels(ADC_TypeDef *adc_reg, const ADCChannel_t *sequence, const uint8_t channel_count) {
  if (channel_count == 0) return;

  // Set up ADC non-injected scan mode
  adc_reg->CR1 |= (1 << ADC_CR1_SCAN_Pos);
  adc_reg->SQR1 |= ((channel_count - 1) << ADC_SQR1_L_Pos);

  // Configure DMA as it's required for scan mode
  adc_reg->CR2 |= (1 << ADC_CR2_DMA_Pos);
  adc_reg->CR2 |= (1 << ADC_CR2_DDS_Pos);

  // Sequence configuration registers
  volatile uint32_t *sqrs[] = {&adc_reg->SQR3, &adc_reg->SQR2, &adc_reg->SQR1};

  // Channel speed registers
  volatile uint32_t *smprs[] = {&adc_reg->SMPR1, &adc_reg->SMPR2};

  for (int i = 0; i < channel_count; i++) {
    uint8_t channel = sequence[i].channel;

    // Configure the SQR sequence position with the channel
    uint8_t sqr_reg = i / 6;
    uint8_t sqr_pos = (i % 6) * 5;
    *sqrs[sqr_reg] |= (channel << sqr_pos);

    // Set sample time for the channel
    uint8_t speed = convert_channel_speed(sequence[i].speed);
    uint8_t smpr_reg = channel / 10;
    uint8_t smpr_pos = (channel % 10) * 3;
    *smprs[smpr_reg] |= (speed << smpr_pos);
  }
}

void init_injected_scan_channels(ADC_TypeDef *adc_reg, const ADCChannel_t *sequence, const uint8_t channel_count) {
  if (channel_count == 0) return;

  // Setup injected channels, scan mode
  adc_reg->CR1 |= (1 << ADC_CR1_SCAN_Pos);
  adc_reg->JSQR |= ((channel_count - 1) << ADC_JSQR_JL_Pos);

  // Configure DMA as it's required for scan mode
  adc_reg->CR2 |= (1 << ADC_CR2_DMA_Pos);
  adc_reg->CR2 |= (1 << ADC_CR2_DDS_Pos);

  // Channel speed registers
  volatile uint32_t *smprs[] = {&adc_reg->SMPR1, &adc_reg->SMPR2};

  // Offset due to JSQR needing to be configured in non-reversed order but starting from 4-n
  uint8_t offset = (4 - channel_count) * 5;

  for (int i = 0; i < channel_count; i++) {
    uint8_t channel = sequence[i].channel;

    // Configure the JSQR sequence position with the channel
    uint8_t jsqr_pos = i * 5 + offset;
    adc_reg->JSQR |= (channel << jsqr_pos);

    // Set sample time for the channel
    // NOTE: This might create a conflict with the normal scan mode but this is unavoidable
    uint8_t speed = convert_channel_speed(sequence[i].speed);
    uint8_t smpr_reg = channel / 10;
    uint8_t smpr_pos = (channel % 10) * 3;
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
    default:
      return 0b000;
  }
}

uint16_t adc_single_sample(ADC_TypeDef *adc_reg, const uint8_t channel, const ADCChannelSpeed_t channel_speed,
                           const ADCBlocking_t blocking) {
  if (adc_reg == NULL) return 0xFFFF;

  // Set up ADC non-injected scan mode
  adc_reg->SQR1 &= ~(0b1111 << ADC_SQR1_L_Pos);

  adc_reg->SQR3 &= ~(0b11111 << ADC_SQR3_SQ1_Pos);
  adc_reg->SQR3 |= ((channel & 0b11111) << ADC_SQR3_SQ1_Pos);

  // Channel speed registers
  volatile uint32_t *smprs[] = {&adc_reg->SMPR1, &adc_reg->SMPR2};

  // Set sample time for the channel
  uint8_t smpr_reg = (channel & 0b11111) / 10;
  uint8_t smpr_pos = ((channel & 0b11111) % 10) * 3;
  *smprs[smpr_reg] |= (convert_channel_speed(channel_speed) << smpr_pos);

  adc_reg->CR2 |= (1 << ADC_CR2_SWSTART_Pos);

  if (blocking == ADC_NON_BLOCKING) return 0xFFFF;

  while (!(adc_reg->SR & (1 << ADC_SR_EOC_Pos)));

  return adc_reg->DR;
}

uint8_t adc_scan_sample(ADC_TypeDef *adc_reg) {
  return adc_base_scan_sample(adc_reg, ADC_CR2_SWSTART_Pos, ADC_SR_EOC_Pos, ADC_NON_BLOCKING);
}

uint8_t adc_inj_scan_sample(ADC_TypeDef *adc_reg, const ADCBlocking_t blocking) {
  return adc_base_scan_sample(adc_reg, ADC_CR2_JSWSTART_Pos, ADC_SR_JEOC_Pos, blocking);
}

uint8_t adc_base_scan_sample(ADC_TypeDef *adc_reg, uint8_t start_bit_pos, uint8_t eoc_bit_pos,
                             const ADCBlocking_t blocking) {
  if (adc_reg == NULL) return -1;

  adc_reg->CR2 |= (1 << start_bit_pos);

  if (blocking != ADC_NON_BLOCKING) {
    while (!(adc_reg->SR & (1 << eoc_bit_pos)));
    adc_reg->SR &= ~(1 << eoc_bit_pos);
  }

  return 0;
}

uint16_t adc_get_inj_data(ADC_TypeDef *adc_reg, const uint8_t channel) {
  if (adc_reg == NULL) return 0xFFFF;

  // Change channel from 1-4 to 0-3 for array, and protect against values over the number of channels allowed
  uint8_t tmp_channel = ((unsigned int)(channel - 1)) > 3 ? 3 : channel - 1;

  volatile uint32_t *inj_regs[] = {&adc_reg->JDR1, &adc_reg->JDR2, &adc_reg->JDR3, &adc_reg->JDR4};
  return *inj_regs[tmp_channel];
}

void adc_dual_sample() { ADC1->CR2 |= (1 << ADC_CR2_SWSTART_Pos); }

uint8_t adc_irq_handling(ADC_TypeDef *adc_reg, const ADCInterruptType_t interrupt_type) {
  if (adc_reg == NULL) return 0;

  if (adc_reg->SR & 1 << interrupt_type) {
    adc_reg->SR &= ~(1 << interrupt_type);

    return 1;
  }

  return 0;
}

float convert_adc_to_temperature(uint16_t adc_val, ADCResolution_t resolution) {
  uint8_t bit_widths[] = {12, 10, 8, 6};
  uint8_t bit_width = bit_widths[(int)resolution];

  // Use the bits per ADC sample to calculate the resolution
  uint16_t max_adc_range = 1;
  for (int i = 0; i < bit_width; i++) max_adc_range *= 2;

  // Find temperature in celsius
  float v_sense = adc_val * 3.3 / max_adc_range;
  return 400 * (v_sense - 0.76) + 25;
}

// NOTE:
// Missing:
// EXTI11 trigger
// Watchdog stuff
