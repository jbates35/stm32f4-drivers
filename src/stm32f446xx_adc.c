#include "stm32f446xx_adc.h"

#include <stdio.h>

#include "stm32f446xx.h"
#include "stm32f446xx_dma.h"

#define ADCS {ADC1, ADC2, ADC3}
#define ADC_RCC_POS {RCC_APB2ENR_ADC1EN_Pos, RCC_APB2ENR_ADC2EN_Pos, RCC_APB2ENR_ADC3EN_Pos}

#define SIZEOF(arr) ((int)sizeof(arr) / sizeof(arr[0]))
#define SIZEOFP(arr) ((int)sizeof(arr) / sizeof(uint32_t))  // Memory size of stm32f4

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
  uint8_t adc_rcc_pos[] = ADC_RCC_POS;

  if (en_state)
    RCC->APB2ENR |= (1 << adc_rcc_pos[i]);
  else
    RCC->APB2RSTR |= (1 << adc_rcc_pos[i]);

  return 0;
}

int adc_stream_init(const ADCHandle_t *adc_handle) { return 0; }
