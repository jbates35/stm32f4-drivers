#include "stm32f446xx_adc.h"

#include <stdio.h>

#include "stm32f446xx.h"
#include "stm32f446xx_dma.h"

#define ADCS {ADC1, ADC2, ADC3}
#define ADC_RCC_POS {RCC_APB2ENR_ADC1EN_Pos, RCC_APB2ENR_ADC2EN_Pos, RCC_APB2ENR_ADC3EN_Pos}

#define SIZEOF(arr) ((int)sizeof(arr) / sizeof(arr[0]))
#define SIZEOFP(arr) ((int)sizeof(arr) / sizeof(uint32_t))  // Memory size of stm32f4

int adc_peri_clock_control(const ADC_TypeDef *base_addr, const uint8_t en_state) { return 0; }

int adc_stream_init(const ADCHandle_t *adc_handle) { return 0; }
