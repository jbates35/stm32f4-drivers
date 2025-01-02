#include "stm32f446xx_dma.h"

#include <stdio.h>

#include "stm32f446xx.h"

#define DMAS {DMA1, DMA2}
#define DMA_RCC_POS {RCC_AHB1ENR_DMA1EN, RCC_AHB1ENR_DMA2EN}

#define SIZEOF(arr) ((int)sizeof(arr) / sizeof(arr[0]))
#define SIZEOFP(arr) ((int)sizeof(arr) / sizeof(uint32_t))  // Memory size of stm32f4

int dma_peri_clock_control(const DMA_TypeDef *base_addr, const uint8_t en_state) {
  // If null pointer, return error code
  if (base_addr == NULL) return -1;

  const DMA_TypeDef *dmas[] = DMAS;

  // Find the DMA peripheral being enabled
  int i = 0;
  for (; i < SIZEOFP(dmas); i++) {
    if (dmas[i] == base_addr) break;
  }

  // Return error if a proper base address wasn't put in
  if (i >= SIZEOFP(dmas)) return -1;

  // Enable or disable DMA peripheral's clock
  const unsigned int dma_reg_pos[] = DMA_RCC_POS;
  if (en_state)
    RCC->AHB1ENR |= (1 << dma_reg_pos[i]);
  else
    RCC->AHB1RSTR |= (1 << dma_reg_pos[i]);

  return 0;
}

