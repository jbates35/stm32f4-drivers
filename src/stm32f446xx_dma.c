#include "stm32f446xx_dma.h"

#include <stdio.h>

#include "stm32f446xx.h"

#define DMAS {DMA1, DMA2}
#define DMA_RCC_POS {RCC_AHB1ENR_DMA1EN_Pos, RCC_AHB1ENR_DMA2EN_Pos}

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

int dma_stream_init(const DMAHandle_t *dma_handle) {
  if (dma_handle == NULL || dma_handle->p_stream_addr == NULL) return -1;

  DMA_Stream_TypeDef *stream = dma_handle->p_stream_addr;
  const DMAConfig_t *cfg = &(dma_handle->cfg);

  // Get direction of transmission and validate
  uint8_t dir = 0;
  if (cfg->in.type == DMA_IO_TYPE_PERIPHERAL && cfg->out.type == DMA_IO_TYPE_PERIPHERAL)
    return -1;
  else if (cfg->in.type == DMA_IO_TYPE_PERIPHERAL)
    dir = 0b00;
  else if (cfg->out.type == DMA_IO_TYPE_PERIPHERAL)
    dir = 0b01;
  else
    dir = 0b10;

  // Reset CR
  stream->CR = 0;

  // Assign channel and priority
  stream->CR |= ((cfg->channel & 0b111) << DMA_SxCR_CHSEL_Pos);
  stream->CR |= ((cfg->priority & 0b11) << DMA_SxCR_PL_Pos);

  // Set the data sizes of the buffers
  stream->CR |= ((cfg->peri_data_size & 0b11) << DMA_SxCR_PSIZE_Pos);
  stream->CR |= ((cfg->mem_data_size & 0b11) << DMA_SxCR_MSIZE_Pos);

  // Set direction
  stream->CR |= (dir << DMA_SxCR_DIR_Pos);

  // Set number of data elements which can be stored in dma buffer
  stream->NDTR = cfg->dma_elements;

  // Set peripheral and memory addresses
  switch (dir) {
    case 0b00:
      stream->PAR = cfg->in.addr;
      stream->CR |= (cfg->in.inc << DMA_SxCR_PINC_Pos);
      stream->M0AR = cfg->out.addr;
      stream->CR |= (cfg->out.inc << DMA_SxCR_MINC_Pos);
      break;
    case 0b01:
      stream->M0AR = cfg->in.addr;
      stream->CR |= (cfg->in.inc << DMA_SxCR_MINC_Pos);
      stream->PAR = cfg->out.addr;
      stream->CR |= (cfg->out.inc << DMA_SxCR_PINC_Pos);
      break;
  }

  stream->CR |= (1 << DMA_SxCR_EN_Pos);

  return 0;
}
