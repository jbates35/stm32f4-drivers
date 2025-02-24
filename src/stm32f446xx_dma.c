#include "stm32f446xx_dma.h"

#include <stdio.h>

#define DMAS {DMA1, DMA2}
#define DMA_RCC_POS {RCC_AHB1ENR_DMA1EN_Pos, RCC_AHB1ENR_DMA2EN_Pos}

#define SIZEOF(arr) ((int)sizeof(arr) / sizeof(arr[0]))
#define SIZEOFP(arr) ((int)sizeof(arr) / sizeof(uint32_t))  // Memory size of stm32f4

/**
 * @brief  Controls the clock for the DMA peripheral.
 * 
 * This function enables or disables the clock for the specified DMA peripheral.
 * 
 * @param base_addr  Pointer to the base address of the DMA peripheral.
 * @param en_state   State to enable or disable the clock.
 * @return int       Returns 0 on success, -1 on error.
 */
int dma_peri_clock_control(const DMA_TypeDef *base_addr, const DMAPeriClockEn_t en_state) {
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
  if (en_state == DMA_PERI_CLOCK_ENABLE)
    RCC->AHB1ENR |= (1 << dma_reg_pos[i]);
  else
    RCC->AHB1RSTR |= (1 << dma_reg_pos[i]);

  return 0;
}

/**
 * @brief  Initializes the DMA stream.
 * 
 * This function initializes the specified DMA stream with the provided configuration.
 * 
 * @param dma_handle  Pointer to the DMA handle structure.
 * @return int        Returns 0 on success, -1 on error.
 */
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

  // Set peripheral and memory addresses
  switch (dir) {
    case 0b00:
    case 0b10:
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

  // Assign channel and priority
  stream->CR |= ((cfg->channel & 0b111) << DMA_SxCR_CHSEL_Pos);
  stream->CR |= ((cfg->priority & 0b11) << DMA_SxCR_PL_Pos);

  // Set the data sizes of the buffers
  stream->CR |= ((cfg->peri_data_size & 0b11) << DMA_SxCR_PSIZE_Pos);
  stream->CR |= ((cfg->mem_data_size & 0b11) << DMA_SxCR_MSIZE_Pos);

  // Set direction
  stream->CR |= (dir << DMA_SxCR_DIR_Pos);

  // Set circular mode
  uint8_t circ_buffer = cfg->circ_buffer ? 1 : 0;
  stream->CR |= (circ_buffer << DMA_SxCR_CIRC_Pos);

  // Set who controls DMA flow control
  uint8_t flow_control = cfg->flow_control ? 1 : 0;
  stream->CR |= (flow_control << DMA_SxCR_PFCTRL_Pos);

  // Set interrupts
  if (cfg->interrupt_en.full_transfer) stream->CR |= (1 << DMA_SxCR_TCIE_Pos);
  if (cfg->interrupt_en.half_transfer) stream->CR |= (1 << DMA_SxCR_HTIE_Pos);
  if (cfg->interrupt_en.transfer_error) stream->CR |= (1 << DMA_SxCR_TEIE_Pos);
  if (cfg->interrupt_en.direct_mode_error) stream->CR |= (1 << DMA_SxCR_DMEIE_Pos);

  // Set number of data elements which can be stored in dma buffer
  stream->NDTR = cfg->dma_elements;

  if (cfg->start_enabled == DMA_START_ENABLED) stream->CR |= (1 << DMA_SxCR_EN_Pos);

  return 0;
}

/**
 * @brief  Handles DMA interrupts.
 * 
 * This function handles the specified DMA interrupt for the given stream.
 * 
 * @param base_addr       Pointer to the base address of the DMA peripheral.
 * @param stream_num      Stream number of the DMA.
 * @param interrupt_type  Type of the interrupt to handle.
 * @return int            Returns 1 if the interrupt was handled, 0 otherwise.
 */
int dma_irq_handling(const DMA_TypeDef *base_addr, uint8_t stream_num, DMAInterruptType_t interrupt_type) {
  if (base_addr == NULL) return 0;
  if (stream_num > 8) return 0;
  if (interrupt_type == 1 || interrupt_type > 5) return 0;

  const volatile uint32_t *set_regs[] = {&base_addr->LISR, &base_addr->HISR};
  volatile uint32_t *clear_regs[] = {(volatile uint32_t *)&base_addr->LIFCR, (volatile uint32_t *)&base_addr->HIFCR};

  uint8_t reg_ind = stream_num / 4;
  uint8_t reg_bit_offset = (stream_num % 4) * 6;

  if (reg_bit_offset >= 12) {
    reg_bit_offset = reg_bit_offset + 4;
  }

  if (*set_regs[reg_ind] & (1 << (interrupt_type + reg_bit_offset))) {
    *(clear_regs[reg_ind]) |= (1 << (interrupt_type + reg_bit_offset));
    return 1;
  }

  return 0;
}
