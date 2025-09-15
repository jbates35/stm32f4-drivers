#include "stm32f446xx_dma.h"

#include <stdio.h>

#define DMAS {DMA1, DMA2}
#define DMA_RCC_POS {RCC_AHB1ENR_DMA1EN_Pos, RCC_AHB1ENR_DMA2EN_Pos}
#define DMA_STREAMS                                                                                                   \
  {                                                                                                                   \
      DMA1_Stream0, DMA1_Stream1, DMA1_Stream2, DMA1_Stream3, DMA1_Stream4, DMA1_Stream5, DMA1_Stream6, DMA1_Stream7, \
      DMA2_Stream0, DMA2_Stream1, DMA2_Stream2, DMA2_Stream3, DMA2_Stream4, DMA2_Stream5, DMA2_Stream6, DMA2_Stream7, \
  }

#define SIZEOF(arr) ((int)sizeof(arr) / sizeof(arr[0]))
#define SIZEOFP(arr) ((int)sizeof(arr) / sizeof(uint32_t))  // Memory size of stm32f4

typedef struct {
  volatile uint32_t *status_reg;
  volatile uint32_t *clear_reg;
  uint8_t bit_offset;
  uint8_t success;
} DMAStatusRegStruct_t;

DMAStatusRegStruct_t get_dma_sr_struct(const DMA_Stream_TypeDef *stream);

/**
 * @brief  Controls the clock for the DMA peripheral.
 *
 * This function enables or disables the clock for the specified DMA peripheral.
 *
 * @param base_addr  Pointer to the base address of the DMA peripheral.
 * @param en_state   State to enable or disable the clock.
 * @return int       Returns 0 on success, -1 on error.
 */
int dma_peri_clock_control(const DMA_TypeDef *base_addr, const DMAEnable_t en_state) {
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
  if (en_state == DMA_ENABLE)
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
  if (dma_handle == NULL || dma_handle->stream_addr == NULL) return -1;

  DMA_Stream_TypeDef *stream = dma_handle->stream_addr;
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
      stream->PAR = (uintptr_t)cfg->in.addr;
      stream->CR |= (cfg->in.inc << DMA_SxCR_PINC_Pos);
      stream->M0AR = (uintptr_t)cfg->out.addr;
      stream->CR |= (cfg->out.inc << DMA_SxCR_MINC_Pos);
      break;
    case 0b01:
      stream->M0AR = (uintptr_t)cfg->in.addr;
      stream->CR |= (cfg->in.inc << DMA_SxCR_MINC_Pos);
      stream->PAR = (uintptr_t)cfg->out.addr;
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

  if (cfg->start_enabled == DMA_ENABLE) stream->CR |= (1 << DMA_SxCR_EN_Pos);

  return 0;
}

/**
 * @brief Start a DMA transaction.
 *
 * This function is used when you need to start a transaction with a certain number of elements to transfer.
 *
 * Note this function (at 16MHz) takes like 50us
 *
 * @param stream Pointer to the DMA stream to be enabled. If NULL, the function returns immediately.
 * @param buffer_size The number of elements to transfer before the DMA disables again.
 */
void dma_start_transfer(DMA_Stream_TypeDef *stream, uint32_t buffer_size) {
  if (stream == NULL) return;

  dma_stream_dis(stream);
  stream->NDTR = (uint32_t)buffer_size;
  dma_stream_en(stream);
}

/**
 * @brief Re-assign memory address.
 *
 * This function is used to assign the memory addresses in the DMA stream peripheral
 * Note: DMA stream must be disabled to re-assign memory address
 *
 * @param stream Pointer to the DMA stream to be enabled. If NULL, the function returns immediately.
 * @param ptr_addr The ptr of the array to assign to the DMA stream memory reg to
 * @param addr_reg Whether mem0 or mem1
 */
void dma_set_buffer(DMA_Stream_TypeDef *stream, volatile void *ptr_addr, DMAAddress_t addr_reg) {
  if (stream == NULL) return;

  uint8_t stream_en = (stream->CR & DMA_SxCR_EN);
  if (stream_en) dma_stream_dis(stream);

  if (addr_reg == DMA_ADDRESS_MEMORY_0) stream->M0AR = (uintptr_t)ptr_addr;
  if (addr_reg == DMA_ADDRESS_MEMORY_1) stream->M1AR = (uintptr_t)ptr_addr;

  if (stream_en) dma_stream_en(stream);
}

/**
 * @brief Enables the specified DMA stream.
 *
 * This function enables a given DMA stream after ensuring that all the flags
 * associated with that stream are cleared. It first identifies the stream and
 * its corresponding DMA controller, then clears the necessary flags before
 * enabling the stream.
 *
 * @param stream Pointer to the DMA stream to be enabled. If the pointer is NULL,
 *               the function returns immediately.
 *
 * @note The DMA stream will not be enabled unless all the flags for that stream
 *       are cleared.
 */
void dma_stream_en(DMA_Stream_TypeDef *stream) {
  if (stream == NULL) return;

  // Get the clear reg index and the bit position so we can clear all the flags
  // NOTE: The stream here will NOT turn on unless all the flags have been cleared
  DMAStatusRegStruct_t sr_info = get_dma_sr_struct(stream);
  if (!sr_info.success) return;
  *sr_info.clear_reg |= (0x3F << sr_info.bit_offset);

  // NOW, we can turn on the DMA stream
  stream->CR |= (1 << DMA_SxCR_EN_Pos);
}

/**
 * @brief Disable the DMA stream.
 *
 * This function clears the enable bit in the control register of the specified DMA stream.
 *
 * @param stream Pointer to the DMA stream to be disabled. If NULL, the function returns immediately.
 */
void dma_stream_dis(DMA_Stream_TypeDef *stream) {
  if (stream == NULL) return;
  stream->CR &= ~(1 << DMA_SxCR_EN_Pos);
}

/**
 * @brief  Initializes the DMA stream.
 * @brief  Handles DMA interrupts.
 *
 * This function handles the specified DMA interrupt for the given stream.
 *
 * @param stream          DMA Stream that the flag is associated with
 * @param interrupt_type  Type of the interrupt to handle.
 * @return int            Returns 1 if the interrupt was handled, 0 otherwise.
 */
int dma_irq_handling(DMA_Stream_TypeDef *stream, DMAInterruptType_t interrupt_type) {
  if (stream == NULL) return 0;
  if (interrupt_type == 1 || interrupt_type > 5) return 0;

  DMAStatusRegStruct_t sr_info = get_dma_sr_struct(stream);
  if (!sr_info.success) return 0;

  int bit_offset = sr_info.bit_offset;
  if (*sr_info.status_reg & (1 << (interrupt_type + bit_offset))) {
    *sr_info.clear_reg |= (1 << (interrupt_type + bit_offset));
    return 1;
  }

  return 0;
}

/**
 * @brief Retrieves the status register structure for a given DMA stream.
 *
 * This function returns a structure containing pointers to the set and clear
 * registers, as well as the bit offset for a specified DMA stream. It first
 * identifies the stream and its corresponding DMA controller, then determines
 * the appropriate registers and bit positions.
 *
 * Codes ugly but it's more optimized this way than trying to do math on pointers.
 *
 * @param stream Pointer to the DMA stream for which the status register structure
 *               is to be retrieved. If the pointer is NULL, the function returns
 *               a structure with default values.
 * @return DMAStatusRegStruct_t Structure containing the set register, clear register,
 *         bit offset, and success flag. If the stream is not found, the success flag
 *         is set to 0.
 */
DMAStatusRegStruct_t get_dma_sr_struct(const DMA_Stream_TypeDef *stream) {
  DMAStatusRegStruct_t return_struct = {.status_reg = NULL, .clear_reg = NULL, .success = 0};

  if (stream == DMA1_Stream0) {
    return_struct.clear_reg = &DMA1->LIFCR;
    return_struct.status_reg = &DMA1->LISR;
    return_struct.bit_offset = 0;
  } else if (stream == DMA1_Stream1) {
    return_struct.clear_reg = &DMA1->LIFCR;
    return_struct.status_reg = &DMA1->LISR;
    return_struct.bit_offset = 6;
  } else if (stream == DMA1_Stream2) {
    return_struct.clear_reg = &DMA1->LIFCR;
    return_struct.status_reg = &DMA1->LISR;
    return_struct.bit_offset = 16;
  } else if (stream == DMA1_Stream3) {
    return_struct.clear_reg = &DMA1->LIFCR;
    return_struct.status_reg = &DMA1->LISR;
    return_struct.bit_offset = 22;
  } else if (stream == DMA1_Stream4) {
    return_struct.clear_reg = &DMA1->HIFCR;
    return_struct.status_reg = &DMA1->HISR;
    return_struct.bit_offset = 0;
  } else if (stream == DMA1_Stream5) {
    return_struct.clear_reg = &DMA1->HIFCR;
    return_struct.status_reg = &DMA1->HISR;
    return_struct.bit_offset = 6;
  } else if (stream == DMA1_Stream6) {
    return_struct.clear_reg = &DMA1->HIFCR;
    return_struct.status_reg = &DMA1->HISR;
    return_struct.bit_offset = 16;
  } else if (stream == DMA1_Stream7) {
    return_struct.clear_reg = &DMA1->HIFCR;
    return_struct.status_reg = &DMA1->HISR;
    return_struct.bit_offset = 22;
  } else if (stream == DMA2_Stream0) {
    return_struct.clear_reg = &DMA2->LIFCR;
    return_struct.status_reg = &DMA2->LISR;
    return_struct.bit_offset = 0;
  } else if (stream == DMA2_Stream1) {
    return_struct.clear_reg = &DMA2->LIFCR;
    return_struct.status_reg = &DMA2->LISR;
    return_struct.bit_offset = 6;
  } else if (stream == DMA2_Stream2) {
    return_struct.clear_reg = &DMA2->LIFCR;
    return_struct.status_reg = &DMA2->LISR;
    return_struct.bit_offset = 16;
  } else if (stream == DMA2_Stream3) {
    return_struct.clear_reg = &DMA2->LIFCR;
    return_struct.status_reg = &DMA2->LISR;
    return_struct.bit_offset = 22;
  } else if (stream == DMA2_Stream4) {
    return_struct.clear_reg = &DMA2->HIFCR;
    return_struct.status_reg = &DMA2->HISR;
    return_struct.bit_offset = 0;
  } else if (stream == DMA2_Stream5) {
    return_struct.clear_reg = &DMA2->HIFCR;
    return_struct.status_reg = &DMA2->HISR;
    return_struct.bit_offset = 6;
  } else if (stream == DMA2_Stream6) {
    return_struct.clear_reg = &DMA2->HIFCR;
    return_struct.status_reg = &DMA2->HISR;
    return_struct.bit_offset = 16;
  } else if (stream == DMA2_Stream7) {
    return_struct.clear_reg = &DMA2->HIFCR;
    return_struct.status_reg = &DMA2->HISR;
    return_struct.bit_offset = 22;
  }

  return_struct.success = 1;
  return return_struct;
}
