/*
 * stm32f446_dma.h
 *
 *  Created on: Dec. 29, 2024
 *      Author: jbates
 */

#ifndef INC_STM32F446XX_DMA_H_
#define INC_STM32F446XX_DMA_H_

#include <stdint.h>
#include <stdio.h>

#include "stm32f446xx.h"

typedef enum { DMA_IO_TYPE_PERIPHERAL = 0, DMA_IO_TYPE_MEMORY } DMAIOType_t;
typedef enum { DMA_DATA_SIZE_8_BIT = 0, DMA_DATA_SIZE_16_BIT = 1, DMA_DATA_SIZE_32_BIT = 2 } DMADataSize_t;
typedef enum { DMA_PRIORITY_LOW = 0, DMA_PRIORITY_MEDIUM, DMA_PRIORITY_HIGH, DMA_PRIORITY_MAX } DMAPriority_t;
typedef enum { DMA_IO_ARR_STATIC = 0, DMA_IO_ARR_INCREMENT } DMAArrIncrement_t;
typedef enum { DMA_BUFFER_FINITE = 0, DMA_BUFFER_CIRCULAR } DMACircBuffer_t;
typedef enum { DMA_PERIPH_NO_FLOW_CONTROL = 0, DMA_PERIPH_FLOW_CONTROL = 1 } DMAFlowControl_t;
typedef enum { DMA_DISABLE = 0, DMA_ENABLE } DMAEnable_t;
typedef enum {
  DMA_INTERRUPT_TYPE_FIFO_ERROR = 0,
  DMA_INTERRUPT_TYPE_DIRECT_MODE_ERROR = 2,
  DMA_INTERRUPT_TYPE_TRANSFER_ERROR = 3,
  DMA_INTERRUPT_TYPE_HALF_TRANSFER_COMPLETE = 4,
  DMA_INTERRUPT_TYPE_FULL_TRANSFER_COMPLETE = 5
} DMAInterruptType_t;

/**
 * @brief  IO Handle structure definition
 *
 * This structure defines the IO handle for DMA operations.
 *
 * @param addr  Address for the IO handle.
 * @param type  Type of the IO handle.
 * @param inc   Address increment setting.
 */
typedef struct {
  uint32_t addr;
  DMAIOType_t type;
  DMAArrIncrement_t inc;
} IOHandle_t;

/**
 * @brief  DMA All Interrupts Enable structure definition
 *
 * This structure defines the interrupt enable settings for DMA.
 *
 * @param direct_mode_error  Direct mode error interrupt enable.
 * @param transfer_error     Transfer error interrupt enable.
 * @param half_transfer      Half transfer interrupt enable.
 * @param full_transfer      Full transfer interrupt enable.
 */
typedef struct {
  DMAEnable_t direct_mode_error;
  DMAEnable_t transfer_error;
  DMAEnable_t half_transfer;
  DMAEnable_t full_transfer;
} DMAAllInterruptsEn_t;

/**
 * @brief  DMA Configuration structure definition
 *
 * This structure defines the configuration settings for DMA.
 *
 * @param in              Input IO handle.
 * @param out             Output IO handle.
 * @param interrupt_en    Interrupt enable settings.
 * @param mem_data_size   Memory data size.
 * @param peri_data_size  Peripheral data size.
 * @param channel         DMA channel.
 * @param priority        DMA priority.
 * @param flow_control    Flow control setting.
 * @param circ_buffer     Circular buffer setting.
 * @param dma_elements    Number of DMA elements to cycle through.
 * @param start_enabled   Whether the DMA should enable after initialized.
 */
typedef struct {
  IOHandle_t in;
  IOHandle_t out;
  DMAAllInterruptsEn_t interrupt_en;
  DMADataSize_t mem_data_size;
  DMADataSize_t peri_data_size;
  uint8_t channel;
  DMAPriority_t priority;
  DMAFlowControl_t flow_control;
  DMACircBuffer_t circ_buffer;
  uint16_t dma_elements;
  DMAEnable_t start_enabled;
} DMAConfig_t;

/**
 * @brief  DMA Handle structure definition
 *
 * This structure defines the handle for DMA operations.
 *
 * @param cfg            DMA configuration settings.
 * @param p_stream_addr  Pointer to the DMA stream address.
 */
typedef struct {
  DMAConfig_t cfg;
  DMA_Stream_TypeDef *p_stream_addr;
} DMAHandle_t;

/**
 * @brief  Controls the clock for the DMA peripheral.
 *
 * This function enables or disables the clock for the specified DMA peripheral.
 *
 * @param base_addr  Pointer to the base address of the DMA peripheral.
 * @param en_state   State to enable or disable the clock.
 * @return int       Returns 0 on success, -1 on error.
 */
int dma_peri_clock_control(const DMA_TypeDef *base_addr, const DMAEnable_t en_state);

/**
 *
 * This function initializes the specified DMA stream with the provided configuration.
 *
 * @param dma_handle  Pointer to the DMA handle structure.
 * @return int        Returns 0 on success, -1 on error.
 */
int dma_stream_init(const DMAHandle_t *dma_handle);

/**
 * @brief Start a DMA transaction.
 *
 * This function is used when you need to start a transaction with a certain number of elements to transfer.
 *
 * @param stream Pointer to the DMA stream to be enabled. If NULL, the function returns immediately.
 * @param buffer_size The number of elements to transfer before the DMA disables again.
 */
void dma_start_transfer(DMA_Stream_TypeDef *stream, uint16_t buffer_size);

/**
 * @brief Enable the DMA stream.
 *
 * This function sets the enable bit in the control register of the specified DMA stream.
 *
 * @param stream Pointer to the DMA stream to be enabled. If NULL, the function returns immediately.
 */
void dma_stream_en(DMA_Stream_TypeDef *stream);

/**
 * @brief Disable the DMA stream.
 *
 * This function clears the enable bit in the control register of the specified DMA stream.
 *
 * @param stream Pointer to the DMA stream to be disabled. If NULL, the function returns immediately.
 */
void dma_stream_dis(DMA_Stream_TypeDef *stream);

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
int dma_irq_handling(DMA_Stream_TypeDef *stream, DMAInterruptType_t interrupt_type);

#endif
