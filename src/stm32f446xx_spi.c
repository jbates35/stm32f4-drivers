#include "stm32f446xx_spi.h"

#include <string.h>

#define SPIS_NUM 4
#define SPIS {SPI1, SPI2, SPI3, SPI4}
#define SPIS_RCC_REGS {&RCC->APB2ENR, &RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB2ENR}
#define SPIS_RCC_POS {RCC_APB2ENR_SPI1EN_Pos, RCC_APB1ENR_SPI2EN_Pos, RCC_APB1ENR_SPI3EN_Pos, RCC_APB2ENR_SPI4EN}
#define SIZEOF(arr) ((int)sizeof(arr) / sizeof(arr[0]))
#define SIZEOFP(arr) ((int)sizeof(arr) / sizeof(uint32_t))  // Memory size of stm32f4

// For keeping track especially when it comes to interrupt instanstiations
typedef struct {
  char *buffer;
  char *buffer_start;
  int eles_left;
  int len;
  SPIEnable_t en;
} SPIInterruptBuffer_t;

typedef struct {
  SPIInterruptBuffer_t tx;
  SPIInterruptBuffer_t rx;
  SPIInterruptStatus_t status;
  SPIInterruptCircular_t circular;
  void (*callback)(void);
} SPIInterruptInfo_t;
volatile SPIInterruptInfo_t spi_interrupt_info[SPIS_NUM] = {0};

int handle_spi_int_buffer(SPI_TypeDef *spi_reg, const SPIInterruptType_t type);
// SPIInterruptType_t spi_irq_handling(const SPI_TypeDef *spi_reg, volatile SPIInterruptInfo_t *spi_info);

/**
 * @brief  Helper function to get the correct SPI Config
 *
 * @param spi_base_addr The SPI peripheral, base address.
 *
 * @return SPIConfig_t  Returns the SPI interrupt info package

 */
volatile static inline SPIInterruptInfo_t *get_spi_int_info(const SPI_TypeDef *spi_base_addr) {
  if (spi_base_addr == SPI1)
    return &spi_interrupt_info[0];
  else if (spi_base_addr == SPI2)
    return &spi_interrupt_info[1];
  else if (spi_base_addr == SPI3)
    return &spi_interrupt_info[2];
  else if (spi_base_addr == SPI4)
    return &spi_interrupt_info[3];
  return NULL;
}

/**
 * @brief Function which retrieves the correct IRQ flag
 * Also it will lock the tx when the rx irq needs to still be retrieved
 * In the future, having the error IRQ be incorporated could be done as well. Quite easily.
 *
 * @param spi_reg The SPI peripheral, base address
 * @param spi_info The SPIInterruptInfo_t package that is called with the above function
 *
 * @return SPIConfig_t  Returns either the IRQ flag, or returns the "NONE" one

 */
static inline SPIInterruptType_t get_spi_irq_type(const SPI_TypeDef *spi_reg, volatile SPIInterruptInfo_t *spi_info) {
  uint8_t spi_rx_busy = ((spi_reg->CR2 & (1 << SPI_CR2_RXNEIE_Pos)) && (spi_info->status == SPI_INTERRUPT_BUSY));

  if (spi_reg->SR & (1 << SPI_SR_TXE_Pos) && !spi_rx_busy) {
    spi_info->status = SPI_INTERRUPT_BUSY;
    return SPI_INTERRUPT_TYPE_TX;
  }

  if (spi_reg->SR & (1 << SPI_SR_RXNE_Pos)) {
    spi_info->status = SPI_INTERRUPT_READY;
    return SPI_INTERRUPT_TYPE_RX;
  }

  return SPI_INTERRUPT_TYPE_NONE;
}

/**
 * @brief Controls the peripheral clock for the specified SPI instance.
 *
 * This function enables or disables the peripheral clock for a given SPI instance
 * based on the provided enable/disable state. It validates the SPI instance and
 * updates the corresponding RCC register to control the clock.
 *
 * @param spi_reg Pointer to the SPI peripheral base address.
 * @param en_state Enable/disable state for the peripheral clock. Use 1 to enable
 *            and 0 to disable.
 *
 * @return int Returns 0 on success, or -1 if the input pointer is NULL or the SPI
 *         instance is invalid.
 *
 * @note Ensure that the SPI instance is valid and corresponds to one of the supported
 *       SPI peripherals before calling this function.
 */
int spi_peri_clock_control(const SPI_TypeDef *spi_reg, const SPIEnable_t en_state) {
  if (spi_reg == NULL) return -1;  // Error: null pointer

  const SPI_TypeDef *spis_arr[] = SPIS;
  int i = 0;

  for (; i < SIZEOFP(spis_arr); i++) {
    if (spis_arr[i] == spi_reg) break;
  }

  if (i >= SIZEOFP(spis_arr)) return -1;

  volatile uint32_t *spi_regs[] = SPIS_RCC_REGS;
  const unsigned int spi_rcc_pos[] = SPIS_RCC_POS;
  if (en_state == SPI_ENABLE)
    *spi_regs[i] |= (1 << spi_rcc_pos[i]);
  else if (en_state == SPI_DISABLE)
    *spi_regs[i] &= ~(1 << spi_rcc_pos[i]);
  else
    return -2;

  return 0;
}

/**
 * @brief Initializes the SPI peripheral with the specified configuration.
 *
 * This function configures the SPI peripheral based on the settings provided
 * in the SPIHandle_t structure. It supports master/slave mode, software/hardware
 * slave management, different bus configurations, baud rate settings, data frame
 * formats, clock polarity/phase, and DMA enablement. Optionally, the SPI can be
 * enabled immediately after initialization.
 *
 * @param spi_handle Pointer to the SPI handle structure containing the
 *            base address of the SPI peripheral and its configuration.
 *
 * @return int Returns 0 on successful initialization, or -1 if the input
 *         pointer is NULL or invalid.
 *
 * @note Ensure that the SPIHandle_t structure is properly initialized before
 *       calling this function. The SPI peripheral must be disabled before
 *       reconfiguring it.
 */
int spi_init(const SPIHandle_t *spi_handle) {
  if (spi_handle == NULL || spi_handle->addr == NULL) return -1;  // NUll pointers

  SPI_TypeDef *spi_reg = spi_handle->addr;
  const SPIConfig_t *cfg = &(spi_handle->cfg);

  spi_reg->CR1 = 0;
  spi_reg->CR2 = 0;

  // Is SPI in master mode or slave mode?
  uint8_t master_mode = cfg->device_mode == SPI_DEVICE_MODE_MASTER ? 1 : 0;
  spi_reg->CR1 |= (master_mode << SPI_CR1_MSTR_Pos);

  // Enable software slave management, or not
  if (cfg->ssm == SPI_SSM_ENABLE) {
    spi_reg->CR1 |= (1 << SPI_CR1_SSM_Pos);
    spi_reg->CR1 |= (1 << SPI_CR1_SSI_Pos);
  } else {
    spi_reg->CR1 |= (1 << SPI_CR2_SSOE_Pos);
  }

  // Handle the different modes
  switch (cfg->bus_config) {
    case SPI_BUS_CONFIG_HALF_DUPLEX:
      spi_reg->CR1 |= (1 << SPI_CR1_BIDIMODE_Pos);
      break;
    case SPI_BUS_CONFIG_SIMPLEX_RX_ONLY:
      spi_reg->CR1 |= (1 << SPI_CR1_RXONLY_Pos);
      break;
    default:
      break;
  }

  // Baud rate
  spi_reg->CR1 |= ((cfg->baud_divisor & 0b111) << SPI_CR1_BR_Pos);

  // Data frame mode
  uint8_t data_frame = cfg->dff ? 1 : 0;
  spi_reg->CR1 |= (data_frame << SPI_CR1_DFF_Pos);

  // Capture mode
  if (cfg->capture_mode == SPI_CAPTURE_MODE_FALLING) {
    spi_reg->CR1 |= (1 << SPI_CR1_CPOL_Pos);
    spi_reg->CR1 |= (1 << SPI_CR1_CPHA_Pos);
  }

  // Enable DMA
  if (cfg->dma_setup.tx == SPI_ENABLE) spi_reg->CR2 |= (1 << SPI_CR2_TXDMAEN_Pos);
  if (cfg->dma_setup.rx == SPI_ENABLE) spi_reg->CR2 |= (1 << SPI_CR2_RXDMAEN_Pos);

  // Enable SPI
  if (cfg->enable_on_init == SPI_ENABLE) spi_reg->CR1 |= (1 << SPI_CR1_SPE_Pos);

  return 0;
}

/**
 * @brief NOT IMPLEMENTED YET
 */
int spi_deinit(const SPI_TypeDef *p_spi_addr) { return 0; }

/**
 * @brief Transmits a single byte over SPI.
 *
 * This function sends a single byte of data through the specified SPI port.
 * It waits until the TX buffer is empty before transmitting the byte.
 *
 * @param spi_port Pointer to the SPI peripheral base address.
 * @param tx_byte The byte to be transmitted.
 *
 * @return int Returns 0 on success, or -1 if the SPI port is NULL.
 */
int spi_tx_byte(SPI_TypeDef *spi_port, const uint16_t tx_byte) {
  if (spi_port == NULL) return -1;

  // While the TX Buffer is not empty...
  while (!(spi_port->SR & (1 << SPI_SR_TXE_Pos)));
  spi_port->DR = tx_byte;

  // Clear flag
  spi_port->SR &= ~(1 << SPI_SR_TXE_Pos);

  return 0;
}

/**
 * @brief Transmits a buffer of data over SPI in full-duplex mode.
 *
 * This function sends a buffer of data through the specified SPI port
 * while simultaneously receiving data. It uses the full-duplex transfer
 * function internally.
 *
 * @param spi_reg Pointer to the SPI peripheral base address.
 * @param tx_buffer Pointer to the buffer containing data to transmit.
 * @param len Length of the data to transmit.
 *
 * @return int Returns 0 on success, or -1 if the SPI port is NULL.
 */
int spi_tx_word(SPI_TypeDef *spi_reg, const void *tx_buffer, int len) {
  return spi_full_duplex_transfer(spi_reg, (void *)tx_buffer, (void *)tx_buffer, len);
}

/**
 * @brief Receives a single byte over SPI.
 *
 * This function waits until the RX buffer is not empty and then reads
 * a single byte of data from the specified SPI port.
 *
 * @param spi_reg Pointer to the SPI peripheral base address.
 *
 * @return uint16_t Returns the received byte, or 0 if the SPI port is NULL.
 */
uint16_t spi_rx_byte(SPI_TypeDef *spi_reg) {
  if (spi_reg == NULL) return 0;

  // Wait until the rx is empty
  while (!(spi_reg->SR & (1 << SPI_SR_RXNE_Pos)));
  uint16_t rx_word = spi_reg->DR;

  spi_reg->SR &= ~(1 << SPI_SR_RXNE_Pos);
  return rx_word;
}

/**
 * @brief Receives a buffer of data over SPI in full-duplex mode.
 *
 * This function receives a buffer of data through the specified SPI port
 * while simultaneously transmitting data. It uses the full-duplex transfer
 * function internally.
 *
 * @param spi_reg Pointer to the SPI peripheral base address.
 * @param rx_buffer Pointer to the buffer to store received data.
 * @param len Length of the data to receive.
 *
 * @return int Returns 0 on success, or -1 if the SPI port is NULL.
 */
int spi_rx_word(SPI_TypeDef *spi_reg, uint8_t *rx_buffer, int len) {
  return spi_full_duplex_transfer(spi_reg, rx_buffer, rx_buffer, len);
}

/**
 * @brief Performs a full-duplex SPI transfer.
 *
 * This function transmits and receives data simultaneously over SPI.
 * It handles both 8-bit and 16-bit data frames based on the DFF setting.
 *
 * @param spi_reg Pointer to the SPI peripheral base address.
 * @param tx_buffer Pointer to the buffer containing data to transmit.
 * @param rx_buffer Pointer to the buffer to store received data.
 * @param len Length of the data to transfer.
 *
 * @return int Returns 0 on success, or -1 if the SPI port is NULL.
 */
int spi_full_duplex_transfer(SPI_TypeDef *spi_reg, void *tx_buffer, void *rx_buffer, int len) {
  if (spi_reg == NULL) return -1;

  // Get the amount of bytes per frame - Should be 1 bytes, or 2 bytes (dff=1)
  uint8_t dff_bytes = ((spi_reg->CR1 >> SPI_CR1_DFF_Pos) & 0b1) + 1;

  while (len > 0) {
    // Prepare the tx part of the transmission
    uint16_t tx_word = 0;
    if (dff_bytes == 1)
      tx_word = *((uint8_t *)tx_buffer);
    else {
      tx_word = *((uint16_t *)tx_buffer);
      if (len == 1) tx_word &= 0xFF00;
    }

    // Send byte
    spi_tx_byte(spi_reg, tx_word);

    // Prepare the rx part of the transmission and receive bytes
    uint16_t rx_byte = spi_rx_byte(spi_reg);

    if (dff_bytes == 1 || len == 1)
      *((uint8_t *)rx_buffer) = rx_byte & 0xFF;
    else
      *((uint16_t *)rx_buffer) = rx_byte;

    // Move buffer along to the next available frame
    tx_buffer = (uint8_t *)tx_buffer + dff_bytes;
    rx_buffer = (uint8_t *)rx_buffer + dff_bytes;
    len -= dff_bytes;
  }

  return 0;
}

/**
 * @brief Sets up an SPI interrupt for transmission or reception.
 *
 * This function configures the SPI interrupt for either TX or RX operations.
 * It initializes the interrupt buffer and enables the interrupt.
 *
 * @param spi_reg Pointer to the SPI peripheral base address.
 * @param type Type of interrupt (TX or RX).
 * @param buffer Pointer to the buffer for interrupt data.
 * @param len Length of the buffer.
 *
 * @return int Returns 0 on success, -1 if the SPI port is NULL, or -2 if the interrupt is busy.
 */
int spi_setup_interrupt(const SPI_TypeDef *spi_reg, const SPIInterruptType_t type, char *buffer, const int len) {
  if (spi_reg == NULL) return -1;

  volatile SPIInterruptInfo_t *int_info = get_spi_int_info(spi_reg);
  if (int_info->status == SPI_INTERRUPT_BUSY) return -2;

  volatile SPIInterruptBuffer_t *buf_info = NULL;
  if (type == SPI_INTERRUPT_TYPE_TX)
    buf_info = &int_info->tx;
  else if (type == SPI_INTERRUPT_TYPE_RX)
    buf_info = &int_info->rx;
  else
    return -1;

  buf_info->buffer = buffer;
  buf_info->buffer_start = buffer;
  buf_info->len = len;
  buf_info->eles_left = len;
  buf_info->en = SPI_ENABLE;

  return 0;
}

/**
 * @brief Configures circular mode for SPI interrupts.
 *
 * This function enables or disables circular mode for SPI interrupts.
 * In circular mode, the interrupt buffers are reused after completion.
 *
 * @param spi_reg Pointer to the SPI peripheral base address.
 * @param circular_en Circular mode enable/disable setting.
 *
 * @return int Returns 0 on success, -1 if the SPI port is NULL, or -2 if the interrupt info is NULL.
 */
int spi_set_circular_interrupt(const SPI_TypeDef *spi_reg, const SPIInterruptCircular_t circular_en) {
  if (spi_reg == NULL) return -1;

  volatile SPIInterruptInfo_t *int_info = get_spi_int_info(spi_reg);
  if (int_info == NULL) return -2;

  if (circular_en == SPI_INTERRUPT_NONCIRCULAR)
    int_info->circular = SPI_INTERRUPT_NONCIRCULAR;
  else
    int_info->circular = SPI_INTERRUPT_CIRCULAR;

  return 0;
}

/**
 * @brief Retrieves the current status of the SPI interrupt.
 *
 * This function returns the current status of the SPI interrupt, such as
 * whether it is ready, busy, or completed.
 *
 * @param spi_reg Pointer to the SPI peripheral base address.
 *
 * @return SPIInterruptStatus_t Returns the interrupt status, or SPI_INTERRUPT_INVALID if the SPI port is NULL.
 */
SPIInterruptStatus_t spi_get_interrupt_status(const SPI_TypeDef *spi_reg) {
  if (spi_reg == NULL) return SPI_INTERRUPT_INVALID;

  volatile const SPIInterruptInfo_t *interrupt_info = get_spi_int_info(spi_reg);
  if (interrupt_info == NULL) return SPI_INTERRUPT_INVALID;

  return interrupt_info->status;
}

/**
 * @brief Handles the SPI interrupt and determines its type.
 *
 * This function processes the SPI interrupt and identifies whether it is
 * a TX or RX interrupt.
 *
 * @param spi_reg Pointer to the SPI peripheral base address.
 *
 * @return SPIInterruptType_t Returns the type of interrupt (TX, RX, or NONE).
 */
SPIInterruptType_t spi_irq_handling(const SPI_TypeDef *spi_reg) {
  volatile SPIInterruptInfo_t *int_info = get_spi_int_info(spi_reg);
  return get_spi_irq_type(spi_reg, int_info);
}

/**
 * @brief Handles SPI interrupt word transfers.
 *
 * This function processes TX and RX interrupts for word transfers. It
 * manages the interrupt buffers and invokes the callback upon completion.
 *
 * @param spi_reg Pointer to the SPI peripheral base address.
 *
 * @return int Returns 1 if the transfer is complete, 0 otherwise, or -1 if the SPI port is NULL.
 */
int spi_irq_word_handling(SPI_TypeDef *spi_reg) {
  // This function is such a piece of garbage. Probably should be re-organized or made smaller in scope.

  if (spi_reg == NULL) return -1;

  // Get the amount of bytes per frame - Should be 1 bytes, or 2 bytes (dff=1)
  uint8_t dff_bytes = ((spi_reg->CR1 >> SPI_CR1_DFF_Pos) & 0b1) + 1;

  volatile SPIInterruptInfo_t *int_info = get_spi_int_info(spi_reg);
  SPIInterruptType_t int_status = get_spi_irq_type(spi_reg, int_info);

  if (int_status == SPI_INTERRUPT_TYPE_NONE)
    return 0;

  else if (int_status == SPI_INTERRUPT_TYPE_TX) {
    volatile SPIInterruptBuffer_t *tx_buf_info = &int_info->tx;

    if (tx_buf_info->eles_left > 0) {
      // Get next element of array and put it in DR
      uint8_t tx_byte;

      if (dff_bytes == 1)
        tx_byte = *((uint8_t *)tx_buf_info->buffer);
      else {
        tx_byte = *((uint16_t *)tx_buf_info->buffer);
        if (tx_buf_info->eles_left == 1) tx_byte &= 0xFF00;
      }
      spi_reg->DR = tx_byte;

      tx_buf_info->buffer = (char *)tx_buf_info->buffer + dff_bytes;
      tx_buf_info->eles_left -= dff_bytes;
    } else {
      spi_reg->DR = 0xFF;
    }
  }

  else if (int_status == SPI_INTERRUPT_TYPE_RX) {
    volatile SPIInterruptBuffer_t *rx_buf_info = &int_info->rx;

    if (rx_buf_info->eles_left > 0) {
      // Get element from DR and parse it, add it into rx buff
      uint16_t rx_byte = spi_reg->DR;

      if (dff_bytes == 1 || rx_buf_info->eles_left == 1)
        *((uint8_t *)rx_buf_info->buffer) = rx_byte & 0xFF;
      else
        *((uint16_t *)rx_buf_info->buffer) = rx_byte;

      rx_buf_info->buffer = (char *)rx_buf_info->buffer + dff_bytes;
      rx_buf_info->eles_left -= dff_bytes;
    } else {
      int throwaway_var = spi_reg->DR;
    }
  }

  uint8_t tx_is_done = ((int_info->tx.eles_left <= 0) || !(spi_reg->CR2 & (1 << SPI_CR2_TXEIE_Pos)));
  uint8_t rx_is_done = ((int_info->rx.eles_left <= 0) || !(spi_reg->CR2 & (1 << SPI_CR2_RXNEIE_Pos)));

  if (tx_is_done && rx_is_done) {
    int_info->status = SPI_INTERRUPT_DONE;
    volatile SPIInterruptBuffer_t *tx_buf_info = &int_info->tx;
    volatile SPIInterruptBuffer_t *rx_buf_info = &int_info->rx;

    uint8_t int_word = (1 << SPI_CR2_TXEIE_Pos) | (1 << SPI_CR2_RXNEIE_Pos);
    spi_reg->CR2 &= ~(int_word);

    int_info->callback();

    // Return pointers back to original position
    tx_buf_info->buffer = tx_buf_info->buffer_start;
    tx_buf_info->eles_left = tx_buf_info->len;
    rx_buf_info->buffer = rx_buf_info->buffer_start;
    rx_buf_info->eles_left = rx_buf_info->len;

    if (int_info->circular == SPI_INTERRUPT_CIRCULAR) {
      spi_start_int_word_transfer(spi_reg);
    }

    return 1;
  }

  return 0;
}

/**
 * @brief Sets the callback function for SPI interrupts.
 *
 * This function assigns a user-defined callback function to be invoked
 * when the SPI interrupt completes.
 *
 * @param spi_reg Pointer to the SPI peripheral base address.
 * @param fnc_ptr Pointer to the callback function.
 *
 * @return int Returns 0 on success, or -1 if the SPI port or interrupt info is NULL.
 */
int spi_set_interrupt_callback(const SPI_TypeDef *spi_reg, void (*fnc_ptr)(void)) {
  if (spi_reg == NULL) return -1;

  volatile SPIInterruptInfo_t *int_info = get_spi_int_info(spi_reg);
  if (int_info == NULL) return -1;

  int_info->callback = fnc_ptr;

  return 0;
}

/**
 * @brief Starts an SPI interrupt word transfer.
 *
 * This function initializes the SPI interrupt for word transfers and
 * enables the necessary interrupt flags.
 *
 * @param spi_reg Pointer to the SPI peripheral base address.
 *
 * @return int Returns 1 if the transfer is started, 0 otherwise, or -1 if the SPI port is NULL.
 */
int spi_start_int_word_transfer(SPI_TypeDef *spi_reg) {
  if (spi_reg == NULL) return -1;

  volatile SPIInterruptInfo_t *int_info = get_spi_int_info(spi_reg);
  int_info->status = SPI_INTERRUPT_READY;

  uint32_t int_word = 0;
  if (int_info->tx.en) int_word |= (1 << SPI_CR2_TXEIE_Pos);
  if (int_info->rx.en) int_word |= (1 << SPI_CR2_RXNEIE_Pos);

  spi_reg->CR2 |= int_word;

  return (int_word != 0);
}

/**
 * @brief Enables SPI interrupt transfers for TX and RX.
 *
 * This function enables the SPI interrupt for TX and/or RX operations
 * based on the provided enable settings.
 *
 * @param spi_reg Pointer to the SPI peripheral base address.
 * @param tx Enable/disable setting for TX interrupt.
 * @param rx Enable/disable setting for RX interrupt.
 *
 * @return int Returns 1 if the interrupt is enabled, 0 otherwise, or -1 if the SPI port is NULL.
 */
int spi_enable_int_transfer(SPI_TypeDef *spi_reg, SPIEnable_t tx, SPIEnable_t rx) {
  if (spi_reg == NULL) return -1;

  volatile SPIInterruptInfo_t *int_info = get_spi_int_info(spi_reg);
  int_info->status = SPI_INTERRUPT_READY;

  uint32_t int_word = 0;
  if (tx == SPI_ENABLE) int_word |= (1 << SPI_CR2_TXEIE_Pos);
  if (rx == SPI_ENABLE) int_word |= (1 << SPI_CR2_RXNEIE_Pos);
  spi_reg->CR2 |= int_word;

  return (int_word != 0);
}

/**
 * @brief Disables SPI interrupt transfers.
 *
 * This function disables the SPI interrupt for both TX and RX operations
 * and resets the interrupt status.
 *
 * @param spi_reg Pointer to the SPI peripheral base address.
 *
 * @return int Returns 1 if the interrupt is disabled, 0 otherwise, or -1 if the SPI port is NULL.
 */
int spi_disable_int_transfer(SPI_TypeDef *spi_reg) {
  if (spi_reg == NULL) return -1;

  volatile SPIInterruptInfo_t *int_info = get_spi_int_info(spi_reg);
  int_info->status = SPI_INTERRUPT_READY;

  uint32_t int_word = 0;
  int_word |= (1 << SPI_CR2_TXEIE_Pos);
  int_word |= (1 << SPI_CR2_RXNEIE_Pos);
  spi_reg->CR2 &= ~int_word;

  return (int_word != 0);
}
