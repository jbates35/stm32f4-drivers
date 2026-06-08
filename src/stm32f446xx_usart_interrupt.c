#include <string.h>

#include "stm32f446xx.h"
#include "stm32f446xx_usart.h"

#define USARTS {USART1, USART2, USART3, UART4, UART5, USART6}
#define USART_RCC_REGS {&RCC->APB2ENR, &RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB2ENR}
#define USART_RSTR_REGS {&RCC->APB2RSTR, &RCC->APB1RSTR, &RCC->APB1RSTR, &RCC->APB1RSTR, &RCC->APB1RSTR, &RCC->APB2RSTR}
#define USART_RCC_POS {4, 17, 18, 19, 20, 5}
#define SIZEOF(arr) ((int)sizeof(arr) / sizeof(arr[0]))
#define SIZEOFP(arr) ((int)sizeof(arr) / sizeof(uint32_t))  // Memory size of stm32f4

#define USART_NUMS 6

typedef enum { USART_INT_TSTATUS_OK, USART_INT_TSTATUS_INVALID_TRANSACTION } USARTIntTransferStatus_t;

// NOTE: This struct could probably use a refacator
typedef struct {
  USARTEnable_t en;  // Whether the interrupt has been enabled (... We need a separate enable byte because the txe in
                     // the CR is required for program control)

  // The following two buffers prevent possible ownership and data corruption issues
  void* out_buff;              // For rx, this gets loaded with memcpy when the rx transmission is done
  uint8_t in_buff[256];        // This gets loaded into during the transmission - NOTE: ONLY RX
  int32_t len;                 // The number of bytes expected in the transimssion
  volatile int32_t eles_left;  // The number of elements left before the tx/rx transmission is considered 'finished'
  USARTInterruptStatus_t
      status;  // Current status of the interrupt - Could be used to be shown to user of API with a getter
  USARTEnable_t tx_circular_en;     // Whether tx transmission should start again once the callback has been called
  uint32_t rx_len_at_config;        // Length as dictated by config so that the len works properly
  USARTEnable_t rx_length_byte_en;  // If enabled, first byte in RX transmission will be used to set the expected length
                                    // of the rx transmission
  uint8_t rx_length_byte_received;  // Whether to take the transmitted byte as the length or the start of the buff
  USARTIgnoreEscapeChars_t rx_ignore_escapes;  // Whether to ignore escapes i.e. throw it away
  void (*callback)(void);                      // Function called when the transmission is finished
} USARTInterruptBuffer_t;

typedef struct {
  USARTInterruptBuffer_t tx;
  USARTInterruptBuffer_t rx;
  // DMA_Stream_TypeDef* tx_stream;
  // DMA_Stream_TypeDef* rx_stream;
  // void (*dma_start_transfer_cb)(DMA_Stream_TypeDef*, uint32_t);
} USARTInterruptInfo_t;

// For recording callbacks, information, etc. Kinda OOP
static USARTInterruptInfo_t usart_interrupt_info[USART_NUMS];

static inline uint8_t get_status(const USART_TypeDef* usart_reg, const uint32_t mask) {
  if (usart_reg->SR & mask) return 1;
  return 0;
}

static inline USARTInterruptInfo_t* get_usart_int_info(const USART_TypeDef* usart_reg) {
  if (usart_reg == USART1)
    return &usart_interrupt_info[0];
  else if (usart_reg == USART2)
    return &usart_interrupt_info[1];
  else if (usart_reg == USART3)
    return &usart_interrupt_info[2];
  else if (usart_reg == UART4)
    return &usart_interrupt_info[3];
  else if (usart_reg == UART5)
    return &usart_interrupt_info[4];
  else if (usart_reg == USART6)
    return &usart_interrupt_info[5];
  return NULL;
}

static inline void clear_usart_info(USARTInterruptInfo_t* int_info) {
  int_info->tx.out_buff = 0;
  int_info->tx.len = 0;
  int_info->tx.eles_left = 0;
  int_info->tx.en = USART_DISABLE;
  int_info->tx.callback = NULL;
  int_info->tx.tx_circular_en = USART_DISABLE;
  int_info->tx.status = USART_INTERRUPT_STATUS_READY;
  int_info->rx.out_buff = 0;
  int_info->rx.len = 0;
  int_info->rx.rx_len_at_config = 0;
  int_info->rx.eles_left = 0;
  int_info->rx.en = USART_DISABLE;
  int_info->rx.callback = NULL;
  int_info->rx.rx_length_byte_received = 0;
  int_info->rx.rx_length_byte_en = USART_DISABLE;
  int_info->rx.status = USART_INTERRUPT_STATUS_READY;
}

USARTStatus_t usart_setup_interrupt(USART_TypeDef* usart_reg, const USARTInterruptConfig_t* setup_info) {
  // NOTE: Mention in docs that any alterations for interrupts that are manual need to be set after this function
  // Such as, perhaps, setting the CTSIE

  // Get interrupt info
  USARTInterruptInfo_t* int_info = get_usart_int_info(usart_reg);
  if (int_info == NULL) return USART_STATUS_INVALID_ADDR;

  clear_usart_info(int_info);

  // Clear all the required reg's first
  usart_reg->CR1 &= ~(USART_CR1_IDLEIE | USART_CR1_PEIE | USART_CR1_TXEIE | USART_CR1_RXNEIE | USART_CR1_TCIE);
  usart_reg->CR3 &= ~(USART_CR3_EIE | USART_CR3_CTSIE);

  // Setup interrupt types
  // Error interrupts
  if (setup_info->error_interrupts_en) {
    usart_reg->CR3 |= USART_CR3_EIE;   // General errors (frame, overrun, noise)
    usart_reg->CR1 |= USART_CR1_PEIE;  // Parity errors
  } else {
    usart_reg->CR3 &= ~USART_CR3_EIE;   // General errors (frame, overrun, noise)
    usart_reg->CR1 &= ~USART_CR1_PEIE;  // Parity errors
  }
  // Idle interrupt
  if (setup_info->idle_en)
    usart_reg->CR1 |= USART_CR1_IDLEIE;
  else
    usart_reg->CR1 &= ~USART_CR1_IDLEIE;
  // Transmission complete interrupt
  if (setup_info->tx_complete_en)
    usart_reg->CR1 |= USART_CR1_TCIE;
  else
    usart_reg->CR1 &= ~USART_CR1_TCIE;

  USARTStatus_t return_status_tx = usart_setup_tx(usart_reg, &setup_info->tx);
  if (return_status_tx != USART_STATUS_OK) return return_status_tx;

  USARTStatus_t return_status_rx = usart_setup_rx(usart_reg, &setup_info->rx);
  if (return_status_rx != USART_STATUS_OK) return return_status_rx;

  return USART_STATUS_OK;
}

USARTStatus_t usart_setup_tx(USART_TypeDef* usart_reg, const USARTBuffer_t* tx) {
  USARTInterruptInfo_t* int_info = get_usart_int_info(usart_reg);
  if (int_info == NULL) return USART_STATUS_INVALID_ADDR;

  usart_reg->CR1 &= ~USART_CR1_TXEIE;

  int_info->tx.out_buff = tx->buff;
  int_info->tx.len = tx->len;
  int_info->tx.eles_left = tx->len;
  int_info->tx.tx_circular_en = tx->tx_circular_en;
  int_info->tx.callback = tx->callback;

  uint8_t enabled = tx->en;
  uint8_t has_buffer = int_info->tx.out_buff != NULL;
  uint8_t has_length = int_info->tx.len > 0;
  int_info->tx.en = enabled && has_buffer && has_length ? USART_ENABLE : USART_DISABLE;

  // Maybe add logic to restart it if tx had been enabled

  return USART_STATUS_OK;
}

USARTStatus_t usart_setup_rx(USART_TypeDef* usart_reg, const USARTBuffer_t* rx) {
  USARTInterruptInfo_t* int_info = get_usart_int_info(usart_reg);
  if (int_info == NULL) return USART_STATUS_INVALID_ADDR;

  // Disable IE in case anything comes here while we're actually resetting the buff and what not
  usart_reg->CR1 &= ~USART_CR1_RXNEIE;

  int_info->rx.out_buff = rx->buff;
  int_info->rx.rx_len_at_config = rx->len;
  int_info->rx.len = rx->len;
  int_info->rx.eles_left = rx->len;
  int_info->rx.rx_length_byte_en = rx->rx_length_byte_en;
  int_info->rx.rx_length_byte_received = 0;
  int_info->rx.callback = rx->callback;

  uint8_t enabled = rx->en;
  uint8_t has_buffer = int_info->rx.out_buff != NULL;
  uint8_t has_length = int_info->rx.len > 0;
  int_info->rx.en = enabled && has_buffer && has_length ? USART_ENABLE : USART_DISABLE;

  if (int_info->rx.en) usart_reg->CR1 |= USART_CR1_RXNEIE;  // This will already be bit shifted essentially

  return USART_STATUS_OK;
}

USARTStatus_t usart_reset_tx_interrupt(USART_TypeDef* usart_reg) {
  USARTInterruptInfo_t* int_info = get_usart_int_info(usart_reg);
  if (int_info == NULL) return USART_STATUS_INVALID_ADDR;

  // Disable TXE to essentially stop any transfer
  usart_reg->CR1 &= ~USART_CR1_TXEIE;

  int_info->tx.eles_left = int_info->tx.len;
  int_info->tx.en = USART_ENABLE;
  int_info->tx.status = USART_INTERRUPT_STATUS_READY;

  return USART_STATUS_OK;
}

USARTStatus_t usart_reset_rx_interrupt(USART_TypeDef* usart_reg) {
  USARTInterruptInfo_t* int_info = get_usart_int_info(usart_reg);
  if (int_info == NULL) return USART_STATUS_INVALID_ADDR;

  int_info->rx.len = int_info->rx.rx_len_at_config;
  int_info->rx.eles_left = int_info->rx.len;
  int_info->rx.en = USART_ENABLE;
  int_info->rx.status = USART_INTERRUPT_STATUS_BUSY;
  int_info->rx.rx_length_byte_received = 0;

  return USART_STATUS_OK;
}

USARTStatus_t usart_start_tx_interrupt(USART_TypeDef* usart_reg) {
  USARTInterruptInfo_t* int_info = get_usart_int_info(usart_reg);
  if (int_info == NULL) return USART_STATUS_INVALID_ADDR;

  // Start transaction, change struct to busy so user knows they shouldn't touch CRs and what not
  int_info->tx.status = USART_INTERRUPT_STATUS_BUSY;
  usart_reg->CR1 |= USART_CR1_TXEIE;

  return USART_STATUS_OK;
}

// NOTE: YO - might need to break this up into separate rx/tx functions as USART tends to be unrelated unlike I2C
USARTInterruptStatus_t usart_irq_tx_word_handling(USART_TypeDef* usart_reg) {
  USARTInterruptInfo_t* int_info = get_usart_int_info(usart_reg);

  if (int_info == NULL) return USART_INTERRUPT_STATUS_INVALID_ADDR;

  uint8_t no_eles_left = (int_info->tx.eles_left == 0);
  uint8_t invalid_eles = (int_info->tx.eles_left > int_info->tx.len);

  if (no_eles_left || invalid_eles) return USART_INTERRUPT_STATUS_ERROR;

  int index = int_info->tx.len - int_info->tx.eles_left;
  uint8_t tx_byte = ((uint8_t*)int_info->tx.out_buff)[index];
  usart_reg->DR = tx_byte;

  int_info->tx.eles_left--;

  no_eles_left = (int_info->tx.eles_left == 0);

  USARTInterruptStatus_t ret_status = int_info->tx.status;
  if (no_eles_left) {
    int_info->tx.status = USART_INTERRUPT_STATUS_DONE;
    ret_status = int_info->tx.status;
    usart_reset_tx_interrupt(usart_reg);

    if (int_info->tx.callback != NULL) int_info->tx.callback();

    if (int_info->tx.tx_circular_en == USART_ENABLE) usart_start_tx_interrupt(usart_reg);
  }

  return ret_status;
}

USARTInterruptStatus_t usart_irq_rx_word_handling(USART_TypeDef* usart_reg) {
  // NOTE:
  // It might be worth having a separate buffer for the tx_rx buffers for the purposes of callbacks
  // If a transmission comes in while performing the callback, there's going to be an error
  USARTInterruptInfo_t* int_info = get_usart_int_info(usart_reg);

  if (int_info == NULL) return USART_INTERRUPT_STATUS_INVALID_ADDR;

  uint8_t rx_byte = usart_reg->DR;

  uint8_t length_byte_en = (int_info->rx.rx_length_byte_en == USART_ENABLE);
  uint8_t length_byte_received = (int_info->rx.rx_length_byte_received);
  if (length_byte_en && !length_byte_received) {
    int_info->rx.len = rx_byte;
    int_info->rx.eles_left = rx_byte;
    int_info->rx.rx_length_byte_received = 1;
    return int_info->rx.status;
  }

  uint8_t ignore_escapes = (int_info->rx.rx_ignore_escapes == USART_IGNORE_ESCAPE_CHARS);
  uint8_t start_of_rx = (int_info->rx.eles_left == int_info->rx.len);
  uint8_t escape_char_sent = (rx_byte == '\0' || rx_byte == '\n' || rx_byte == '\r');

  if (ignore_escapes && escape_char_sent)
    return int_info->rx.status;
  else if (!ignore_escapes && !start_of_rx && escape_char_sent) {
    int_info->rx.len = int_info->rx.len - int_info->rx.eles_left;
    int_info->rx.eles_left = 0;
  } else {
    int index = int_info->rx.len - int_info->rx.eles_left;
    ((uint8_t*)int_info->rx.in_buff)[index] = rx_byte;
    int_info->rx.eles_left--;
  }

  uint8_t no_eles_left = (int_info->rx.eles_left == 0);
  USARTInterruptStatus_t ret_status = int_info->rx.status;
  if (no_eles_left) {
    // Temporarirly change status to DONE, and ret_status to too so user can do something if need be
    int_info->rx.status = USART_INTERRUPT_STATUS_DONE;
    ret_status = int_info->rx.status;

    // Disable interrupt to ensure memcpy is valid, then enable once done
    usart_reg->CR1 &= ~USART_CR1_RXNEIE;
    memcpy(int_info->rx.out_buff, int_info->rx.in_buff, (size_t)int_info->rx.len);
    usart_reg->CR1 |= USART_CR1_RXNEIE;

    // Callback and reset
    if (int_info->rx.callback != NULL) int_info->rx.callback();
    usart_reset_rx_interrupt(usart_reg);
  }

  return ret_status;
}

USARTIRQType_t usart_irq_handling(const USART_TypeDef* usart_reg) {
  // Normal transmissions
  if (get_status(usart_reg, USART_SR_RXNE)) return USART_IRQ_TYPE_RXNE;
  if (get_status(usart_reg, USART_SR_TXE)) return USART_IRQ_TYPE_TXE;
  if (get_status(usart_reg, USART_SR_TC)) return USART_IRQ_TYPE_TRANSMISSION_COMPLETE;

  // Errors
  if (get_status(usart_reg, USART_SR_FE)) return USART_IRQ_TYPE_FRAMING_ERROR;
  if (get_status(usart_reg, USART_SR_IDLE)) return USART_IRQ_TYPE_IDLE;
  if (get_status(usart_reg, USART_SR_LBD)) return USART_IRQ_TYPE_LINE_BREAK_DETECTED;
  if (get_status(usart_reg, USART_SR_ORE)) return USART_IRQ_TYPE_OVERRUN_ERROR;
  if (get_status(usart_reg, USART_SR_PE)) return USART_IRQ_TYPE_PARITY_ERROR;
  if (get_status(usart_reg, USART_SR_NE)) return USART_IRQ_TYPE_NOISE_DETECTED;

  return USART_IRQ_TYPE_NONE;
}

USARTInterruptStatus_t usart_get_tx_interrupt_status(USART_TypeDef* usart_reg) {
  USARTInterruptInfo_t* int_info = get_usart_int_info(usart_reg);
  if (int_info == NULL) return USART_INTERRUPT_STATUS_INVALID_ADDR;
  return int_info->tx.status;
}

USARTInterruptStatus_t usart_get_rx_interrupt_status(USART_TypeDef* usart_reg) {
  USARTInterruptInfo_t* int_info = get_usart_int_info(usart_reg);
  if (int_info == NULL) return USART_INTERRUPT_STATUS_INVALID_ADDR;
  return int_info->rx.status;
}

uint16_t usart_get_rx_interrupt_length(USART_TypeDef* usart_reg) {
  USARTInterruptInfo_t* int_info = get_usart_int_info(usart_reg);
  if (int_info == NULL) return 0;
  return int_info->rx.len;
}

void usart_set_tx_interrupt_length(USART_TypeDef* usart_reg, uint16_t len) {
  USARTInterruptInfo_t* int_info = get_usart_int_info(usart_reg);
  if (int_info == NULL) return;
  int_info->tx.len = len;
}
