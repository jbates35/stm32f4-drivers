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

typedef struct {
  void* buff;
  int32_t len;
  int32_t eles_left;
  USARTEnable_t en;
  USARTInterruptStatus_t status;
  USARTInterruptCircular_t circular;
  void (*callback)(void);
} USARTInterruptBuffer_t;

typedef struct {
  USARTInterruptBuffer_t tx;
  USARTInterruptBuffer_t rx;
  // DMA_Stream_TypeDef* tx_stream;
  // DMA_Stream_TypeDef* rx_stream;
  // void (*dma_start_transfer_cb)(DMA_Stream_TypeDef*, uint32_t);
} USARTInterruptInfo_t;

// For recording callbacks, information, etc. Kinda OOP
static volatile USARTInterruptInfo_t usart_interrupt_info[USART_NUMS];

static inline int get_usart_index(const USART_TypeDef* addr) {
  const volatile USART_TypeDef* usart_addrs[] = USARTS;
  int usart_index = -1;
  for (int i = 0; i < SIZEOFP(usart_addrs); i++) {
    if (addr == usart_addrs[i]) {
      usart_index = i;
      break;
    }
  }
  return usart_index;
}

static inline uint8_t get_status(const USART_TypeDef* usart_reg, const uint32_t mask) {
  if (usart_reg->SR & mask) return 1;
  return 0;
}

static inline int is_usart_instance(const USART_TypeDef* usart_reg) {
  return (usart_reg == USART1 || usart_reg == USART2 || usart_reg == USART3 || usart_reg == UART4 ||
          usart_reg == UART5 || usart_reg == USART6);
}

volatile static inline USARTInterruptInfo_t* get_usart_int_info(const USART_TypeDef* usart_reg) {
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

static inline void clear_usart_info(volatile USARTInterruptInfo_t* int_info) {
  int_info->tx.buff = 0;
  int_info->tx.len = 0;
  int_info->tx.eles_left = 0;
  int_info->tx.en = USART_DISABLE;
  int_info->tx.callback = 0;
  int_info->tx.circular = 0;
  int_info->tx.status = USART_INTERRUPT_STATUS_READY;
  int_info->rx.buff = 0;
  int_info->rx.len = 0;
  int_info->rx.eles_left = 0;
  int_info->rx.en = USART_DISABLE;
  int_info->rx.callback = 0;
  int_info->rx.circular = 0;
  int_info->rx.status = USART_INTERRUPT_STATUS_READY;
}

USARTStatus_t usart_setup_interrupt(USART_TypeDef* usart_reg, const USARTInterruptConfig_t* setup_info) {
  // Get interrupt info
  volatile USARTInterruptInfo_t* int_info = get_usart_int_info(usart_reg);
  if (int_info == NULL) return USART_STATUS_INVALID_ADDR;

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

  usart_setup_tx(usart_reg, &setup_info->tx);
  usart_setup_rx(usart_reg, &setup_info->rx);

  return USART_STATUS_OK;
}

USARTStatus_t usart_reset_tx_interrupt(USART_TypeDef* usart_reg) {
  volatile USARTInterruptInfo_t* int_info = get_usart_int_info(usart_reg);
  if (int_info == NULL) return USART_STATUS_INVALID_ADDR;

  // Disable TXE to essentially stop any transfer
  usart_reg->CR1 &= ~USART_CR1_TXEIE;

  int_info->tx.eles_left = int_info->tx.len;
  int_info->tx.en = USART_ENABLE;
  int_info->tx.status = USART_INTERRUPT_STATUS_READY;

  return USART_STATUS_OK;
}

USARTStatus_t usart_reset_rx_interrupt(USART_TypeDef* usart_reg) {
  volatile USARTInterruptInfo_t* int_info = get_usart_int_info(usart_reg);
  if (int_info == NULL) return USART_STATUS_INVALID_ADDR;

  int_info->rx.eles_left = int_info->rx.len;
  int_info->rx.en = USART_ENABLE;
  int_info->rx.status = USART_INTERRUPT_STATUS_READY;

  return USART_STATUS_OK;
}

USARTStatus_t usart_start_tx_interrupt(USART_TypeDef* usart_reg) {
  volatile USARTInterruptInfo_t* int_info = get_usart_int_info(usart_reg);
  if (int_info == NULL) return USART_STATUS_INVALID_ADDR;

  // Start transaction, change struct to busy so user knows they shouldn't touch CRs and what not
  int_info->tx.status = USART_INTERRUPT_STATUS_BUSY;
  usart_reg->CR1 |= USART_CR1_TXEIE;

  return USART_STATUS_OK;
}

USARTStatus_t usart_setup_tx(USART_TypeDef* usart_reg, const USARTBuffer_t* tx) {
  volatile USARTInterruptInfo_t* int_info = get_usart_int_info(usart_reg);
  if (int_info == NULL) return USART_STATUS_INVALID_ADDR;

  usart_reg->CR1 &= ~USART_CR1_TXEIE;

  int_info->tx.buff = tx->buff;
  int_info->tx.len = tx->len;
  int_info->tx.eles_left = tx->len;
  int_info->tx.circular = tx->circular;
  int_info->tx.callback = tx->callback;
  int_info->tx.en = (int_info->tx.buff != NULL && int_info->tx.len) ? USART_ENABLE : USART_DISABLE;

  // Maybe add logic to restart it if tx had been enabled

  return USART_STATUS_OK;
}

USARTStatus_t usart_setup_rx(USART_TypeDef* usart_reg, const USARTBuffer_t* rx) {
  volatile USARTInterruptInfo_t* int_info = get_usart_int_info(usart_reg);
  if (int_info == NULL) return USART_STATUS_INVALID_ADDR;

  // Keep track of whether rx was enabled
  uint8_t rx_enabled = (usart_reg->CR1 & USART_CR1_RXNEIE);

  // Disable, the enable after setting
  usart_reg->CR1 &= ~USART_CR1_RXNEIE;

  int_info->rx.buff = rx->buff;
  int_info->rx.len = rx->len;
  int_info->rx.eles_left = rx->len;
  int_info->rx.circular = rx->circular;
  int_info->rx.callback = rx->callback;
  int_info->rx.en = (int_info->rx.buff != NULL && int_info->rx.len) ? USART_ENABLE : USART_DISABLE;

  usart_reg->CR1 |= rx_enabled;  // This will already be bit shifted essentially

  return USART_STATUS_OK;
}

// NOTE: YO - might need to break this up into separate rx/tx functions as USART tends to be unrelated unlike I2C
USARTInterruptStatus_t usart_irq_tx_word_handling(USART_TypeDef* usart_reg) {
  volatile USARTInterruptInfo_t* int_info = get_usart_int_info(usart_reg);

  if (int_info == NULL) return USART_INTERRUPT_STATUS_INVALID_ADDR;

  uint8_t no_eles_left = (int_info->tx.eles_left == 0);
  uint8_t invalid_eles = (int_info->tx.eles_left > int_info->tx.len);

  if (no_eles_left || invalid_eles) return USART_INTERRUPT_STATUS_ERROR;

  int index = int_info->tx.len - int_info->tx.eles_left;
  uint8_t tx_byte = ((uint8_t*)int_info->tx.buff)[index];
  usart_reg->DR = tx_byte;

  int_info->tx.eles_left--;

  no_eles_left = (int_info->tx.eles_left == 0);

  if (no_eles_left) {
    if (int_info->tx.circular == USART_INTERRUPT_CIRCULAR) {
      usart_reset_tx_interrupt(usart_reg);
      int_info->tx.callback();
      usart_start_tx_interrupt(usart_reg);
    } else {
      usart_reg->CR1 &= ~USART_CR1_TXEIE;
      int_info->tx.callback();
      int_info->tx.status = USART_INTERRUPT_STATUS_DONE;
    }
  }

  return int_info->tx.status;
}

USARTInterruptStatus_t usart_irq_rx_word_handling(USART_TypeDef* usart_reg) {
  USARTIRQType_t irq_reason = usart_irq_handling(usart_reg);
  volatile USARTInterruptInfo_t* int_info = get_usart_int_info(usart_reg);

  if (int_info == NULL) return USART_INTERRUPT_STATUS_INVALID_ADDR;

  USARTIntTransferStatus_t tstatus = USART_INT_TSTATUS_OK;

  return int_info->rx.status;
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
