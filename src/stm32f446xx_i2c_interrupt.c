#include <stdio.h>

#include "stm32f446xx.h"
#include "stm32f446xx_i2c.h"

#define I2CS_NUM 3
#define I2CS {I2C1, I2C2, I2C3}
#define I2CS_RCC_REGS {&RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB1ENR}
#define I2CS_RSTR_REGS {&RCC->APB1RSTR, &RCC->APB1RSTR, &RCC->APB1RSTR}
#define I2CS_RCC_POS {RCC_APB1ENR_I2C1EN_Pos, RCC_APB1ENR_I2C2EN_Pos, RCC_APB1ENR_I2C3EN_Pos}
#define SIZEOF(arr) ((int)sizeof(arr) / sizeof(arr[0]))
#define SIZEOFP(arr) ((int)sizeof(arr) / sizeof(uint32_t))  // Memory size of stm32f4

typedef enum { I2C_WRITE = 0, I2C_READ = 1 } I2CWriteOrRead_t;

typedef struct {
  void *buff;
  int32_t len;
  int32_t eles_left;
  I2CEnable_t en;
} I2CInterruptBuffer_t;

typedef struct {
  I2CInterruptBuffer_t tx;
  I2CInterruptBuffer_t rx;
  I2CInterruptStatus_t status;
  uint8_t address;
  I2CInterruptCircular_t circular;
  void (*callback)(void);
} I2CInterruptInfo_t;

static volatile I2CInterruptInfo_t i2c_interrupt_info[I2CS_NUM] = {0};

static inline int get_i2c_index(const I2C_TypeDef *addr) {
  const volatile I2C_TypeDef *i2c_addrs[] = I2CS;
  int i2c_index = -1;
  for (int i = 0; i < SIZEOFP(i2c_addrs); i++) {
    if (addr == i2c_addrs[i]) {
      i2c_index = i;
      break;
    }
  }
  return i2c_index;
}

static inline uint8_t get_status(const I2C_TypeDef *i2c_reg, const uint32_t mask) {
  if (i2c_reg->SR1 & mask) return 1;
  return 0;
}

static inline void clear_flag(I2C_TypeDef *i2c_reg, const uint32_t mask) { i2c_reg->SR1 &= ~mask; }

volatile static inline I2CInterruptInfo_t *get_i2c_int_info(const I2C_TypeDef *i2c_reg) {
  if (i2c_reg == I2C1)
    return &i2c_interrupt_info[0];
  else if (i2c_reg == I2C2)
    return &i2c_interrupt_info[1];
  else if (i2c_reg == I2C3)
    return &i2c_interrupt_info[2];
  return NULL;
}

I2CStatus_t i2c_setup_interrupt(I2C_TypeDef *i2c_reg, const I2CInterruptConfig_t *setup_info) {
  volatile I2CInterruptInfo_t *int_info = get_i2c_int_info(i2c_reg);
  if (int_info == NULL) return I2C_STATUS_I2C_ADDR_INVALID;

  // Maybe have a look at this line ...
  if (int_info->status == I2C_INTERRUPT_STATUS_BUSY) return I2C_STATUS_INTERRUPT_BUSY;

  // TX specific buffer
  int_info->tx.buff = setup_info->tx.buff;
  int_info->tx.len = setup_info->tx.len;
  int_info->tx.eles_left = setup_info->tx.len;

  if (int_info->tx.buff != NULL && int_info->tx.len >= 0)
    int_info->tx.en = I2C_ENABLE;
  else
    int_info->tx.en = I2C_DISABLE;

  // Enable if tx stuff has buffer was assigned and length isn't 0
  int_info->tx.en = (int_info->tx.buff != NULL && int_info->tx.len) ? I2C_ENABLE : I2C_DISABLE;

  // RX specific buffer
  int_info->rx.buff = setup_info->rx.buff;
  int_info->rx.len = setup_info->rx.len;
  int_info->rx.eles_left = setup_info->rx.len;

  // Enable if rx stuff has buffer was assigned and length isn't 0
  int_info->rx.en = (int_info->rx.buff != NULL && int_info->rx.len) ? I2C_ENABLE : I2C_DISABLE;

  // General interrupt stuff
  int_info->address = setup_info->address;
  int_info->circular = setup_info->circular;
  int_info->callback = setup_info->callback;

  return I2C_STATUS_OK;
}

I2CIRQType_t i2c_irq_event_handling(const I2C_TypeDef *i2c_reg) {
  if (get_status(i2c_reg, I2C_SR1_SB)) return I2C_IRQ_TYPE_STARTED;
  if (get_status(i2c_reg, I2C_SR1_ADDR)) return I2C_IRQ_TYPE_ADDR_SENT;
  if (get_status(i2c_reg, I2C_SR1_RXNE)) return I2C_IRQ_TYPE_RXNE;
  if (get_status(i2c_reg, I2C_SR1_TXE)) return I2C_IRQ_TYPE_TXE;
  if (get_status(i2c_reg, I2C_SR1_BTF)) return I2C_IRQ_TYPE_BTF;

  return I2C_IRQ_TYPE_NONE;
}

I2CIRQType_t i2c_irq_error_handling(I2C_TypeDef *i2c_reg) {
  if (get_status(i2c_reg, I2C_SR1_ARLO)) {
    clear_flag(i2c_reg, I2C_SR1_ARLO);
    return I2C_IRQ_TYPE_ERROR_ARBLOST;
  }

  if (get_status(i2c_reg, I2C_SR1_BERR)) {
    clear_flag(i2c_reg, I2C_SR1_BERR);
    return I2C_IRQ_TYPE_ERROR_BUS;
  }

  if (get_status(i2c_reg, I2C_SR1_AF)) {
    clear_flag(i2c_reg, I2C_SR1_AF);
    return I2C_IRQ_TYPE_ERROR_ACKFAIL;
  }

  if (get_status(i2c_reg, I2C_SR1_TIMEOUT)) {
    clear_flag(i2c_reg, I2C_SR1_TIMEOUT);
    return I2C_IRQ_TYPE_ERROR_TIMEOUT;
  }

  if (get_status(i2c_reg, I2C_SR1_OVR)) {
    clear_flag(i2c_reg, I2C_SR1_OVR);
    return I2C_IRQ_TYPE_ERROR_OVERRUN;
  }

  return I2C_IRQ_TYPE_NONE;
}

I2CStatus_t i2c_start_interrupt(I2C_TypeDef *i2c_reg) {
  volatile I2CInterruptInfo_t *int_info = get_i2c_int_info(i2c_reg);
  if (int_info == NULL) return I2C_STATUS_I2C_ADDR_INVALID;

  // Start transaction, change struct to busy so user knows they shouldn't touch CRs and what not
  int_info->status = I2C_INTERRUPT_STATUS_BUSY;
  i2c_reg->CR1 |= I2C_CR1_START;

  return I2C_STATUS_OK;
}

typedef enum { I2C_INT_TSTATUS_OK = 0, I2C_INT_TSTATUS_INVALID_TRANSACTION } I2CIntTransferStatus_t;

static inline void send_addr(I2C_TypeDef *i2c_reg, uint8_t addr, uint8_t lsb) { i2c_reg->DR = ((addr << 1) | lsb); }

static inline void init_xmission(I2C_TypeDef *i2c_reg, uint8_t ack) {
  // If rx, it needs acks sent
  if (ack) i2c_reg->CR1 |= I2C_CR1_ACK;

  // Enable data IRQs
  i2c_reg->CR2 |= I2C_CR2_ITBUFEN;

  // Clear SR regs
  (void)i2c_reg->SR1;
  (void)i2c_reg->SR2;
}

static inline I2CIntTransferStatus_t send_data(I2C_TypeDef *i2c_reg, volatile I2CInterruptBuffer_t *tx_buff) {
  // Return if no elements left or if for some reason len is below eles_left
  uint8_t invalid_eles = (tx_buff->eles_left > tx_buff->len);
  uint8_t no_eles_left = (tx_buff->eles_left <= 0);
  if (invalid_eles || no_eles_left) return I2C_INT_TSTATUS_INVALID_TRANSACTION;

  // Load byte from buff into DR register
  int i = tx_buff->len - tx_buff->eles_left;
  i2c_reg->DR = ((uint8_t *)tx_buff->buff)[i];
  tx_buff->eles_left--;

  return I2C_INT_TSTATUS_OK;
}

static inline I2CIntTransferStatus_t end_send_data(I2C_TypeDef *i2c_reg, volatile I2CInterruptInfo_t *int_info) {
  // After TXE, need to make sure shift register is also cleared
  if (!(i2c_reg->SR1 & I2C_SR1_BTF)) return I2C_INT_TSTATUS_OK;

  // Disable data from triggering isr
  i2c_reg->CR2 &= ~I2C_CR2_ITBUFEN;

  uint8_t rx_enabled = (int_info->rx.en == I2C_ENABLE);
  uint8_t rx_has_eles = (int_info->rx.eles_left > 0);
  if (rx_enabled && rx_has_eles)
    // If rx is locked and loaded, we need a repeated start
    i2c_reg->CR1 |= I2C_CR1_START;
  else
    // If rx is weak and disabled, we need a stop bit sent
    i2c_reg->CR1 |= I2C_CR1_STOP;

  int_info->tx.en = I2C_DISABLE;

  return I2C_INT_TSTATUS_OK;
}

static inline I2CIntTransferStatus_t single_byte_setup(I2C_TypeDef *i2c_reg, volatile I2CInterruptBuffer_t *rx_buff) {
  // Return if no elements left or if for some reason len is below eles_left
  // Also make sure it's actually a single byte transfer
  uint8_t invalid_eles = (rx_buff->eles_left > rx_buff->len);
  uint8_t no_eles_left = (rx_buff->eles_left <= 0);
  uint8_t len_isnt_1 = (rx_buff->len != 1);
  if (invalid_eles || no_eles_left || len_isnt_1) return I2C_INT_TSTATUS_INVALID_TRANSACTION;

  // Set ack low and set stop bit high
  i2c_reg->CR1 &= ~(1 << I2C_CR1_ACK_Pos);
  i2c_reg->CR1 |= (1 << I2C_CR1_STOP_Pos);

  // Clear all flags from SR regs
  (void)i2c_reg->SR1;
  (void)i2c_reg->SR2;

  return I2C_INT_TSTATUS_OK;
}

static inline I2CIntTransferStatus_t receive_data(I2C_TypeDef *i2c_reg, volatile I2CInterruptBuffer_t *rx_buff) {
  // Make sure it's actually a single byte transfer
  uint8_t invalid_eles = (rx_buff->eles_left > rx_buff->len);
  uint8_t no_eles_left = (rx_buff->eles_left <= 0);
  if (invalid_eles || no_eles_left) return I2C_INT_TSTATUS_INVALID_TRANSACTION;

  // STM32 I2C implementation requires this procedure when 2 bytes left
  // Refer to manual
  if (rx_buff->eles_left == 2) {
    i2c_reg->CR1 &= ~(1 << I2C_CR1_ACK_Pos);
    i2c_reg->CR1 |= (1 << I2C_CR1_STOP_Pos);
  }

  // Load byte from DR into given buffer
  int i = rx_buff->len - rx_buff->eles_left;
  ((uint8_t *)rx_buff->buff)[i] = (uint8_t)i2c_reg->DR;
  rx_buff->eles_left--;

  // Disable data IRQ and disable RX buffer when transfer done
  if (!rx_buff->eles_left) {
    i2c_reg->CR2 &= ~I2C_CR2_ITBUFEN;
    rx_buff->en = I2C_DISABLE;
  }

  return I2C_INT_TSTATUS_OK;
}

I2CInterruptStatus_t i2c_irq_word_handling(I2C_TypeDef *i2c_reg) {
  I2CIRQType_t irq_reason = i2c_irq_event_handling(i2c_reg);
  volatile I2CInterruptInfo_t *int_info = get_i2c_int_info(i2c_reg);

  if (irq_reason == I2C_IRQ_TYPE_NONE || int_info->status == I2C_INTERRUPT_STATUS_ERROR) return int_info->status;

  I2CIntTransferStatus_t tstatus = I2C_INT_TSTATUS_OK;

  if (int_info->tx.en) {
    // TX:
    if (irq_reason == I2C_IRQ_TYPE_STARTED) {
      send_addr(i2c_reg, 0x68, 0);
    } else if (irq_reason == I2C_IRQ_TYPE_ADDR_SENT) {
      init_xmission(i2c_reg, 0);
    } else if (irq_reason == I2C_IRQ_TYPE_TXE && int_info->tx.eles_left) {
      tstatus = send_data(i2c_reg, &int_info->tx);
    } else if (irq_reason == I2C_IRQ_TYPE_TXE) {
      tstatus = end_send_data(i2c_reg, int_info);
    }
  } else if (int_info->rx.en) {
    // RX:
    if (irq_reason == I2C_IRQ_TYPE_STARTED) {
      send_addr(i2c_reg, 0x68, 1);
    } else if (irq_reason == I2C_IRQ_TYPE_ADDR_SENT) {
      init_xmission(i2c_reg, 1);
      // Special method required to set up if length of transfer is just 1
      if (int_info->rx.len == 1) tstatus = single_byte_setup(i2c_reg, &int_info->rx);
    } else if (irq_reason == I2C_IRQ_TYPE_RXNE && int_info->rx.eles_left) {
      tstatus = receive_data(i2c_reg, &int_info->rx);
    }
  }

  // If there was a problem, set to error and return as such
  if (tstatus != I2C_INT_TSTATUS_OK) {
    int_info->status = I2C_INTERRUPT_STATUS_ERROR;
    return int_info->status;
  }

  // If both tx and rx are done, we need to cleanup logic, set callback, and load buffer again if set to circular
  uint8_t tx_done = !int_info->tx.en || !int_info->tx.eles_left;
  uint8_t rx_done = !int_info->rx.en || !int_info->rx.eles_left;

  // Check for this to ensure following logic is done only once
  uint8_t int_busy = (int_info->status == I2C_INTERRUPT_STATUS_BUSY);

  if (tx_done && rx_done && int_busy) {
    int_info->status = I2C_INTERRUPT_STATUS_DONE;

    if (int_info->callback) int_info->callback();

    // Re-setup and re-enable interrupt if circular
    if (int_info->circular == I2C_INTERRUPT_CIRCULAR) {
      // Restart interrupt
      i2c_reset_interrupt(i2c_reg);
      i2c_start_interrupt(i2c_reg);
    }
  }

  return int_info->status;
}

I2CStatus_t i2c_reset_interrupt(const I2C_TypeDef *i2c_reg) {
  volatile I2CInterruptInfo_t *int_info = get_i2c_int_info(i2c_reg);
  if (int_info == NULL) return I2C_STATUS_I2C_ADDR_INVALID;

  // TX specific buffer
  int_info->tx.eles_left = int_info->tx.len;
  int_info->tx.en = I2C_ENABLE;

  // RX specific buffer
  int_info->rx.eles_left = int_info->rx.len;
  int_info->rx.en = I2C_ENABLE;

  return I2C_STATUS_OK;
}

  volatile I2CInterruptInfo_t *int_info = get_i2c_int_info(i2c_reg);

}
