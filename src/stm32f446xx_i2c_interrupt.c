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
  int32_t i;
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

volatile I2CInterruptInfo_t i2c_interrupt_info[I2CS_NUM] = {0};

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

I2CStatus_t i2c_setup_interrupt(const I2C_TypeDef *i2c_reg, const I2CInterruptConfig_t setup_info) {
  volatile I2CInterruptInfo_t *int_info = get_i2c_int_info(i2c_reg);
  if (int_info == NULL) return I2C_STATUS_I2C_ADDR_INVALID;

  if (int_info->status == I2C_INTERRUPT_STATUS_BUSY) return I2C_STATUS_INTERRUPT_BUSY;

  int_info->tx.buff = setup_info.tx.buff;
  int_info->tx.len = setup_info.tx.len;
  int_info->tx.i = 0;
  int_info->rx.buff = setup_info.tx.buff;
  int_info->rx.len = setup_info.tx.len;
  int_info->rx.i = 0;
  int_info->address = setup_info.address;
  int_info->circular = setup_info.circular;

  return I2C_STATUS_OK;
}

I2CStatus_t i2c_enable_interrupt(const I2C_TypeDef *i2c_reg, I2CTxRxDirection_t type, I2CEnable_t en) {
  volatile I2CInterruptInfo_t *int_info = get_i2c_int_info(i2c_reg);
  if (int_info == NULL) return I2C_STATUS_I2C_ADDR_INVALID;

  if (type == I2C_TXRX_DIR_SEND)
    int_info->tx.en = en;
  else if (type == I2C_TXRX_DIR_RECEIVE)
    int_info->rx.en = en;
  else
    return I2C_STATUS_INVALID_I2C_INT_TYPE;

  return I2C_STATUS_OK;
}

I2CInterruptType_t i2c_irq_event_handling(const I2C_TypeDef *i2c_reg) {
  if (get_status(i2c_reg, I2C_SR1_SB)) return I2C_INT_TYPE_STARTED;
  if (get_status(i2c_reg, I2C_SR1_ADDR)) return I2C_INT_TYPE_ADDR_SENT;
  if (get_status(i2c_reg, I2C_SR1_RXNE)) return I2C_INT_TYPE_RXNE;
  if (get_status(i2c_reg, I2C_SR1_TXE)) return I2C_INT_TYPE_TXE;
  if (get_status(i2c_reg, I2C_SR1_BTF)) return I2C_INT_TYPE_BTF;

  return I2C_INT_TYPE_NONE;
}

I2CInterruptType_t i2c_irq_error_handling(I2C_TypeDef *i2c_reg) {
  if (get_status(i2c_reg, I2C_SR1_ARLO)) {
    clear_flag(i2c_reg, I2C_SR1_ARLO);
    return I2C_INT_TYPE_ERROR_ARBLOST;
  }

  if (get_status(i2c_reg, I2C_SR1_BERR)) {
    clear_flag(i2c_reg, I2C_SR1_BERR);
    return I2C_INT_TYPE_ERROR_BUS;
  }

  if (get_status(i2c_reg, I2C_SR1_AF)) {
    clear_flag(i2c_reg, I2C_SR1_AF);
    return I2C_INT_TYPE_ERROR_ACKFAIL;
  }

  if (get_status(i2c_reg, I2C_SR1_TIMEOUT)) {
    clear_flag(i2c_reg, I2C_SR1_TIMEOUT);
    return I2C_INT_TYPE_ERROR_TIMEOUT;
  }

  if (get_status(i2c_reg, I2C_SR1_OVR)) {
    clear_flag(i2c_reg, I2C_SR1_OVR);
    return I2C_INT_TYPE_ERROR_OVERRUN;
  }

  return I2C_INT_TYPE_NONE;
}

I2CInterruptStatus_t i2c_irq_word_handling(I2C_TypeDef *i2c_reg) {
  I2CInterruptType_t int_type = i2c_irq_event_handling(i2c_reg);
  volatile I2CInterruptInfo_t *int_info = get_i2c_int_info(i2c_reg);

  if (int_info->tx.en) {
    if (int_type == I2C_INT_TYPE_STARTED) {
      // send_addr(i2c_reg, 0x68, 0);
    } else if (int_type == I2C_INT_TYPE_ADDR_SENT) {
      // init_xmission(i2c_reg, 0);
    } else if (int_type == I2C_INT_TYPE_TXE && int_info->tx.eles_left) {
      // send_data(i2c_reg, &int_info->tx);
    } else if (int_type == I2C_INT_TYPE_TXE) {
      // end_tx(i2c_reg, &int_info->tx, &int_info->rx);
    }
  } else if (int_info->rx.en) {
    // RX:
    if (int_type == I2C_INT_TYPE_STARTED) {
      // send_addr(i2c_reg, 0x68, 1);
    } else if (int_type == I2C_INT_TYPE_ADDR_SENT) {
      // init_xmission(i2c_reg, 1);
      // single_byte_rx_handler(i2c_reg, &int_info->rx);
    } else if (int_type == I2C_INT_TYPE_RXNE && int_info->rx.eles_left) {
      // receive_data(i2c_reg, &int_info->rx);
    }
  }

  return I2C_INTERRUPT_STATUS_DONE;
}
