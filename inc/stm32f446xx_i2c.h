#ifndef INC_STM34F446XX_I2C_H_
#define INC_STM34F446XX_I2C_H_

#include <stdint.h>
#include <stdio.h>

#include "stm32f446xx.h"

// NOTE: I2C Resistor calculation: https://www.ti.com/lit/an/slva689/slva689.pdf?ts=1750740074661
// For some other electrical characteristic information,
// check page 700 of the reference manual for the stm32f446re

typedef enum {
  I2C_STATUS_OK = 0,
  I2C_STATUS_I2C_ADDR_INVALID = -1,
  I2C_STATUS_INVALID_PERI_FREQUENCY = -2,
  I2C_STATUS_INVALID_CCR_CCR_VAL = -3,
  I2C_STATUS_ACK_FAIL = -4,
  I2C_STATUS_INVALID_I2C_INT_TYPE = -5,
  I2C_STATUS_INTERRUPT_BUSY = -6
} I2CStatus_t;
typedef enum { I2C_NO_STOP = 0, I2C_STOP } I2CStop_t;
typedef enum { I2C_DEVICE_MODE_SLAVE = 0, I2C_DEVICE_MODE_MASTER } I2CDeviceMode_t;
typedef enum { I2C_SCL_MODE_SPEED_SM = 0, I2C_SCL_MODE_SPEED_FM } I2CSclMode_t;
typedef enum { I2C_FM_DUTY_CYCLE_2 = 0, I2C_FM_DUTY_CYCLE_16_9 } I2CFMDutyCycle_t;
typedef enum { I2C_TXRX_DIR_SEND = 0, I2C_TXRX_DIR_RECEIVE } I2CTxRxDirection_t;
typedef enum {
  I2C_IRQ_TYPE_NONE = 0,
  I2C_IRQ_TYPE_STARTED,
  I2C_IRQ_TYPE_ADDR_SENT,
  I2C_IRQ_TYPE_TXE,
  I2C_IRQ_TYPE_RXNE,
  I2C_IRQ_TYPE_BTF,
  I2C_IRQ_TYPE_ERROR_ARBLOST,
  I2C_IRQ_TYPE_ERROR_BUS,
  I2C_IRQ_TYPE_ERROR_ACKFAIL,
  I2C_IRQ_TYPE_ERROR_OVERRUN,
  I2C_IRQ_TYPE_ERROR_TIMEOUT
} I2CIRQType_t;
typedef enum {
  I2C_INTERRUPT_STATUS_READY = 0,
  I2C_INTERRUPT_STATUS_BUSY,
  I2C_INTERRUPT_STATUS_DONE,
  I2C_INTERRUPT_STATUS_ERROR
} I2CInterruptStatus_t;
typedef enum { I2C_INTERRUPT_NON_CIRCULAR = 0, I2C_INTERRUPT_CIRCULAR } I2CInterruptCircular_t;
typedef enum { I2C_DISABLE = 0, I2C_ENABLE } I2CEnable_t;

typedef struct {
  uint8_t device_address;
  I2CEnable_t clock_stretch;
} I2CSlaveSetup_t;

typedef struct {
  void *buff;
  int32_t len;
} I2CBuffer_t;

typedef struct {
  I2CBuffer_t tx;
  I2CBuffer_t rx;
  uint8_t address;
  I2CInterruptCircular_t circular;
} I2CInterruptConfig_t;

/**
 * @brief I2C configuration structure
 * @param peri_clock_speed The speed of the peripheral clock (APB1);
 * @param scl_speed Whether I2C in standard, 100kHz, or fast, 400kHz, mode;
 * @param fm_duty_cycle If fast mode, whether t_low = 2t_high, or 9t_low = 16t_high;
 * @param device_address;
 * @param ack_control;
 */
typedef struct {
  uint32_t peri_clock_freq_hz;
  I2CDeviceMode_t device_mode;
  I2CSlaveSetup_t slave_setup;
  I2CSclMode_t scl_mode;
  I2CFMDutyCycle_t fm_duty_cycle;
  I2CEnable_t dma_enable;
  I2CEnable_t enable_on_init;
} I2CConfig_t;

typedef struct {
  I2C_TypeDef *addr;
  I2CConfig_t cfg;
} I2CHandle_t;

I2CStatus_t i2c_peri_clock_control(const I2C_TypeDef *i2c_reg, const I2CEnable_t en);
I2CStatus_t i2c_init(I2CHandle_t *i2c_handle);
I2CStatus_t i2c_deinit(const I2C_TypeDef *i2c_reg);
I2CStatus_t i2c_master_send(I2C_TypeDef *i2c_reg, void *tx_buffer, int32_t len, uint8_t slave_addr,
                            I2CStop_t stop_at_end);
I2CStatus_t i2c_master_receive(I2C_TypeDef *i2c_reg, void *rx_buffer, int32_t len, uint8_t slave_addr);

I2CStatus_t i2c_setup_interrupt(const I2C_TypeDef *i2c_reg, const I2CInterruptConfig_t setup_info);
I2CStatus_t i2c_enable_interrupt(const I2C_TypeDef *i2c_reg, I2CTxRxDirection_t type, I2CEnable_t en);
I2CStatus_t i2c_reset_interrupt(const I2C_TypeDef *i2c_reg);
I2CStatus_t i2c_start_interrupt(I2C_TypeDef *i2c_reg);
I2CInterruptStatus_t i2c_irq_word_handling(I2C_TypeDef *i2c_reg);
I2CIRQType_t i2c_irq_event_handling(const I2C_TypeDef *i2c_reg);
I2CIRQType_t i2c_irq_error_handling(I2C_TypeDef *i2c_reg);
I2CStatus_t i2c_assign_interrupt_cb(const I2C_TypeDef *i2c_reg, void (*callback)(void));

#endif
