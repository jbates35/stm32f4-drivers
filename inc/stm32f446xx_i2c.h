
#ifndef INC_STM34F446XX_I2C_H_
#define INC_STM34F446XX_I2C_H_

#include <stdatomic.h>
#include <stdint.h>

#include "stm32f446xx.h"

typedef enum { I2C_MODE_SLAVE = 0, I2C_MODE_MASTER = 1 } I2CMode_t;
typedef enum { I2C_SCL_SPEED_SM = 100000, I2C_SCL_SPEED_FM = 400000 } I2CSclSpeed_t;
typedef enum { I2C_FM_DUTY_CYCLE_2 = 0, I2C_FM_DUTY_CYCLE_16_9 } I2CFMDutyCycle_t;
typedef enum { I2C_DISABLE = 0, I2C_ENABLE } I2CEnable_t;

typedef struct {
  uint8_t device_address;
  I2CEnable_t clock_stretch;
} I2CSlaveSetup_t;
/**
 * @brief I2C configuration structure
 * @param peri_clock_speed The speed of the peripheral clock (APB1);
 * @param scl_speed Whether I2C in standard, 100kHz, or fast, 400kHz, mode;
 * @param fm_duty_cycle If fast mode, whether t_low = 2t_high, or 9t_low = 16t_high;
 * @param device_address;
 * @param ack_control;
 */
typedef struct {
  uint32_t peri_clock_speed;
  uint16_t fm_duty_cycle;
  I2CMode_t mode;
  I2CSlaveSetup_t slave_setup;
  I2CEnable_t dma_enable;
  I2CEnable_t interrupt_enable;
  I2CSclSpeed_t scl_speed;
  I2CEnable_t ack_control;
  I2CEnable_t enable_on_init;
} I2CConfig_t;

typedef struct {
  I2C_TypeDef *addr;
  I2CConfig_t cfg;
} I2CHandle_t;

int i2c_peri_clock_control(const I2C_TypeDef *i2c_reg, const I2CEnable_t en);
int i2c_init(I2CHandle_t *i2c_handle);
int i2c_deinit(I2C_TypeDef *i2c_reg);

#endif
