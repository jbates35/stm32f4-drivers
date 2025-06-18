
#ifndef INC_STM34F446XX_I2C_H_
#define INC_STM34F446XX_I2C_H_

#include <stdint.h>

#include "stm32f446xx.h"

typedef enum { I2C_SCL_SPEED_SM = 100000, I2C_SCL_SPEED_FM = 400000 } I2CSclSpeed_t;
typedef enum { I2C_FM_DUTY_CYCLE_2 = 0, I2C_FM_DUTY_CYCLE_16_9 } I2CFMDutyCycle_t;
typedef enum { I2C_DISABLE = 0, I2C_ENABLE } I2CEnable_t;

typedef struct {
  I2CSclSpeed_t scl_speed;
  uint16_t fm_duty_cycle;
  uint8_t device_address;
  I2CEnable_t ack_control;
} I2CConfig_t;

typedef struct {
  I2C_TypeDef *addr;
  I2CConfig_t cfg;
} I2CHandle_t;

int i2c_peri_clock_control(const I2C_TypeDef *i2c_reg, const I2CEnable_t en);
int i2c_init(I2CHandle_t *i2c_handle);
void i2c_deinit(I2C_TypeDef *i2c_reg);

#endif
