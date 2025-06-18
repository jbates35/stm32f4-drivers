
#ifndef INC_STM34F446XX_I2C_H_
#define INC_STM34F446XX_I2C_H_

#include <stdint.h>

#include "stm32f446xx.h"

typedef struct {
  uint32_t scl_speed;
  uint16_t fm_duty_cycle;
  uint8_t device_address;
  uint8_t ack_control;
} I2CConfig_t;

typedef struct {
  I2C_TypeDef *addr;
  I2CConfig_t cfg;
} I2CHandle_t;

#endif
