#include "stm32f446xx_i2c.h"

#include <string.h>

#include "stm32f446xx.h"

#define I2CS {I2C1, I2C2, I2C3}
#define I2CS_RCC_REGS {&RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB1ENR}
#define I2CS_RCC_POS {RCC_APB1ENR_I2C1EN_Pos, RCC_APB1ENR_I2C2EN_Pos, RCC_APB1ENR_I2C3EN_Pos}
#define SIZEOF(arr) ((int)sizeof(arr) / sizeof(arr[0]))
#define SIZEOFP(arr) ((int)sizeof(arr) / sizeof(uint32_t))  // Memory size of stm32f4

int i2c_peri_clock_control(const I2C_TypeDef *i2c_reg, const I2CEnable_t en) { return 0; }

int i2c_init(I2CHandle_t *i2c_handle) {
  if (i2c_handle == NULL) return -1;

  I2C_TypeDef *addr = i2c_handle->addr;
  I2CConfig_t *cfg = &i2c_handle->cfg;

  return 0;
}

void i2c_deinit(I2C_TypeDef *i2c_reg) {}
