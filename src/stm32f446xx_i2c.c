#include "stm32f446xx_i2c.h"

#include <string.h>

#include "stm32f446xx.h"

#define I2CS {I2C1, I2C2, I2C3}
#define I2CS_RCC_REGS {&RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB1ENR}
#define I2CS_RCC_POS {RCC_APB1ENR_I2C1EN_Pos, RCC_APB1ENR_I2C2EN_Pos, RCC_APB1ENR_I2C3EN_Pos}
#define SIZEOF(arr) ((int)sizeof(arr) / sizeof(arr[0]))
#define SIZEOFP(arr) ((int)sizeof(arr) / sizeof(uint32_t))  // Memory size of stm32f4

int i2c_peri_clock_control(const I2C_TypeDef *i2c_reg, const I2CEnable_t en) {
  if (i2c_reg == NULL) return -1;  // Error: null pointer

  const I2C_TypeDef *i2cs_arr[] = I2CS;
  int i = 0;

  for (; i < SIZEOFP(i2cs_arr); i++) {
    if (i2cs_arr[i] == i2c_reg) break;
  }

  if (i >= SIZEOFP(i2cs_arr)) return -1;

  volatile uint32_t *i2c_regs[] = I2CS_RCC_REGS;
  const unsigned int i2c_rcc_pos[] = I2CS_RCC_POS;

  if (en == I2C_ENABLE)
    *i2c_regs[i] |= (1 << i2c_rcc_pos[i]);
  else if (en == I2C_DISABLE)
    *i2c_regs[i] &= ~(1 << i2c_rcc_pos[i]);
  else
    return -2;

  return 0;
}

int i2c_init(I2CHandle_t *i2c_handle) {
  if (i2c_handle == NULL) return -1;

  I2C_TypeDef *addr = i2c_handle->addr;
  I2CConfig_t *cfg = &i2c_handle->cfg;

  // Make sure freq can be set
  uint8_t freq_mhz = cfg->peri_clock_speed / (int)1E6;
  if (freq_mhz < 2 || freq_mhz > 50) return -2;

  ///// CR1 register: /////
  int32_t cr1_word = 0;

  // Whether an acknowledge should be received after matched address or data
  if (cfg->ack_control) cr1_word |= (1 << I2C_CR1_ACK_Pos);

  // If in slave mode, whether to set in stretch mode (0 is stretch on stm32)
  if (!cfg->slave_setup.clock_stretch) cr1_word |= (1 << I2C_CR1_NOSTRETCH_Pos);

  addr->CR1 = cr1_word;

  ///// CR2 register: /////
  uint32_t cr2_word = 0;

  // DMA Enable
  if (cfg->dma_enable) cr2_word = (1 << I2C_CR2_DMAEN_Pos);

  // Interrupt enable
  if (cfg->interrupt_enable) {
    cr2_word |= (1 << I2C_CR2_ITBUFEN_Pos);
    cr2_word |= (1 << I2C_CR2_ITEVTEN_Pos);
  }

  cr2_word |= (freq_mhz << I2C_CR2_FREQ_Pos);

  addr->CR2 = cr2_word;

  // Slave address
  addr->OAR1 = ((cfg->slave_setup.device_address & 0x7F) << I2C_OAR1_ADD1_Pos);

  // NEED SPEED STUFF HERE

  if (cfg->enable_on_init) addr->CR1 |= (1 << I2C_CR1_PE_Pos);
  return 0;
}

int i2c_deinit(I2C_TypeDef *i2c_reg) {
  if (i2c_reg == NULL) return -1;

  memset(i2c_reg, 0, sizeof(I2C_TypeDef));

  return 0;
}
