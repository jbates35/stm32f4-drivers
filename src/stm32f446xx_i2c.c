#include "stm32f446xx_i2c.h"

#include <string.h>

#include "stm32f446xx.h"

#define I2CS {I2C1, I2C2, I2C3}
#define I2CS_RCC_REGS {&RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB1ENR}
#define I2CS_RSTR_REGS {&RCC->APB1RSTR, &RCC->APB1RSTR, &RCC->APB1RSTR}
#define I2CS_RCC_POS {RCC_APB1ENR_I2C1EN_Pos, RCC_APB1ENR_I2C2EN_Pos, RCC_APB1ENR_I2C3EN_Pos}
#define SIZEOF(arr) ((int)sizeof(arr) / sizeof(arr[0]))
#define SIZEOFP(arr) ((int)sizeof(arr) / sizeof(uint32_t))  // Memory size of stm32f4

static inline int get_i2c_index(const I2C_TypeDef *addr) {
  volatile I2C_TypeDef *i2c_addrs[] = I2CS;
  int i2c_index = -1;
  for (int i = 0; i < SIZEOFP(i2c_addrs); i++) {
    if (addr == i2c_addrs[i]) {
      i2c_index = i;
      break;
    }
  }
  return i2c_index;
}

static inline uint16_t get_i2c_ccr_ccr_val(I2CSclMode_t scl_mode, I2CFMDutyCycle_t dc, uint32_t peri_freq) {
  uint16_t ccr_val = 0;
  if (scl_mode == I2C_SCL_MODE_SPEED_SM) {
    // In standard mode: T_high = T_low = CCR * T_peri_clock
    uint32_t scl_freq = 100000;
    ccr_val = peri_freq / (scl_freq * 2);

    // Basically, ceil the value - to ensure no i2c clock speed over 100k
    if (peri_freq % (scl_freq * 2)) ccr_val++;

    // Under 4 is invalid here, so 0 it so error handling can be done
    if (ccr_val < 4) ccr_val = 0;
  } else if (dc == I2C_FM_DUTY_CYCLE_2) {
    // In fast mode with DC=0, T_low = 2T_high, therefore:
    // T_i2c = 3*T_high
    // And therefore:
    // T_i2c = 3 * CCR * T_peri_clock
    uint32_t scl_freq = 400000;
    ccr_val = peri_freq / (scl_freq * 3);

    // Basically, ceil the value - to ensure no i2c clock speed over 100k
    if (peri_freq % (scl_freq * 3)) ccr_val++;

    // Under 4 is invalid here, so 0 it so error handling can be done
    if (ccr_val < 4) ccr_val = 0;
  } else if (dc == I2C_FM_DUTY_CYCLE_16_9) {
    // In fast mode with DC=1, 9T_low = 16T_high, therefore:
    // T_i2c = T_high + 16*T_high/9 = 25*T_high/9
    // And therefore:
    // 9 * T_i2c / 25 = 9 * CCR * T_peri_clock
    uint32_t scl_freq = 400000;
    ccr_val = peri_freq / (scl_freq * 25);

    // Basically, ceil the value - to ensure no i2c clock speed over 100k
    if (peri_freq % (scl_freq * 25)) ccr_val++;
  }

  if (ccr_val > 0xFFF) ccr_val = 0xFFF;

  return ccr_val;
}

I2CStatus_t i2c_peri_clock_control(const I2C_TypeDef *i2c_reg, const I2CEnable_t en) {
  // Get the index of the I2C reg in the array defined at top
  int index = get_i2c_index(i2c_reg);
  if (index < 0) return I2C_STATUS_I2C_ADDR_INVALID;

  volatile uint32_t *i2c_reg_arr[] = I2CS_RCC_REGS;
  const unsigned int i2c_rcc_pos[] = I2CS_RCC_POS;

  if (en == I2C_DISABLE)
    *i2c_reg_arr[index] &= ~(1 << i2c_rcc_pos[index]);
  else
    *i2c_reg_arr[index] |= (1 << i2c_rcc_pos[index]);

  return I2C_STATUS_OK;
}

I2CStatus_t i2c_init(I2CHandle_t *i2c_handle) {
  I2C_TypeDef *addr = i2c_handle->addr;

  int index = get_i2c_index(addr);
  if (index < 0) return I2C_STATUS_I2C_ADDR_INVALID;

  I2CConfig_t *cfg = &i2c_handle->cfg;

  // Make sure freq can be set
  uint8_t freq_mhz = cfg->peri_clock_freq_hz / (int)1E6;
  if (freq_mhz < 2 || freq_mhz > 50) return I2C_STATUS_INVALID_PERI_FREQUENCY;

  uint16_t ccr_ccr_val = get_i2c_ccr_ccr_val(cfg->scl_mode, cfg->fm_duty_cycle, cfg->peri_clock_freq_hz);
  if (ccr_ccr_val == 0) return I2C_STATUS_INVALID_CCR_CCR_VAL;

  // First, need I2C reg disabled
  addr->CR1 |= (1 << I2C_CR1_SWRST_Pos);
  addr->CR1 = 0;

  ///// CR1 register: /////
  uint16_t cr1_word = 0;

  // Whether an acknowledge should be received after matched address or data
  if (cfg->ack_enable) cr1_word |= (1 << I2C_CR1_ACK_Pos);

  // If in slave mode, whether to set in stretch mode (0 is stretch on stm32)
  if (cfg->device_mode == I2C_DEVICE_MODE_SLAVE && !cfg->slave_setup.clock_stretch)
    cr1_word |= (1 << I2C_CR1_NOSTRETCH_Pos);

  if (cfg->enable_on_init) cr1_word |= (1 << I2C_CR1_PE_Pos);

  ///// CR2 register: /////
  uint16_t cr2_word = 0;

  // DMA Enable
  if (cfg->dma_enable) cr2_word |= (1 << I2C_CR2_DMAEN_Pos);

  // Interrupt enable
  // if (cfg->interrupt_enable) {
  //   cr2_word |= (1 << I2C_CR2_ITBUFEN_Pos);
  //   cr2_word |= (1 << I2C_CR2_ITEVTEN_Pos);
  // }

  // Tell i2c peripeheral how fast frequency is
  cr2_word |= (freq_mhz << I2C_CR2_FREQ_Pos);

  // Slave address
  if (cfg->device_mode == I2C_DEVICE_MODE_SLAVE)
    addr->OAR1 = ((cfg->slave_setup.device_address & 0x7F) << I2C_OAR1_ADD1_Pos);

  ///// CCR register: /////
  uint16_t ccr_word = 0;

  // Whether i2c set in standard mode (100kHz), or fast mode (400kHz)
  if (cfg->scl_mode) ccr_word |= (1 << I2C_CCR_FS_Pos);

  // Duty cycle mode, only when in fast mode
  if (cfg->fm_duty_cycle == I2C_FM_DUTY_CYCLE_16_9) ccr_word |= (1 << I2C_CCR_DUTY_Pos);

  // Set CCR reg - formulas shows in function above
  ccr_word |= (ccr_ccr_val << I2C_CCR_CCR_Pos);

  // Initalize all regs - need to set CR1 last as it has the peripheral enable reg
  addr->CCR = ccr_word;
  addr->CR2 = cr2_word;
  addr->CR1 = cr1_word;

  return I2C_STATUS_OK;
}

I2CStatus_t i2c_deinit(const I2C_TypeDef *i2c_reg) {
  int index = get_i2c_index(i2c_reg);
  if (index < 0) return I2C_STATUS_I2C_ADDR_INVALID;

  volatile uint32_t *i2c_rstr_arr[] = I2CS_RSTR_REGS;
  const unsigned int i2c_rcc_pos[] = I2CS_RCC_POS;

  *i2c_rstr_arr[index] |= (1 << i2c_rcc_pos[index]);

  return 0;
}
