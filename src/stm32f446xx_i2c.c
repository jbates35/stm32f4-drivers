// TODO:
// - Clear sr flags from master tx
// - Calculate tRise

#include "stm32f446xx_i2c.h"

#include "stm32f446xx.h"

#define I2CS {I2C1, I2C2, I2C3}
#define I2CS_RCC_REGS {&RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB1ENR}
#define I2CS_RSTR_REGS {&RCC->APB1RSTR, &RCC->APB1RSTR, &RCC->APB1RSTR}
#define I2CS_RCC_POS {RCC_APB1ENR_I2C1EN_Pos, RCC_APB1ENR_I2C2EN_Pos, RCC_APB1ENR_I2C3EN_Pos}
#define SIZEOF(arr) ((int)sizeof(arr) / sizeof(arr[0]))
#define SIZEOFP(arr) ((int)sizeof(arr) / sizeof(uint32_t))  // Memory size of stm32f4

typedef enum { I2C_WRITE = 0, I2C_READ = 1 } I2CWriteOrRead_t;

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

static inline uint8_t get_status(const I2C_TypeDef *i2c_reg, const uint32_t mask) { return i2c_reg->SR1 & mask; }
static inline uint8_t get_status(const I2C_TypeDef *i2c_reg, const uint32_t mask) {

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

  // Tell i2c peripheral how fast frequency is
  cr2_word |= (freq_mhz << I2C_CR2_FREQ_Pos);

  // Slave address
  if (cfg->device_mode == I2C_DEVICE_MODE_SLAVE)
    addr->OAR1 = ((cfg->slave_setup.device_address & 0x7F) << I2C_OAR1_ADD1_Pos);

  ///// CCR register: /////
  uint16_t ccr_word = 0;

  // Whether i2c set in standard mode (100kHz), or fast mode (400kHz)
  if (cfg->scl_mode == I2C_SCL_MODE_SPEED_FM) ccr_word |= (1 << I2C_CCR_FS_Pos);

  // Duty cycle mode, only when in fast mode
  if (cfg->fm_duty_cycle == I2C_FM_DUTY_CYCLE_16_9) ccr_word |= (1 << I2C_CCR_DUTY_Pos);

  // Set CCR reg - formulas shows in function above
  ccr_word |= (ccr_ccr_val << I2C_CCR_CCR_Pos);

  ////// TRise reg //////
  uint16_t trise_ns = 0;
  if (cfg->scl_mode == I2C_SCL_MODE_SPEED_SM)
    trise_ns = 1000;
  else if (cfg->scl_mode == I2C_SCL_MODE_SPEED_FM)
    trise_ns = 300;

  uint16_t trise_word = freq_mhz * trise_ns / 1000 + 1;

  // Essentially, ceil the number if it isn't perfectly divided
  if (freq_mhz * trise_ns % 1000) trise_word++;

  // Initialize all regs - need to set CR1 last as it has the peripheral enable reg
  addr->TRISE = trise_word;
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
  __NOP();
  __NOP();
  *i2c_rstr_arr[index] &= ~(1 << i2c_rcc_pos[index]);

  return I2C_STATUS_OK;
}

static inline void i2c_start_blocking(I2C_TypeDef *i2c_reg, I2CEnable_t ack_en) {
  // Initiate transfer with start byte
  i2c_reg->CR1 |= (ack_en << I2C_CR1_ACK_Pos) | (1 << I2C_CR1_START_Pos);
  while (!(i2c_reg->SR1 & (1 << I2C_SR1_SB_Pos)));
}

static inline I2CStatus_t i2c_send_addr_blocking(I2C_TypeDef *i2c_reg, uint8_t slave_addr, const I2CWriteOrRead_t wr) {
  // Load the slave address into the I2C data register
  // NOTE: if reading, a 1 should be set to LSB
  uint8_t slave_addr_tx = (slave_addr << 1) | (wr & 1);
  i2c_reg->DR = slave_addr_tx;

  // Wait for ADDR bit in SR to be set, meaning address phase is done. Need to read SR1, and then SR2
  while (!get_status(i2c_reg, I2C_SR1_ADDR)) {
    if (get_status(i2c_reg, I2C_SR1_AF)) return I2C_STATUS_ACK_FAIL;
  }

  (void)i2c_reg->SR1;
  (void)i2c_reg->SR2;

  return I2C_STATUS_OK;
}

I2CStatus_t i2c_master_send(I2C_TypeDef *i2c_reg, void *tx_buffer, int32_t len, uint8_t slave_addr,
                            I2CStop_t stop_at_end) {
  int index = get_i2c_index(i2c_reg);
  if (index < 0) return I2C_STATUS_I2C_ADDR_INVALID;

  // S and EV5 according to diagram
  i2c_start_blocking(i2c_reg, I2C_DISABLE);

  // Addr, A, and EV6 according to diagram
  if (i2c_send_addr_blocking(i2c_reg, slave_addr, I2C_WRITE) == I2C_STATUS_ACK_FAIL) return I2C_STATUS_ACK_FAIL;

  // Next steps can be repeated until end of tx_buffer
  while (len) {
    // Send data - Data and EV8 event according to diagram
    while (!get_status(i2c_reg, I2C_SR1_TXE));
    i2c_reg->DR = *((uint8_t *)tx_buffer);

    // Move buffer position for next iteration
    tx_buffer = (uint8_t *)tx_buffer + 1;
    len--;
  }

  // EV8_2 event according to diagram
  // Wait for TxE==1 and BTF==1 (Byte frame)
  while (!get_status(i2c_reg, I2C_SR1_TXE) || !get_status(i2c_reg, I2C_SR1_BTF));

  if (stop_at_end == I2C_STOP) {
    // Stop transfer using stop byte
    i2c_reg->CR1 |= (1 << I2C_CR1_STOP_Pos);
  }

  return I2C_STATUS_OK;
}

I2CStatus_t i2c_master_receive(I2C_TypeDef *i2c_reg, void *rx_buffer, int32_t len, uint8_t slave_addr) {
  int index = get_i2c_index(i2c_reg);
  if (index < 0) return I2C_STATUS_I2C_ADDR_INVALID;

  // S and EV5 according to diagram
  i2c_start_blocking(i2c_reg, I2C_ENABLE);

  // Addr, A, and EV6 according to diagram
  if (i2c_send_addr_blocking(i2c_reg, slave_addr, I2C_WRITE) == I2C_STATUS_ACK_FAIL) return I2C_STATUS_ACK_FAIL;

  // Handle differently if single byte reception
  if (len == 1) {
    // Set ack low and set stop bit high
    i2c_reg->CR1 &= ~(1 << I2C_CR1_ACK_Pos);
    i2c_reg->CR1 |= (1 << I2C_CR1_STOP_Pos);

    // Clear flags
    (void)i2c_reg->SR1;
    (void)i2c_reg->SR2;

    // Data and EV7 Event from diagram here
    while (!get_status(i2c_reg, I2C_SR1_RXNE));
    *((uint8_t *)rx_buffer) = (uint8_t)i2c_reg->DR;

    return I2C_STATUS_OK;
  }

  // Next steps can be repeated until end of rx_buffer
  while (len) {
    // Wait for data
    while (!get_status(i2c_reg, I2C_SR1_RXNE));

    // EV7_1 event handled here:
    if (len == 2) {
      i2c_reg->CR1 &= ~(1 << I2C_CR1_ACK_Pos);
      i2c_reg->CR1 |= (1 << I2C_CR1_STOP_Pos);
    }

    // Data and EV7 Event from diagram here
    *((uint8_t *)rx_buffer) = (uint8_t)i2c_reg->DR;
    rx_buffer = (uint8_t *)rx_buffer + 1;
    len--;
  }

  return I2C_STATUS_OK;
}
