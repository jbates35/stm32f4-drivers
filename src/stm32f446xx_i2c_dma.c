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
  uint8_t *buff;
  uint8_t *buff_start;
  int32_t len;
  int32_t eles_left;
  I2CEnable_t en;
} I2CInterruptBuffer_t;

typedef struct {
  I2CInterruptBuffer_t tx;
  I2CInterruptBuffer_t rx;
  I2CInterruptStatus_t status;
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
