#ifndef __VEML7700_H
#define __VEML7700_H

#include "stm32f4xx_hal.h"

#define VEML7700_I2C_ADDR 0x10 << 1 // I2C address for VEML7700
#define VEML7700_REG_CONF 0x00 // Configuration register
#define VEML7700_REG_ALS  0x04 // ALS register

extern I2C_HandleTypeDef hi2c1;

void VEML7700_Init(void);
float VEML7700_ReadLuminosity(void);

#endif // __VEML7700_H
