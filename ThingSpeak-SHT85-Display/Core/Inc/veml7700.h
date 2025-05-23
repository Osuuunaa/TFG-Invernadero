#ifndef __VEML7700_H
#define __VEML7700_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>

extern I2C_HandleTypeDef hi2c2; // Usa I2C2 como en el SHT85

extern uint16_t lux_raw;
extern float totalLuminosity;
extern uint8_t sampleCountLight;

void VEML7700_Init(void);
void ReadVEML7700(float *lux);
void calculateAverageLuminosity(float newLux, float* averageLuminosity);

#endif // __VEML7700_H
