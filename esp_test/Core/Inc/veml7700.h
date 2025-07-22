/*
 * veml7700.h
 *
 *  Created on: Jul 22, 2025
 *      Author: carlo
 */

#ifndef INC_VEML7700_H_
#define INC_VEML7700_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>

extern I2C_HandleTypeDef hi2c2; // Usa I2C2

extern uint16_t lux_raw;
extern float totalLuminosity;
extern uint8_t sampleCountLight;

void VEML7700_Init(void);
void ReadVEML7700(float *lux);
void calculateAverageLuminosity(float newLux, float* averageLuminosity);


#endif /* INC_VEML7700_H_ */
