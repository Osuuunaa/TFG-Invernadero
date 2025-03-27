#ifndef __SHT85_H
#define __SHT85_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>

#define SHT85_I2C_ADDR 0x44 << 1 // Dirección I2C del SHT85
#define SHT85_CMD_MEASURE_HIGHREP 0x2400 // Comando para iniciar la medición

extern I2C_HandleTypeDef hi2c1; // Declaración del manejador I2C

extern uint16_t temp_raw, hum_raw;
extern float totalTemperature;
extern uint8_t sampleCount;

void SHT85_Init();
void ReadSHT85_Periodic(float* temperature, float* humidity);
void SHT85_ErrorReset(float* temperature, float* humidity);
void ResetSHT85();
void calculatorAverageTemperature(float newTemperature, float* averageTemperature);
void calculatorAverageHumidity(float newHumidity, float *averageHumidity);

#endif // __SHT85_H
