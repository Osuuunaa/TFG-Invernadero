#ifndef __SHT85_H
#define __SHT85_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>

#define SHT85_I2C_ADDR 0x44 << 1 // Dirección I2C del SHT85
#define SHT85_CMD_MEASURE_HIGHREP 0x2400 // Comando para iniciar la medición

extern I2C_HandleTypeDef hi2c1; // Declaración del manejador I2C

// Declaraciones de variables globales
extern float temperature;
extern float humidity;
extern uint16_t temp_raw, hum_raw;
extern bool alarm;
extern float temperature_max;
extern uint32_t lastTimeAlarmUp;
extern uint8_t counterTempAlarmUp;
extern uint32_t lastTimeAlarmDown;
extern uint8_t counterTempAlarmDown;
extern uint8_t maxCounterTempAlarm;
extern uint32_t lastTimeSHT;
extern uint32_t lastTimeIncreasingLight;
extern uint16_t duty;
extern bool increasingLight;
extern float totalTemperature;
extern float averageTemperature;
extern uint8_t sampleCount;

void ReadSHT85(float *temperature, float *humidity);
bool setAlarm(float temp);
void calculatorAverageTemperature(float newTemperature);

#endif // __SHT85_H
