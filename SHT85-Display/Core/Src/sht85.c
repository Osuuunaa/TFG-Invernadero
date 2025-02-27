#include "sht85.h"

float temperature = 0.0f;
float humidity = 0.0f;
uint16_t temp_raw = 0, hum_raw = 0;
bool alarm = true; // Estado de la alarma (TRUE: apagada, FALSE: activa)
float temperature_max = 20.4;
// Variables para antirrebotes en control de alarma
uint32_t lastTimeAlarmUp = 0; // Tiempo desde que se incrementó el counterTempAlarmUp
uint8_t counterTempAlarmUp = 0; // Contador de mediciones consecutivas cuando la temperatura se sale del rango
uint32_t lastTimeAlarmDown = 0; // Tiempo desde que se incrementó el counterTempAlarmDown
uint8_t counterTempAlarmDown = 0; // Contador de mediciones consecutivas cuando la temperatura entra en rango de nuevo
uint8_t maxCounterTempAlarm = 5; // Máximo número de mediciones necesarias
// Variables para concurrencia entre lecturas del sensor SHT85 y control de alarma
uint32_t lastTimeSHT = 0; // Tiempo de la última lectura del sensor
uint32_t lastTimeIncreasingLight = 0; // Tiempo desde que se incrementó o decrementó el duty en el led rojo
uint16_t duty = 1000; // Ciclo de trabajo del led rojo
bool increasingLight = true; // Control de dirección del ciclo de trabajo. TRUE incrementa luz. FALSE decrementa luz
// Cálculo de temperaturas medias cada 1 segundo (5 muestras recogidas cada 200ms cada una)
float totalTemperature = 0.0f;
float averageTemperature = 0.0f;
uint8_t sampleCount = 0;

void ReadSHT85(float *temperature, float *humidity) {
    uint8_t cmd[2] = { (SHT85_CMD_MEASURE_HIGHREP >> 8) & 0xFF, SHT85_CMD_MEASURE_HIGHREP & 0xFF };
    HAL_I2C_Master_Transmit(&hi2c1, SHT85_I2C_ADDR, cmd, 2, HAL_MAX_DELAY);
    HAL_Delay(15); // Esperar a que la medición se complete

    uint8_t buffer[6];
    HAL_I2C_Master_Receive(&hi2c1, SHT85_I2C_ADDR, buffer, 6, HAL_MAX_DELAY);

    temp_raw = (buffer[0] << 8) | buffer[1];
    hum_raw = (buffer[3] << 8) | buffer[4];

    *temperature = -45.0f + 175.0f * ((float)temp_raw / 65535.0f);
    *humidity = 100.0f * ((float)hum_raw / 65535.0f);
}

bool setAlarm(float temp) {
    if (temp > temperature_max) {
        if (HAL_GetTick() - lastTimeAlarmUp > 200) {
            lastTimeAlarmUp = HAL_GetTick();
            counterTempAlarmUp++;
            if (counterTempAlarmUp >= maxCounterTempAlarm) {
                return false; // Activa la alarma
            }
        }
    } else {
        counterTempAlarmUp = 0;
        if (HAL_GetTick() - lastTimeAlarmDown > 200) {
            lastTimeAlarmDown = HAL_GetTick();
            counterTempAlarmDown++;
            if (counterTempAlarmDown >= maxCounterTempAlarm) {
                return true; // Apaga la alarma
            }
        }
    }
    return alarm;
}

void calculatorAverageTemperature(float newTemperature) {
    totalTemperature += newTemperature;
    sampleCount++;
    if (sampleCount >= 5) { // Calcular promedio cada 1 segundo (5 muestras)
        averageTemperature = totalTemperature / 5;
        totalTemperature = 0.0f;
        sampleCount = 0;
    }
}
