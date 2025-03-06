#include "veml7700.h"

extern I2C_HandleTypeDef hi2c2; // Usamos I2C2

#define VEML7700_I2C_ADDR (0x10 << 1) // Dirección ajustada para STM32
#define VEML7700_REG_CONF 0x00        // Registro de configuración
#define VEML7700_REG_ALS  0x04        // Registro de lectura de luz

// Variables globales
uint16_t lux_raw = 0;
//float luminosity = 0.0;
float totalLuminosity = 0.0f;
float averageLuminosity = 0.0f;
uint8_t sampleCountLight = 0;
float luminosity_max = 500.0f;  // Umbral para alarma

// Configuración del sensor
void VEML7700_Init(void) {
    uint8_t config[2] = { 0x00, 0x00 };  // GAIN x1, IT 100ms, sensor encendido
    HAL_I2C_Mem_Write(&hi2c2, VEML7700_I2C_ADDR, VEML7700_REG_CONF, I2C_MEMADD_SIZE_8BIT, config, 2, HAL_MAX_DELAY);
    HAL_Delay(5);
}

// Lectura de luminosidad
void ReadVEML7700(float *lux) {
    uint8_t buffer[2] = { 0x00, 0x00 };
    HAL_I2C_Mem_Read(&hi2c2, VEML7700_I2C_ADDR, VEML7700_REG_ALS, I2C_MEMADD_SIZE_8BIT, buffer, 2, HAL_MAX_DELAY);

    lux_raw = (buffer[1] << 8) | buffer[0]; // Convertir a 16 bits
    *lux = lux_raw * 0.0036; // Factor de conversión del datasheet
}

// Alarma basada en luminosidad
bool setLightAlarm(float lux) {
    return (lux <= luminosity_max);
}

// Calculo del promedio de luminosidad
void calculateAverageLuminosity(float newLux) {
    totalLuminosity += newLux;
    sampleCountLight++;
    if (sampleCountLight >= 5) { // Calcular promedio cada 1 segundo (5 muestras)
        averageLuminosity = totalLuminosity / 5;
        totalLuminosity = 0.0f;
        sampleCountLight = 0;
    }
}
