/*
 * sht85.c
 *
 *  Created on: Jul 22, 2025
 *      Author: carlo
 */

#include "sht85.h"

uint16_t temp_raw = 0, hum_raw = 0;
// Cálculo de temperaturas medias
float totalTemperature = 0.0f;
uint8_t sampleCountTemp = 0;
float totalHumidity = 0.0f;
uint8_t sampleCountHum = 0;

void SHT85_Init() {
    // No se requiere inicialización en modo single shot, pero puede hacerse un soft reset por seguridad
    ResetSHT85();
}

uint8_t SHT85_CalculateCRC(uint8_t* data, uint8_t len) {
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x31;
            else
                crc <<= 1;
        }
    }
    return crc;
}

void SHT85_ReadSingleShot(float* temperature, float* humidity) {
    uint8_t cmd[2] = {0x24, 0x00};  // High repeatability, no clock stretching
    uint8_t data[6];

    HAL_I2C_Master_Transmit(&hi2c1, SHT85_I2C_ADDR, cmd, 2, HAL_MAX_DELAY);
    HAL_Delay(20);  // Tiempo típico de conversión (~15 ms para alta repetibilidad)

    HAL_I2C_Master_Receive(&hi2c1, SHT85_I2C_ADDR, data, 6, HAL_MAX_DELAY);

    // Validar CRC
    if (SHT85_CalculateCRC(&data[0], 2) != data[2]) return;
    if (SHT85_CalculateCRC(&data[3], 2) != data[5]) return;

    temp_raw = (data[0] << 8) | data[1];
    hum_raw  = (data[3] << 8) | data[4];

    *temperature = -45 + 175 * (float)temp_raw / 65535.0f;
    *humidity    = 100 * (float)hum_raw / 65535.0f;
}

void SHT85_ErrorReset(float* temperature, float* humidity) {
    static uint8_t out_of_range_count = 0;

    if (*temperature <= -44.0f || *humidity <= 1) {
        if (out_of_range_count > 5) {
            ResetSHT85();
            out_of_range_count = 0;
        } else {
            out_of_range_count++;
        }
    } else {
        out_of_range_count = 0;
    }
}

void ResetSHT85() {
    uint8_t reset_command[2] = {0x30, 0xA2};
    HAL_I2C_Master_Transmit(&hi2c1, SHT85_I2C_ADDR, reset_command, 2, HAL_MAX_DELAY);
    HAL_Delay(50);
}

void calculatorAverageTemperature(float newTemperature, float *averageTemperature) {
    static float lastValidAverage = 20.0f;

    if (newTemperature > -40.0f) {
        totalTemperature += newTemperature;
        sampleCountTemp++;
    }

    if (sampleCountTemp > 0) {
        *averageTemperature = totalTemperature / sampleCountTemp;
        lastValidAverage = *averageTemperature;
    } else {
        *averageTemperature = lastValidAverage;
    }

    if (sampleCountTemp >= 40) {
        totalTemperature = 0.0f;
        sampleCountTemp = 0;
    }
}

void calculatorAverageHumidity(float newHumidity, float *averageHumidity) {
    static float lastValidAverage = 50.0f;

    if (newHumidity > 1.0f) {
        totalHumidity += newHumidity;
        sampleCountHum++;
    }

    if (sampleCountHum > 0) {
        *averageHumidity = totalHumidity / sampleCountHum;
        lastValidAverage = *averageHumidity;
    } else {
        *averageHumidity = lastValidAverage;
    }

    if (sampleCountHum >= 40) {
        totalHumidity = 0.0f;
        sampleCountHum = 0;
    }
}


