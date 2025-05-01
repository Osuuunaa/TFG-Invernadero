//#include "sht85.h"
//
//uint16_t temp_raw = 0, hum_raw = 0;
//// Cálculo de temperaturas medias
//float totalTemperature = 0.0f;
//uint8_t sampleCountTemp = 0;
//float totalHumidity = 0.0f;
//uint8_t sampleCountHum = 0;
//
//void SHT85_Init() {		// Inicio en modo periodico
//    uint8_t cmd[2] = {0x21, 0x30};  // 1 mps, high repeatability
//    HAL_I2C_Master_Transmit(&hi2c1, SHT85_I2C_ADDR, cmd, 2, HAL_MAX_DELAY);
//}
//
//void ReadSHT85_Periodic(float* temperature, float* humidity) {		// Leer datos periódicos
//    uint8_t fetch_command[2] = {0xE0, 0x00}; // Comando Fetch Data
//    uint8_t data[6]; // Espacio para almacenar datos (temperatura y humedad)
//
//    HAL_I2C_Master_Transmit(&hi2c1, SHT85_I2C_ADDR, fetch_command, 2, HAL_MAX_DELAY);	// Enviar comando Fetch Data
//    HAL_Delay(20); // Espera típica para conversión (alta repetibilidad)
//
//    HAL_I2C_Master_Receive(&hi2c1, SHT85_I2C_ADDR, data, 6, HAL_MAX_DELAY);		// Leer datos del sensor
//
//    uint16_t temp_raw = (data[0] << 8) | data[1]; // Procesar temperatura
//    uint16_t hum_raw = (data[3] << 8) | data[4]; // Procesar humedad
//
//    *temperature = -45 + 175 * (float)temp_raw / 65535.0;		// Convertir temperatura
//    *humidity = 100 * (float)hum_raw / 65535.0;		// Convertir humedad
//}
//
//void SHT85_ErrorReset(float* temperature, float* humidity) {		// Manejo de fallo del sensor
//    static uint8_t out_of_range_count = 0; // Contador de mediciones fuera de rango
//
//    if (*temperature <= -44.0f || *humidity <= 1) {		// Verificar fallo (-45°C o 0.37%HR)
//    	if (out_of_range_count > 5) { // Reiniciar si se alcanzan 5 fallos consecutivos
//            ResetSHT85();		// Reiniciar sensor con Soft Reset
//            SHT85_Init(); // Reactivar modo periódico después del reset
//            out_of_range_count = 0;		// Resetear contador tras el reinicio
//        } else{
//        	out_of_range_count++; // Incrementar contador
//        }
//    } else {
//        out_of_range_count = 0; // Resetear contador si la medición es válida
//    }
//}
//
//
//
//void ResetSHT85() {
//    uint8_t break_command[2] = {0x30, 0x93};  // Comando para detener modo periódico
//    uint8_t reset_command[2] = {0x30, 0xA2};  // Comando soft reset
//
//    // Parar modo periódico
//    HAL_I2C_Master_Transmit(&hi2c1, SHT85_I2C_ADDR, break_command, 2, HAL_MAX_DELAY);
//    HAL_Delay(2);  // Espera mínima antes del reset
//
//    // Soft reset
//    HAL_I2C_Master_Transmit(&hi2c1, SHT85_I2C_ADDR, reset_command, 2, HAL_MAX_DELAY);
//    HAL_Delay(50);  // Esperar después del reset
//
//}
//
//void calculatorAverageTemperature(float newTemperature, float *averageTemperature) {
//    static float lastValidAverage = 20.0f;  // Valor por defecto inicial
//
//    if (newTemperature > -40.0f) {
//        totalTemperature += newTemperature;
//        sampleCountTemp++;
//    }
//
//    if (sampleCountTemp > 0) {
//        *averageTemperature = totalTemperature / sampleCountTemp;
//        lastValidAverage = *averageTemperature;
//    } else {
//        *averageTemperature = lastValidAverage;  // Usar la última válida si no hay muestras
//    }
//
//    if (sampleCountTemp >= 40) {
//        totalTemperature = 0.0f;
//        sampleCountTemp = 0;
//    }
//}
//
//void calculatorAverageHumidity(float newHumidity, float *averageHumidity) {
//    static float lastValidAverage = 50.0f;  // Valor por defecto inicial
//
//    if (newHumidity > 1.0f) {
//        totalHumidity += newHumidity;
//        sampleCountHum++;
//    }
//
//    if (sampleCountHum > 0) {
//        *averageHumidity = totalHumidity / sampleCountHum;
//        lastValidAverage = *averageHumidity;
//    } else {
//        *averageHumidity = lastValidAverage;  // Usar última válida si no hay muestras
//    }
//
//    if (sampleCountHum >= 40) {
//        totalHumidity = 0.0f;
//        sampleCountHum = 0;
//    }
//}


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


