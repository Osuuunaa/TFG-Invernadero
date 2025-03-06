#include "veml7700.h"
#include <stdio.h>

// Function to initialize the VEML7700 sensor
void VEML7700_Init(void) {
    uint8_t config[2] = {0x00, 0x00}; // Example configuration
    HAL_I2C_Mem_Write(&hi2c1, VEML7700_I2C_ADDR, VEML7700_REG_CONF, I2C_MEMADD_SIZE_8BIT, config, 2, HAL_MAX_DELAY);
}

// Function to read luminosity from the VEML7700 sensor
float VEML7700_ReadLuminosity(void) {
    uint8_t buffer[2] = {0x00, 0x00};
    HAL_I2C_Mem_Read(&hi2c1, VEML7700_I2C_ADDR, VEML7700_REG_ALS, I2C_MEMADD_SIZE_8BIT, buffer, 2, HAL_MAX_DELAY);
    uint16_t raw_lux = (buffer[1] << 8) | buffer[0];
    float luminosity = raw_lux * 0.0576; // Convert raw value to lux (example conversion factor)
    return luminosity;
}
