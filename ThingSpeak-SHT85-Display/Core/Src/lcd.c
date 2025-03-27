#include "lcd.h"
#include "stm32f4xx_hal.h"
#include <string.h>

static void LCD_EnablePulse(void);
static void LCD_Send(uint8_t value, uint8_t mode);

static uint32_t lastTick = 0;

void LCD_Init(void) {
//    HAL_Delay(20); // Wait for LCD to power up
	// Esperar 20 ms sin bloquear
	lastTick = HAL_GetTick();
	while ((HAL_GetTick() - lastTick) < 20);  // Espera activa de 20 ms

	// Inicializar en modo de 4 bits
	LCD_Send(0x33, 0);  // Inicializa en modo de 8 bits
	LCD_Send(0x32, 0);  // Cambia a modo de 4 bits
	LCD_SendCommand(0x28);  // 2 líneas, matriz de 5x7
	LCD_SendCommand(0x0C);  // Mostrar encendido, cursor apagado
	LCD_SendCommand(0x06);  // Incrementar cursor
	LCD_SendCommand(0x01);  // Limpiar pantalla
    //HAL_Delay(2);
	lastTick = HAL_GetTick();
	while ((HAL_GetTick() - lastTick) < 2);  // Espera activa de 2 ms

}

void LCD_SendCommand(uint8_t cmd) {
    LCD_Send(cmd, 0);
}

void LCD_SendData(uint8_t data) {
    LCD_Send(data, 1);
}

void LCD_Print(const char* str) {
    while (*str) {
        LCD_SendData((uint8_t)(*str));
        str++;
    }
}

void LCD_SetCursor(uint8_t row, uint8_t col) {
    uint8_t address = (row == 0) ? col : (col + 0x40);
    LCD_SendCommand(0x80 | address);
}

void LCD_Clear(void) {
    LCD_SendCommand(0x01); // Clear display command
    //HAL_Delay(2);          // Delay to allow the command to process
    lastTick = HAL_GetTick();
	while ((HAL_GetTick() - lastTick) < 2);  // Espera activa de 2 ms
}

static void LCD_EnablePulse(void) {
    HAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, GPIO_PIN_SET);
    //HAL_Delay(1);
    lastTick = HAL_GetTick();
	while ((HAL_GetTick() - lastTick) < 1);  // Espera activa de 1 ms
    HAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, GPIO_PIN_RESET);
//    HAL_Delay(1);
    lastTick = HAL_GetTick();
    while ((HAL_GetTick() - lastTick) < 1);  // Espera activa de 1 ms
}

static void LCD_Send(uint8_t value, uint8_t mode) {
    HAL_GPIO_WritePin(LCD_RS_PORT, LCD_RS_PIN, (mode ? GPIO_PIN_SET : GPIO_PIN_RESET));

    // Send high nibble
    HAL_GPIO_WritePin(LCD_D4_PORT, LCD_D4_PIN, (value & 0x10) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_D5_PORT, LCD_D5_PIN, (value & 0x20) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_D6_PORT, LCD_D6_PIN, (value & 0x40) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_D7_PORT, LCD_D7_PIN, (value & 0x80) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    LCD_EnablePulse();

    // Send low nibble
    HAL_GPIO_WritePin(LCD_D4_PORT, LCD_D4_PIN, (value & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_D5_PORT, LCD_D5_PIN, (value & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_D6_PORT, LCD_D6_PIN, (value & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_D7_PORT, LCD_D7_PIN, (value & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    LCD_EnablePulse();
}
