//#include "lcd.h"
//#include "stm32f4xx_hal.h"
//#include <string.h>
//
//static void LCD_EnablePulse(void);
//static void LCD_Send(uint8_t value, uint8_t mode);
//
//void LCD_Init(void) {
//	HAL_Delay(40); // Esperar más de 40ms después de encender
//
//	LCD_SendCommand(0x30); // Function Set (8-bit, 2-line mode)
//	HAL_Delay(5); // Esperar más de 39 µs
//
//	LCD_SendCommand(0x30); // Repetir Function Set
//	HAL_Delay(1);
//
//	LCD_SendCommand(0x30); // Repetir Function Set
//	HAL_Delay(1);
//
//	LCD_SendCommand(0x20); // Cambiar a modo 4 bits
//	HAL_Delay(1);
//
//	// Configurar parámetros del display
//	LCD_SendCommand(0x28); // Function Set (4-bit, 2-line mode)
//	LCD_SendCommand(0x0C); // Display ON, Cursor OFF
//	LCD_SendCommand(0x01); // Clear Display
//	HAL_Delay(2); // Esperar 1.53ms mínimo
//	LCD_SendCommand(0x06); // Entry Mode Set
//
//}
//
//void LCD_SendCommand(uint8_t cmd) {
//    LCD_Send(cmd, 0);
//}
//
//void LCD_SendData(uint8_t data) {
//    LCD_Send(data, 1);
//}
//
//void LCD_Print(const char* str) {
//    while (*str) {
//        LCD_SendData((uint8_t)(*str));
//        str++;
//    }
//}
//
//void LCD_SetCursor(uint8_t row, uint8_t col) {
//    uint8_t address = (row == 0) ? col : (col + 0x40);
//    LCD_SendCommand(0x80 | address);
//}
//
//void LCD_Clear(void) {
//    LCD_SendCommand(0x01); // Clear display command
//    HAL_Delay(2);          // Delay to allow the command to process
//}
//
//static void LCD_EnablePulse(void) {
//    HAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, GPIO_PIN_SET);
//    HAL_Delay(1);
//    HAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, GPIO_PIN_RESET);
//    HAL_Delay(1);
//}
//
//static void LCD_Send(uint8_t value, uint8_t mode) {
//    HAL_GPIO_WritePin(LCD_RS_PORT, LCD_RS_PIN, (mode ? GPIO_PIN_SET : GPIO_PIN_RESET));
//
//    // Send high nibble
//    HAL_GPIO_WritePin(LCD_D4_PORT, LCD_D4_PIN, (value & 0x10) ? GPIO_PIN_SET : GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(LCD_D5_PORT, LCD_D5_PIN, (value & 0x20) ? GPIO_PIN_SET : GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(LCD_D6_PORT, LCD_D6_PIN, (value & 0x40) ? GPIO_PIN_SET : GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(LCD_D7_PORT, LCD_D7_PIN, (value & 0x80) ? GPIO_PIN_SET : GPIO_PIN_RESET);
//    LCD_EnablePulse();
//
//    // Send low nibble
//    HAL_GPIO_WritePin(LCD_D4_PORT, LCD_D4_PIN, (value & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(LCD_D5_PORT, LCD_D5_PIN, (value & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(LCD_D6_PORT, LCD_D6_PIN, (value & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(LCD_D7_PORT, LCD_D7_PIN, (value & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
//    LCD_EnablePulse();
//}
#include "lcd.h"
#include "stm32f4xx_hal.h"
#include <string.h>

static void LCD_EnablePulse(void);
static void LCD_Send(uint8_t value, uint8_t mode);

void LCD_Init(void) {
    HAL_Delay(20); // Wait for LCD to power up
    LCD_SendCommand(0x02); // Initialize in 4-bit mode
    LCD_SendCommand(0x28); // 2 lines, 5x7 matrix
    LCD_SendCommand(0x0C); // Display on, cursor off
    LCD_SendCommand(0x06); // Increment cursor
    LCD_SendCommand(0x01); // Clear display
    HAL_Delay(2);
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
    HAL_Delay(2);          // Delay to allow the command to process
}

static void LCD_EnablePulse(void) {
    HAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, GPIO_PIN_RESET);
    HAL_Delay(1);
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
