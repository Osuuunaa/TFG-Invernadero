#ifndef __LCD_H
#define __LCD_H

#include "stm32f4xx_hal.h"

// Define the pins used for the LCD
#define LCD_RS_PIN GPIO_PIN_0
#define LCD_RS_PORT GPIOC
#define LCD_E_PIN GPIO_PIN_1
#define LCD_E_PORT GPIOC
#define LCD_D4_PIN GPIO_PIN_2
#define LCD_D4_PORT GPIOC
#define LCD_D5_PIN GPIO_PIN_3
#define LCD_D5_PORT GPIOC
#define LCD_D6_PIN GPIO_PIN_4
#define LCD_D6_PORT GPIOC
#define LCD_D7_PIN GPIO_PIN_5
#define LCD_D7_PORT GPIOC

void LCD_Init(void);
void LCD_SendCommand(uint8_t cmd);
void LCD_SendData(uint8_t data);
void LCD_Print(const char* str);
void LCD_SetCursor(uint8_t row, uint8_t col);
void LCD_Clear(void);

#endif // __LCD_H
