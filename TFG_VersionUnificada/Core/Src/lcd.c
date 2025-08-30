#include "lcd.h"
#include <string.h>
#include <stdio.h>

static void lcd_write4(uint8_t nibble);
static void lcd_pulseE(void);
static void lcd_us(uint32_t us);

static void lcd_us(uint32_t us){               // delay “suficiente” sin timers
  uint32_t n=SystemCoreClock/3000000U*us;      // ~0.33 ciclos por iteración aprox
  while(n--){__NOP();}
}

static void lcd_pulseE(void){
  HAL_GPIO_WritePin(LCD_E_PORT,LCD_E_PIN,GPIO_PIN_SET);
  lcd_us(2);                                   // t_EH >= 230 ns → 2 µs holgado
  HAL_GPIO_WritePin(LCD_E_PORT,LCD_E_PIN,GPIO_PIN_RESET);
  lcd_us(40);                                  // t_cyc >= 500 ns, y settle
}

static void lcd_write4(uint8_t n){             // pone D7..D4 = n[3..0]
  HAL_GPIO_WritePin(LCD_D4_PORT,LCD_D4_PIN, (n&0x01)?GPIO_PIN_SET:GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_D5_PORT,LCD_D5_PIN, (n&0x02)?GPIO_PIN_SET:GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_D6_PORT,LCD_D6_PIN, (n&0x04)?GPIO_PIN_SET:GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_D7_PORT,LCD_D7_PIN, (n&0x08)?GPIO_PIN_SET:GPIO_PIN_RESET);
  lcd_pulseE();
}

void LCD_Send(uint8_t v, uint8_t rs){
  HAL_GPIO_WritePin(LCD_RS_PORT,LCD_RS_PIN, rs?GPIO_PIN_SET:GPIO_PIN_RESET);
  lcd_write4(v>>4);                            // **SIEMPRE** nibble alto primero
  lcd_write4(v&0x0F);                          // luego nibble bajo
}

void LCD_SendCommand(uint8_t cmd){             // wrappers
  LCD_Send(cmd,0);
  if(cmd==0x01 || cmd==0x02) HAL_Delay(2);     // Clear/Home tardan >1.5 ms
}

void LCD_SendData(uint8_t data){
  LCD_Send(data,1);
}

void LCD_Clear(void){
  LCD_SendCommand(0x01);
}

void LCD_SetCursor(uint8_t row, uint8_t col){
  uint8_t addr = (row?0x40:0x00) + col;        // 16x2: 0x00 y 0x40
  LCD_SendCommand(0x80 | addr);
}

void LCD_Print(const char* s){
  while(*s) LCD_SendData((uint8_t)*s++);
}

void LCD_Init(void){
  HAL_Delay(20);                               // power-on >15 ms

  // Modo 4-bit “wake-up” según HD44780
  HAL_GPIO_WritePin(LCD_RS_PORT,LCD_RS_PIN,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN,  GPIO_PIN_RESET);

  lcd_write4(0x03); HAL_Delay(5);              // 0x30 x3 en 8-bit por nibbles
  lcd_write4(0x03); lcd_us(150);
  lcd_write4(0x03); lcd_us(150);
  lcd_write4(0x02); lcd_us(150);               // ahora 4-bit

  // Function set: 4-bit, 2 líneas, 5x8
  LCD_SendCommand(0x28);
  // Display ON, cursor OFF, blink OFF
  LCD_SendCommand(0x0C);
  // Entry mode: incremento, sin shift
  LCD_SendCommand(0x06);
  // Clear
  LCD_Clear();
}

//const char* floatToStr(float num, int prec){
//  static char buf[16];
//  if(prec<0) prec=0; if(prec>3) prec=3;
//  snprintf(buf,sizeof(buf),"%.*f",prec,num);
//  return buf;
//}
const char* floatToStr(float num, int precision) {
    static char str[20]; // Buffer estático para almacenar la cadena resultante
    sprintf(str, "%.*f", precision, num);
    return str;
}

void LCD_Update_Variables(float* temperature, float* humidity, float* luminosity){
//  const char* st = floatToStr(*t,1);
//  const char* sh = floatToStr(*h,0);
//  const char* sl = floatToStr(*l,0);
//  LCD_Clear();
//  LCD_SetCursor(0,0); LCD_Print("T:"); LCD_Print(st); LCD_Print("C ");
//  LCD_Print("| H:"); LCD_Print(sh); LCD_Print("%");
//  LCD_SetCursor(1,0); LCD_Print("L: "); LCD_Print(sl); LCD_Print(" lux");
//


  const char* strTemp = floatToStr(*temperature, 2);
  LCD_Clear();
  LCD_SetCursor(0, 0);
  LCD_Print("T:");
  LCD_Print(strTemp);
  //LCD_SendData(223);  // Enviar código ASCII del símbolo de grados para HD44780
  LCD_Print("C | H:");
  const char* strHum = floatToStr(*humidity, 0);
  LCD_Print(strHum);
  LCD_Print("%");
  const char* strLux = floatToStr(*luminosity, 3);
  LCD_SetCursor(1, 0);
  LCD_Print("  L: ");
	LCD_Print(strLux);
	LCD_Print(" lux");
}


/*
#include "lcd.h"
#include "stm32f4xx_hal.h"
#include <string.h>

static void LCD_EnablePulse(void);
static void LCD_Send(uint8_t value, uint8_t mode);

static uint32_t lastTick = 0;


void LCD_Init(void) {
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
//    uint8_t address = (row == 0) ? col : (col + 0x40);
	uint8_t address = (row?0x40:0x00) + col;        // 16x2: 0x00 y 0x40
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

const char* floatToStr(float num, int precision) {
    static char str[20]; // Buffer estático para almacenar la cadena resultante
    sprintf(str, "%.*f", precision, num);
    return str;
}

void LCD_Update_Variables(float* temperature, float* humidity, float* luminosity) {
    const char* strTemp = floatToStr(*temperature, 2);
    LCD_Clear();
    LCD_SetCursor(0, 0);
    LCD_Print("T:");
    LCD_Print(strTemp);
    //LCD_SendData(223);  // Enviar código ASCII del símbolo de grados para HD44780
    LCD_Print("C | H:");
    const char* strHum = floatToStr(*humidity, 0);
    LCD_Print(strHum);
    LCD_Print("%");
    const char* strLux = floatToStr(*luminosity, 3);
    LCD_SetCursor(1, 0);
    LCD_Print("  L: ");
	LCD_Print(strLux);
	LCD_Print(" lux");
}
*/
