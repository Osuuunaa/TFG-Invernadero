/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Dirección I2C del SHT85 (ver datasheet para confirmar)
#define SHT85_I2C_ADDR 0x44 << 1 // Cambia si usas otra configuración

// Comando para iniciar la medición
#define SHT85_CMD_MEASURE_HIGHREP 0x2400

// DISPLAY INICIO
#define RS_PIN  GPIO_PIN_0
#define E_PIN   GPIO_PIN_1
#define D4_PIN  GPIO_PIN_2
#define D5_PIN  GPIO_PIN_3
#define D6_PIN  GPIO_PIN_4
#define D7_PIN  GPIO_PIN_5
#define LCD_PORT GPIOA
// DISPLAY FINAL

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

// Variables USUARIO INICIO. Para ajustar tiempos de muestreo de temperatura
uint16_t samplingPeriod = 20; // tiempo de muestreo de temperaturas (en ms)
uint16_t averagePeriod = 2; // tiempo deseado para calcular las temperaturas medias (en segundos)
bool alarmWithActualTemp = true; // TRUE: alarma se detecta mas rapido (con temp actual). FALSE: alarma se detecta mas lento (con temp media, se espera a leer varios datos)
// Variables USUARIO FIN

//Variables GLOBALES:
// SENSOR SHT85 TEMPERATURA INICIO
volatile float temperature = 0.0f;
float humidity = 0.0f;
volatile uint16_t temp_raw = 0, hum_raw = 0;
bool alarm = true; // Estado de la alarma (TRUE: apagada, FALSE: activa)
float temperature_max = 20.4;
// Variables para antirrebotes en control de alarma
uint32_t lastTimeAlarmUp = 0; // Tiempo desde que se incremento el counterTempAlarmUp
uint8_t counterTempAlarmUp = 0; // Contador de mediciones consecutivas cuando la temperatura se sale del rango
uint32_t lastTimeAlarmDown = 0; // Tiempo desde que se incremento el counterTempAlarmDown
uint8_t counterTempAlarmDown = 0; // Contador de mediciones consecutivas cuando la temperatura entra en rango de nuevo
uint8_t maxCounterTempAlarm = 5; // Máximo número de mediciones necesarias
//Variables para concurrencia entre lecturas sensor SHT85 y control de alarma
uint32_t lastTimeSHT = 0; // Tiempo de la última lectura del sensor
uint32_t lastTimeIncreasingLight = 0; // Tiempo desde que se incremento o decremento el duty en el led rojo
uint16_t duty = 1000; // Ciclo de trabajo del led rojo
bool increasingLight = true; // Control de dirección del ciclo de trabajo. TRUE incrementa luz. FALSE decrementa luz
// Calculo de temperaturas medias cada 1 segundo (5 muestras recogidas cada 200ms cada una)
volatile float totalTemperature = 0.0f;
float averageTemperature = 0.0f;
volatile uint8_t sampleCount = 0;
// SENSOR SHT85 TEMPERATURA FIN

// DISPLAY INICIO
// Variables para depuración en Live Expressions
uint8_t lastSentCommand = 0;  // Último comando enviado
uint8_t lastSentChar = 0;     // Último carácter enviado
uint8_t lastSent4Bits = 0;    // Últimos 4 bits enviados


// DISPLAY FINAL

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

// SENSOR SHT85 TEMPERATURA INICIO
void ReadSHT85(float *temperature, float *humidity);
bool setAlarm(float temp);
void calculatorAverageTemperature(float newTemperature);
// SENSOR SHT85 TEMPERATURA FIN


// DISPLAY INICIO
/*void LCD_PulseEnable(void) {
    HAL_GPIO_WritePin(LCD_PORT, E_PIN, GPIO_PIN_SET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(LCD_PORT, E_PIN, GPIO_PIN_RESET);
    HAL_Delay(1000);
}

void LCD_Send4Bits(uint8_t data) {
    lastSent4Bits = data; // Guardar valor para depuración
    HAL_GPIO_WritePin(LCD_PORT, D4_PIN, (data >> 0) & 0x01);
    HAL_GPIO_WritePin(LCD_PORT, D5_PIN, (data >> 1) & 0x01);
    HAL_GPIO_WritePin(LCD_PORT, D6_PIN, (data >> 2) & 0x01);
    HAL_GPIO_WritePin(LCD_PORT, D7_PIN, (data >> 3) & 0x01);
    LCD_PulseEnable();
}*/

void send_to_lcd(char data, int rs)
{
    HAL_GPIO_WritePin(LCD_PORT, RS_PIN, rs); // rs = 1 para datos, rs = 0 para comandos

    /* Escribir los datos en los pines respectivos */
    HAL_GPIO_WritePin(LCD_PORT, D4_PIN, (data >> 0) & 0x01);
	HAL_GPIO_WritePin(LCD_PORT, D5_PIN, (data >> 1) & 0x01);
	HAL_GPIO_WritePin(LCD_PORT, D6_PIN, (data >> 2) & 0x01);
	HAL_GPIO_WritePin(LCD_PORT, D7_PIN, (data >> 3) & 0x01);


    // Si el LCD no funciona, aumentar el retardo a 50, 80
	HAL_GPIO_WritePin(LCD_PORT, E_PIN, GPIO_PIN_SET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(LCD_PORT, E_PIN, GPIO_PIN_RESET);
	HAL_Delay(50);

	lastSent4Bits = data; // Guardar valor para depuración
}



/*
void LCD_SendCommand(uint8_t cmd) {
    lastSentCommand = cmd; // Guardar último comando para depuración
    HAL_GPIO_WritePin(LCD_PORT, RS_PIN, GPIO_PIN_RESET); // RS = 0 (Comando)
    LCD_Send4Bits(cmd >> 4); // Parte alta
    LCD_Send4Bits(cmd);      // Parte baja
    HAL_Delay(20); // Esperar mínimo 39µs según datasheet
}
*/

void lcd_send_cmd(char cmd)
{
    char datatosend;

    /* Enviar primero el nibble superior */
    datatosend = ((cmd >> 4) & 0x0F);
    send_to_lcd(datatosend, 0); // RS debe estar en 0 al enviar un comando

    /* Enviar el nibble inferior */
    datatosend = (cmd & 0x0F);
    send_to_lcd(datatosend, 0);
}


void lcd_send_data(char data)
{
    char datatosend;

    /* Enviar el nibble superior */
    datatosend = ((data >> 4) & 0x0F);
    send_to_lcd(datatosend, 1); // rs = 1 para enviar datos

    /* Enviar el nibble inferior */
    datatosend = (data & 0x0F);
    send_to_lcd(datatosend, 1);
}

void lcd_clear(void)
{
    lcd_send_cmd(0x01);
    HAL_Delay(2);
}

void lcd_put_cur(int row, int col)
{
    switch (row)
    {
        case 0:
            col |= 0x80;
            break;
        case 1:
            col |= 0xC0;
            break;
    }

    lcd_send_cmd(col);
}

void lcd_init(void)
{
    // Inicialización en modo de 4 bits
    HAL_Delay(50);  // Esperar >40ms
    lcd_send_cmd(0x30);

    HAL_Delay(5);   // Esperar >4.1ms
    lcd_send_cmd(0x30);

    HAL_Delay(1);   // Esperar >100us
    lcd_send_cmd(0x30);

    HAL_Delay(10);
    lcd_send_cmd(0x20);  // Modo 4 bits
    HAL_Delay(10);

    // Inicialización de la pantalla
    lcd_send_cmd(0x28);  // Configuración de funciones: DL=0 (modo 4 bits), N=1 (pantalla de 2 líneas), F=0 (5x8 caracteres)
    HAL_Delay(1);

    lcd_send_cmd(0x08);  // Control de encendido/apagado de la pantalla: D=0, C=0, B=0 --> pantalla apagada
    HAL_Delay(1);

    lcd_send_cmd(0x01);  // Limpiar pantalla
    HAL_Delay(2);

    lcd_send_cmd(0x06);  // Modo de entrada: I/D=1 (incrementa cursor), S=0 (sin desplazamiento)
    HAL_Delay(2);

    lcd_send_cmd(0x0C);  // Control de encendido/apagado: D=1 (pantalla encendida), C=0, B=0 (sin cursor ni parpadeo)
}

void lcd_send_string(char *str)
{
    while (*str)
        lcd_send_data(*str++);
}


/*

void LCD_SendChar(char c) {
    lastSentChar = c; // Guardar el carácter enviado para depuración

    uint8_t highNibble = (c >> 4) & 0x0F;  // Extraer parte alta (4 bits)
    uint8_t lowNibble = c & 0x0F;         // Extraer parte baja (4 bits)

    HAL_GPIO_WritePin(LCD_PORT, RS_PIN, GPIO_PIN_SET); // RS = 1 (Dato)

    lastSent4Bits = highNibble; // Guardar parte alta para depuración
    HAL_Delay(1000); // Mantener el valor visible más tiempo
    LCD_Send4Bits(highNibble); // Enviar parte alta al LCD
    HAL_Delay(1000); // Mantener la pausa

    lastSent4Bits = lowNibble; // Guardar parte baja para depuración
    HAL_Delay(1000); // Mantener el valor visible más tiempo
    LCD_Send4Bits(lowNibble); // Enviar parte baja al LCD
    HAL_Delay(1000); // Mantener la pausa
}



void LCD_SendString(char *str) {
    while (*str) {
        LCD_SendChar(*str++);
    }
}


void LCD_Init(void) {
    HAL_Delay(50); // 🔴 Esperar más de 40 ms tras encendido (datasheet)

    // 🔵 Secuencia para forzar modo 8 bits
    LCD_Send4Bits(0x03);
    HAL_Delay(50);
    LCD_Send4Bits(0x03);
    HAL_Delay(10);
    LCD_Send4Bits(0x03);
    HAL_Delay(10);

    // 🔵 Cambiar a modo 4 bits
    LCD_Send4Bits(0x02);
    HAL_Delay(10);

    // 🔵 Configuración inicial
    LCD_SendCommand(0x28); // Modo 4 bits, 2 líneas, 5x8 caracteres
    HAL_Delay(10);
    LCD_SendCommand(0x0C); // Display ON, Cursor OFF
    HAL_Delay(10);
    LCD_SendCommand(0x01); // Limpiar pantalla
	HAL_Delay(20); // 🔴 Esperar 1.53 ms según datasheet
    LCD_SendCommand(0x06); // Incremento de cursor
    HAL_Delay(10);

    LCD_SendCommand(0x0E); // Display ON, Cursor ON (verifica si algo aparece)
    HAL_Delay(10);
}
 */

// DISPLAY FINAL


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int row = 0;
int col = 0;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  // SENSOR SHT85 TEMPERATURA INICIO
//  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET); // Apagar ventilador
//  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); // Set Apaga el led verde del RGB
//
//  	// Inicia el PWM en el canal 2 (PA1 - LED Rojo)
//  	//HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
//  	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1000); // Configura el ciclo de trabajo a 1000 (Rojo apagado)

  	lastTimeSHT = HAL_GetTick();
  	lastTimeIncreasingLight = HAL_GetTick();
  	//tiempo_inicio_led_rojo = HAL_GetTick();

  	// SENSOR SHT85 TEMPERATURA FIN

  	// DISPLAY INICIO

    //LCD_Init();
    //LCD_SendString("Hola, STM32!");

    lcd_init();
    lcd_put_cur(0, 0);
    lcd_send_string("HELLO ");
    lcd_send_string("WORLD ");
    lcd_send_string("FROM");

    lcd_put_cur(1, 0);
    lcd_send_string("CONTROLLERS TECH");
    HAL_Delay(3000);
    lcd_clear();


  	// DISPLAY FINAL


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // SENSOR SHT85 TEMPERATURA INICIO

	  		// Ajuste del valor de temperatura para encender el ventilador

	  		  //HAL_ADC_Start(&hadc1);

	  		  // Esperar a que la conversión finalice
//	  		  if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
//	  			  // Leer el valor ADC convertido
//	  			  adcValue = HAL_ADC_GetValue(&hadc1);
//	  			  temperature_max = 15.0 + (adcValue/4095.0)*15.0; // Obtiene un valor entre 15 y 30 grados centigrados
//	  		  }
	  		  // Detener la conversión
//	  		  HAL_ADC_Stop(&hadc1);

	  		// Control de ventilador y leds

	  /*
	  		if (HAL_GetTick() - lastTimeSHT >= samplingPeriod) { // Leer los sensores cada 200 ms (periodo de muestreo)
	  			lastTimeSHT = HAL_GetTick(); // Se actualiza referencia
	  			ReadSHT85(&temperature, &humidity); // Leer sensores
	  			calculatorAverageTemperature(temperature); // Se calculan temperaturas medias
	  			float temperatureToCheck =
	  					alarmWithActualTemp ? temperature : averageTemperature;
	  			alarm = setAlarm(temperatureToCheck); // Verificar alarma. Si se quiere controlar la alarma con la temperatura media. Se activa más tarde
	  		}

	  		if (!alarm) { // La alarma se activa pq se supera temperatura maxima
	  					  // Incrementa gradualmente la intensidad del led rojo del RGB
	  			if (HAL_GetTick() - lastTimeIncreasingLight >= 20) {
	  				lastTimeIncreasingLight = HAL_GetTick();

	  				if (increasingLight) {
	  					duty -= 50; // Decrementar el ciclo de trabajo (lógica inversa)
	  					if (duty <= 0) {
	  						duty = 0;
	  						// Insertar bocina, aqui llega al pico maximo de luz
	  						increasingLight = false; // Cambiar dirección
	  					}
	  				} else {
	  					duty += 50; // Incrementar el ciclo de trabajo
	  					if (duty >= 1000) {
	  						duty = 1000;
	  						increasingLight = true; // Cambiar dirección
	  					}
	  				}
*/
	  				//__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, duty); // Ajusta el ciclo de trabajo del led rojo
	  			//}
	  		//	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET); // Encender led
//	  			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET); // Encender VENTILADOR
//	  			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); // Verde apagado

/*	  		} else {
	  			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET); // Apagar led
//	  			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET); // Apagar VENTILADOR
//	  			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0); // AMARILLO (ROJO + VERDE)
//	  			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET); // AMARILLO (ROJO + VERDE)
	  		}
	  		// SENSOR SHT85 TEMPERATURA FIN
*/

	  		// DISPLAY INICIO


	  for (int i = 0; i < 128; i++)
	  {
	      lcd_put_cur(row, col);
	      lcd_send_data(i + 48);

	      col++;

	      if (col > 15) { row++; col = 0; }
	      if (row > 1) row = 0;

	      HAL_Delay(250);
	  }


	  		// DISPLAY FINAL

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// SENSOR SHT85 TEMPERATURA INICIO
void ReadSHT85(float *temperature, float *humidity) {
	uint8_t cmd[2] = {
	SHT85_CMD_MEASURE_HIGHREP >> 8, SHT85_CMD_MEASURE_HIGHREP & 0xFF };
	uint8_t data[6];

	// Enviar el comando para iniciar la medición
	HAL_I2C_Master_Transmit(&hi2c1, SHT85_I2C_ADDR, cmd, 2, HAL_MAX_DELAY);

	// Esperar unos 10 ms para completar la medición
	HAL_Delay(20);

	// Leer los 6 bytes de datos
	HAL_I2C_Master_Receive(&hi2c1, SHT85_I2C_ADDR, data, 6, HAL_MAX_DELAY);

	// Convertir los datos recibidos
	temp_raw = (data[0] << 8) | data[1];
	hum_raw = (data[3] << 8) | data[4];

	// Calcular temperatura y humedad (según el datasheet del SHT85)
	*temperature = -45 + 175 * ((float) temp_raw / 65535);
	*humidity = 100 * ((float) hum_raw / 65535);
}

bool setAlarm(float temp) {		// TRUE apagado. FALSE alarma encendida
	static bool alarmState = true; // Estado de la alarma (true: apagada, false: activa)
	uint32_t alarmPeriod =
			alarmWithActualTemp ? samplingPeriod : (averagePeriod * 1000);

	if (temp > temperature_max) {
		counterTempAlarmDown = 0; // Si la temperatura se sale de rango, reiniciar el contador para cuando entra en el rango

		if (HAL_GetTick() - lastTimeAlarmUp >= alarmPeriod) { //si se quiere controlar la alarma con la temperatura acutal o media
			lastTimeAlarmUp = HAL_GetTick(); // Se actualiza el tiempo de referencia
			if (temp <= temperature_max) {
				counterTempAlarmUp = 0;
			} //Si entra dentro de rango: "falsa" medicion
			else {
				counterTempAlarmUp++;
			}	// Lectura correcta: fuera de rango

			if (counterTempAlarmUp >= maxCounterTempAlarm) { //Se da por buena la lectura: FUERA DE RANGO VERIFICADO
				counterTempAlarmUp = maxCounterTempAlarm;
				alarmState = false; // Alarma activada (fuera de rango confirmado)
			}
		}
	} else {
		counterTempAlarmUp = 0; // Si la temperatura vuelve al rango normal, reiniciar el contador para cuando se sale de rango
		// Debouncer
		if (HAL_GetTick() - lastTimeAlarmDown >= alarmPeriod) { //si se quiere controlar la alarma con la temperatura actual o media
			lastTimeAlarmDown = HAL_GetTick(); // Se actualiza el tiempo de referencia
			if (temp > temperature_max) {
				counterTempAlarmDown = 0;
			} //Si sale fuera de rango: "falsa" medicion
			else {
				counterTempAlarmDown++;
			}	// Lectura correcta: dentro de rango

			if (counterTempAlarmDown >= maxCounterTempAlarm) { //Se da por buena la lectura: DENTRO DE RANGO VERIFICADO
				counterTempAlarmDown = maxCounterTempAlarm; // Limitar el contador
				alarmState = true; // TRUE: alarma se apaga
			}
		}
	}
	return alarmState;
}

void calculatorAverageTemperature(float newTemperature) {
	totalTemperature += newTemperature;
	sampleCount++;

	if (sampleCount >= averagePeriod * 1000 / samplingPeriod) { // Se leen por ejemplo 25 muestras (durante 5 segundos. 200ms de tiempo de muestreo)
		averageTemperature = totalTemperature / (averagePeriod * 1000 / samplingPeriod);
		totalTemperature = 0.0f; // Reinicia acumulador
		sampleCount = 0;
	}
}

// SENSOR SHT85 TEMPERATURA FIN

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
