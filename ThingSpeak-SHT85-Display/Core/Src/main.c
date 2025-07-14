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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include "lcd.h"
#include "sht85.h"
#include "veml7700.h"
#include "rele.h"

#include "wifi_thingspeak.h"
#include "mqtt_raspberry.h"
#include "esp_8266.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define WIFI_SSID "MOVISTAR_1DD2"
//#define WIFI_PASS "55253A2D16DDBF32D47B"
const char* WIFI_SSID = "MOVISTAR_1DD2";
const char* WIFI_PASS = "55253A2D16DDBF32D47B";
#define THINGSPEAK_API_KEY "6K0OKMK195CFJ8SQ"

#define LUMINOSITY_THRESHOLD 1.5f
#define MEASUREMENT_INTERVAL 1000 // 0.2 segundos. Frecuencia para tomar mediciones con los sensores
#define DISPLAY_INTERVAL 3000 // 3 segundos. Frecuencia de muestreo de datos en el display
#define SEND_THINGSPEAK_INTERVAL 20000 // 20 segundos. Linea 184 de system_stm32f4xx.c modificada manualmente
#define SEND_MQTT_INTERVAL 8000 // 20 segundos. Linea 184 de system_stm32f4xx.c modificada manualmente

#define WIFI_RESET_INTERVAL 3600000 // 1h en ms

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
float temperature = 0.0f;
float averageTemperature = 0.0f;
float humidity = 0.0f;
float averageHumidity = 0.0f;
float luminosity = 0.0f;
float averageLuminosity = 0.0f;
uint8_t countAverage = 0;
volatile uint32_t lastTimeMeasurement = 0;
volatile uint32_t lastTimeDisplay = 0;
volatile uint32_t lastThingSpeakSend = 0;
volatile uint32_t lastMQTTSend = 0;
volatile uint32_t lastWiFiReset = 0;
volatile uint8_t displayState = 0; // 0: Temperature, 1: Humidity, 2: Luminosity
volatile bool releOn = false; // Variable global para monitorear el estado del relé

uint32_t marblack = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
static void MX_NVIC_Init(void);
const char* floatToStr(float num, int precision);
void LCD_Update_Variables(float* temperature, float* humidity, float* luminosity);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int __io_putchar(int ch) {
    ITM_SendChar(ch);
    return ch;
}



int _write(int file, char *ptr, int len){
	int i=0;
	for (i=0; i<len;i++){
		ITM_SendChar((*ptr));

	}
	return len;
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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  lastTimeMeasurement = HAL_GetTick();
  lastTimeDisplay = HAL_GetTick();
  lastThingSpeakSend = HAL_GetTick();
  lastMQTTSend = HAL_GetTick();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  // Habilitar el ITM y DWT (necesario para printf por SWO). NO ESTOY SEGURO SI HAY QUE PONERLO. LO DICE CHATGPT
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // Enable TRC
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;             // Enable DWT cycle counter


  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  MX_NVIC_Init();

  LCD_Init();
  //ResetSHT85();      // Añade esto antes de iniciar en modo periódico


  SHT85_Init();

  HAL_Delay(500);

  VEML7700_Init();
  connectToWiFi(WIFI_SSID, WIFI_PASS);


  if(isWiFiConnected() == 1){
	LCD_Clear();
	LCD_SetCursor(0, 0);
	LCD_Print("Wifi conectado");
	LCD_SetCursor(1, 0);
	LCD_Print("correctamente");
  }

  printf("Inicio del programa...\n");

  Rele_Off(); // Relé inicializado en OFF


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  uint32_t currentTick = HAL_GetTick();

	  // Chequea el WiFi periódicamente
//	  if (!isWiFiConnected()) {
//		  connectToWiFi(WIFI_SSID, WIFI_PASS);
//	  }















	  // Medición de sensores
	  if (currentTick - lastTimeMeasurement >= MEASUREMENT_INTERVAL) {
		  //lastTimeMeasurement = currentTick;
		  lastTimeMeasurement += MEASUREMENT_INTERVAL; // Incrementar en lugar de reiniciar
		  //ReadSHT85_Periodic(&temperature, &humidity);
		  SHT85_ReadSingleShot(&temperature, &humidity);

		  //calculatorAverageTemperature(temperature, &averageTemperature);
		  //calculatorAverageHumidity(humidity, &averageHumidity);
		  SHT85_ErrorReset(&temperature, &humidity);

		  ReadVEML7700(&luminosity);
		  averageTemperature += temperature;
		  averageHumidity += humidity;
		  averageLuminosity += luminosity;
		  countAverage++;
		  //calculateAverageLuminosity(luminosity, &averageLuminosity);

		  // Display LCD
		  LCD_Update_Variables(&temperature, &humidity, &luminosity);


		  // Controlar el rele basado en la luminosidad
		  Control_Rele(luminosity);
	  }




	  // Envío de datos a ThingSpeak
	  /*if (currentTick - lastThingSpeakSend >= SEND_THINGSPEAK_INTERVAL) {
		  //lastTimeSend = currentTick;

		  if(countAverage == 0) {
			  averageTemperature = temperature;
			  averageHumidity = humidity;
			  averageLuminosity = luminosity;
		  } else {
			  averageTemperature = averageTemperature/countAverage;
			  averageHumidity = averageHumidity/countAverage;
			  averageLuminosity = averageLuminosity/countAverage;
		  }

		  lastThingSpeakSend += SEND_THINGSPEAK_INTERVAL ; // es recomendable utilizar el método de acumulación para evitar deriva en el tiempo
		  //sendDataToThingSpeak(THINGSPEAK_API_KEY, averageTemperature, averageHumidity, averageLuminosity);
		  sendDataToThingSpeak(THINGSPEAK_API_KEY, temperature, humidity, luminosity);

		  averageTemperature = 0.0f;
		  averageHumidity = 0.0f;
		  averageLuminosity = 0.0f;
		  countAverage = 0;
		  // INICIO COMUNICACIÓN MQTT CON RASPBERRY
		  // Activar MQTT si no lo está
		  //mqtt_raspberry_set_enabled(1);

		  // Enviar los datos a la Pi en JSON
		  //mqtt_raspberry_send(temperature, humidity, luminosity);

		  // FIN COMUNICACIÓN MQTT CON RASPBERRY
	  }
*/
	  // Envío de datos MQTT cada X segundos
	  	  if (currentTick - lastMQTTSend >= SEND_MQTT_INTERVAL) {
	  		  marblack = 10;
	  		lastMQTTSend += SEND_MQTT_INTERVAL;

	  		  // Si tienes medias, cámbialo por tus promedios aquí
	  		  float t = temperature;
	  		  float h = humidity;
	  		  float l = luminosity;

	  		  // Activa la máquina de estados MQTT y envía los datos
	  		  mqtt_raspberry_set_enabled(1);
	  		  mqtt_raspberry_send(t, h, l);

	  		  // Espera a que termine el envío
	  		  while (mqtt_raspberry_is_busy() == 1) {
	  			  mqtt_raspberry_process();
	  			  HAL_Delay(10);
	  		  }
	  		marblack++;
	  	  }

	  	  // Procesa la máquina de estados MQTT (importante si usas envío asíncrono)
	  	  mqtt_raspberry_process();

	  	  HAL_Delay(10); // Pequeño delay para no saturar el MCU



	  // Reseteo de la conexión ESP8266-router cada 1 hora para asegurar que no se sature
	  if (currentTick - lastWiFiReset > WIFI_RESET_INTERVAL) {	//Forzando reinicio WiFi por mantenimiento
		  lastWiFiReset = HAL_GetTick();
		  esp8266_reset_and_reconnect(WIFI_SSID,WIFI_PASS);
	  }

	  processThingSpeakStateMachine(); // Procesar la máquina de estados de ThingSpeak

	  // INICIO COMUNICACIÓN MQTT CON RASPBERRY
	  //mqtt_raspberry_process();
	  // FIN COMUNICACIÓN MQTT CON RASPBERRY

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
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */


  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3
                           PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD14 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */



static void MX_NVIC_Init(void)
{
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn); // Habilitar interrupciones globales USART2 para ESP8266
}
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
#ifdef USE_FULL_ASSERT
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
