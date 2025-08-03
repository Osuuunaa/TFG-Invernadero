/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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

#include "ESP_DATA_HANDLER.h"
#include "sht85.h"
#include "veml7700.h"
#include "wifi_thingspeak.h"
#include "mqtt_raspberry.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MEASUREMENT_INTERVAL 1000 // 0.2 segundos. Frecuencia para tomar mediciones con los sensores
#define THINGSPEAK_API_KEY "6K0OKMK195CFJ8SQ"
#define SEND_THINGSPEAK_INTERVAL 60000 // 30 segundos. Linea 184 de system_stm32f4xx.c modificada manualmente
#define SEND_RASPBERRY_INTERVAL 60000
#define ANTIBLOCK_STATE_MACHINE 10000 // 10 segundos puede estar como maximo en una maquina de estados. Sino -> reseteo del esp8266
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
// ----------------- SENSORS -----------------
float temperature = 0.0f;
float averageTemperature = 0.0f;
float humidity = 0.0f;
float averageHumidity = 0.0f;
float luminosity = 0.0f;
float averageLuminosity = 0.0f;

uint8_t countAverage = 0;
// ----------------- TIME MEASUREMENT -----------------
volatile uint32_t lastTimeMeasurement = 0;
volatile uint32_t lastThingSpeakSend = 0;
volatile uint32_t lastRpiSend = 0;

// ----------------- STATE MACHINE -----------------
typedef enum {
    TASK_IDLE,
    TASK_THINGSPEAK,
    TASK_RPI
} CommTask;

static CommTask activeTask = TASK_IDLE;

volatile bool alertaBloqueo = false;

// BORRAR SI NO FUNCIONA
volatile uint32_t nextThingSpeakSend = 0;  // o HAL_GetTick() + intervalo si no arranca en 0
volatile uint32_t nextRpiSend = 0;

volatile uint32_t currentTick = 0;

volatile uint32_t tsStart = 0;
volatile uint32_t rpiStart = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	lastTimeMeasurement = HAL_GetTick();
	lastThingSpeakSend = HAL_GetTick();
	lastRpiSend = HAL_GetTick();
	tsStart = 0;
	rpiStart = 0;
	nextThingSpeakSend = 30000;
	nextRpiSend = 60000;
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
  MX_USART6_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  // ----------------- ESP8266 WIFI -----------------
  ESP_Init("MOVISTAR_1DD2", "55253A2D16DDBF32D47B", "192.168.1.222");

  // ----------------- SENSORS -----------------
  SHT85_Init();
  HAL_Delay(500);
  VEML7700_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  currentTick = HAL_GetTick();

	  // ----------------- SENSORS -----------------
	  if (currentTick - lastTimeMeasurement >= MEASUREMENT_INTERVAL) {
//		  lastTimeMeasurement += MEASUREMENT_INTERVAL; // Incrementar en lugar de reiniciar
		  lastTimeMeasurement = currentTick;

		  SHT85_ReadSingleShot(&temperature, &humidity);
		  SHT85_ErrorReset(&temperature, &humidity);
		  averageTemperature += temperature;
		  averageHumidity += humidity;

		  ReadVEML7700(&luminosity);
		  averageLuminosity += luminosity;
		  countAverage++;
	  }


	  // ----------------- THINGSPEAK + MQTT RASPBERRY -----------------
//	  if (activeTask == TASK_IDLE && currentTick - lastThingSpeakSend >= SEND_THINGSPEAK_INTERVAL) {
//	      lastThingSpeakSend += SEND_THINGSPEAK_INTERVAL;

	  if (activeTask == TASK_IDLE && currentTick >= nextThingSpeakSend) {
	      nextThingSpeakSend += SEND_THINGSPEAK_INTERVAL;
	      lastThingSpeakSend = currentTick;	// Actualiza el tiempo del ultimo envio (opcional)

	      // Calcular promedios
	      if(countAverage == 0) {
			  averageTemperature = temperature;
			  averageHumidity = humidity;
			  averageLuminosity = luminosity;
		  } else {
			  averageTemperature = averageTemperature/countAverage;
			  averageHumidity = averageHumidity/countAverage;
			  averageLuminosity = averageLuminosity/countAverage;
		  }
	      sendDataToThingSpeak(THINGSPEAK_API_KEY, temperature, humidity, luminosity);
		  averageTemperature = 0.0f;
		  averageHumidity = 0.0f;
		  averageLuminosity = 0.0f;
		  countAverage = 0;

	      activeTask = TASK_THINGSPEAK;
	  }

	  // Ejecutar la máquina que esté activa. Evita bloques en las máquinas de estado
	  if (activeTask == TASK_THINGSPEAK) {
	      if (thingSpeakState != 0 && tsStart == 0)		// Empieza maquina estados
	    	  tsStart = HAL_GetTick();

	      processThingSpeakStateMachine();

	      if (thingSpeakState == 0) {
	          tsStart = 0;  // reiniciar si finaliza bien
	          if(currentTick >= nextRpiSend){
				  nextRpiSend += SEND_RASPBERRY_INTERVAL;
				  lastRpiSend = nextRpiSend - SEND_RASPBERRY_INTERVAL;	// Actualiza el tiempo del ultimo envio (opcional)
				  sendDataToRpi(temperature, humidity, luminosity);
				  activeTask = TASK_RPI;
	          }

	      } else if ((HAL_GetTick() - tsStart) > ANTIBLOCK_STATE_MACHINE) {
	          alertaBloqueo = true;
	      }

	  } else if (activeTask == TASK_RPI) {
	      if (rpiState != 0 && rpiStart == 0)		// Empieza maquina estados
	          rpiStart = HAL_GetTick();

	      processRpiStateMachine();

	      if (rpiState == 0) {
	          rpiStart = 0;
	          activeTask = TASK_IDLE;
	      } else if ((HAL_GetTick() - rpiStart) > ANTIBLOCK_STATE_MACHINE) {
	          alertaBloqueo = true;
	      }
	  }


	  if (alertaBloqueo) {
	      // 1. Resetear variables de estado
	      thingSpeakState = 0;
	      rpiState = 0;
	      activeTask = TASK_IDLE;

	      // 2. Limpiar buffers UART
	      Uart_flush();
	      Uart_clear();

	      ESP_Init("MOVISTAR_1DD2", "55253A2D16DDBF32D47B", "192.168.1.222");

	      // 5. Reset de flags y temporizadores
	      alertaBloqueo = false;
	      tsStart = 0;
	      rpiStart = 0;
	  }





    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  Server_Start();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
