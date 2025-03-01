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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>  // Necesario para usar bool en C

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char response[256];  // Buffer para almacenar la respuesta (aumentado a 256 bytes)
volatile uint8_t rxIndex = 0; // Índice de recepción
volatile uint8_t rxComplete = 0; // Bandera de recepción completa
uint8_t flagConexion = 0; // Bandera de recepción completa


char http_request[150]; // BOOORRAR


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_NVIC_Init(void); // Agregar prototipo para NVIC init
/* USER CODE BEGIN PFP */

void sendATCommand(char *cmd) {
    HAL_UART_Transmit(&huart2, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);
}

void receiveResponse_IT() {
    rxIndex = 0;  // Reiniciar índice
    rxComplete = 0;  // Limpiar bandera de recepción completa
    memset(response, 0, sizeof(response));  // Limpiar el buffer
    HAL_UART_Receive_IT(&huart2, (uint8_t*)&response[rxIndex], 1);  // Recibir primer byte
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {  // Verificar si es el UART correcto
        if (rxIndex < sizeof(response) - 1) {  // Evitar desbordamiento del buffer
            rxIndex++;  // Avanzar índice
            if (response[rxIndex - 1] == '\n') {  // Detecta el fin de la respuesta
                rxComplete = 1;  // Marcar recepción completa
            } else {
                HAL_UART_Receive_IT(&huart2, (uint8_t*)&response[rxIndex], 1);  // Recibir siguiente byte
            }
        } else {
            rxComplete = 1;  // Si se llena el buffer, finalizar recepción
        }
    }
}

void connectToWiFi() {
    HAL_Delay(3000);  // Esperar 3s después de encender el ESP8266

    // Enviar comando AT y esperar respuesta con interrupciones
    sendATCommand("AT\r\n");
    receiveResponse_IT();
    while (!rxComplete);
    printf("Respuesta AT: %s\n", response);
    rxComplete = 0;

    // Reiniciar el ESP8266
    sendATCommand("AT+RST\r\n");
    receiveResponse_IT();
    HAL_Delay(5000);
    while (!rxComplete);
    printf("Respuesta RST: %s\n", response);
    rxComplete = 0;

    // Configurar modo Station
    sendATCommand("AT+CWMODE=1\r\n");
    receiveResponse_IT();
    HAL_Delay(1000);
    while (!rxComplete);
    printf("Respuesta CWMODE: %s\n", response);
    rxComplete = 0;

    // Conectar a WiFi
    sendATCommand("AT+CWJAP=\"MOVISTAR_1DD2\",\"55253A2D16DDBF32D47B\"\r\n");
    receiveResponse_IT();
    HAL_Delay(15000);
    while (!rxComplete);
    printf("Respuesta CWJAP: %s\n", response);
    rxComplete = 0;

    // Verificar si la conexión fue exitosa
    if (strstr(response, "WIFI CONNECTED") != NULL) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);  // Encender LED
    } else {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);  // Apagar LED
    }
}

void sendDataToThingSpeak(float temperature) {



    // 1. Iniciar conexión TCP con ThingSpeak
    sendATCommand("AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",80\r\n");
    receiveResponse_IT();
    HAL_Delay(3000);  // Aumentar tiempo de espera
    while (!rxComplete);  // Esperar a recibir la respuesta
    printf("Respuesta CIPSTART: %s\n", response);
    rxComplete = 0;  // Resetear bandera


    HAL_Delay(16000);
    bzero(response, sizeof(response));
    flagConexion = 2;


    // 2. Crear la solicitud HTTP con el valor de temperatura en `field1`
//    char http_request[150];
   // sprintf(http_request, "GET https://api.thingspeak.com/update?api_key=NHNNS6OWGDSBEJF1&field1=%.2f\r\n", temperature); // Sustituye con tu API Key
    sprintf(http_request,
        "GET /update?api_key=NHNNS6OWGDSBEJF1&field1=%.2f HTTP/1.1\r\n"
        "Host: api.thingspeak.com\r\n"
        "Connection: close\r\n\r\n",
        temperature);


    HAL_Delay(16000);
    bzero(response, sizeof(response));
    flagConexion = 3;

    // 3. Indicar la cantidad de bytes a enviar
    char cmd[30];
    sprintf(cmd, "AT+CIPSEND=%d\r\n", strlen(http_request));
    sendATCommand(cmd);
    receiveResponse_IT();
    HAL_Delay(1000);
    while (!rxComplete);
    printf("Respuesta CIPSEND: %s\n", response);
    rxComplete = 0;


    HAL_Delay(16000);
    bzero(response, sizeof(response));
    flagConexion = 4;

//    // Verificar si está listo para enviar datos
//    if (strstr(response, ">") == NULL) {
//        printf("Error al preparar el envío de datos\n");
//        return;
//    }

    // 4. Enviar la petición HTTP
    sendATCommand(http_request);
    receiveResponse_IT();
    HAL_Delay(2000);
    while (!rxComplete);
    printf("Respuesta HTTP: %s\n", response);
    rxComplete = 0;

    HAL_Delay(2000);
    bzero(response, sizeof(response));
    flagConexion = 5;

    // 5. Cerrar la conexión TCP
    sendATCommand("AT+CIPCLOSE\r\n");
    receiveResponse_IT();
    HAL_Delay(1000);
    while (!rxComplete);
    printf("Respuesta CIPCLOSE: %s\n", response);
    rxComplete = 0;
}

int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

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
  float temperature = 23.45; // Valor de temperatura de ejemplo
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
  MX_USART2_UART_Init();
  MX_NVIC_Init(); // Inicializar NVIC
  /* USER CODE BEGIN 2 */
  flagConexion = 0;
  HAL_Delay(3000);  // Esperar 3s después de encender el ESP8266



  //bzero(response, sizeof(response)); limpia buffer

//  for (int i = 0; i < sizeof(response); i++) {
//      response[i] = 0;
//  }

  connectToWiFi();
  flagConexion = 1;




  bzero(response, sizeof(response));

  HAL_Delay(5000);
  sendDataToThingSpeak(temperature);
  flagConexion = 6;



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_Delay(5000);
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
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV8;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart2.Init.BaudRate = 115200 ;
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

static void MX_NVIC_Init(void)
{
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn); // Habilitar interrupciones globales USART2
}


static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PD12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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

#ifdef __GNUC__
    #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
    #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
