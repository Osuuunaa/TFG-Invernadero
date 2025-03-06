#include "wifi_thingspeak.h"
#include <string.h>
#include <stdio.h>

extern UART_HandleTypeDef huart2;

static char response[256];  // Buffer para almacenar la respuesta
static volatile uint8_t rxIndex = 0; // Índice de recepción
static volatile uint8_t rxComplete = 0; // Bandera de recepción completa

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

void connectToWiFi(const char* ssid, const char* password) {
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
    char connectCmd[100];
    sprintf(connectCmd, "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, password);
    sendATCommand(connectCmd);
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

void sendDataToThingSpeak(const char* apiKey, float temperature) {
    char cmd[100];
    sprintf(cmd, "AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",80\r\n");
    sendATCommand(cmd);
    HAL_Delay(2000);

    char http_request[150];
    sprintf(http_request, "GET /update?api_key=%s&field1=%.2f HTTP/1.1\r\nHost: api.thingspeak.com\r\nConnection: close\r\n\r\n", apiKey, temperature);

    char http_cmd[50];
    sprintf(http_cmd, "AT+CIPSEND=%d\r\n", strlen(http_request));
    sendATCommand(http_cmd);
    HAL_Delay(1000);
    sendATCommand(http_request);
    HAL_Delay(2000);
}
