/*
 * esp_8266.c
 *
 *  Created on: Mar 22, 2025
 *      Author: carlo
 */

#include "esp_8266.h"
#include <string.h>
#include <stdio.h>

extern UART_HandleTypeDef huart2;

static char response[256];  // Buffer para almacenar la respuesta
static volatile uint8_t rxIndex = 0; // Índice de recepción
static volatile uint8_t rxComplete = 0; // Bandera de recepción completa

void esp8266_send_command(const char* cmd) {		// Envía un comando AT al ESP8266 a través de UART
    HAL_UART_Transmit(&huart2, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);
}

void esp8266_receive_response_IT(void) {	//Prepara la recepción de una respuesta del ESP8266 utilizando interrupciones
	rxIndex = 0;  // Reiniciar índice
	rxComplete = 0;  // Limpiar bandera de recepción completa
	memset(response, 0, sizeof(response));  // Limpiar el buffer
	HAL_UART_Receive_IT(&huart2, (uint8_t*)&response[rxIndex], 1);  // Recibir primer byte
}

uint8_t esp8266_is_response_ready(void) { 	//Verifica si la respuesta del ESP8266 está lista
    return rxComplete;		//En lugar de acceder directamente a rxComplete (que es static y privado en esp8266.c), accedes por función.
}

char* esp8266_get_response(void) {		// Devuelve el contenido de la respuesta recibida del ESP8266
    return response;
}

void esp8266_clear_response_flag(void) {		// Limpia la bandera que indica que la respuesta ha sido recibida
    rxComplete = 0;
}

bool esp8266_wait_response_timeout(uint32_t timeout_ms) {		// Espera una respuesta del ESP8266 durante un tiempo máximo dado
    uint32_t start = HAL_GetTick();			// Para no bloquear el resto del programa en reconexión de esp8266
    while (!esp8266_is_response_ready()) {
        if ((HAL_GetTick() - start) > timeout_ms) return false;
    }
    return true;
}

void esp8266_deep_sleep(uint32_t sleep_time_ms) {		// Pone el ESP8266 en modo deep sleep durante el tiempo especificado
    char command[50];
    sprintf(command, "AT+SLEEP=%lu\r\n", sleep_time_ms);  // sleep_time_ms en milisegundos
    esp8266_send_command(command);
    esp8266_receive_response_IT();
    esp8266_wait_response_timeout(5000);  // Espera una respuesta del ESP8266
}

void esp8266_reset_and_reconnect(const char* ssid, const char* password) {
	esp8266_send_command("AT+RST\r\n");
	esp8266_wait_response_timeout(2000);		// Espera
	esp8266_clear_response_flag();
	// Conectar a WiFi (sin configurar el modo station nuevamente)
	char connectCmd[100];
	sprintf(connectCmd, "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, password);
	esp8266_send_command(connectCmd);
	esp8266_wait_response_timeout(5000);			// Espera
	esp8266_clear_response_flag();
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {	//Callback que se activa cuando se recibe un byte de datos a través de UART
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
