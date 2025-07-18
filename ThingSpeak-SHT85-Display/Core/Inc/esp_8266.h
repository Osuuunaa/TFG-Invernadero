/*
 * esp_8266.h
 *
 *  Created on: Mar 22, 2025
 *      Author: carlo
 */

#ifndef INC_ESP_8266_H_
#define INC_ESP_8266_H_


#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#define ESP_RST_PIN GPIO_PIN_1
#define ESP_RST_GPIO_PORT GPIOA
#define ESP_RX_SIZE 512
#define ESP_TX_SIZE 256
#define ESP_RESET_INTERVAL 1800000 // 30mins en ms
#define ESP_JOIN_TIMEOUT_MS 10000  // 10 segundos
#define ESP_SEND_TIMEOUT_MS    2000		// Timeout para el SEND OK
#define ESP_CMD_TIMEOUT_MS     1000   // Timeout para el OK inicial
#define ESP_PROMPT_TIMEOUT_MS  1000   // Timeout para recibir '>'
#define ESP_CLOSE_TIMEOUT_MS   1000
#define ESP_RESET_PULSE_MS      100
#define ESP_OPEN_TIMEOUT_MS   10000  // // esperar en AT+CIPSTART (en milisegundos)

#define ESP_AT_TIMEOUT_MS    500
#define ESP_RST_TIMEOUT_MS   2000

#define ESP_CIPSEND_TIMEOUT_MS    2000	// Tiempo (ms) para que responda al CIPSEND con “>”

extern const char* WIFI_SSID;
extern const char* WIFI_PASS;



// - - -  Variables globales (definidas en esp_8266.c)
extern uint8_t  usartBuff_Rx;
extern uint8_t  RxBuffer[ESP_RX_SIZE];
extern uint8_t  TxBuffer[ESP_TX_SIZE];
extern uint16_t RxIndex;
extern bool     RxIsData;
extern uint8_t	comprob;

// ── Prototipos de la API del ESP8266 ────────────────────────────────────────────

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);


/**
 * @brief Inicializa UART2 y el módulo ESP8266.
 * @return true si la secuencia de inicio (AT, CWMODE…) fue exitosa.
 */
bool ESP_Init(void);

/**
 * @brief Limpia RxBuffer y resetea sus indicadores.
 */
void ESP_RxClear(void);

/**
 * @brief Limpia TxBuffer.
 */
void ESP_TxClear(void);

/**
 * @brief Conecta el ESP8266 a la red Wi-Fi.
 * @param ssid     Cadena con el SSID.
 * @param password Cadena con la contraseña.
 * @return true si obtuvo IP y “OK” al final de CWJAP.
 */
bool ESP_ConnectWiFi(const char *ssid, const char *password);


bool ESP_SendRaw(uint8_t *data,uint16_t len);
/**
 * @brief Envía un comando AT al ESP8266.
 * @param cmd Cadena terminada en '\0' (sin "\r\n").
 * @return true si se transmitió correctamente.
 */
bool ESP_SendCommand(char *cmd);

/**
 * @brief Espera una o varias respuestas del módulo.
 * @param timeout_ms Tiempo máximo (ms) a esperar.
 * @param result Puntero donde se devolverá el índice de la cadena coincidente.
 * @param paramCount Número de cadenas a comprobar.
 * @param ... Listado de punteros a cadenas ANSI que buscaremos en RxBuffer.
 * @return true si alguna de las cadenas apareció antes de expirar el timeout.
 */
bool ESP_WaitForResponse(uint32_t timeout_ms, uint8_t *result, uint8_t paramCount, ...);
bool ESP_WaitForChar(uint32_t timeout_ms, char target);
/**
 * @brief Indica si hay datos nuevos en RxBuffer desde la última limpieza.
 * @return true si hay datos listos para procesar.
 */
bool ESP_ResponseReady(void);



/**
 * @brief Abre una conexión TCP simple al servidor especificado.
 * @param host  Dirección (IP o nombre) a la que conectarse.
 * @param port  Puerto TCP.
 * @return true si respondió “OK” + “Linked” sin error.
 */
bool ESP_OpenConnection(const char *host, uint16_t port);

/**
 * @brief Envía datos por la conexión abierta.
 * @param data   Puntero a los bytes a enviar.
 * @param length Longitud en bytes.
 * @return true si transmitió y recibió “SEND OK”.
 */
bool ESP_SendData(const uint8_t *data, uint16_t length);

/**
 * @brief Cierra la conexión TCP abierta.
 * @return true si obtuvo “OK” tras CIPCLOSE.
 */
bool ESP_CloseConnection(uint8_t unusedLinkId);


void ESP_RESET(void);


const char *ESP_GetResponse(void);
void ESP_ClearResponseFlag(void);


#endif /* INC_ESP_8266_H_ */
