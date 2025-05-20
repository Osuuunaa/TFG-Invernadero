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

static volatile char response[512];  // Buffer para almacenar la respuesta
static volatile uint8_t rxIndex = 0; // Índice de recepción
static volatile uint8_t rxComplete = 0; // Bandera de recepción completa

void esp8266_send_command(const char* cmd);
void esp8266_receive_response_IT(void);
uint8_t esp8266_is_response_ready(void);
char* esp8266_get_response(void);
void esp8266_clear_response_flag(void);
bool esp8266_wait_response_timeout(uint32_t timeout_ms);
void esp8266_deep_sleep(uint32_t sleep_time_ms);
void esp8266_reset_and_reconnect(const char* ssid, const char* password);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);




#endif /* INC_ESP_8266_H_ */
