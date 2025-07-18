/*
 * mqtt_raspberry.h
 *
 *  Created on: Mar 22, 2025
 *      Author: carlo
 */

#ifndef INC_MQTT_RASPBERRY_H_
#define INC_MQTT_RASPBERRY_H_

#include "stm32f4xx_hal.h"


#include "mqtt_raspberry.h"
#include "esp_8266.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

extern UART_HandleTypeDef huart2;


static bool mqtt_wait_connack(void);

void mqtt_raspberry_set_enabled(uint8_t enable) ;

void mqtt_raspberry_send(float t, float h, float l) ;

//static void mqtt_send_connect_packet(void);
//
//static void mqtt_publish_unified_json(const char* topic);
//
//// Espera y lee 4 bytes del CONNACK MQTT (binario, no texto)
//static uint8_t mqtt_wait_connack(void);

void mqtt_raspberry_process(void);

uint8_t mqtt_raspberry_is_busy(void);

#endif /* INC_MQTT_RASPBERRY_H_ */
