/*
 * mqtt_raspberry.h
 *
 *  Created on: Jul 23, 2025
 *      Author: carlo
 */

#ifndef INC_MQTT_RASPBERRY_H_
#define INC_MQTT_RASPBERRY_H_

#include "stm32f4xx_hal.h"
#include "UartRingbuffer.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#define MQTT_BROKER_IP     "192.168.1.220"
#define MQTT_BROKER_PORT   1883
#define MQTT_CLIENT_ID     "STM32"

#define MQTT_USERNAME      "osuna"
#define MQTT_PASSWORD      "tfg"
#define MQTT_TOPIC_JSON    "Invernadero/sensores"


void sendDataToRpi(float t, float h, float l);
void mqtt_send_connect_packet(void);
void mqtt_publish_unified_json(const char* topic);
bool mqtt_wait_connack(void);
void processRpiStateMachine(void);

extern uint8_t rpiState;

#endif /* INC_MQTT_RASPBERRY_H_ */
