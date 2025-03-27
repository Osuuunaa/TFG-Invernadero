/*
 * mqtt_raspberry.h
 *
 *  Created on: Mar 22, 2025
 *      Author: carlo
 */

#ifndef INC_MQTT_RASPBERRY_H_
#define INC_MQTT_RASPBERRY_H_

#include "stm32f4xx_hal.h"

extern char esp8266_rx_buffer[256];
void mqtt_raspberry_set_enabled(uint8_t enable);
void mqtt_raspberry_send(float t, float h, float l);
void mqtt_raspberry_process(void); // Llamar de forma cíclica


#endif /* INC_MQTT_RASPBERRY_H_ */
