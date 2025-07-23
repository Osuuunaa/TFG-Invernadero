/*
 * wifi_thingspeak.h
 *
 *  Created on: Jul 22, 2025
 *      Author: carlo
 */

#ifndef INC_WIFI_THINGSPEAK_H_
#define INC_WIFI_THINGSPEAK_H_

#include "stm32f4xx_hal.h"
#include "UartRingbuffer.h"
#include <string.h>
#include <stdio.h>

void sendDataToThingSpeak(const char* apiKey, float averageTemperature, float humidity, float luminosity);
void processThingSpeakStateMachine(void);

extern uint8_t thingSpeakState; // Estado de la máquina de estados de Thingspeak

#endif /* INC_WIFI_THINGSPEAK_H_ */
