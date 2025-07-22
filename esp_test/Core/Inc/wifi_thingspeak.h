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



void sendATCommand(char *cmd);
void receiveResponse_IT(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void connectToWiFi(const char* ssid, const char* password);

void sendDataToThingSpeak(const char* apiKey, float averageTemperature, float humidity, float luminosity);
void processThingSpeakStateMachine(void);

#endif /* INC_WIFI_THINGSPEAK_H_ */
