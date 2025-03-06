#ifndef __WIFI_THINGSPEAK_H
#define __WIFI_THINGSPEAK_H

#include "stm32f4xx_hal.h"

void sendATCommand(char *cmd);
void receiveResponse_IT(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void connectToWiFi(const char* ssid, const char* password);
void sendDataToThingSpeak(const char* apiKey, float temperature);

#endif // __WIFI_THINGSPEAK_H
