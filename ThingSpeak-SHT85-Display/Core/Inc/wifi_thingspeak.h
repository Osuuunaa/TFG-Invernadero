#ifndef __WIFI_THINGSPEAK_H
#define __WIFI_THINGSPEAK_H

#include "stm32f4xx_hal.h"

extern const char* WIFI_SSID;
extern const char* WIFI_PASS;


void sendATCommand(char *cmd);
void receiveResponse_IT(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void connectToWiFi(const char* ssid, const char* password);
uint8_t isWiFiConnected(void);
void sendDataToThingSpeak(const char* apiKey, float averageTemperature, float humidity, float luminosity);
void processThingSpeakStateMachine(void);

#endif // __WIFI_THINGSPEAK_H
