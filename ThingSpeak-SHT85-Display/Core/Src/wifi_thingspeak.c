#include "wifi_thingspeak.h"
#include "esp_8266.h"
#include <string.h>
#include <stdio.h>

extern UART_HandleTypeDef huart2;


static volatile uint8_t state = 0; // Estado de la máquina de estados
static const char* wifiApiKey;
static float wifiTemperature;
static float wifiHumidity;
static float wifiLuminosity;
uint32_t marblack1 = 0;
uint32_t marblack2 = 0;
uint32_t marblack3 = 0;
uint32_t marblack4 = 0;
uint32_t marblack5 = 0;

//static uint8_t conf = 0; // Bandera de recepción completa






void sendDataToThingSpeak(const char* apiKey, float averageTemperature, float humidity, float luminosity) {	// Almacena los datos a enviar a ThingSpeak y establece el estado para iniciar la máquina de estados
    wifiApiKey = apiKey;
    wifiTemperature = averageTemperature;
    wifiHumidity = humidity;
    wifiLuminosity = luminosity;
    state = 1; // Iniciar la máquina de estados
}




void processThingSpeakStateMachine() {	// Maneja el proceso de comunicación con ThingSpeak a través de comandos AT y gestiona las respuestas del ESP8266 en una máquina de estados
//    switch (state) {
//        case 1: {
//        	printf("Dentro de la maquina de estados \r\n");
//            char cmd[100];
//            sprintf(cmd, "AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",80\r\n");
//            esp8266_send_command(cmd);
//			esp8266_receive_response_IT();
//            state = 2;
//            memset(response, 0, sizeof(response));  // Limpiar el buffer
//            break;
//        }
//        case 2: {
//            if (esp8266_is_response_ready()) {
//            	esp8266_clear_response_flag();	//rxComplete = 0;
//                char http_request[250];
//                sprintf(http_request, "GET /update?api_key=%s&field1=%.2f&field2=%.2f&field3=%.2f HTTP/1.1\r\nHost: api.thingspeak.com\r\nConnection: close\r\n\r\n", wifiApiKey, wifiTemperature, wifiHumidity, wifiLuminosity);
//
//                char http_cmd[50];
//                sprintf(http_cmd, "AT+CIPSEND=%d\r\n", strlen(http_request));	// Envio longitud del mensaje
//                esp8266_send_command(http_cmd);
//				esp8266_receive_response_IT();
//                state = 3;
//                memset(response, 0, sizeof(response));  // Limpiar el buffer
//            }
//            break;
//        }
//        case 3: {				// Envio mensaje con los campos
//            if (esp8266_is_response_ready()) {
//            	esp8266_clear_response_flag();	//rxComplete = 0;
//                char http_request[250];
//                sprintf(http_request, "GET /update?api_key=%s&field1=%.2f&field2=%.2f&field3=%.2f HTTP/1.1\r\nHost: api.thingspeak.com\r\nConnection: close\r\n\r\n", wifiApiKey, wifiTemperature, wifiHumidity, wifiLuminosity);
//                esp8266_send_command(http_request);
//				esp8266_receive_response_IT();
//                state = 4;
//            }
//            break;
//        }
//        case 4: {
//			if (esp8266_is_response_ready()) {
//				esp8266_clear_response_flag();	//rxComplete = 0;
//				state = 5; // Comprobación de conexión WiFi
//			}
//			break;
//		}
//		case 5: {
//			if (isWiFiConnected()) {
//				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_14, GPIO_PIN_RESET);	// DEPURACIÓN
//
//				//esp8266_deep_sleep(20000); // Poner el ESP8266 en deep sleep durante 20 segundos (ajustable)
//				// Reiniar el módulo ESP8266 después de deep sleep para asegurar que esté disponible
//				//esp8266_reset_and_reconnect(WIFI_SSID, WIFI_PASS); // Agregar esta línea para reiniciar y reconectar
//			} else {
//
//				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_14, GPIO_PIN_SET);	// DEPURACIÓN
//
//				connectToWiFi(WIFI_SSID, WIFI_PASS); // Reintentar conexión
//			}
//			state = 0; // Finalizar la máquina de estados
//
//			break;
//		}
//    }
}

