/*
 * wifi_thingspeak.c
 *
 *  Created on: Jul 22, 2025
 *      Author: carlo
 */


#include "wifi_thingspeak.h"

uint8_t thingSpeakState = 0;
static const char* wifiApiKey;
static float wifiTemperature;
static float wifiHumidity;
static float wifiLuminosity;


void sendDataToThingSpeak(const char* apiKey, float averageTemperature, float humidity, float luminosity) {	// Almacena los datos a enviar a ThingSpeak y establece el estado para iniciar la máquina de estados
    wifiApiKey = apiKey;
    wifiTemperature = averageTemperature;
    wifiHumidity = humidity;
    wifiLuminosity = luminosity;
    thingSpeakState = 1; // Iniciar la máquina de estados
}

void processThingSpeakStateMachine() {	// Maneja el proceso de comunicación con ThingSpeak a través de comandos AT y gestiona las respuestas del ESP8266 en una máquina de estados
    switch (thingSpeakState) {
        case 1: {
        	Uart_flush();
        	Uart_sendstring("AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",80\r\n");
        	while (!(Wait_for("OK\r\n")));
        	thingSpeakState = 2;
            break;
        }
        case 2: {
        	Uart_flush();

            // Generar la petición primero
            char http_request[250];
            sprintf(http_request, "GET /update?api_key=%s&field1=%.2f&field2=%.2f&field3=%.2f HTTP/1.1\r\nHost: api.thingspeak.com\r\nConnection: close\r\n\r\n", wifiApiKey, wifiTemperature, wifiHumidity, wifiLuminosity);

            // Enviar AT+CIPSEND con longitud correcta
            Uart_sendstring("AT+CIPSEND=");
            Uart_printbase(strlen(http_request), 10);
            Uart_sendstring("\r\n");

            while (!Wait_for(">"));
            thingSpeakState = 3;
            break;
        }
        case 3: {				// Envio mensaje con los campos
        	Uart_flush();
        	char http_request[250];
        	sprintf(http_request, "GET /update?api_key=%s&field1=%.2f&field2=%.2f&field3=%.2f HTTP/1.1\r\nHost: api.thingspeak.com\r\nConnection: close\r\n\r\n", wifiApiKey, wifiTemperature, wifiHumidity, wifiLuminosity);

        	Uart_sendstring(http_request);
        	if (Wait_for("SEND OK\r\n")) {
				thingSpeakState = 4;
			} else {
				thingSpeakState = 0;  // Error → reiniciar
			}

//
//			while (!(Wait_for("OK\r\n")));
//			thingSpeakState = 4;
			break;
        }
        case 4: {  // Cerrar la conexión TCP con AT+CIPCLOSE
            Uart_flush();
            Uart_sendstring("AT+CIPCLOSE\r\n");
            Wait_for("OK\r\n");

            thingSpeakState = 0;  // Reiniciar máquina de estados
            break;
        }
        default: {
        	thingSpeakState = 0; // Finalizar la máquina de estados
			break;
        }

	}
}
