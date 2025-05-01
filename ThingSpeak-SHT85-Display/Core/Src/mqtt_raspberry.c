/*
 * mqtt_raspberry.c
 *
 *  Created on: Mar 22, 2025
 *      Author: carlo
 */

#include "mqtt_raspberry.h"
#include "esp_8266.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#define MQTT_BROKER_IP     "192.168.1.220"		// Mi router por cable. Raspberry Pi está conectada
#define MQTT_BROKER_PORT   1883
#define MQTT_CLIENT_ID     "STM32"
#define MQTT_USERNAME      "mqttuser"
#define MQTT_PASSWORD      "49RpCqjsuwm7a4"

extern UART_HandleTypeDef huart2;
char esp8266_rx_buffer[256];


static volatile uint8_t mqttEnabled = 0;
//static uint8_t mqttState = 0;
volatile uint8_t mqttState = 0;

static float lastTemp = 0, lastHum = 0, lastLux = 0;
static volatile uint32_t lastConnackTime = 0;

void mqtt_raspberry_set_enabled(uint8_t enable) {
    mqttEnabled = enable;
}

void mqtt_raspberry_send(float t, float h, float l) {
    lastTemp = t;
    lastHum = h;
    lastLux = l;
    mqttState = 1;
}

static void mqtt_publish_unified_json(const char* topic) {
    char payload[128];
    sprintf(payload, "{\"temp\":%.2f,\"hum\":%.2f,\"lux\":%.2f}", lastTemp, lastHum, lastLux);

    uint8_t header[] = {
        0x30,
        (uint8_t)(2 + strlen(topic) + strlen(payload))
    };

    char cipCmd[32];
    sprintf(cipCmd, "AT+CIPSEND=%d\r\n", header[1] + 2);
    esp8266_send_command(cipCmd);
    esp8266_wait_response_timeout(3000);

    uint8_t topic_len[2] = {0x00, strlen(topic)};

    HAL_UART_Transmit(&huart2, header, sizeof(header), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, topic_len, 2, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t*)topic, strlen(topic), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t*)payload, strlen(payload), HAL_MAX_DELAY);
}

static void mqtt_send_connect_packet(void) {
    uint8_t client_id_len = strlen(MQTT_CLIENT_ID);
    uint8_t username_len = strlen(MQTT_USERNAME);
    uint8_t password_len = strlen(MQTT_PASSWORD);
    uint16_t payload_len = 2 + client_id_len + 2 + username_len + 2 + password_len;

    uint8_t fixed_header[2] = { 0x10, 12 + payload_len };
    uint8_t variable_header[] = {
        0x00, 0x04, 'M', 'Q', 'T', 'T', // Protocol Name
        0x04,                          // Protocol Level
        0xC2,                          // Connect Flags: username, password, clean session
        0x00, 0x3C                     // Keep Alive
    };

    char at_cmd[32];
    sprintf(at_cmd, "AT+CIPSEND=%d\r\n", fixed_header[1] + 2);
    esp8266_send_command(at_cmd);
    esp8266_wait_response_timeout(3000);

    HAL_UART_Transmit(&huart2, fixed_header, sizeof(fixed_header), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, variable_header, sizeof(variable_header), HAL_MAX_DELAY);

    uint8_t id_len[2] = {0x00, client_id_len};
    HAL_UART_Transmit(&huart2, id_len, 2, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t*)MQTT_CLIENT_ID, client_id_len, HAL_MAX_DELAY);

    uint8_t usr_len[2] = {0x00, username_len};
    HAL_UART_Transmit(&huart2, usr_len, 2, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t*)MQTT_USERNAME, username_len, HAL_MAX_DELAY);

    uint8_t pwd_len[2] = {0x00, password_len};
    HAL_UART_Transmit(&huart2, pwd_len, 2, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t*)MQTT_PASSWORD, password_len, HAL_MAX_DELAY);
}


void mqtt_raspberry_process(void) {
    if (!mqttEnabled) return;

    static uint32_t stateTimeout = 0; // Variable para manejar el timeout

    switch (mqttState) {
        case 1: {		// Cerrar conexión anterior y abrir nueva conexión TCP con el broker
        	esp8266_send_command("AT+CIPCLOSE\r\n");
			HAL_Delay(100);
			esp8266_send_command("AT+CIPSTART=\"TCP\",\"" MQTT_BROKER_IP "\",1883\r\n");
			esp8266_receive_response_IT();

            stateTimeout = HAL_GetTick(); // Guardar el tiempo actual
			mqttState = 2;
			break;
        }
        case 2: {
        	if (esp8266_is_response_ready()) {
        		esp8266_clear_response_flag();	//rxComplete = 0;
				if (strstr(esp8266_get_response(), "CONNECT") || strstr(esp8266_get_response(), "ALREADY CONNECTED")) {
					mqttState = 3;
				} else {
					mqttState = 0; // error
				}
			} else if (HAL_GetTick() - stateTimeout > 5000) { // Timeout de 5 segundos
                printf("MQTT State 2 timeout. Reiniciando conexión...\r\n");
                mqttState = 0; // Reiniciar el estado en caso de timeout
            }
			break;
        }
        case 3: { // SEND CONNECT packet
        	mqtt_send_connect_packet();
        	esp8266_receive_response_IT();
			stateTimeout = HAL_GetTick(); // Reiniciar el tiempo para el siguiente estado
            mqttState = 4;
            break;
        }
        case 4: { // Esperar CONNACK sin bloquear
        	if (esp8266_is_response_ready()) {
				esp8266_clear_response_flag(); // Limpiar el flag de respuesta
				char* connack = esp8266_get_response(); // Leer el buffer de respuesta

				// Verificar si el paquete recibido es un CONNACK válido
				if (connack[0] == 0x20 && connack[1] == 0x02) {
					if ((uint8_t)connack[3] == 0x00) {
						printf("CONNACK recibido correctamente. MQTT conectado.\r\n");
						mqttState = 5; // Avanzar al siguiente estado
					} else {
						printf("Error en CONNACK. Código: %d\r\n", connack[3]);
						mqttState = 0; // Reiniciar en caso de error
					}
				} else {
					printf("Paquete inesperado en CONNACK: %s\r\n", connack);
					mqttState = 0; // Reiniciar en caso de paquete no válido
				}
			} else if (HAL_GetTick() - stateTimeout > 1000) { // Timeout esperando CONNACK
				printf("Timeout esperando CONNACK. Reiniciando...\r\n");
				mqttState = 0;
			}
			break;
        }

        case 5: { // Enviar PUBLISH (JSON)
        	mqtt_publish_unified_json("Invernadero/sensores");
        	 esp8266_receive_response_IT();
			mqttState = 6;
			break;
        }

        case 6: { // Cerrar conexión TCP con el broker
            esp8266_send_command("AT+CIPCLOSE\r\n");

            if (!esp8266_wait_response_timeout(3000)) {
                printf("Cerrando conexión MQTT\r\n");
            } else {
                printf("MQTT conexión cerrada correctamente\r\n");
            }
            mqttEnabled = 0; // Desactivar al terminar el envío
            mqttState = 0;
            break;
        }

        default:
            break;
    }
}
