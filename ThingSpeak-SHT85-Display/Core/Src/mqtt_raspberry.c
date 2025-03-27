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

#define MQTT_BROKER_IP     "192.168.1.87"		// Mi router por cable. Raspberry Pi está conectada
#define MQTT_BROKER_PORT   1883
#define MQTT_CLIENT_ID     "STM32"
#define MQTT_TOPIC         "Invernadero/estado"

extern UART_HandleTypeDef huart2;
char esp8266_rx_buffer[256];


static uint8_t mqttEnabled = 0;
static uint8_t mqttState = 0;

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

void mqtt_raspberry_process(void) {
    if (!mqttEnabled) return;

    switch (mqttState) {
        case 1: {
            esp8266_send_command("AT+CIPSTART=\"TCP\",\"" MQTT_BROKER_IP "\",1883\r\n");
            esp8266_receive_response_IT();
            mqttState = 2;
            break;
        }
        case 2: {
            if (esp8266_is_response_ready()) {
                if (strstr(esp8266_get_response(), "CONNECT")) {
                    mqttState = 3;
                } else {
                    mqttState = 0; // error
                }
            }
            break;
        }
        case 3: { // SEND CONNECT packet
            uint8_t mqtt_connect[] = {
                0x10,
                16 + strlen(MQTT_CLIENT_ID),
                0x00, 0x04, 'M', 'Q', 'T', 'T',
                0x04,
                0x02,
                0x00, 0x3C,
                0x00, (uint8_t)strlen(MQTT_CLIENT_ID)
            };

            uint16_t total_len = sizeof(mqtt_connect) + strlen(MQTT_CLIENT_ID);

			char at_cmd[32];
			sprintf(at_cmd, "AT+CIPSEND=%d\r\n", total_len);
			esp8266_send_command(at_cmd);


            HAL_UART_Transmit(&huart2, mqtt_connect, sizeof(mqtt_connect), HAL_MAX_DELAY);
            HAL_UART_Transmit(&huart2, (uint8_t*)MQTT_CLIENT_ID, strlen(MQTT_CLIENT_ID), HAL_MAX_DELAY);
            mqttState = 4;
            break;
        }
        case 4: { // Esperar CONNACK sin bloquear
            if (HAL_GetTick() - lastConnackTime > 1000) {
                mqttState = 0; // timeout esperando CONNACK
                break;
            }

            char* connack = esp8266_rx_buffer;

            if (connack[0] == 0x20 && connack[1] == 0x02) {
                if ((uint8_t)connack[3] == 0x00) {
                    mqttState = 5; // CONNACK OK
                } else {		// ERROR
                    mqttState = 0;
                }
            }

            break;
        }

        case 5: { // Enviar PUBLISH (JSON)
            char json[64];
            sprintf(json, "{\"t\":%.2f,\"h\":%.2f,\"l\":%.2f}", lastTemp, lastHum, lastLux);

            uint8_t header[] = {
                0x30, // PUBLISH
                (uint8_t)(2 + strlen(MQTT_TOPIC) + strlen(json))
            };

            char cipCmd[32];
            sprintf(cipCmd, "AT+CIPSEND=%d\r\n", header[1] + 2);
            esp8266_send_command(cipCmd);
            esp8266_receive_response_IT();

            if (!esp8266_wait_response_timeout(3000) || strstr(esp8266_rx_buffer, ">") == NULL) {
                mqttState = 0;
                break;
            }

            uint8_t topic_len[2] = {0x00, strlen(MQTT_TOPIC)};

            // Enviar paquete MQTT PUBLISH completo
            HAL_UART_Transmit(&huart2, header, sizeof(header), HAL_MAX_DELAY);
            HAL_UART_Transmit(&huart2, topic_len, 2, HAL_MAX_DELAY);
            HAL_UART_Transmit(&huart2, (uint8_t*)MQTT_TOPIC, strlen(MQTT_TOPIC), HAL_MAX_DELAY);
            HAL_UART_Transmit(&huart2, (uint8_t*)json, strlen(json), HAL_MAX_DELAY);

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

            mqttState = 0;
            break;
        }

        default:
            break;
    }
}
