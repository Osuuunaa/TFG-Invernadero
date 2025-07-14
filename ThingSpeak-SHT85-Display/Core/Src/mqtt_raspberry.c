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

extern UART_HandleTypeDef huart2;

#define MQTT_BROKER_IP     "192.168.1.220"
#define MQTT_BROKER_PORT   1883
#define MQTT_CLIENT_ID     "STM32"

#define MQTT_USERNAME      "osuna"
#define MQTT_PASSWORD      "tfg"
#define MQTT_TOPIC_JSON    "Invernadero/sensores"

static volatile uint8_t mqttEnabled = 0;
volatile uint8_t mqttState = 0;
volatile uint8_t err = 0;

static float lastTemp = 0, lastHum = 0, lastLux = 0;

// Prototipos auxiliares
static uint8_t mqtt_wait_connack(void);

void mqtt_raspberry_set_enabled(uint8_t enable) {
    mqttEnabled = enable;
}

void mqtt_raspberry_send(float t, float h, float l) {
    lastTemp = t;
    lastHum = h;
    lastLux = l;
    mqttState = 1;
}

//static void mqtt_send_connect_packet(void) {
//    const char* clientId = MQTT_CLIENT_ID;
//    const char* username = MQTT_USERNAME;
//    const char* password = MQTT_PASSWORD;
//
//    uint8_t packet[256];
//    uint8_t idx = 0;
//
//    packet[idx++] = 0x10;  // CONNECT
//    uint8_t remLenPos = idx++;  // Placeholder for Remaining Length
//
//    // Variable header
//    packet[idx++] = 0x00; packet[idx++] = 0x04;
//    packet[idx++] = 'M'; packet[idx++] = 'Q'; packet[idx++] = 'T'; packet[idx++] = 'T';
//    packet[idx++] = 0x04;      // Protocol level 4
//
////    packet[idx++] = 0xC2;      // Flags: username + password + clean session
////    packet[idx++] = 0xC0;
//    packet[idx++] = 0x02;  // Solo Clean Session
//
//
//    packet[idx++] = 0x00; packet[idx++] = 0x3C;  // Keep Alive: 60s
//
//    // Payload
//    packet[idx++] = 0x00;
//    packet[idx++] = strlen(clientId);
//    memcpy(&packet[idx], clientId, strlen(clientId));
//    idx += strlen(clientId);
//
////    packet[idx++] = 0x00; packet[idx++] = strlen(username);
////    memcpy(&packet[idx], username, strlen(username)); idx += strlen(username);
////
////    packet[idx++] = 0x00; packet[idx++] = strlen(password);
////    memcpy(&packet[idx], password, strlen(password)); idx += strlen(password);
//
//
//
//    packet[remLenPos] = idx - 2; // MQTT remaining length (OK si <127)
//    // Codificar Remaining Length (variable-length encoding)
////    uint32_t len = idx - 2;
////    uint8_t len_bytes[4];
////    uint8_t len_idx = 0;
////
////    do {
////        uint8_t byte = len % 128;
////        len /= 128;
////        if (len > 0) byte |= 0x80;
////        len_bytes[len_idx++] = byte;
////    } while (len > 0);
////
////    // Mover el payload para hacer hueco
////    memmove(&packet[remLenPos + len_idx], &packet[remLenPos + 1], idx - (remLenPos + 1));
////    memcpy(&packet[remLenPos], len_bytes, len_idx);
////    idx += (len_idx - 1);
//
//
//
//    // Enviar por AT+CIPSEND
//    char cmd[32];
//    sprintf(cmd, "AT+CIPSEND=%d\r\n", idx);
//    esp8266_send_command(cmd);
//    esp8266_wait_response_timeout(3000);
//
//    HAL_UART_Transmit(&huart2, packet, idx, HAL_MAX_DELAY);
//}

static void mqtt_send_connect_packet(void) {
    const char* clientId = MQTT_CLIENT_ID;

    uint8_t packet[256];
    uint8_t idx = 0;

    packet[idx++] = 0x10;  // CONNECT
    uint8_t remLenPos = idx++;  // Placeholder for Remaining Length

    // Variable header
    packet[idx++] = 0x00; packet[idx++] = 0x04;
    packet[idx++] = 'M'; packet[idx++] = 'Q'; packet[idx++] = 'T'; packet[idx++] = 'T';
    packet[idx++] = 0x05;      // Protocol level 5 (MQTT 5.0)

    packet[idx++] = 0x02;      // Flags: Solo Clean Session
    packet[idx++] = 0x00; packet[idx++] = 0x3C;  // Keep Alive: 60s

    // MQTT 5: Añadir campo "Properties" (de longitud 0)
    packet[idx++] = 0x00;      // Properties length (0 = no properties)

    // Payload
    packet[idx++] = 0x00; packet[idx++] = strlen(clientId);
    memcpy(&packet[idx], clientId, strlen(clientId)); idx += strlen(clientId);

    // Rellenar remaining length (solo válido para paquetes <127 bytes)
    packet[remLenPos] = idx - 2; // MQTT remaining length (OK si <127)

    // Enviar por AT+CIPSEND
    char cmd[32];
    sprintf(cmd, "AT+CIPSEND=%d\r\n", idx);
    esp8266_send_command(cmd);
    esp8266_wait_response_timeout(3000);

    HAL_UART_Transmit(&huart2, packet, idx, HAL_MAX_DELAY);
}

static void mqtt_publish_unified_json(const char* topic) {
    char payload[128];
    sprintf(payload, "{\"temp\":%.2f,\"hum\":%.2f,\"lux\":%.2f}", lastTemp, lastHum, lastLux);

    uint16_t topicLen = strlen(topic);
    uint16_t payloadLen = strlen(payload);

    // +2 -> longitud del tema, +topicLen -> nombre del tema, +1 -> propiedades (MQTT 5), +payloadLen -> datos
    uint16_t msgLen = 2 + topicLen + 1 + payloadLen;

    uint8_t header[5];
    uint8_t hdrLen = 0;
    header[hdrLen++] = 0x30;  // PUBLISH, QoS 0

    uint32_t len = msgLen;
    do {
        uint8_t byte = len % 128;
        len /= 128;
        if (len > 0) byte |= 0x80;
        header[hdrLen++] = byte;
    } while (len > 0);

    char cipCmd[32];
    sprintf(cipCmd, "AT+CIPSEND=%d\r\n", hdrLen + msgLen);
    esp8266_send_command(cipCmd);
    esp8266_wait_response_timeout(3000);

    HAL_UART_Transmit(&huart2, header, hdrLen, HAL_MAX_DELAY);

    // Nombre del tema
    uint8_t topicLenBytes[2] = { topicLen >> 8, topicLen & 0xFF };
    HAL_UART_Transmit(&huart2, topicLenBytes, 2, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t*)topic, topicLen, HAL_MAX_DELAY);

    // Propiedades (MQTT 5) -- longitud 0
    uint8_t propertiesLen = 0x00;
    HAL_UART_Transmit(&huart2, &propertiesLen, 1, HAL_MAX_DELAY);

    // Payload
    HAL_UART_Transmit(&huart2, (uint8_t*)payload, payloadLen, HAL_MAX_DELAY);
}

// Espera y lee 4 bytes del CONNACK MQTT (binario, no texto)
static uint8_t mqtt_wait_connack(void) {
    uint8_t connack[4] = {0};
    // Espera hasta 1 segundo a recibir el CONNACK
    if (HAL_UART_Receive(&huart2, connack, 4, 1000) == HAL_OK) {
        // CONNACK esperado: 0x20 0x02 0x00 0x00 (aceptado)
        if (connack[0] == 0x20 && connack[1] == 0x02 && connack[3] == 0x00) {
            return 1;
        }
    }
    return 0;
}

void mqtt_raspberry_process(void) {
//    if (!mqttEnabled) return;

    static uint32_t stateTimeout = 0;

    switch (mqttState) {
        case 1: {
            // Cierra cualquier conexión previa (mejor limpieza)
//            esp8266_send_command("AT+CIPCLOSE\r\n");
//            mqttState = 9;
//            HAL_Delay(1500);
//            esp8266_wait_response_timeout(1500);

//            mqttState = 10;
            char cmd[64];
            sprintf(cmd, "AT+CIPSTART=\"TCP\",\"%s\",%d\r\n", MQTT_BROKER_IP, MQTT_BROKER_PORT);
            mqttState = 11;
            esp8266_send_command(cmd);
            esp8266_receive_response_IT();
            stateTimeout = HAL_GetTick();
            mqttState = 2;
            memset(response, 0, sizeof(response));  // Limpiar el buffer
            break;
        }
        case 2: {
            if (esp8266_is_response_ready()) {

                esp8266_clear_response_flag();

                char* resp = esp8266_get_response();

                if (strstr(resp, "CONNECT") || strstr(resp, "ALREADY CONNECTED") || strstr(resp, "AAT+CIPSTART=\"TCP\",\"192.168.1.220\",1883")) {
                    mqttState = 3;
                    memset(response, 0, sizeof(response));  // Limpiar el buffer
                }
                else {
                    printf("Error conectando al broker\r\n");
                    mqttState = 0;
                }
            } else if (HAL_GetTick() - stateTimeout > 5000) {
                printf("Timeout en conexión TCP\r\n");
                mqttState = 0;
            }
            break;
        }
        case 3: {		// MANDA EL CONNECT
            mqtt_send_connect_packet();
            // Espera a que el ESP8266 mande "SEND OK" antes de leer CONNACK
            esp8266_receive_response_IT();
            stateTimeout = HAL_GetTick();
            mqttState = 4;
            memset(response, 0, sizeof(response));  // Limpiar el buffer
            break;
        }
        case 4: {		// ESPERA AL CONNACK
            if (esp8266_is_response_ready()) {
            	mqttState = 50;
                esp8266_clear_response_flag();
                // Importante: leer CONNACK binario MQTT
//                if (mqtt_wait_connack()) {
//                    printf("MQTT conectado\r\n");
//                    mqttState = 5;
//                } else {
//                    printf("CONNACK inválido\r\n");
//                    mqttState = 0;
//                }

                mqttState = 5;
            } else if (HAL_GetTick() - stateTimeout > 2000) {
                printf("Timeout esperando CONNACK\r\n");
                mqttState = 0;
            }
            break;
        }
        case 5: {		// MANDA EL PUBLISH
            mqtt_publish_unified_json(MQTT_TOPIC_JSON);
            esp8266_receive_response_IT(); // Espera a "SEND OK"
            mqttState = 6;
            stateTimeout = HAL_GetTick();
            break;
        }
        case 6: {
            if (esp8266_is_response_ready()) {
                esp8266_clear_response_flag();
                // Da tiempo a que el broker procese el publish antes de cerrar
                HAL_Delay(150);
                esp8266_send_command("AT+CIPCLOSE\r\n");
                esp8266_wait_response_timeout(2000);
                mqttEnabled = 0;
                mqttState = 0;
            } else if (HAL_GetTick() - stateTimeout > 2000) {
                // Si no llega "SEND OK", de todas formas cierra por seguridad
                esp8266_send_command("AT+CIPCLOSE\r\n");
                esp8266_wait_response_timeout(2000);
                mqttEnabled = 0;
                mqttState = 0;
            }
            break;
        }
        default:
//            mqttState = 0;
            mqttEnabled = 0;
            break;
    }
}

uint8_t mqtt_raspberry_is_busy(void) {
    return mqttEnabled;
}
