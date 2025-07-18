/*
 * mqtt_raspberry.c
 *
 *  Created on: Mar 22, 2025
 *      Author: carlo
 */


#include "mqtt_raspberry.h"
#include "esp_8266.h"


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


void mqtt_raspberry_set_enabled(uint8_t enable) {
    mqttEnabled = enable;
    mqttState = 1;
}

void mqtt_raspberry_send(float t, float h, float l) {
    lastTemp = t;
    lastHum = h;
    lastLux = l;
    mqttEnabled = 1;
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
    uint8_t  packet[256];
    uint16_t idx = 0;

    // Fixed header
    packet[idx++] = 0x10;         // MQTT CONNECT
    uint16_t remLenPos = idx++;   // hueco para Remaining Length

    // Variable header
    packet[idx++] = 0x00; packet[idx++] = 0x04;
    packet[idx++] = 'M'; packet[idx++] = 'Q';
    packet[idx++] = 'T'; packet[idx++] = 'T';
    packet[idx++] = 0x05;         // Protocol Level 5.0

    packet[idx++] = 0x02;         // Connect Flags: Clean Start
    packet[idx++] = 0x00; packet[idx++] = 0x3C; // Keep-alive = 60s

    // MQTT5: Properties length = 0
    packet[idx++] = 0x00;

    // Payload: Client Identifier
    packet[idx++] = (uint8_t)((strlen(clientId) >> 8) & 0xFF);
    packet[idx++] = (uint8_t)(strlen(clientId) & 0xFF);
    memcpy(&packet[idx], clientId, strlen(clientId));
    idx += strlen(clientId);

    // Rellenar Remaining Length (para <128 basta un byte)
    packet[remLenPos] = idx - 2;

    // 1) Preparar AT+CIPSEND
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%u", idx);

    // 2) Lanzar CIPSEND y esperar prompt ">"
    ESP_RxClear();
    ESP_TxClear();
    ESP_SendCommand(cmd);
    {
        uint8_t result;
        // Timeout de 3s debería bastar para el prompt
        if (!ESP_WaitForResponse(ESP_CIPSEND_TIMEOUT_MS, &result, 2, ">", "ERROR\r\n")) {
            // fallo en CIPSEND
            return;
        }
        if (result != 1) {
            // recibió "ERROR"
            return;
        }
    }

    // 3) Enviar el paquete MQTT
    if (!ESP_SendData(packet, idx)) {
        // Error al transmitir el payload
        return;
    }

    // 4) (Opcional) esperar "SEND OK"
    {
        uint8_t result;
        ESP_WaitForResponse(ESP_SEND_TIMEOUT_MS, &result, 2, "SEND OK\r\n", "ERROR\r\n");
    }
}

static void mqtt_publish_unified_json(const char* topic) {
    // 1) Formatear el payload JSON
    char payload[128];
    int  payloadLen = snprintf(payload, sizeof(payload),
                               "{\"temp\":%.2f,\"hum\":%.2f,\"lux\":%.2f}",
                               lastTemp, lastHum, lastLux);
    if (payloadLen < 0 || payloadLen >= sizeof(payload)) return;

    // 2) Calcular longitudes
    uint16_t topicLen = strlen(topic);
    // MQTT5 PUBLISH:
    //   2 bytes topic length + topic + 1 byte props-len + payload
    uint16_t msgLen   = 2 + topicLen + 1 + payloadLen;

    // 3) Construir encabezado MQTT (fixed header + remaining length var-int)
    uint8_t header[5];
    uint8_t hdrLen = 0;
    header[hdrLen++] = 0x30;  // PUBLISH QoS0

    // Remaining Length
    uint32_t rem = msgLen;
    do {
        uint8_t b = rem & 0x7F;
        rem >>= 7;
        if (rem) b |= 0x80;
        header[hdrLen++] = b;
    } while (rem);

    // 4) Lanzar AT+CIPSEND
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%u", hdrLen + msgLen);

    ESP_RxClear();
    ESP_TxClear();
    ESP_SendCommand(cmd);

    // 5) Esperar prompt ">"
    {
        uint8_t result;
        if (!ESP_WaitForResponse(ESP_CIPSEND_TIMEOUT_MS, &result, 2, ">", "ERROR\r\n")) {
            return;  // fallo en CIPSEND
        }
        if (result != 1) {
            return;  // ERROR
        }
    }

    // 6) Enviar todo el paquete MQTT (header + topic-len + topic + props + payload)
    //    Lo metemos en un único buffer para usar ESP_SendData():
    uint8_t buf[5 + 2 + 64 + 1 + 128]; // suficiente para header+rest
    uint16_t pos = 0;

    // header
    memcpy(&buf[pos], header, hdrLen);
    pos += hdrLen;
    // topic length
    buf[pos++] = (uint8_t)(topicLen >> 8);
    buf[pos++] = (uint8_t)(topicLen & 0xFF);
    // topic
    memcpy(&buf[pos], topic, topicLen);
    pos += topicLen;
    // properties length (MQTT5)
    buf[pos++] = 0x00;
    // payload
    memcpy(&buf[pos], payload, payloadLen);
    pos += payloadLen;

    // 7) Transmitir vía AT driver
    if (!ESP_SendData(buf, pos)) {
        // error en la transmisión
        return;
    }

    // 8) (Opcional) esperar confirmación "SEND OK"
    {
        uint8_t result;
        ESP_WaitForResponse(ESP_SEND_TIMEOUT_MS, &result, 2, "SEND OK\r\n", "ERROR\r\n");
    }
}

static bool mqtt_wait_connack(void) {
    // El CONNACK MQTT es un paquete binario de 4 bytes: 0x20,0x02,0x00,0x00
    const uint32_t timeout = 1000;  // 1s
    uint32_t start = HAL_GetTick();

    ESP_RxClear();  // arrancamos la recepción IT

    while ((HAL_GetTick() - start) < timeout) {
        if (ESP_ResponseReady()) {
            // ya tenemos datos en RxBuffer[0..rxIndex-1]
            if (RxIndex >= 4 &&
                RxBuffer[0] == 0x20 &&
                RxBuffer[1] == 0x02 &&
                RxBuffer[2] == 0x00 &&
                RxBuffer[3] == 0x00) {
                ESP_RxClear();
                return true;
            }
            break;  // llegó algo pero no es CONNACK válido
        }
    }
    return false;
}


void mqtt_raspberry_process(void) {
    static uint32_t stateTimeout = 0;
    char cmd[80];
    const char *resp;

    switch (mqttState) {
        case 1: {
            // 1) Iniciar conexión TCP con el broker
            ESP_RxClear();
            snprintf(cmd, sizeof(cmd),
                     "AT+CIPSTART=\"TCP\",\"%s\",%d",
                     MQTT_BROKER_IP, MQTT_BROKER_PORT);
            if (!ESP_SendCommand(cmd)) {
                printf("Error enviando CIPSTART\r\n");
                mqttState = 0;
                break;
            }
            stateTimeout = HAL_GetTick();
            mqttState = 2;
            break;
        }
        case 2: {
            // 2) Esperar respuesta de AT+CIPSTART
            if (ESP_ResponseReady()) {
                ESP_ClearResponseFlag();
                resp = ESP_GetResponse();
                if ( strstr(resp, "CONNECT")
                  || strstr(resp, "ALREADY CONNECTED")
                  || strstr(resp, "OK") )
                {
                    mqttState = 3;
                    ESP_RxClear();
                }
                else {
                    printf("Error conectando al broker: %s\r\n", resp);
                    mqttState = 0;
                }
            }
            else if (HAL_GetTick() - stateTimeout > 5000) {
                printf("Timeout TCP\r\n");
                mqttState = 0;
            }
            break;
        }
        case 3: {
            // 3) Enviar paquete CONNECT MQTT
            ESP_RxClear();
            mqtt_send_connect_packet();
            stateTimeout = HAL_GetTick();
            mqttState = 4;
            break;
        }
        case 4: {
            // 4) Esperar CONNACK (binario)
            if (mqtt_wait_connack()) {
                printf("MQTT CONNECT OK\r\n");
                mqttState = 5;
            }
            else if (HAL_GetTick() - stateTimeout > 2000) {
                printf("Timeout CONNACK\r\n");
                mqttState = 0;
            }
            break;
        }
        case 5: {
            // 5) Publicar mensaje JSON
            ESP_RxClear();
            mqtt_publish_unified_json(MQTT_TOPIC_JSON);
            stateTimeout = HAL_GetTick();
            mqttState = 6;
            break;
        }
        case 6: {
            // 6) Esperar "SEND OK" y cerrar
            if (ESP_ResponseReady()) {
                ESP_ClearResponseFlag();
                // damos un pequeño retardo para que se procese bien
                HAL_Delay(150);
                ESP_SendCommand("AT+CIPCLOSE");
                ESP_WaitForResponse(2000, NULL, 1, "OK");
                mqttEnabled = false;
                mqttState   = 0;
            }
            else if (HAL_GetTick() - stateTimeout > 2000) {
                // forzamos cierre igual
                ESP_SendCommand("AT+CIPCLOSE");
                ESP_WaitForResponse(2000, NULL, 1, "OK");
                mqttEnabled = false;
                mqttState   = 0;
            }
            break;
        }
        default: {
            mqttEnabled = false;
            mqttState   = 0;
            break;
        }
    }
}



uint8_t mqtt_raspberry_is_busy(void) {
    return mqttEnabled;
}
