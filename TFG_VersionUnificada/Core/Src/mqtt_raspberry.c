/*
 * mqtt_raspberry.c
 *
 *  Created on: Jul 23, 2025
 *      Author: carlo
 */


#include "mqtt_raspberry.h"


//extern UART_HandleTypeDef huart6;



uint8_t rpiState = 0;  // Define la variable
volatile uint8_t err = 0;

static float wifiTemperature = 0, wifiHumidity = 0, wifiLuminosity = 0;


void sendDataToRpi(float t, float h, float l) {
	wifiTemperature = t;
	wifiHumidity = h;
	wifiLuminosity = l;
    rpiState = 1;	// Iniciar la máquina de estados
}

void mqtt_send_connect_packet(void)
{
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

    // MQTT 5: propiedades vacías
    packet[idx++] = 0x00;

    // Payload: Client ID
    uint16_t len = strlen(clientId);
    packet[idx++] = (uint8_t)(len >> 8);
    packet[idx++] = (uint8_t)(len & 0xFF);
    memcpy(&packet[idx], clientId, len);
    idx += len;

    // Rellenar Remaining Length
    packet[remLenPos] = idx - 2;

    // 1) Preparar CIPSEND
    char cmd[32];
    sprintf(cmd, "AT+CIPSEND=%u\r\n", idx);

    Uart_flush();
    Uart_sendstring(cmd);

    // 2) Esperar prompt ">"
    if (!Wait_for(">")) {
        printf("❌ Error esperando '>' en CIPSEND\r\n");
        return;
    }

    // 3) Enviar el paquete MQTT
    for (uint16_t i = 0; i < idx; i++) {
        Uart_write(packet[i]);
    }

    // 4) (Opcional) esperar "SEND OK"
    Wait_for("SEND OK\r\n");
}


bool mqtt_wait_connack(void)
{
    const uint32_t timeout = 1000;  // 1s
    uint32_t start = HAL_GetTick();



    uint8_t expected[4] = {0x20, 0x02, 0x00, 0x00};
    uint8_t received[4];
    int idx = 0;

    while ((HAL_GetTick() - start) < timeout)
    {
        if (IsDataAvailable())
        {
            int c = Uart_read();
            if (c < 0) continue;

            received[idx++] = (uint8_t)c;

            // Si recibimos 4 bytes, comparamos
            if (idx == 4)
            {
                if (memcmp(received, expected, 4) == 0)
                {
                    return true;
                }
                else
                {
                    break;  // llegó algo pero no es CONNACK válido
                }
            }
        }
    }
    return false;
}

void mqtt_publish_unified_json(const char* topic)
{
    // 1) Formatear payload JSON
    char payload[128];
    int payloadLen = snprintf(payload, sizeof(payload),
                              "{\"temp\":%.2f,\"hum\":%.2f,\"lux\":%.2f}",
							  wifiTemperature, wifiHumidity, wifiLuminosity);
    if (payloadLen < 0 || payloadLen >= sizeof(payload)) return;

    // 2) Longitudes
    uint16_t topicLen = strlen(topic);
    uint16_t msgLen = 2 + topicLen + 1 + payloadLen;  // topic len (2) + topic + props (1) + payload

    // 3) Fixed header MQTT
    uint8_t header[5];
    uint8_t hdrLen = 0;
    header[hdrLen++] = 0x30;  // PUBLISH QoS 0

    // Remaining Length (formato variable)
    uint32_t rem = msgLen;
    do {
        uint8_t b = rem & 0x7F;
        rem >>= 7;
        if (rem) b |= 0x80;
        header[hdrLen++] = b;
    } while (rem);

    // 4) Enviar AT+CIPSEND
    char cmd[32];
    sprintf(cmd, "AT+CIPSEND=%u\r\n", hdrLen + msgLen);

    Uart_flush();
    Uart_sendstring(cmd);

    if (!Wait_for(">")) {
        printf("❌ No llegó '>' en CIPSEND MQTT\r\n");
        return;
    }

    // 5) Enviar el paquete MQTT completo (header + topic + props + payload)

    // → header
    for (uint8_t i = 0; i < hdrLen; i++) {
        Uart_write(header[i]);
    }

    // → topic length (2 bytes)
    Uart_write((uint8_t)(topicLen >> 8));
    Uart_write((uint8_t)(topicLen & 0xFF));

    // → topic
    for (uint16_t i = 0; i < topicLen; i++) {
        Uart_write(topic[i]);
    }

    // → propiedades (0x00 para MQTT5 sin propiedades)
    Uart_write(0x00);

    // → payload (el JSON)
    for (int i = 0; i < payloadLen; i++) {
        Uart_write(payload[i]);
    }

    // 6) (Opcional) esperar "SEND OK"
    Wait_for("SEND OK\r\n");
}


void processRpiStateMachine(void) {
    static uint32_t stateTimeout = 0;

    switch (rpiState) {
    case 1: {  // Iniciar conexión TCP
        Uart_flush();
        char cmd[100];
        sprintf(cmd, "AT+CIPSTART=\"TCP\",\"%s\",%d\r\n", MQTT_BROKER_IP, MQTT_BROKER_PORT);
        Uart_sendstring(cmd);

        if (!Wait_for("OK\r\n")) {
        	closeConnection();
            rpiState = 0;
            break;
        }

        rpiState = 2;
        break;
    }

    case 2: {  // enviar paquete CONNECT (MQTT 5.0)
        Uart_flush();
        mqtt_send_connect_packet();

        rpiState = 3;
        stateTimeout = HAL_GetTick();
        break;
    }

    case 3: {  // Esperar CONNACK
        if (mqtt_wait_connack()) {
            rpiState = 4;
            break;
        } else if (HAL_GetTick() - stateTimeout > 2000) {
        	closeConnection();
			rpiState = 0;
			break;
		}
        break;
    }

    case 4: {  // MQTT PUBLISH JSON
        Uart_flush();
        mqtt_publish_unified_json(MQTT_TOPIC_JSON);

        rpiState = 5;
        stateTimeout = HAL_GetTick();
        break;
    }

    case 5: {  // Cerrar conexión
        closeConnection();
        rpiState = 0;
        break;

	}
    default:{
    	closeConnection();
		rpiState = 0;
		break;

    }
    }
}
