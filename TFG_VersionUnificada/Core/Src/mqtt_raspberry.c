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

static float wifiTemperatureRpi = 0, wifiHumidityRpi = 0, wifiLuminosityRpi = 0;


void sendDataToRpi(float t, float h, float l) {
	Uart_clear ();
	wifiTemperatureRpi = t;
	wifiHumidityRpi = h;
	wifiLuminosityRpi = l;
    rpiState = 1;	// Iniciar la máquina de estados
}

void mqtt_send_connect_packet(void)
{
    uint8_t packet[] = {
        0x10, 0x2c,                         // Fixed header: CONNECT + Remaining Length = 47
        0x00, 0x04, 'M', 'Q', 'T', 'T',     // Protocol Name
        0x05,                               // Protocol Level = 5
        0xC2,                               // Connect Flags: username + password + clean start
        0x00, 0x3C,                         // Keep alive = 60s

        0x05,                               // Properties length
        0x11, 0x00, 0x00, 0x01, 0x2C,       // Session Expiry Interval = 300

        0x00, 0x0E,                         // Client ID length = 14
        'm','q','t','t','x','_','0','c','6','6','8','d','0','d',

        0x00, 0x05,                         // Username length = 5
        'o','s','u','n','a',

        0x00, 0x03,                         // Password length = 6
        't','f','g'
    };

    // 1) AT+CIPSEND con longitud total
    char cmd[32];
    sprintf(cmd, "AT+CIPSEND=%u\r\n", sizeof(packet));

    Uart_flush();
    Uart_sendstring(cmd);

    if (!Wait_for(">")) return;

    // 2) Enviar byte a byte
    for (uint16_t i = 0; i < sizeof(packet); i++) {
        Uart_write(packet[i]);
    }

    Wait_for("SEND OK\r\n");
}

/*
bool mqtt_wait_connack(void)
{
    const uint32_t timeout = 3000;  // 1s
    uint32_t start = HAL_GetTick();

    // CONNACK recibida (20 13 00 00 10 27 00 10 00 00 25 01 2a 01 29 01 22 ff ff 28 01)
    const uint8_t expected[] = {
        0x20, 0x13, 0x00, 0x00, 0x10, 0x27, 0x00, 0x10, 0x00, 0x00,
        0x25, 0x01, 0x2A, 0x01, 0x29, 0x01, 0x22, 0xFF, 0xFF, 0x28, 0x01
    };
    const int expected_len = sizeof(expected);
    int idx = 0;

    while ((HAL_GetTick() - start) < timeout)
    {
        if (IsDataAvailable())
        {
            int c = Uart_read();
            if (c < 0) continue;

            if ((uint8_t)c != expected[idx])
                return false;

            idx++;

            if (idx == expected_len)
                return true;
        }
    }

    return false;
}
*/

bool mqtt_wait_connack(void)
{
    const uint8_t expected[] = { 0x20, 0x13, 0x00, 0x00 }; // CONNACK mínimo
    uint8_t received[4];
    int idx = 0;
    uint32_t start = HAL_GetTick();
    const uint32_t timeout = 3000;  // Más tiempo

    while ((HAL_GetTick() - start) < timeout)
    {
        if (IsDataAvailable())
        {
        	uint8_t c = (uint8_t)Uart_read();

			// Solo empezar a almacenar si detectamos el primer byte correcto
			if (idx == 0 && c != expected[0])
				continue;

			received[idx++] = c;

			if (idx == 4)
				return (received[0] == 0x20 && received[2] == 0x00 && received[3] == 0x00);
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
							  wifiTemperatureRpi, wifiHumidityRpi, wifiLuminosityRpi);
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
        } else if (HAL_GetTick() - stateTimeout > 5000) {
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
