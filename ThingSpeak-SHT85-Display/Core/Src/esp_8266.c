/*
 * esp_8266.c
 *
 *  Created on: Mar 22, 2025
 *      Author: carlo
 */

#include "esp_8266.h"

extern UART_HandleTypeDef huart2;

// - - -  Variables globales (definidas en esp_8266.c)
uint8_t  usartBuff_Rx;
uint8_t  RxBuffer[ESP_RX_SIZE];
uint8_t  TxBuffer[ESP_TX_SIZE];
uint16_t RxIndex;
bool     RxIsData;
uint8_t	comprob;

// Callback de recepción por interrupción
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
    	if (RxIndex < ESP_RX_SIZE) {
    	            RxBuffer[RxIndex++] = usartBuff_Rx;
    	            RxBuffer[RxIndex] = '\0';

		}
		// 2) rearma la interrupción para seguir recibiendo
		HAL_UART_Receive_IT(&huart2, &usartBuff_Rx, 1);
    }
}



/**
 * @brief Inicializa UART2 y el módulo ESP8266.
 * @return true si la secuencia de inicio (AT, CWMODE…) fue exitosa.
 */
bool ESP_Init(void)
{
    uint8_t result;
    bool ok = false;



    do {
    	// Limpiamos estados
		ESP_RxClear();
//		ESP_TxClear();
        // 1) Verificar basic AT
    	if (ESP_SendCommand("AT")==false) {
        	comprob = 1;
            break;
        }
        if (ESP_WaitForResponse(ESP_AT_TIMEOUT_MS, &result, 2,
                                 "OK", "ERROR")==false) {
        	comprob=2;
            break;
        }
        if (result == 2) {  // recibió "ERROR"
        	comprob=3;
            break;
        }
        ok = true;
        ESP_RxClear();
        ESP_TxClear();

        HAL_UART_Receive_IT(&huart2,&usartBuff_Rx,1);

//        // 2) Poner modo estación
//        if (!ESP_SendCommand("AT+CWMODE=1")) {
//        	comprob=4;
//            break;
//        }
//        if (!ESP_WaitForResponse(ESP_AT_TIMEOUT_MS, &result, 2,
//                                 "OK\r\n", "ERROR\r\n")) {
//        	comprob=5;
//            break;
//        }
//        if (result != 1) {  // recibió "ERROR"
//        	comprob=6;
//            break;
//        }
//        ESP_RxClear();

        // Si llegamos aquí, OK en ambos pasos
//        ok = true;
    } while (0);

    return ok;
}


void ESP_RxClear(void)
{
    // Reinicia el índice
    RxIndex = 0;
    // Marca que no hay datos completos aún
    RxIsData = false;
    // Vacía el buffer de recepción
	memset(RxBuffer, 0, ESP_RX_SIZE);
    // Rearma la interrupción UART para seguir recibiendo
    HAL_UART_Receive_IT(&huart2, &usartBuff_Rx, 1);
}

void ESP_TxClear(void)
{
    // Vacía el buffer de transmisión
    memset(TxBuffer, 0, ESP_TX_SIZE);
    // Si tuvieras un índice de TX, aquí lo reiniciarías
    // TxIndex = 0;
}

bool ESP_ConnectWiFi(const char *ssid, const char *password)
{
    uint8_t result;
    char    cmd[128];

    // 1) AT
    ESP_RxClear();
//    ESP_StartReceptionIT();                        // HAL_UART_Receive_IT(&huart2, &usartBuff, 1);
    ESP_SendCommand("AT");                         // envía "AT\r\n"
    if (!ESP_WaitForResponse(ESP_AT_TIMEOUT_MS,     // espera “OK”
                             &result,
                             1,
                             "OK\r\n"))
    {
        printf("ESP8266 no responde a AT\r\n");
        return false;
    }

    // 2) Soft reset
    ESP_RxClear();
//    ESP_StartReceptionIT();
    ESP_SendCommand("AT+RST");                     // envía "AT+RST\r\n"
    if (!ESP_WaitForResponse(ESP_RST_TIMEOUT_MS,    // espera “ready”
                             &result,
                             1,
                             "ready\r\n"))
    {
        printf("ESP8266 no se reinició bien\r\n");
        return false;
    }

    // 3) Station mode
    ESP_RxClear();
//    ESP_StartReceptionIT();
    ESP_SendCommand("AT+CWMODE=1");                // envía "AT+CWMODE=1\r\n"
    if (!ESP_WaitForResponse(ESP_AT_TIMEOUT_MS,     // espera “OK”
                             &result,
                             1,
                             "OK\r\n"))
    {
        printf("Modo station no configurado\r\n");
        return false;
    }

    // 4) Join AP
    ESP_RxClear();
//    ESP_StartReceptionIT();
    snprintf(cmd, sizeof(cmd),
             "AT+CWJAP=\"%s\",\"%s\"",
             ssid, password);
    ESP_SendCommand(cmd);                          // envía "AT+CWJAP="ssid","pass"\r\n"
    // esperamos tanto WIFI CONNECTED como OK final
    if (!ESP_WaitForResponse(ESP_JOIN_TIMEOUT_MS,
                             &result,
                             2,
                             "WIFI CONNECTED\r\n",
                             "OK\r\n"))
    {
    	comprob = 50;
        printf("Error conectando al WiFi\r\n");
        return false;
    }


    printf("Conectado al WiFi ✔\r\n");
    return true;
}

bool ESP_SendRaw(uint8_t *data,uint16_t len)
{
	if(len <= ESP_TX_SIZE)
	{
		// Send the information in data through the UART of the ESP8266
		memcpy(TxBuffer,data,len);
		if(HAL_UART_Transmit(&huart2,data,len,900) == HAL_OK)
			return true;
		else
			return false;
	}
	else
		return false;
}


bool ESP_SendCommand(char *cmd)
{
	return ESP_SendRaw((uint8_t*)cmd,strlen(cmd));
}


/**
 * @brief Espera una o varias respuestas del módulo.
 * @param timeout_ms Tiempo máximo (ms) a esperar.
 * @param result Puntero donde se devolverá el índice de la cadena coincidente.
 * @param paramCount Número de cadenas a comprobar.
 * @param ... Listado de punteros a cadenas ANSI que buscaremos en RxBuffer.
 * @return true si alguna de las cadenas apareció antes de expirar el timeout.
 */
bool ESP_WaitForResponse(uint32_t timeout_ms, uint8_t *result, uint8_t paramCount, ...)
{
    if (result == NULL || paramCount == 0) {
        return false;
    }
    *result = 0;

    va_list tag;
		va_start (tag,paramCount);
		char *arg[paramCount];
		for(uint8_t i=0; i<paramCount ; i++)
			arg[i] = va_arg (tag, char *);
	va_end (tag);

	for(uint32_t t=0 ; t<timeout_ms ; t+=20)
	{
		HAL_Delay(20);
		for(uint8_t	mx=0 ; mx<paramCount ; mx++)
		{
			if(strstr((char*)RxBuffer,arg[mx])!=NULL)
			{
				*result = mx+1;
				return true;
			}
		}
	}
	// timeout
	return false;

}

bool ESP_WaitForChar(uint32_t timeout_ms, char target)
{
    uint32_t start = HAL_GetTick();

    // Bucle hasta agotar timeout
    while ((HAL_GetTick() - start) < timeout_ms) {
        // Recorre todos los bytes recibidos hasta ahora
        for (uint16_t i = 0; i < RxIndex; i++) {
            if ((char)RxBuffer[i] == target) {
                return true;
            }
        }
        // Pequeña pausa para no bloquear totalmente el bus
        HAL_Delay(1);
    }

    // Timeout sin encontrar el carácter
    return false;
}


bool ESP_ResponseReady(void) {
	return RxIsData;
}



bool ESP_OpenConnection(const char *host, uint16_t port)
{
    uint8_t result;
    char    cmd[128];

    // 1) Preparamos buffers e interrupción
    ESP_RxClear();
    ESP_TxClear();

    // 2) Montamos el comando CIPSTART
    //    Modo single-connection (CIPMUX=0 por defecto)
    snprintf(cmd, sizeof(cmd),
             "AT+CIPSTART=\"TCP\",\"%s\",%d\"\r\n",
             host, port);

    // 3) Lo enviamos
    if (!ESP_SendCommand(cmd)) {
        return false;
    }

    // 4) Esperamos "CONNECT" o "ERROR"
    if (!ESP_WaitForResponse(ESP_OPEN_TIMEOUT_MS,
                             &result,
                             2,
                             "CONNECT\r\n",
                             "ERROR\r\n"))
    {
        return false;  // timeout
    }

    // 5) Sólo CONNECT (result==1) es éxito
    return (result == 1);
}

/**
 * @brief Envía datos por la conexión abierta.
 * @param data   Puntero a los bytes a enviar.
 * @param length Longitud en bytes.
 * @return true si transmitió y recibió “SEND OK”.
 */
bool ESP_SendData(const uint8_t *data, uint16_t length)
{
    uint8_t result;
    char    cmd[32];

    // 1) Reiniciar buffers e IT
    ESP_RxClear();
    ESP_TxClear();

    // 2) Pido espacio con CIPSENDBUF=<length>
    //    Para single-connection:
    snprintf(cmd, sizeof(cmd), "AT+CIPSENDBUF=%u", length);

    if (!ESP_SendCommand(cmd)) {
        return false;
    }
    // 3) Espero OK o ERROR
    if (!ESP_WaitForResponse(ESP_CMD_TIMEOUT_MS, &result, 2,
                             "OK\r\n", "ERROR\r\n"))
    {
        return false;
    }
    if (result != 1) {
        // ERROR
        return false;
    }

    // 4) Espero prompt '>' o "ERROR" o "busy"
    if (!ESP_WaitForResponse(ESP_PROMPT_TIMEOUT_MS, &result, 3,
                             ">", "ERROR", "busy"))
    {
        return false;
    }
    if (result != 1) {
        // si result==2 o 3 => ERROR o busy
        return false;
    }

    // 5) Envío los datos crudos
    ESP_RxClear();
    if (HAL_UART_Transmit(&huart2, (uint8_t*)data, length, 1000) != HAL_OK) {
        return false;
    }

    // 6) Espero "SEND OK" o "ERROR"
    if (!ESP_WaitForResponse(ESP_SEND_TIMEOUT_MS, &result, 2,
                             "SEND OK\r\n", "ERROR\r\n"))
    {
        return false;
    }
    return (result == 1);
}



/**
 * @brief Cierra la conexión TCP abierta.
 * @return true si obtuvo “OK” tras CIPCLOSE.
 */
bool ESP_CloseConnection(uint8_t unusedLinkId)
{
    uint8_t result;

    // 1) Limpiar buffers e IT
    ESP_RxClear();
    ESP_TxClear();

    // 2) Enviar AT+CIPCLOSE
    if (!ESP_SendCommand("AT+CIPCLOSE")) {
        return false;
    }

    // 3) Esperar OK o ERROR
    if (!ESP_WaitForResponse(ESP_CLOSE_TIMEOUT_MS,
                             &result,
                             2,
                             "OK\r\n",
                             "ERROR\r\n"))
    {
        return false;
    }

    // 4) Sólo OK (result == 1) es éxito
    return (result == 1);
}

void ESP_RESET(void)
{
    static uint8_t  state = 0;
    static uint32_t t0, t1;
    uint32_t now = HAL_GetTick();

    switch (state)
    {
        case 0:
            // Iniciar pulso de reset
            HAL_GPIO_WritePin(ESP_RST_GPIO_PORT, ESP_RST_PIN, GPIO_PIN_RESET);
            t0 = now;
            state = 1;
            break;

        case 1:
            // Mantener reset durante ESP_RESET_PULSE_MS
            if (now - t0 >= ESP_RESET_PULSE_MS)
            {
                // Soltar reset
                HAL_GPIO_WritePin(ESP_RST_GPIO_PORT, ESP_RST_PIN, GPIO_PIN_SET);
                t1 = now;
                state = 2;

                // Tras reset, re-inicializar módulo y WiFi
                ESP_RxClear();
                ESP_TxClear();
                if (ESP_Init())
                {
                    ESP_ConnectWiFi(WIFI_SSID, WIFI_PASS);
                }
            }
            break;

        case 2:
            // Esperar hasta el siguiente ciclo de reset
            if (now - t1 >= ESP_RESET_INTERVAL)
            {
                state = 0;
            }
            break;
    }
}


const char *ESP_GetResponse(void)
{
    // Asegúrate de que el buffer esté null-terminated:
    RxBuffer[RxIndex < ESP_RX_SIZE ? RxIndex : ESP_RX_SIZE - 1] = '\0';
    return (const char *)RxBuffer;
}


void ESP_ClearResponseFlag(void)
{
	RxIsData = false;
}
