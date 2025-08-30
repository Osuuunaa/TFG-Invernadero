/**
  ******************************************************************************

  File:		ESP DATA HANDLER
  Author:   ControllersTech
  Updated:  3rd Aug 2020

  ******************************************************************************
  Copyright (C) 2017 ControllersTech.com

  This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
  of the GNU General Public License version 3 as published by the Free Software Foundation.
  This software library is shared with public for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
  or indirectly by this software, read more about this on the GNU General Public License.

  ******************************************************************************
*/



#include "ESP_DATA_HANDLER.h"
#include "UartRingbuffer.h"
#include "stdio.h"
#include "string.h"

extern UART_HandleTypeDef huart6;

#define wifi_uart &huart6


void ESP_Init (char *SSID, char *PASSWD, char *STAIP)
{
	char data[80];

	Ringbuf_init();

	Uart_sendstring("AT+RST\r\n");
	HAL_Delay(2000);

	/********* AT **********/
	//Uart_flush();
	Uart_sendstring("AT\r\n");
	while(!(Wait_for("OK\r\n")));


	/********* AT+CWMODE=1 **********/
	Uart_flush();
	Uart_sendstring("AT+CWMODE=1\r\n");
	while (!(Wait_for("OK\r\n")));

	/* Set Static IP Address */
	/********* AT+CWSTAIP=IPADDRESS **********/
	Uart_flush();
	sprintf (data, "AT+CIPSTA=\"%s\"\r\n", STAIP);
	Uart_sendstring(data);
	while (!(Wait_for("OK\r\n")));

	/********* AT+CWJAP="SSID","PASSWD" **********/
	Uart_flush();
	sprintf (data, "AT+CWJAP=\"%s\",\"%s\"\r\n", SSID, PASSWD);
	Uart_sendstring(data);
//	while (!(Wait_for("OK\r\n")));

	// Escuchar múltiples posibles respuestas
//	while (!(Wait_for("WIFI CONNECTED")));
//	while (!(Wait_for("WIFI GOT IP")));
	while (!(Wait_for("OK\r\n")));



	  Uart_flush();
	  Uart_sendstring("AT+CWJAP?\r\n");
	  while (!(Wait_for("OK\r\n")));
}


int Server_Send (char *str, int Link_ID)
{
	int len = strlen (str);
	char data[80];
	Uart_flush();
	sprintf (data, "AT+CIPSEND=%d,%d\r\n", Link_ID, len);
	Uart_sendstring(data);
	while (!(Wait_for(">")));
	Uart_sendstring (str);
	while (!(Wait_for("SEND OK")));
	Uart_flush();
	sprintf (data, "AT+CIPCLOSE=%d\r\n",Link_ID);
	Uart_sendstring(data);
	while (!(Wait_for("OK\r\n")));
	return 1;
}


