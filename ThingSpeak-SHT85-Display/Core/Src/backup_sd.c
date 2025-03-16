/*
 * backup_sd.c
 *
 *  Created on: Mar 14, 2025
 *      Author: carlo
 */


//#include "backup_sd.h"
//#include "wifi_thingspeak.h"
//#include <stdio.h>
//#include <string.h>
//
//static SensorData buffer[BUFFER_SIZE];
//static uint8_t bufferIndex = 0;
//static FATFS fs;
//static FIL file;

//void backup_init() {
//    f_mount(&fs, "", 1);
//}
//
//void storeData(float temperature, float humidity, float luminosity) {
//    SensorData data = {temperature, humidity, luminosity, HAL_GetTick()};
//    buffer[bufferIndex++] = data;
//    if (bufferIndex >= BUFFER_SIZE) {
//        if (!isWiFiConnected()) {
//            saveToSD();
//        }
//        bufferIndex = 0;
//    }
//}
//
//void saveToSD() {
//    if (f_open(&file, SD_FILENAME, FA_OPEN_APPEND | FA_WRITE) == FR_OK) {
//        for (int i = 0; i < BUFFER_SIZE; i++) {
//            char line[50];
//            sprintf(line, "%.2f,%.2f,%.2f,%lu\n", buffer[i].temperature, buffer[i].humidity, buffer[i].luminosity, buffer[i].timestamp);
//            f_puts(line, &file);
//        }
//        f_close(&file);
//    }
//}
//
//void readFromSDAndSend() {
//    if (f_open(&file, SD_FILENAME, FA_READ) == FR_OK) {
//        char line[50];
//        while (f_gets(line, sizeof(line), &file)) {
//            SensorData data;
//            //sscanf(line, "%f,%f,%f,%lu", &data.temperature, &data.humidity, &data.luminosity, &data.timestamp);
//            sendDataToThingSpeak(data.temperature, data.humidity, data.luminosity);
//        }
//        f_close(&file);
//        f_unlink(SD_FILENAME);
//    }
//}
//
//uint8_t isWiFiConnected() {
//    sendATCommand("AT+CIPSTATUS\r\n");
//    receiveResponse_IT();
//    HAL_Delay(1000);
//    return strstr(response, "STATUS:2") != NULL;
//}
