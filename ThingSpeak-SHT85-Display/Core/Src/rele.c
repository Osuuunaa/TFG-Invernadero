/*
 * rele.c
 *
 *  Created on: Mar 16, 2025
 *      Author: carlo
 */


#include "rele.h"


void Rele_On(void) {
    HAL_GPIO_WritePin(RELE_GPIO_PORT, RELE_PIN, GPIO_PIN_SET);
}

void Rele_Off(void) {
    HAL_GPIO_WritePin(RELE_GPIO_PORT, RELE_PIN, GPIO_PIN_RESET);
}

void Control_Rele(float luminosity) {
    if (luminosity < 0.5) {
        Rele_On();
    } else {
        Rele_Off();
    }
}
