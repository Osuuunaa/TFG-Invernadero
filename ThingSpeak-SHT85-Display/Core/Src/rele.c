/*
 * rele.c
 *
 *  Created on: Mar 16, 2025
 *      Author: carlo
 */


#include "rele.h"

extern volatile bool releOn; // Declarar la variable global

void Rele_On(void) {
    HAL_GPIO_WritePin(RELE_GPIO_PORT, RELE_PIN, GPIO_PIN_SET);
    releOn = true; // Actualizar el estado del relé
}

void Rele_Off(void) {
    HAL_GPIO_WritePin(RELE_GPIO_PORT, RELE_PIN, GPIO_PIN_RESET);
    releOn = false; // Actualizar el estado del relé
}

void Control_Rele(float luminosity) {
    if (luminosity > 5.5) {
        Rele_On();
    } else {
        Rele_Off();
    }
}
