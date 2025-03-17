/*
 * rele.h
 *
 *  Created on: Mar 16, 2025
 *      Author: carlo
 */

#ifndef INC_RELE_H_
#define INC_RELE_H_

#include "stm32f4xx_hal.h"

// Define el pin y el puerto del rele
#define RELE_PIN GPIO_PIN_5
#define RELE_GPIO_PORT GPIOB

void Rele_On(void);
void Rele_Off(void);
void Control_Rele(float luminosity);

#endif /* INC_RELE_H_ */
