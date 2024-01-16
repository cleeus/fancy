/*
 * fancy.h
 *
 *  Created on: Jan 13, 2024
 *      Author: kai.dietrich
 */

#ifndef SRC_FANCY_H_
#define SRC_FANCY_H_

#include "main.h"

void fancy(
		TIM_HandleTypeDef *dht11_tim,
		TIM_HandleTypeDef *buzzer_tim,
		uint8_t            buzzer_tim_channel);


#endif /* SRC_FANCY_H_ */
