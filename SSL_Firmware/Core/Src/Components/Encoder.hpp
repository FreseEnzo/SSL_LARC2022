/*
 * Encoder.hpp
 *
 *  Created on: Mar 21, 2021
 *      Author: Gabriel Marcellino
 */

#ifndef SRC_COMPONENTS_ENCODER_HPP_
#define SRC_COMPONENTS_ENCODER_HPP_

#include "main.h"

class Encoder{
public:
	Encoder(uint8_t encoderId);
	int32_t ReadEncoder();
private:
	TIM_HandleTypeDef* encTimer;
	volatile int32_t cntDif;
	volatile uint32_t direction;
	volatile uint32_t* encVal;
	volatile uint32_t timCntVal;
	volatile uint32_t timCntPast;
};



#endif /* SRC_COMPONENTS_ENCODER_HPP_ */
