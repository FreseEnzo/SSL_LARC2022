/*
 * Encoder.cpp
 *
 *  Created on: Mar 21, 2021
 *      Author: Gabriel Marcellino
 */

# include "Encoder.hpp"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;

Encoder::Encoder (uint8_t encoderId){
	switch(encoderId)
	{
		case 0:
		{
			encTimer = &htim3;
			encVal = &(TIM3->CNT);
			break;
		}
		case 1:
		{
			encTimer = &htim2;
			encVal = &(TIM2->CNT);
			break;
		}
		case 2:
		{
			encTimer = &htim5;
			encVal = &(TIM5->CNT);
			break;
		}
		case 3:
		{
			encTimer = &htim4;
			encVal = &(TIM4->CNT);
			break;
		}
		default:
			break;
	}
	timCntVal = 0;
	timCntPast = 0;
	*encVal = 20000;
}

int32_t Encoder::ReadEncoder(){
	/*direction = __HAL_TIM_IS_TIM_COUNTING_DOWN(encTimer); //Detectar direção de rotação do encoder. 0 contador aumenta, 1 diminui
	timCntPast = timCntVal;
	timCntVal = *encVal;
	if(direction){
		if (timCntVal <= timCntPast) {
			cntDif = timCntVal - timCntPast;
		}
		else {
			cntDif = timCntVal - timCntPast - 65535;
		}
	}
	else {
		if (timCntVal >= timCntPast) {
			cntDif = timCntVal - timCntPast;
		}
		else {
			cntDif = timCntVal - timCntPast + 65535;
		}
	}*/
	timCntVal = *encVal;
	*encVal = 20000;
	cntDif = timCntVal - 20000;
	return cntDif;
}

