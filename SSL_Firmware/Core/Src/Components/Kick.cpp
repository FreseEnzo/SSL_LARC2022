/*
 * Kick.cpp
 *
 *  Created on: 20 de mar de 2022
 *      Author: Vinicius Moraes
 *
 *
 *     Modified by Léo
 *
 */

#include "Kick.hpp"
#include "Defines.hpp"
#include <math.h>


Kick::Kick(GPIO_TypeDef* _KICK_H_GPIO_Port, GPIO_TypeDef* _KICK_L_GPIO_Port, GPIO_TypeDef* _KICK_C_GPIO_Port, uint16_t _KICK_H_Pin,uint16_t _KICK_L_Pin,uint16_t _KICK_C_Pin, TIM_HandleTypeDef* _KICK_HL_TIM, TIM_HandleTypeDef* _KICK_RC_TIM, TIM_HandleTypeDef* _KICK_C_TIM, uint32_t _rechargeTime){
	KICK_H_GPIO_Port = _KICK_H_GPIO_Port;
	KICK_L_GPIO_Port = _KICK_L_GPIO_Port;
	KICK_C_GPIO_Port = _KICK_C_GPIO_Port;
	KICK_HL_TIM = _KICK_HL_TIM;
	KICK_RC_TIM = _KICK_RC_TIM;
	KICK_C_TIM = _KICK_C_TIM;
	KICK_H_Pin = _KICK_H_Pin;
	KICK_L_Pin = _KICK_L_Pin;
	KICK_C_Pin = _KICK_C_Pin;
	kickCharged = GPIO_PIN_RESET;
	kickEnable = GPIO_PIN_RESET;
	KICK_RC_TIM->Instance->ARR=10000*_rechargeTime;
}

void Kick::SetKick(float kickspeedx, float kickspeedz){			//Argumento em decimetros por segundo
	if(kickspeedz > 0){
#ifdef KICKTIME
		KickHigh(kickspeedz);				//kickspeed = tempo de chute em seg
#else
		//conversao speed->time
		if(kickspeedx >= 23.85){
			KickHigh((6-(log(34-kickspeedx*1.41421356237)/log(2))));	//argumento em milisegundos
		}else{
			KickHigh(10);
		}

#endif

	}else if(kickspeedx > 0.1 && kickspeedx < 33.75){
#ifdef KICKTIME
		KickLow(kickspeedx);				//kickspeed = tempo de chute em mseg
#else
		//conversao speed->time
		//kickpower = 6-log2(34-kickspeed))
		//kickpower em ms e kickspeed em dm/s
		KickLow(6-(log(34-kickspeedx)/log(2)));							//argumento em milisegundos
	}else if(kickspeedx >= 33.75){
		//força máxima
		KickLow(10);
#endif
	}
}

void Kick::KickHigh(float kickPower){
	if(!kickEnable && HAL_GPIO_ReadPin(S1_GPIO_Port, S1_Pin)){
		HAL_GPIO_WritePin(KICK_C_GPIO_Port, KICK_C_Pin, GPIO_PinState(RESET));

		kickCharged = GPIO_PIN_RESET;

		kickEnable = GPIO_PIN_SET;

		KICK_HL_TIM->Instance->ARR=10*kickPower;

		__HAL_TIM_SET_COUNTER(KICK_HL_TIM,0);
		__HAL_TIM_SET_COUNTER(KICK_RC_TIM,0);

		HAL_GPIO_WritePin(KICK_H_GPIO_Port, KICK_H_Pin, GPIO_PinState(SET));
	}
}

void Kick::KickLow(float kickPower){
	if(!kickEnable && HAL_GPIO_ReadPin(S1_GPIO_Port, S1_Pin)){
		HAL_GPIO_WritePin(CHIP_KICK_GPIO_Port, CHIP_KICK_Pin, GPIO_PinState(RESET));

		KICK_HL_TIM->Instance->ARR=10*kickPower;

		__HAL_TIM_SET_COUNTER(KICK_HL_TIM,0);

		__HAL_TIM_SET_COUNTER(KICK_RC_TIM,0);
		kickCharged = GPIO_PIN_RESET;
		kickEnable=GPIO_PIN_SET;

		HAL_GPIO_WritePin(KICK_L_GPIO_Port, KICK_L_Pin, GPIO_PinState(SET));
	}
}

void Kick::Charge(float chargeTime){
	__HAL_TIM_SET_COUNTER(KICK_C_TIM,0);
	KICK_C_TIM->Instance->ARR=10000*chargeTime;
	HAL_GPIO_WritePin(KICK_C_GPIO_Port, KICK_C_Pin, GPIO_PinState(SET));
}
