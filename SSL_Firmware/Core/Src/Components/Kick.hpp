/*
 * Kick.hpp
 *
 *  Created on: 20 de mar de 2022
 *      Author: Vinicius Moraes
 *
 *      Modified by Léo
 */

#ifndef SRC_COMPONENTS_KICK_HPP_
#define SRC_COMPONENTS_KICK_HPP_

#include "main.h"

class Kick {
	public:
	Kick(GPIO_TypeDef* _KICK_H_GPIO_Port, GPIO_TypeDef* _KICK_L_GPIO_Port, GPIO_TypeDef* _KICK_C_GPIO_Port, uint16_t _KICK_H_Pin,uint16_t _KICK_L_Pin,uint16_t _KICK_C_Pin, TIM_HandleTypeDef* _KICK_HL_TIM, TIM_HandleTypeDef* _KICK_RC_TIM, TIM_HandleTypeDef* _KICK_C_TIM, uint32_t _rechargeTime);
		void SetKick(float kickspeedx, float kickspeedz);
		void Charge(float chargeTime);
		GPIO_PinState kickCharged;
		GPIO_PinState kickEnable;
		GPIO_TypeDef* KICK_H_GPIO_Port;
		GPIO_TypeDef* KICK_L_GPIO_Port;
		GPIO_TypeDef* KICK_C_GPIO_Port;
		TIM_HandleTypeDef* KICK_HL_TIM;
		TIM_HandleTypeDef* KICK_RC_TIM;
		TIM_HandleTypeDef* KICK_C_TIM;
		uint16_t KICK_H_Pin;
		uint16_t KICK_L_Pin;
		uint16_t KICK_C_Pin;
	private:
		void KickHigh(float kickPower);
		void KickLow(float kickPower);
};

#endif /* SRC_COMPONENTS_KICK_HPP_ */
