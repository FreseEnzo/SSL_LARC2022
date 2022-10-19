/*
 * motor.c
 *
 *  Created on: Mar 13, 2021
 *      Author: Moraes
 */

#include "motor_c.h"

//spd>0 A em VCC e B em GND

void setMotorSpd(int32_t spd, uint8_t motorId){
	switch (motorId){
		case 0:
			if(spd>0){
				TIM8->CCR4 = TIM8->ARR - spd; //MAH
				TIM8->CCR2 = TIM8->ARR; //MBH
				HAL_GPIO_WritePin(M0_MAL_GPIO_Port, M0_MAL_Pin, RESET);
				HAL_GPIO_WritePin(M0_MBL_GPIO_Port, M0_MBL_Pin, SET);
			}else if(spd<0){
				TIM8->CCR4 = TIM8->ARR; //MAH
				TIM8->CCR2 =TIM8->ARR + spd; //MBH
				HAL_GPIO_WritePin(M0_MAL_GPIO_Port, M0_MAL_Pin, SET);
				HAL_GPIO_WritePin(M0_MBL_GPIO_Port, M0_MBL_Pin, RESET);
			}else{
				TIM8->CCR4 = TIM8->ARR; //MAH
				TIM8->CCR2 = TIM8->ARR; //MBH
				HAL_GPIO_WritePin(M0_MAL_GPIO_Port, M0_MAL_Pin, SET);
				HAL_GPIO_WritePin(M0_MBL_GPIO_Port, M0_MBL_Pin, SET);
			}
			break;
		case 1:
			if(spd>0){
				TIM1->CCR1 = TIM1->ARR - spd; //MAH
				TIM8->CCR3 = TIM8->ARR; //MBH
				HAL_GPIO_WritePin(M1_MAL_GPIO_Port, M1_MAL_Pin, RESET);
				HAL_GPIO_WritePin(M1_MBL_GPIO_Port, M1_MBL_Pin, SET);
			}else if(spd<0){
				TIM1->CCR1 = TIM1->ARR; //MAH
				TIM8->CCR3 =TIM8->ARR + spd; //MBH
				HAL_GPIO_WritePin(M1_MAL_GPIO_Port, M1_MAL_Pin, SET);
				HAL_GPIO_WritePin(M1_MBL_GPIO_Port, M1_MBL_Pin, RESET);
			}else{
				TIM1->CCR1 = TIM1->ARR; //MAH
				TIM8->CCR3 = TIM8->ARR; //MBH
				HAL_GPIO_WritePin(M1_MAL_GPIO_Port, M1_MAL_Pin, SET);
				HAL_GPIO_WritePin(M1_MBL_GPIO_Port, M1_MBL_Pin, SET);
			}
			break;
		case 2:
			break;
		case 3:
			break;
		case 4:
			break;
		default:
			break;
	}
}
