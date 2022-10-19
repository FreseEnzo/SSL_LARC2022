/*
 * Dribble.cpp
 *
 *  Created on: Mar 20, 2022
 *      Author: Vinicius Moraes
 */

#include "Dribble.hpp"

#ifdef DEEPWEB
float Dribble::duty=0.81;     //0.375 para a bateria de 12V
#else
float Dribble::duty=0.81;
#endif


Dribble::Dribble() {
	MD_Pwm = &(TIM9->CCR1);
}

void Dribble::SetSpeed(bool spinner){
	Pwm_Max = TIM9->ARR;
	if(spinner){
		*MD_Pwm = (1-duty)*Pwm_Max; //O drible só tem um pino e a spd é cte
	}else{
		*MD_Pwm = Pwm_Max;
	}
}
