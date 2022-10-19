/*
 * Dribble.hpp
 *
 *  Created on: Mar 20, 2022
 *      Author: Vinicius Moraes
 */

#ifndef SRC_COMPONENTS_DRIBBLE_HPP_
#define SRC_COMPONENTS_DRIBBLE_HPP_

#include "main.h"
#include "Defines.hpp"

class Dribble {
public:
	Dribble();
	void SetSpeed(bool spinner);

private:
	__IO uint32_t* MD_Pwm;
	uint32_t Pwm_Max;
	static float duty;
};

#endif /* SRC_COMPONENTS_DRIBBLE_HPP_ */
