/*
 * Robo.hpp
 *
 *  Created on: 20 de mar de 2022
 *      Author: Vinicius Moraes
 */

#ifndef SRC_COMPONENTS_ROBO_HPP_
#define SRC_COMPONENTS_ROBO_HPP_

#include "main.h"
#include "Motor.hpp"
#include "Kick.hpp"
#include "Dribble.hpp"

class Robo {
public:
	Robo(uint8_t roboId);
	Kick *high_kick;
	Kick *low_kick;
	void high_kick_cmd(float power);
	void low_kick_cmd(float power);

    /********************************    CONTROLE   ********************************/
	int pos[4];
	float speed[4];	//velocidades desejadas para cada motor
	float real_wheel_speed[4];	//armazenar as velocidades medidas (m/s) das RODAS
	Motor* R_Motors[4];
	Kick* R_Kick;
	Dribble* R_Dribble;

    void get_wheel_speed(); //armazena as velocidades lineares das RODAS em *real_wheel_speed

    void set_robo_speed(float v_r, float v_t, float w); //converte as velocidades v_r, v_t e wR desejadas em speed[4]
    void set_robo_speed(float *v);
    void set_dribble(bool spinner);

    void control_speed(); //deve ser deletado no futuro
	void control_robo_speed(float v_r, float v_t, float w); //realiza controle PID das velocidades do robo
	void control_robo_speed(float *v);

	float robo_pid(float cp, float ci, float cd, float error, float ierror, float derror, float *last_error); //malha de controle do PID

    void set_motor_speed(uint8_t motnr, float vel);
    void set_motor_speed();

	void control_pos();

	float calc_vbat();

	/********************************     CHUTE     ********************************/
	void set_kick(float kickspeedx, float kickspeedz);
	bool hasBall(void);
	/*******************************************************************************/
private:

	uint8_t roboId;
};

#endif /* SRC_COMPONENTS_ROBO_HPP_ */
