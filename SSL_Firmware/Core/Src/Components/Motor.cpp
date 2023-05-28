/*
 * Motor.cpp
 *
 *  Created on: 20 de mar de 2021
 *      Author: Moraes
 */

#include "Motor.hpp"

#ifdef DEEPWEB
	float Motor::cp=(1)*65536;
	float Motor::ci=(0.2)*65536;
	float Motor::cd=(0.5)*65536; //0.3
	float Motor::cl=(0.15)*65536; // 0.15
#else
	float Motor::cp=(10000.0f/10000)*65536;           //Valores do código antigo
	float Motor::ci=(1500.0f/10000)*65536;
	float Motor::cd=(10000.0f/10000)*65536;
	float Motor::cl=(0.36)*65536;
#endif
extern SX1280_Feedback_Packet_t SX1280_Feedback_Packet;

Motor::Motor (uint8_t motorId){
	motorId_attrib = motorId;
	M_Enc =  new Encoder(motorId);
	switch(motorId){
		case 0:
		{
			MAL_GPIO_Port = M0_MAL_GPIO_Port;
			MBL_GPIO_Port = M0_MBL_GPIO_Port;
			MAL_Pin = M0_MAL_Pin;
			MBL_Pin = M0_MBL_Pin;
			MAH_Pwm = &(TIM8->CCR4);
			MBH_Pwm = &(TIM8->CCR2);
			break;
		}
		case 1:
		{
			MAL_GPIO_Port = M1_MAL_GPIO_Port;
			MBL_GPIO_Port = M1_MBL_GPIO_Port;
			MAL_Pin = M1_MAL_Pin;
			MBL_Pin = M1_MBL_Pin;
			MAH_Pwm = &(TIM1->CCR1);
			MBH_Pwm = &(TIM8->CCR3);
			break;
		}

		case 2:
		{
			MAL_GPIO_Port = M2_MAL_GPIO_Port;
			MBL_GPIO_Port = M2_MBL_GPIO_Port;
			MAL_Pin = M2_MAL_Pin;
			MBL_Pin = M2_MBL_Pin;
			MAH_Pwm = &(TIM8->CCR1);
			MBH_Pwm = &(TIM1->CCR2);
			break;
		}
		case 3:
		{
			MAL_GPIO_Port = M3_MAL_GPIO_Port;
			MBL_GPIO_Port = M3_MBL_GPIO_Port;
			MAL_Pin = M3_MAL_Pin;
			MBL_Pin = M3_MBL_Pin;
			MAH_Pwm = &(TIM1->CCR4);
			MBH_Pwm = &(TIM1->CCR3);
			break;
		}
		default:
			break;
	}
}


void Motor::SetSpeed(int32_t spd){
	Pwm_Max = TIM8->ARR;
	if(speed_anterior*spd<=0){
//		TIM8-> CNT &= 0x0; // restart the timer
		speed_anterior = spd;
		if(spd>0){
			HAL_GPIO_WritePin(MAL_GPIO_Port, MAL_Pin, GPIO_PinState(RESET));
				*MBH_Pwm = Pwm_Max; //MBH
				TIM8-> CNT = Pwm_Max; // restart the timer
				TIM1-> CNT = Pwm_Max; // restart the timer
			HAL_GPIO_WritePin(MBL_GPIO_Port, MBL_Pin, GPIO_PinState(SET));
				*MAH_Pwm = Pwm_Max - spd; //MAH
		}else if(spd<0){
				HAL_GPIO_WritePin(MBL_GPIO_Port, MBL_Pin, GPIO_PinState(RESET));
				*MAH_Pwm = Pwm_Max; //MAH
				TIM8-> CNT = Pwm_Max; // restart the timer
				TIM1-> CNT = Pwm_Max; // restart the timer
				HAL_GPIO_WritePin(MAL_GPIO_Port, MAL_Pin, GPIO_PinState(SET));
				*MBH_Pwm = Pwm_Max + spd; //MBH
		}else{
			*MBH_Pwm = Pwm_Max; //MBH
			*MAH_Pwm = Pwm_Max; //MAH
			TIM8-> CNT = Pwm_Max; // restart the timer
			TIM1-> CNT = Pwm_Max; // restart the timer
			HAL_GPIO_WritePin(MBL_GPIO_Port, MBL_Pin, GPIO_PinState(SET));
			HAL_GPIO_WritePin(MAL_GPIO_Port, MAL_Pin, GPIO_PinState(SET));
			}
	}
	else{
		speed_anterior = spd;
		if(spd>0){
					HAL_GPIO_WritePin(MAL_GPIO_Port, MAL_Pin, GPIO_PinState(RESET));
							*MBH_Pwm = Pwm_Max; //MBH
					HAL_GPIO_WritePin(MBL_GPIO_Port, MBL_Pin, GPIO_PinState(SET));
							*MAH_Pwm = Pwm_Max - spd; //MAH
			}else if(spd<0){
						HAL_GPIO_WritePin(MBL_GPIO_Port, MBL_Pin, GPIO_PinState(RESET));
						*MAH_Pwm = Pwm_Max; //MAH
						HAL_GPIO_WritePin(MAL_GPIO_Port, MAL_Pin, GPIO_PinState(SET));
						*MBH_Pwm = Pwm_Max + spd; //MBH
			}else{
				*MBH_Pwm = Pwm_Max; //MBH
				*MAH_Pwm = Pwm_Max; //MAH
				HAL_GPIO_WritePin(MBL_GPIO_Port, MBL_Pin, GPIO_PinState(SET));
				HAL_GPIO_WritePin(MAL_GPIO_Port, MAL_Pin, GPIO_PinState(SET));
			}
	}

}


void Motor::GetSpeed(){
#ifdef DEEPWEB
	int32_t distance=M_Enc->ReadEncoder();
#else
	int32_t distance=-M_Enc->ReadEncoder();
#endif

	float speed=(float)distance*CONVERSION; //converte da unidade da roda para m/s (vel do centro da roda)
	                                     //talvez seja melhor converter de m/s pra unidade da roda
	real_wheel_speed=(float)speed;
};

void Motor::ControlSpeed(float desired_speed){
	//real_wheel_speed=0;
	last_real_wheel_speed = real_wheel_speed;
	GetSpeed();
	error = desired_speed-real_wheel_speed;
	ierror = 0;
	for(int j = 20; j >= 0; j--){
		last_error[j+1]=last_error[j];
		ierror += (j*last_error[j+1])/20;
	}
	if((cp*error +  (desired_speed/2.75)*65535) < 65535 && (cp*error +  (desired_speed/2.75)*65535) > -65535){
		last_error[0]=error;
		ierror += last_error[0];
	}else{
		last_error[0]=0;
	}
	if(ierror > 65535) ierror = 65535;
	if(ierror < -65535) ierror = -65535;

	//derror=error-last_error[1];
	derror=-(real_wheel_speed - last_real_wheel_speed);


	float out = cp*error + ci * ierror + cd * derror + cl*desired_speed; //Soma de duty cycle (linear)
	switch (motorId_attrib){
	case 0:
		SX1280_Feedback_Packet.encoder1 = real_wheel_speed;
		break;
	case 1:
		SX1280_Feedback_Packet.encoder2 = real_wheel_speed;
		break;
	case 2:
		SX1280_Feedback_Packet.encoder3 = real_wheel_speed;
		break;
	case 3:
		SX1280_Feedback_Packet.encoder4 = real_wheel_speed;
		break;
	}
#ifdef DEEPWEB
#ifdef SEMCONTROLE
	dutycycle=-(desired_speed/2.75)*65535;	//73,3333 de angular coloca duty 100%//2.75
#else
	dutycycle=-out; // -out
#endif
#else
#ifdef SEMCONTROLE
	dutycycle=(desired_speed/2.75)*65535;	//73,3333 de angular coloca duty 100%//2.75
#else
	dutycycle=out;
#endif
#endif

	if(dutycycle>65535) dutycycle=65535;
	if(dutycycle<-65535) dutycycle=-65535;
	SetSpeed(dutycycle);
};
