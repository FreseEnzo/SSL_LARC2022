/*
 * CommunicationNRF.hpp
 *
 *  Created on: Mar 22, 2022
 *      Author: Gabriel
 */

#ifndef SRC_COMPONENTS_COMMUNICATIONNRF_HPP_
#define SRC_COMPONENTS_COMMUNICATIONNRF_HPP_

//Type definitions
class nRF_Send_Packet_t{
public:
	float kickspeedx;
	float kickspeedz;
	float veltangent;
	float velnormal;
	float velangular;
	bool spinner;
	uint8_t packetId = 0;
};
class nRF_Feedback_Packet_t{
public:
	uint32_t status = 0;
	float battery;
	float encoder1;
	float encoder2;
	float encoder3;
	float encoder4;
	uint8_t packetId = 0;
};

#endif /* SRC_COMPONENTS_COMMUNICATIONNRF_HPP_ */
