/*
 * ComunicationUSB.hpp
 *
 *  Created on: Mar 21, 2021
 *      Author: Gabriel Marcellino
 */

#ifndef SRC_COMPONENTS_COMMUNICATIONUSB_HPP_
#define SRC_COMPONENTS_COMMUNICATIONUSB_HPP_

#include "main.h"
#include "Start.hpp"

class CommunicationUSB{
public:
	CommunicationUSB(void (**usbRecvCallback)(uint8_t*, uint32_t*));
	void TransmitEncoderReadingRPM(int32_t reading);
	void TransmitFeedbackPacket(uint32_t i, uint32_t id);
	static void ReceiveCallback(uint8_t* Buf, uint32_t* Len);
private:
	uint8_t sendBuffer[16][64];
};



#endif /* SRC_COMPONENTS_COMMUNICATIONUSB_HPP_ */
