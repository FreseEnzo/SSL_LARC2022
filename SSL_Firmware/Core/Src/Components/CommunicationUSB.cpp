/*
 * ComunicationUSB.cpp
 *
 *  Created on: Mar 21, 2021
 *      Author: Gabriel Marcellino
 */

#include "main.h"
#include "Start.hpp"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "CommunicationUSB.hpp"
#include "grSim_Commands.pb.h"
#include "Feedback.pb.h"
#include "pb_decode.h"
#include "pb_encode.h"

extern grSim_Robot_Command receivedPacket;
extern Feedback sendPacket[];

/**
 * @param _usbRecvCallback pointer to global variable which is a function pointer to be called inside usbd_cdc_if.h
 */
CommunicationUSB::CommunicationUSB(void (**_usbRecvCallback)(uint8_t*, uint32_t*))
{
	*_usbRecvCallback = &CommunicationUSB::ReceiveCallback;
}

void CommunicationUSB::ReceiveCallback(uint8_t* Buf, uint32_t* Len){
	pb_istream_t stream = pb_istream_from_buffer(Buf, *Len);
	pb_decode(&stream, grSim_Robot_Command_fields, &receivedPacket);
	USBpacketReceivedCallback();
}

void CommunicationUSB::TransmitFeedbackPacket(uint32_t i, uint32_t id){
	pb_ostream_t stream = pb_ostream_from_buffer(sendBuffer[id], 64);
	pb_encode(&stream, Feedback_fields, &sendPacket[i]);
	CDC_Transmit_FS(sendBuffer[id], stream.bytes_written);
}

void CommunicationUSB::TransmitEncoderReadingRPM(int32_t reading){
	float readingRPM = (float)(reading)/(float)1; //Explicar no comentário

	/*sendPacket.id = receivedPacket.id;
	sendPacket.status = 1;
	sendPacket.battery = 20.0;
	sendPacket.encoder1 = readingRPM;
	sendPacket.encoder2 = receivedPacket.velangular;
	sendPacket.encoder3 = 0;
	sendPacket.encoder4 = 0;*/
}


