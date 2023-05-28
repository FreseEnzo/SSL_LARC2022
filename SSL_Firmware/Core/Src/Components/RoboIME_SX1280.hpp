/*
 * RoboIME_SX1280.hpp
 *
 *  Created on: Apr 25, 2023
 *      Author: Frese
 */

#ifndef SRC_COMPONENTS_ROBOIME_SX1280_HPP_
#define SRC_COMPONENTS_ROBOIME_SX1280_HPP_


#include "usbd_cdc_if.h"
#include "string.h"
#include <cstring>
#include "SerialDebug.hpp"
#include "sx1280/sx1280-hal.h"

class RoboIME_SX1280 {
public:
	int  setRobotId(uint8_t id);
	void GPIOCallback(void);
	void setPayload( uint8_t *buffer, uint8_t size, uint8_t offset );
	uint8_t  sendPayload (uint8_t* payload, uint8_t payloadSize);
	uint8_t receivePayload(uint8_t* payload);
	void setRX(void);
	int setupDataRadio();
	int setupFeedbackRadio();
	uint8_t receiveFeedback(uint8_t* payload);
	uint8_t  sendFeedback(void);
	void  OnTxDone( void );
	void  OnRxDone( void );
	void  OnTxTimeout( void );
	void OnRxTimeout( void );
	void  OnRxError( IrqErrorCode_t errorCode );

	static const uint8_t bufferSize = 128;
private:
	uint8_t payloadTemp[bufferSize];
		uint8_t roboId;
		uint8_t oldCount;
		PacketParams_t PacketParams;
		PacketStatus_t PacketStatus;
		ModulationParams_t ModulationParams;
		//uint8_t syncWord[5] = { 0xDD, 0xA0, 0x96, 0x69, 0xDD };

		uint16_t TxIrqMask = IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT;
		uint16_t RxIrqMask = IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT;


	//Private variables
	uint8_t spiBuf[64];
	uint8_t status;
};






#endif /* SRC_COMPONENTS_ROBOIME_SX1280_HPP_ */
