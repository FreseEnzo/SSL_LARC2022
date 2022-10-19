/*
 * RoboIME_RF24.hpp
 *
 *  Created on: Jan 1, 2022
 *      Author: Gabriel
 */

#ifndef SRC_COMPONENTS_ROBOIME_RF24_HPP_
#define SRC_COMPONENTS_ROBOIME_RF24_HPP_

#include "main.h"
#include "Defines.hpp"

typedef enum : uint8_t{
	PWRDN = 1,
	PWRUP_TX,
	PWRUP_RX
}RF24_Direction;

class RoboIME_RF24 {
public:
	RoboIME_RF24(GPIO_TypeDef* CSN_GPIO_PORT, uint16_t CSN_GPIO_PIN, GPIO_TypeDef* CE_GPIO_PORT, uint16_t CE_GPIO_PIN, GPIO_TypeDef* IRQ_GPIO_PORT, uint16_t IRQ_GPIO_PIN, SPI_HandleTypeDef* SPI_HANDLE);
	void extiCallback();
	int setup();
	int setRobotId(uint8_t id);
	int setDirection(RF24_Direction direction);
	int sendPayload(uint8_t* payload, uint8_t numBytes);
	uint8_t getReceivedPayload(uint8_t* payload);
	int UploadAckPayload(uint8_t* payload, uint8_t numBytes);
	void ce(GPIO_PinState state);

	//Setup parameters = default
	uint8_t REG_CONFIG = 0b00001000;		//Default config (power down)
	uint8_t REG_EN_AA = 0b00000001;			//Only pipe 0 auto-acknowledge
	uint8_t REG_EN_RXADDR = 0b00000001;		//Only pipe 0 enabled
	uint8_t REG_SETUP_AW = 0b00000010;		//4-byte address
	uint8_t REG_SETUP_RETR = 0b00010000;	//500us, 0 retransmit
#ifdef INTEL
	uint8_t REG_RF_CH = 53;					//2400 + 8 = 2408MHz		//Canal 8 em geral e para testes da ELO, canal 80
#else
	uint8_t REG_RF_CH = 111;					//2400 + 8 = 2408MHz		//Canal 8 em geral e para testes da ELO, canal 80
#endif
	uint8_t REG_RF_SETUP = 0b00001110;		//2mbps, 0dBm
	uint8_t REG_STATUS = 0b01111110;		//Clears all interrupts
	uint8_t REG_RX_ADDR_P0[4] =
		{0x00, 0x00, 0xE7, 0xE7};			//This robot id (LSByte first)
	uint8_t REG_TX_ADDR[4] =
		{0x00, 0x00, 0xE7, 0xE7};			//Will send to this id (LSByte first)
	uint8_t REG_RX_PW_P0 = 32;				//32 byte pipe 0 width
	uint8_t REG_DYNPD = 0b00000001;			//Dynamic payload size enabled
	uint8_t REG_FEATURE = 0b00000111;		//Features

	//Attributes
	bool ready = false;

private:
	//Ports and pins
	GPIO_TypeDef* CSN_PORT;
	uint16_t CSN_PIN;
	GPIO_TypeDef* CE_PORT;
	uint16_t CE_PIN;
	GPIO_TypeDef* IRQ_PORT;
	uint16_t IRQ_PIN;
	SPI_HandleTypeDef* hspi;

	//Private variables
	uint8_t spiBuf[64];
	uint8_t status;

	//Methods
	void delayMicroseconds(uint32_t delay);
	void csn(GPIO_PinState state);
	uint8_t readRxPayloadWidth(void);
	int spiCommand(uint8_t command);
	int writeRegister(uint8_t regAddr, uint8_t* data, uint8_t length);
	int readRegister(uint8_t regAddr, uint8_t* data, uint8_t length);
	int writeAckPayload(uint8_t* payload, uint8_t numBytes);
	int readRxPayload(uint8_t* payload, uint8_t numBytes);
	int writeTxPayload(uint8_t* payload, uint8_t numBytes);
};

#endif /* SRC_COMPONENTS_ROBOIME_RF24_HPP_ */
