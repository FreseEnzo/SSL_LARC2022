/*
 * RoboIME_RF24.cpp
 *
 *  Created on: Jan 1, 2022
 *      Author: Gabriel
 */

#include "RoboIME_RF24.hpp"

#include <cstring>

//Constructors
RoboIME_RF24::RoboIME_RF24(
		GPIO_TypeDef* CSN_GPIO_PORT, uint16_t CSN_GPIO_PIN,
		GPIO_TypeDef* CE_GPIO_PORT, uint16_t CE_GPIO_PIN,
		GPIO_TypeDef* IRQ_GPIO_PORT, uint16_t IRQ_GPIO_PIN,
		SPI_HandleTypeDef* SPI_HANDLE
		)
: CSN_PORT(CSN_GPIO_PORT), CSN_PIN(CSN_GPIO_PIN),
  CE_PORT(CE_GPIO_PORT), CE_PIN(CE_GPIO_PIN),
  IRQ_PORT(IRQ_GPIO_PORT), IRQ_PIN(IRQ_GPIO_PIN),
  hspi(SPI_HANDLE)
{

}

//Public methods
void RoboIME_RF24::extiCallback(){
	uint8_t mask = 0b01111110;
	writeRegister(0x07, &mask, 1);
	if (status & 0b00010000){
		csn(GPIO_PIN_RESET);
		spiCommand(0b11100001);	//FLUSH_TX
		csn(GPIO_PIN_SET);
	}
}

int RoboIME_RF24::setup(){
	writeRegister(0x00, &REG_CONFIG, 1);
	writeRegister(0x01, &REG_EN_AA, 1);
	writeRegister(0x02, &REG_EN_RXADDR, 1);
	writeRegister(0x03, &REG_SETUP_AW, 1);
	writeRegister(0x04, &REG_SETUP_RETR, 1);
	writeRegister(0x05, &REG_RF_CH, 1);
	writeRegister(0x06, &REG_RF_SETUP, 1);
	writeRegister(0x07, &REG_STATUS, 1);
	/*writeRegister(0x0A, REG_RX_ADDR_P0, 4);
	writeRegister(0x10, REG_TX_ADDR, 4);*/
	writeRegister(0x11, &REG_RX_PW_P0, 1);
	writeRegister(0x1C, &REG_DYNPD, 1);
	writeRegister(0x1D, &REG_FEATURE, 1);
	return 0;
}

int RoboIME_RF24::setRobotId(uint8_t id){
	REG_RX_ADDR_P0[0] = id;
	REG_TX_ADDR[0] = id;
	writeRegister(0x0A, REG_RX_ADDR_P0, 4);
	writeRegister(0x10, REG_TX_ADDR, 4);
	return 0;
}

int RoboIME_RF24::setDirection(RF24_Direction direction){
	REG_CONFIG &= 0b11111100;
	REG_CONFIG |= direction;
	writeRegister(0x00, &REG_CONFIG, 1);
	if(direction == PWRUP_RX){
		ce(GPIO_PIN_SET);
	}
	return 0;
}

int RoboIME_RF24::sendPayload(uint8_t* payload, uint8_t numBytes){
	csn(GPIO_PIN_RESET);
	spiCommand(0b11100001);	//FLUSH_TX
	csn(GPIO_PIN_SET);
	delayMicroseconds(1);
	writeTxPayload(payload, numBytes);
	ce(GPIO_PIN_SET);
	delayMicroseconds(10);
	//ce(GPIO_PIN_RESET); //nRF da SSL só ativa o LNA quando CE=1 (não respeita o datasheet do chip)
	return numBytes;
}

uint8_t RoboIME_RF24::getReceivedPayload(uint8_t* payload){
	uint8_t availableBytes = readRxPayloadWidth();
	delayMicroseconds(1);
	if(availableBytes > 32){
		csn(GPIO_PIN_RESET);
		spiCommand(0b11100010);	//FLUSH_RX
		csn(GPIO_PIN_SET);
	}else if(availableBytes){
		readRxPayload(payload, availableBytes);
	}
	return availableBytes;
}

int RoboIME_RF24::UploadAckPayload(uint8_t* payload, uint8_t numBytes){
	csn(GPIO_PIN_RESET);
	spiCommand(0b11100001);	//FLUSH_TX
	csn(GPIO_PIN_SET);
	delayMicroseconds(1);
	writeAckPayload(payload, numBytes);
	delayMicroseconds(1);
	return numBytes;
}

//Private methods
void RoboIME_RF24::delayMicroseconds(uint32_t delay){
	uint32_t start = DWT->CYCCNT;
	while(DWT->CYCCNT - start < delay*(HAL_RCC_GetHCLKFreq()/1000000));
}

inline void RoboIME_RF24::csn(GPIO_PinState state){
	HAL_GPIO_WritePin(CSN_PORT, CSN_PIN, state);
}

inline void RoboIME_RF24::ce(GPIO_PinState state){
	HAL_GPIO_WritePin(CE_PORT, CE_PIN, state);
}

uint8_t RoboIME_RF24::readRxPayloadWidth(void){
	uint8_t availableBytes = 0;
	csn(GPIO_PIN_RESET);
	spiCommand(0b01100000);	//R_RX_PL_WID
	if(!(status & 0b00001110)){
		HAL_SPI_Receive_IT(hspi, &availableBytes, 1);
		while (hspi->State == HAL_SPI_STATE_BUSY_RX);
	}
	csn(GPIO_PIN_SET);
	return availableBytes;
}

int RoboIME_RF24::spiCommand(uint8_t command){
	HAL_SPI_TransmitReceive_IT(hspi, &command, &status, 1);
	while (hspi->State == HAL_SPI_STATE_BUSY_TX_RX);
	return 0;
}

int RoboIME_RF24::writeRegister(uint8_t regAddr, uint8_t* data, uint8_t length){
	//Least significant byte first
	if(regAddr > 31){
		return 1;
	}else if(length > 5){
		return 2;
	}
	/*spiBuf[0] = 0x20 | regAddr;
	memcpy(&spiBuf[1], data, length);
	csn(GPIO_PIN_RESET);
	HAL_SPI_Transmit_IT(hspi, spiBuf, length + 1);
	while (hspi->State == HAL_SPI_STATE_BUSY_TX);
	csn(GPIO_PIN_SET);*/
	spiBuf[0] = 0x20 | regAddr;
	memcpy(&spiBuf[1], data, length);
	csn(GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive_IT(hspi, spiBuf, &status, 1);
	while (hspi->State == HAL_SPI_STATE_BUSY_TX_RX);
	HAL_SPI_Transmit_IT(hspi, spiBuf+1, length);
	while (hspi->State == HAL_SPI_STATE_BUSY_TX);
	csn(GPIO_PIN_SET);
	return 0;
}

int RoboIME_RF24::readRegister(uint8_t regAddr, uint8_t* data, uint8_t length){
	if(regAddr > 31){
		return 1;
	}else if(length > 5){
		return 2;
	}
	spiBuf[0] = regAddr;
	csn(GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive_IT(hspi, spiBuf, &status, 1);
	while (hspi->State == HAL_SPI_STATE_BUSY_TX_RX);
	HAL_SPI_Receive_IT(hspi, data, length);
	while (hspi->State == HAL_SPI_STATE_BUSY_RX);
	csn(GPIO_PIN_SET);
	return 0;
}

int RoboIME_RF24::writeAckPayload(uint8_t* payload, uint8_t numBytes){
	csn(GPIO_PIN_RESET);
	spiCommand(0b10101000);	//W_ACK_PAYLOAD pipe 0
	HAL_SPI_Transmit_IT(hspi, payload, numBytes);
	while (hspi->State == HAL_SPI_STATE_BUSY_TX);
	csn(GPIO_PIN_SET);
	return numBytes;
}

int RoboIME_RF24::readRxPayload(uint8_t* payload, uint8_t numBytes){
	csn(GPIO_PIN_RESET);
	spiCommand(0b01100001);	//R_RX_PAYLOAD
	HAL_SPI_Receive_IT(hspi, payload, numBytes);
	while (hspi->State == HAL_SPI_STATE_BUSY_RX);
	csn(GPIO_PIN_SET);
	return numBytes;
}

int RoboIME_RF24::writeTxPayload(uint8_t* payload, uint8_t numBytes){
	csn(GPIO_PIN_RESET);
	spiCommand(0b10100000);	//W_TX_PAYLOAD
	HAL_SPI_Transmit_IT(hspi, payload, numBytes);
	while (hspi->State == HAL_SPI_STATE_BUSY_TX);
	csn(GPIO_PIN_SET);
	return numBytes;
}
