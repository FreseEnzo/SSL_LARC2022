/*
 * UARTDebug.hpp
 *
 *  Created on: Feb 28, 2022
 *      Author: Gabriel
 */

#ifndef SRC_COMPONENTS_SERIALDEBUG_HPP_
#define SRC_COMPONENTS_SERIALDEBUG_HPP_

#include "main.h"
#include <cstdio>

class SerialDebug
{
public:
	SerialDebug(UART_HandleTypeDef* huartptr);
	void setLevel(uint32_t level);
	void debug(const char* data);
	void info(const char* data);
	void warn(const char* data);
	void error(const char* data);

	enum debugLevel{
		DEBUG_LEVEL_DEBUG,
		DEBUG_LEVEL_INFO,
		DEBUG_LEVEL_WARN,
		DEBUG_LEVEL_ERROR
	};
private:
	UART_HandleTypeDef* huartptr;
	uint8_t uartBuf[64];
	uint32_t debugLevel = DEBUG_LEVEL_ERROR;
};






#endif /* SRC_COMPONENTS_SERIALDEBUG_HPP_ */
