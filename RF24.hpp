/*
 * RF24.hpp
 *
 *  Created on: Dec 31, 2021
 *      Author: Gabriel
 */

#ifndef SRC_COMPONENTS_RF24_HPP_
#define SRC_COMPONENTS_RF24_HPP_

#include "main.h"
#include "nRF24L01.h"
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

//1. Pinout Ports and Pin
//#define nrf_CSN_PORT		GPIOD
//#define nrf_CSN_PIN			GPIO_PIN_0

//#define nrf_CE_PORT			GPIOD
//#define nrf_CE_PIN			GPIO_PIN_1

//**** TypeDefs ****//
//1. Power Amplifier function, setPALevel()
typedef enum {
	RF24_PA_m18dB = 0,
	RF24_PA_m12dB,
	RF24_PA_m6dB,
	RF24_PA_0dB,
	RF24_PA_ERROR
}rf24_pa_dbm_e ;
//2. setDataRate() input
typedef enum {
	RF24_1MBPS = 0,
	RF24_2MBPS,
	RF24_250KBPS
}rf24_datarate_e;
//3. setCRCLength() input
typedef enum {
	RF24_CRC_DISABLED = 0,
	RF24_CRC_8,
	RF24_CRC_16
}rf24_crclength_e;
//4. Pipe address registers
const uint8_t ADDR_REGS[7] = {
		REG_RX_ADDR_P0,
		REG_RX_ADDR_P1,
		REG_RX_ADDR_P2,
		REG_RX_ADDR_P3,
		REG_RX_ADDR_P4,
		REG_RX_ADDR_P5,
		REG_TX_ADDR
};
//5. RX_PW_Px registers addresses
const uint8_t RF24_RX_PW_PIPE[6] = {
		REG_RX_PW_P0,
		REG_RX_PW_P1,
		REG_RX_PW_P2,
		REG_RX_PW_P3,
		REG_RX_PW_P4,
		REG_RX_PW_P5
};

class RF24{
public:
	RF24();
	//TODO: Alguns métodos podem ser privados

	//**** Functions prototypes ****//
	//Microsecond delay function
	void DelayMicroSeconds(uint32_t uSec);

	//1. Chip Select function
	void csn(int mode);
	//2. Chip Enable
	void ce(int level);
	//3. Read single byte from a register
	uint8_t read_register(uint8_t reg);
	//4. Read multiple bytes register
	void read_registerN(uint8_t reg, uint8_t *buf, uint8_t len);
	//5. Write single byte register
	void write_register(uint8_t reg, uint8_t value);
	//6. Write multipl bytes register
	void write_registerN(uint8_t reg, const uint8_t* buf, uint8_t len);
	//7. Write transmit payload
	void write_payload(const void* buf, uint8_t len);
	//8. Read receive payload
	void read_payload(void* buf, uint8_t len);
	//9. Flush Tx buffer
	void flush_tx(void);
	//10. Flush Rx buffer
	void flush_rx(void);
	//11. Get status register value
	uint8_t get_status(void);

	//12. Begin function
	void begin(GPIO_TypeDef *nrf24CSNPORT, uint16_t nrfCSN_Pin, GPIO_TypeDef *nrf24CEPORT, uint16_t nrfCE_Pin, SPI_HandleTypeDef nrfSPI);
	//13. Listen on open pipes for reading (Must call openReadingPipe() first)
	void startListening(void);
	//14. Stop listening (essential before any write operation)
	void stopListening(void);

	//15. Write(Transmit data), returns true if successfully sent
	bool write( const void* buf, uint8_t len );
	//16. Check for available data to read
	bool available(void);
	//17. Read received data
	bool read( void* buf, uint8_t len );
	//18. Open Tx pipe for writing (Cannot perform this while Listenning, has to call stopListening)
	void openWritingPipe(uint64_t address);
	//19. Open reading pipe
	void openReadingPipe(uint8_t number, uint64_t address);
	//20 set transmit retries (rf24_Retries_e) and delay
	void setRetries(uint8_t delay, uint8_t count);
	//21. Set RF channel frequency
	void setChannel(uint8_t channel);
	//22. Set payload size
	void setPayloadSize(uint8_t size);
	//23. Get payload size
	uint8_t getPayloadSize(void);
	//24. Get dynamic payload size, of latest packet received
	uint8_t getDynamicPayloadSize(void);
	//25. Enable payload on Ackknowledge packet
	void enableAckPayload(void);
	//26. Enable dynamic payloads
	void enableDynamicPayloads(void);
	void disableDynamicPayloads(void);
	//27. Check if module is NRF24L01+ or normal module
	bool isNRF_Plus(void) ;
	//28. Set Auto Ack for all
	void setAutoAck(bool enable);
	//29. Set Auto Ack for certain pipe
	void setAutoAckPipe( uint8_t pipe, bool enable ) ;
	//30. Set transmit power level
	void setPALevel( rf24_pa_dbm_e level ) ;
	//31. Get transmit power level
	rf24_pa_dbm_e getPALevel( void ) ;
	//32. Set data rate (250 Kbps, 1Mbps, 2Mbps)
	bool setDataRate(rf24_datarate_e speed);
	//33. Get data rate
	rf24_datarate_e getDataRate( void );
	//34. Set crc length (disable, 8-bits or 16-bits)
	void setCRCLength(rf24_crclength_e length);
	//35. Get CRC length
	rf24_crclength_e getCRCLength(void);
	//36. Disable CRC
	void disableCRC( void ) ;
	//37. power up
	void powerUp(void) ;
	//38. power down
	void powerDown(void);
	//39. Check if data are available and on which pipe (Use this for multiple rx pipes)
	bool availablePipe(uint8_t* pipe_num);
	//40. Start write (for IRQ mode)
	void startWrite( const void* buf, uint8_t len );
	//41. Write acknowledge payload
	void writeAckPayload(uint8_t pipe, const void* buf, uint8_t len);
	//42. Check if an Ack payload is available
	bool isAckPayloadAvailable(void);
	//43. Check interrupt flags
	void whatHappened(bool *tx_ok,bool *tx_fail,bool *rx_ready);
	//44. Test if there is a carrier on the previous listenning period (useful to check for intereference)
	bool testCarrier(void);
	//45. Test if a signal carrier exists (=> -64dB), only for NRF24L01+
	bool testRPD(void) ;
	//46. Reset Status
	void resetStatus(void);
	//47. ACTIVATE cmd
	void ACTIVATE_cmd(void);
	//48. Get AckPayload Size
	uint8_t GetAckPayloadSize(void);

	//**********  DEBUG Functions **********//
	//1. Print radio settings
	void printRadioSettings(void);
	//2. Print Status
	void printStatusReg(void);
	//3. Print Config
	void printConfigReg(void);
	//4. Init Variables
	void DebugUART_Init(UART_HandleTypeDef nrf24Uart);
	//5. FIFO Status
	void printFIFOstatus(void);

private:
	//*** Library variables ***//
	uint64_t pipe0_reading_address;
	bool ack_payload_available; /**< Whether there is an ack payload waiting */
	uint8_t ack_payload_length; /**< Dynamic size of pending ack payload. */
	uint8_t payload_size; /**< Fixed size of payloads */
	bool dynamic_payloads_enabled; /**< Whether dynamic payloads are enabled. */
	bool p_variant; /* False for RF24L01 and true for RF24L01P */
	bool wide_band; /* 2Mbs data rate in use? */

	//*** NRF24L01 pins and handles ***//
	//CE and CSN pins
	GPIO_TypeDef			*nrf24_CSN_PORT;
	GPIO_TypeDef			*nrf24_CE_PORT;
	uint16_t				nrf24_CSN_PIN;
	uint16_t				nrf24_CE_PIN;
	//SPI handle
	SPI_HandleTypeDef nrf24_hspi;
	//Debugging UART handle
	UART_HandleTypeDef nrf24_huart;

};


#endif /* SRC_COMPONENTS_RF24_HPP_ */
