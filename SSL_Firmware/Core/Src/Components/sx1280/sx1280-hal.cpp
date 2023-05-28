/*
  ______                              _
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2016 Semtech

Description: Handling of the node configuration protocol

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis, Gregory Cristian and Matthieu Verdy
*/
#include "sx1280-hal.h"

/*!
 * \brief Helper macro to create Interrupt objects only if the pin name is
 *        different from NC
 */
#define CreateDioPin( pinName, dio )                 \
            if( pinName == NC )                      \
            {                                        \
                dio = NULL;                          \
            }                                        \
            else                                     \
            {                                        \
                dio = new InterruptIn( pinName );    \
            }

/*!
 * \brief Used to block execution waiting for low state on radio busy pin.
 *        Essentially used in SPI communications
 */
#define WaitOnBusy( )          while( HAL_GPIO_ReadPin(BUSYPort, BUSYPin) ){ }

// This code handles cases where assert_param is undefined
#ifndef assert_param
#define assert_param( ... )
#endif

SX1280Hal::SX1280Hal( SPI_HandleTypeDef* hspi,
					  GPIO_TypeDef* nssPort, uint16_t nssPin,
                      GPIO_TypeDef* busyPort, uint16_t busyPin,
					  GPIO_TypeDef* rstPort, uint16_t rstPin,
                      RadioCallbacks_t *callbacks )
        :   SX1280( callbacks ),
            RadioNssPort( nssPort ), RadioNssPin( nssPin ),
            RadioResetPort( rstPort ), RadioResetPin( rstPin ),
            RadioCtsnPort( nullptr ), RadioCtsnPin( 0 ),
            BUSYPort( busyPort ), BUSYPin( busyPin )
{
    RadioSpi = hspi;

    //HAL_GPIO_WritePin(RadioNssPort, RadioNssPin, GPIO_PIN_SET);
    //HAL_GPIO_WritePin(RadioResetPort, RadioResetPin, GPIO_PIN_SET);
}

SX1280Hal::~SX1280Hal( void )
{
};

void SX1280Hal::IoIrqInit( DioIrqHandler irqHandler )
{
    assert_param( RadioSpi != NULL);
    // BUSY.mode( PullNone );
    //Conexão entre interrupt do HAL e da biblioteca
    //irqHandler é o ponteiro para a função que deve ser chamada em caso de interrupt
    RadioIrqHandler = irqHandler;
}

void SX1280Hal::Reset( void )
{
	//Por que desliga o IRQ?
	/* Habilitar o delay de outra forma*/
    //__disable_irq( );
    HAL_Delay( 20 );
    HAL_GPIO_WritePin(RadioResetPort, RadioResetPin, GPIO_PIN_RESET);
    HAL_Delay( 50 );
    HAL_GPIO_WritePin(RadioResetPort, RadioResetPin, GPIO_PIN_SET);
    HAL_Delay( 20 );
   // __enable_irq( );
}

void SX1280Hal::Wakeup( void )
{
    __disable_irq( );

    //Don't wait for BUSY here

    if( RadioSpi != NULL )
    {
        HAL_GPIO_WritePin(RadioNssPort, RadioNssPin, GPIO_PIN_RESET);
        SpiTransmitReceiveByte( RADIO_GET_STATUS );
        SpiTransmitReceiveByte( 0 );
        HAL_GPIO_WritePin(RadioNssPort, RadioNssPin, GPIO_PIN_SET);
    }

    // Wait for chip to be ready.
    WaitOnBusy( );

    __enable_irq( );
}

void SX1280Hal::WriteCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
    WaitOnBusy( );

    if( RadioSpi != NULL )
    {
        HAL_GPIO_WritePin(RadioNssPort, RadioNssPin, GPIO_PIN_RESET);
        SpiTransmitReceiveByte( ( uint8_t )command );
        for( uint16_t i = 0; i < size; i++ )
        {
            SpiTransmitReceiveByte( buffer[i] );
        }
        HAL_GPIO_WritePin(RadioNssPort, RadioNssPin, GPIO_PIN_SET);
    }

    if( command != RADIO_SET_SLEEP )
    {
        WaitOnBusy( );
    }
}

void SX1280Hal::ReadCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
    WaitOnBusy( );

    if( RadioSpi != NULL )
    {
        HAL_GPIO_WritePin(RadioNssPort, RadioNssPin, GPIO_PIN_RESET);
        if( command == RADIO_GET_STATUS )
        {
            buffer[0] = SpiTransmitReceiveByte( ( uint8_t )command );
            SpiTransmitReceiveByte( 0 );
            SpiTransmitReceiveByte( 0 );
        }
        else
        {
            SpiTransmitReceiveByte( ( uint8_t )command );
            SpiTransmitReceiveByte( 0 );
            for( uint16_t i = 0; i < size; i++ )
            {
                 buffer[i] = SpiTransmitReceiveByte( 0 );
            }
        }
        HAL_GPIO_WritePin(RadioNssPort, RadioNssPin, GPIO_PIN_SET);
    }

    WaitOnBusy( );
}

void SX1280Hal::WriteRegister( uint16_t address, uint8_t *buffer, uint16_t size )
{
    WaitOnBusy( );

    if( RadioSpi != NULL )
    {
        HAL_GPIO_WritePin(RadioNssPort, RadioNssPin, GPIO_PIN_RESET);
        SpiTransmitReceiveByte( RADIO_WRITE_REGISTER );
        SpiTransmitReceiveByte( ( address & 0xFF00 ) >> 8 );
        SpiTransmitReceiveByte( address & 0x00FF );
        for( uint16_t i = 0; i < size; i++ )
        {
            SpiTransmitReceiveByte( buffer[i] );
        }
        HAL_GPIO_WritePin(RadioNssPort, RadioNssPin, GPIO_PIN_SET);
    }

    WaitOnBusy( );
}

void SX1280Hal::WriteRegister( uint16_t address, uint8_t value )
{
    WriteRegister( address, &value, 1 );
}

void SX1280Hal::ReadRegister( uint16_t address, uint8_t *buffer, uint16_t size )
{
    WaitOnBusy( );

    if( RadioSpi != NULL )
    {
        HAL_GPIO_WritePin(RadioNssPort, RadioNssPin, GPIO_PIN_RESET);
        SpiTransmitReceiveByte( RADIO_READ_REGISTER );
        SpiTransmitReceiveByte( ( address & 0xFF00 ) >> 8 );
        SpiTransmitReceiveByte( address & 0x00FF );
        SpiTransmitReceiveByte( 0 );
        for( uint16_t i = 0; i < size; i++ )
        {
            buffer[i] = SpiTransmitReceiveByte( 0 );
        }
        HAL_GPIO_WritePin(RadioNssPort, RadioNssPin, GPIO_PIN_SET);
    }

    WaitOnBusy( );
}

uint8_t SX1280Hal::ReadRegister( uint16_t address )
{
    uint8_t data;

    ReadRegister( address, &data, 1 );
    return data;
}

void SX1280Hal::WriteBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    WaitOnBusy( );

    if( RadioSpi != NULL )
    {
        HAL_GPIO_WritePin(RadioNssPort, RadioNssPin, GPIO_PIN_RESET);
        SpiTransmitReceiveByte( RADIO_WRITE_BUFFER );
        SpiTransmitReceiveByte( offset );
        for( uint16_t i = 0; i < size; i++ )
        {
            SpiTransmitReceiveByte( buffer[i] );
        }
        HAL_GPIO_WritePin(RadioNssPort, RadioNssPin, GPIO_PIN_SET);
    }

    WaitOnBusy( );
}

void SX1280Hal::ReadBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    WaitOnBusy( );

    if( RadioSpi != NULL )
    {
        HAL_GPIO_WritePin(RadioNssPort, RadioNssPin, GPIO_PIN_RESET);
        SpiTransmitReceiveByte( RADIO_READ_BUFFER );
        SpiTransmitReceiveByte( offset );
        SpiTransmitReceiveByte( 0 );
        for( uint16_t i = 0; i < size; i++ )
        {
            buffer[i] = SpiTransmitReceiveByte( 0 );
        }
        HAL_GPIO_WritePin(RadioNssPort, RadioNssPin, GPIO_PIN_SET);
    }

    WaitOnBusy( );
}

uint8_t SX1280Hal::SpiTransmitReceiveByte( uint8_t byte )
{
	//Se for IT ou DMA a função acaba antes de terminar de transmitir
	uint8_t receivedByte;
	HAL_SPI_TransmitReceive(RadioSpi, &byte, &receivedByte, 1, 100);
	return receivedByte;
}

void SX1280Hal::HalInterruptCallback(void){
	if(RadioIrqHandler != NULL){
		(this->*RadioIrqHandler)();
	}
}

/*
uint8_t SX1280Hal::GetDioStatus( void )
//Conferir o que deve retornar
{
    return ( *DIO3 << 3 ) | ( *DIO2 << 2 ) | ( *DIO1 << 1 ) | ( BUSY << 0 );
}
*/
