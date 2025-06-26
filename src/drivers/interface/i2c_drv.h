#ifndef I2C_H
#define I2C_H

#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"
/* ST includes */
#include "stm32fxxx.h"

#define I2C_NO_INTERNAL_ADDRESS   0xFFFF

typedef enum
{
  i2cAck,
  i2cNack
} I2cStatus;

typedef enum
{
  i2cWrite,
  i2cRead
} I2cDirection;

/**
 * Structure used to capture the I2C message details.  The structure is then
 * queued for processing by the I2C ISR.
 */
typedef struct _I2cMessage
{
	uint32_t         messageLength;		  //< How many bytes of data to send or received.
	uint8_t          slaveAddress;		  //< The slave address of the device on the I2C bus.
  uint8_t          nbrOfRetries;      //< The slave address of the device on the I2C bus.
	I2cDirection     direction;         //< Direction of message
  I2cStatus        status;            //< i2c status
  xQueueHandle     clientQueue;       //< Queue to send received messages to.
  bool             isInternal16bit;   //< Is internal address 16 bit. If false 8 bit.
  uint16_t         internalAddress;   //< Internal address of device.
  uint8_t          *buffer;           //< Pointer to the buffer from where data will be read for transmission, or into which received data will be placed.
} I2cMessage;

typedef struct
{
  I2C_TypeDef*        i2cPort;
  uint32_t            i2cPerif;
  uint32_t            i2cEVIRQn;
  uint32_t            i2cERIRQn;
  uint32_t            i2cClockSpeed;
  uint32_t            gpioSCLPerif;
  GPIO_TypeDef*       gpioSCLPort;
  uint32_t            gpioSCLPin;
  uint32_t            gpioSCLPinSource;
  uint32_t            gpioSDAPerif;
  GPIO_TypeDef*       gpioSDAPort;
  uint32_t            gpioSDAPin;
  uint32_t            gpioSDAPinSource;
  uint32_t            gpioAF;
  uint32_t            dmaPerif;
  uint32_t            dmaChannel;
  DMA_Stream_TypeDef* dmaRxStream;
  uint32_t            dmaRxIRQ;
  uint32_t            dmaRxTCFlag;
  uint32_t            dmaRxTEFlag;

} I2cDef;

typedef struct
{
  const I2cDef *def;                    //< Definition of the i2c
  I2cMessage txMessage;                 //< The I2C send message
  uint32_t messageIndex;                //< Index of bytes sent/received
  uint32_t nbrOfretries;                //< Retries done
  SemaphoreHandle_t isBusFreeSemaphore; //< Semaphore to block during transaction.
  StaticSemaphore_t isBusFreeSemaphoreBuffer;
  SemaphoreHandle_t isBusFreeMutex;     //< Mutex to protect bus
  StaticSemaphore_t isBusFreeMutexBuffer;
  DMA_InitTypeDef DMAStruct;            //< DMA configuration structure used during transfer setup.
} I2cDrv;

// Definitions of i2c busses found in c file.
extern I2cDrv deckBus;
extern I2cDrv sensorsBus;

/**
 * Initialize i2c peripheral as defined by static I2cDef structs.
 */
void i2cdrvInit(I2cDrv* i2c);

/**
 * Send or receive a message over the I2C bus.
 *
 * The message is synchrony by semapthore and uses interrupts to transfer the message.
 *
 * @param i2c      i2c bus to use.
 * @param message	 An I2cMessage struct containing all the i2c message
 *                 Information. Message status will be altered if nack.
 * @return         true if successful, false otherwise.
 */
bool i2cdrvMessageTransfer(I2cDrv* i2c, I2cMessage* message);


/**
 * Create a message to transfer
 *
 * @param message       pointer to message struct that will be filled in.
 * @param slaveAddress  i2c slave address
 * @param direction     i2cWrite or i2cRead
 * @param length        Length of message
 * @param buffer        pointer to buffer of send/receive data
 */
void i2cdrvCreateMessage(I2cMessage *message,
                      uint8_t  slaveAddress,
                      I2cDirection  direction,
                      uint32_t length,
                      const uint8_t  *buffer);

/**
 * Create a message to transfer with internal "reg" address. Will first do a write
 * of one or two bytes depending of IsInternal16 and then write/read the data.
 *
 * @param message       pointer to message struct that will be filled in.
 * @param slaveAddress  i2c slave address
 * @param IsInternal16  It true 16bit reg address else 8bit.
 * @param intAddress    Internal address of device.
 * @param direction     i2cWrite or i2cRead
 * @param length        Length of message
 * @param buffer        pointer to buffer of send/receive data
 */
void i2cdrvCreateMessageIntAddr(I2cMessage *message,
                             uint8_t  slaveAddress,
                             bool IsInternal16,
                             uint16_t intAddress,
                             I2cDirection  direction,
                             uint32_t length,
                             const uint8_t  *buffer);

#endif
