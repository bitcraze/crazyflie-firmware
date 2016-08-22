#ifndef I2C_H
#define I2C_H

#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"

#define I2C_BUFFER_SIZE 8

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
  bool             ackDisableBeforeSR;//< Is this the 1 byte special case reading then disable ack before SR1 & SR2 read
} I2cMessage;


#define I2C_NO_INTERNAL_ADDRESS   0xFFFF

/**
 * Must be called once before any calls to i2cMessage.
 */
void i2cInit(I2C_TypeDef* I2Cx);

/**
 * Send or receive a message over the I2C bus.
 *
 * The message is asynchrony and uses interrupts to transfer the message.
 * Receive messages will be sent to the registered receive queue.
 *
 * @param message	 An I2cMessage struct containing all the i2c message
 *                 Information.
 *
 * @param xBlockTime	 The time to wait for a space in the message queue to
 *						 become available should one not be available
 *						 immediately.
 */
void i2cMessageTransfer(I2cMessage* message, portTickType xBlockTime );

/**
 * Register your queue witch will receive incoming i2c messages.
 *
 * @param queue   The queue that will receive the messages.
 */
void i2cRegister(xQueueHandle queue);

void i2cCreateMessage(I2cMessage *message,
                      uint8_t  slaveAddress,
                      uint8_t  direction,
                      xQueueHandle queue,
                      uint32_t length,
                      uint8_t  *buffer);

void i2cCreateMessageSem(I2cMessage *message,
                         uint8_t  slaveAddress,
                         uint8_t  direction,
                         xQueueHandle queue,
                         uint32_t length,
                         uint8_t  *buffer);

void i2cCreateMessageIntAddr(I2cMessage *message,
                             uint8_t  slaveAddress,
                             bool IsInternal16,
                             uint16_t intAddress,
                             uint8_t  direction,
                             xQueueHandle queue,
                             uint32_t length,
                             uint8_t  *buffer);

#endif

