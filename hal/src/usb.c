/**
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * uart.c - uart CRTP link and raw access functions
 */
#include <string.h>

/*ST includes */
#include "stm32fxxx.h"

/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "config.h"
#include "usblink.h"
#include "radiolink.h"
#include "usb.h"

#include "usbd_usr.h"
#include "usb_conf.h"
#include "usbd_desc.h"
#include "usb_dcd.h"
#include "ledseq.h"

#include "crtp.h"


__ALIGN_BEGIN USB_OTG_CORE_HANDLE    USB_OTG_dev __ALIGN_END ;

static bool isInit = false;

static xQueueHandle usbDataRx;
static xQueueHandle usbDataTx;

/* Endpoints */
#define IN_EP                       0x81  /* EP1 for data IN */
#define OUT_EP                      0x01  /* EP1 for data OUT */

#define DEVICE_DESCRIPTOR           0x01
#define CONFIGURATION_DESCRIPTOR    0x02
#define STRING_DESCRIPTOR           0x03
#define INTERFACE_DESCRIPTOR        0x04
#define ENDPOINT_DESCRIPTOR         0x05

__ALIGN_BEGIN uint8_t  usbd_cf_CfgDesc[57] __ALIGN_END = {
  /***** Configuration descriptor ******/
  9,                         //bLength
  CONFIGURATION_DESCRIPTOR,  //bDescriptorType
  32,0x00,                   //wTotalLength
  1,                         //bNumInterfaces
  1,                         //bConfigurationValue
  0,                         //iConfiguration
  0x80,                      //bmAttribute (Bus powered, no remote wakeup)
  50,                        //bMaxPower (100mA, shall be enough)
  /***** Interface 0 descriptor: Crazyradio EPs ******/
  9,                         //bLength
  INTERFACE_DESCRIPTOR,      //bDescriptorType
  0,                         //bInterfaceNumber
  0,                         //bAlternateSetting
  2,                         //bNumEndpoint (one in, one out)
  0xFF,                      //bInterfaceClass (VENDOR=0xFF)
  0xFF,                      //bInterfaceSubClass (VENDOR=0xFF)
  0,                         //bInterfaceProtocol (None)
  0,                         //iInterface
  /***** Endpoint 1 IN descriptor ******/
  7,                         //bLength
  ENDPOINT_DESCRIPTOR,       //bDescriptorType
  0x81,                      //bEndpointAddess (EP1 IN)
  0x02,                      //bmAttributes (Bulk endpoint)
  0x40, 0x00,                //wMaxPacketSize (64 bytes)
  6,                         //bInterval (irrelevant for bulk endpoint)
  /***** Endpoint 1 OUT descriptor ******/
  7,                         //bLength
  ENDPOINT_DESCRIPTOR,       //bDescriptorType
  0x01,                      //bEndpointAddess (EP1 OUT)
  0x02,                      //bmAttributes (Bulk endpoint)
  0x40, 0x00,                //wMaxPacketSize (64 bytes)
  6,                         //bInterval (irrelevant for bulk endpoint)
};

static uint8_t  usbd_cf_Init        (void  *pdev, uint8_t cfgidx);
static uint8_t  usbd_cf_DeInit      (void  *pdev, uint8_t cfgidx);
static uint8_t  usbd_cf_DataIn      (void *pdev, uint8_t epnum);
static uint8_t  usbd_cf_DataOut     (void *pdev, uint8_t epnum);
static uint8_t  *usbd_cf_GetCfgDesc (uint8_t speed, uint16_t *length);
static uint8_t  usbd_cf_SOF         (void *pdev);
static uint8_t usbd_cf_Setup        (void *pdev , USB_SETUP_REQ  *req);

static USBPacket inPacket;
static USBPacket outPacket;

/* CDC interface class callbacks structure */
USBD_Class_cb_TypeDef cf_usb_cb =
{
  usbd_cf_Init,
  usbd_cf_DeInit,
  usbd_cf_Setup,
  NULL,
  NULL,
  usbd_cf_DataIn,
  usbd_cf_DataOut,
  usbd_cf_SOF,
  NULL,
  NULL,
  usbd_cf_GetCfgDesc,
};

int command = 0xFF;

static uint8_t usbd_cf_Setup(void *pdev , USB_SETUP_REQ  *req)
{
  command = req->wIndex;
  if (command == 0x01) {
    crtpSetLink(usblinkGetLink());
  } else {
    crtpSetLink(radiolinkGetLink());
  }

  return USBD_OK;
}

static uint8_t  usbd_cf_Init (void  *pdev,
                               uint8_t cfgidx)
{
  /* Open EP IN */
  DCD_EP_Open(pdev,
              IN_EP,
              USB_RX_TX_PACKET_SIZE,
              USB_OTG_EP_BULK);

  /* Open EP OUT */
  DCD_EP_Open(pdev,
              OUT_EP,
              USB_RX_TX_PACKET_SIZE,
              USB_OTG_EP_BULK);

  /* Prepare Out endpoint to receive next packet */
  DCD_EP_PrepareRx(pdev,
                   OUT_EP,
                   (uint8_t*)(inPacket.data),
                   USB_RX_TX_PACKET_SIZE);

  return USBD_OK;
}

/**
  * @brief  usbd_cdc_Init
  *         DeInitialize the CDC layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  usbd_cf_DeInit (void  *pdev,
                                 uint8_t cfgidx)
{
  /* Open EP IN */
  DCD_EP_Close(pdev, IN_EP);

  /* Open EP OUT */
  DCD_EP_Close(pdev, OUT_EP);

  return USBD_OK;
}

/**
  * @brief  usbd_audio_DataIn
  *         Data sent on non-control IN endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t  usbd_cf_DataIn (void *pdev, uint8_t epnum)
{
  portBASE_TYPE xTaskWokenByReceive = pdFALSE;

  if (xQueueReceiveFromISR(usbDataTx, &outPacket, &xTaskWokenByReceive) == pdTRUE)
  {
    DCD_EP_Tx (pdev,
               IN_EP,
               (uint8_t*)outPacket.data,
               outPacket.size);
  }

  return USBD_OK;
}

static uint8_t  usbd_cf_SOF (void *pdev)
{
  portBASE_TYPE xTaskWokenByReceive = pdFALSE;

  if (xQueueReceiveFromISR(usbDataTx, &outPacket, &xTaskWokenByReceive) == pdTRUE)
  {
    DCD_EP_Tx (pdev,
               IN_EP,
               (uint8_t*)outPacket.data,
               outPacket.size);
  }

  return USBD_OK;
}

/**
  * @brief  usbd_cf_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t  usbd_cf_DataOut (void *pdev, uint8_t epnum)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  /* Get the received data buffer and update the counter */
  inPacket.size = ((USB_OTG_CORE_HANDLE*)pdev)->dev.out_ep[epnum].xfer_count;

  xQueueSendFromISR(usbDataRx, &inPacket, &xHigherPriorityTaskWoken);

  /* Prepare Out endpoint to receive next packet */
  DCD_EP_PrepareRx(pdev,
                   OUT_EP,
                   (uint8_t*)(inPacket.data),
                   USB_RX_TX_PACKET_SIZE);

  return USBD_OK;
}

/**
  * @brief  USBD_cdc_GetCfgDesc
  *         Return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *usbd_cf_GetCfgDesc (uint8_t speed, uint16_t *length)
{
  *length = sizeof (usbd_cf_CfgDesc);
  return usbd_cf_CfgDesc;
}

void usbInit(void)
{
  USBD_Init(&USB_OTG_dev,
            USB_OTG_FS_CORE_ID,
            &USR_desc,
            &cf_usb_cb,
            &USR_cb);

  // This should probably be reduced to a CRTP packet size
  usbDataRx = xQueueCreate(5, sizeof(USBPacket)); /* Buffer USB packets (max 64 bytes) */
  usbDataTx = xQueueCreate(1, sizeof(USBPacket)); /* Buffer USB packets (max 64 bytes) */

  isInit = true;
}

bool usbTest(void)
{
  return isInit;
}

bool usbGetDataBlocking(USBPacket *in)
{
  while (xQueueReceive(usbDataRx, in, portMAX_DELAY) != pdTRUE)
    ; // Don't return until we get some data on the USB
  return true;
}

static USBPacket outStage;

bool usbSendData(uint32_t size, uint8_t* data)
{
  outStage.size = size;
  memcpy(outStage.data, data, size);
  // Dont' block when sending
  return (xQueueSend(usbDataTx, &outStage, M2T(100)) == pdTRUE);
}
