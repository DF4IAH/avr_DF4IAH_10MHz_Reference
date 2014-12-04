/*
 * df4iah_usb.h
 *
 *  Created on: 01.11.2014
 *      Author: espero
 */

#ifndef DF4IAH_USB_H_
#define DF4IAH_USB_H_


#include "chipdef.h"
#include "usbdrv_fw/usbdrv.h"


void init_usb();
void close_usb();

uint8_t recvchar_usb(void);
void sendchar_usb(uint8_t data);
void recvBuffer_usb(uint16_t addr, uint8_t* dptr, uint8_t len);
void sendBuffer_usb(uint16_t addr, const uint8_t* sptr, uint8_t len);

#endif /* DF4IAH_USB_H_ */
