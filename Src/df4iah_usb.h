/*
 * df4iah_usb.h
 *
 *  Created on: 01.11.2014
 *      Author: espero
 */

#ifndef DF4IAH_USB_H_
#define DF4IAH_USB_H_


#include "chipdef.h"
#include "usbdrv/usbdrv.h"


void replyContent(uchar replyBuffer[], uchar data[]);

void init_usb();
void close_usb();

#if 0
uint8_t recvchar_usb(void);
void sendchar_usb(uint8_t data);
void recvBuffer_usb(uint16_t addr, uint8_t* dptr, uint8_t len);
void sendBuffer_usb(uint16_t addr, const uint8_t* sptr, uint8_t len);
#endif

#endif /* DF4IAH_USB_H_ */
