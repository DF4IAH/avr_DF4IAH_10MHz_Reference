/*
 * df4iah_usb.h
 *
 *  Created on: 01.11.2014
 *      Author: espero
 */

#ifndef DF4IAH_FW_USB_H_
#define DF4IAH_FW_USB_H_


#include "chipdef.h"
#include "usbdrv_fw/usbdrv.h"


void usb_fw_init();
void usb_fw_close();

uint8_t usb_fw_recvchar(void);
void usb_fw_sendchar(uint8_t data);
void usb_fw_recvBuffer(uint16_t addr, uint8_t* dptr, uint8_t len);
void usb_fw_sendBuffer(uint16_t addr, const uint8_t* sptr, uint8_t len);

#endif /* DF4IAH_FW_USB_H_ */
