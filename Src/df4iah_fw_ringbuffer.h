/*
 * df4iah_fw_ringbuffer.h
 *
 *  Created on: 19.12.2014
 *      Author: DF4IAH, Ulrich Habel
 */

#ifndef DF4IAH_FW_RINGBUFFER_H_
#define DF4IAH_FW_RINGBUFFER_H_


#include <stdint.h>

#include "usbdrv_fw/usbdrv.h"


#define RINGBUFFER_SEND_SIZE	254
#define RINGBUFFER_RCV_SIZE		254


uint8_t ringBufferPush(uchar isSend, uchar inData[], uint8_t len);
uint8_t ringBufferPull(uchar isSend, uchar outData[], uint8_t size);



#endif /* DF4IAH_FW_RINGBUFFER_H_ */
