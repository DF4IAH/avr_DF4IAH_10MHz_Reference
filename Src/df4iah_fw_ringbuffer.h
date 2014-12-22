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


#ifndef true
# define true 1
#endif
#ifndef false
# define false 0
#endif


#define RINGBUFFER_SEND_SIZE								128
#define RINGBUFFER_RCV_SIZE									128
#define RINGBUFFER_HOOK_SIZE								128

#define MSG_PATTERN_NMEA									'$'


enum RINGBUFFER_MSG_STATUS_t {
	RINGBUFFER_MSG_STATUS_AVAIL								= 0x01,
	RINGBUFFER_MSG_STATUS_IS_NMEA							= 0x10,
	RINGBUFFER_MSG_STATUS_IS_MASK							= 0xF0
};


uint8_t getSemaphore(uint8_t isSend);
void freeSemaphore(uint8_t isSend);

uint8_t ringBufferPush(uint8_t isSend, const uchar inData[], uint8_t len);
void ringBufferPushAddHook(uint8_t isSend, const uchar inData[], uint8_t len);
uint8_t ringBufferPull(uint8_t isSend, uchar outData[], uint8_t size);
enum RINGBUFFER_MSG_STATUS_t getStatusNextMsg(uint8_t isSend);

#endif /* DF4IAH_FW_RINGBUFFER_H_ */
