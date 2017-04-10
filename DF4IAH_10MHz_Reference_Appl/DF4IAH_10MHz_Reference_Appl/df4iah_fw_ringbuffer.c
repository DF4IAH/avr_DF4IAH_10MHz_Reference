/*
 * df4iah_fw_ringbuffer.c
 *
 *  Created on: 19.12.2014
 *      Author: DF4IAH, Ulrich Habel
 */
// tabsize: 4


#include "df4iah_fw_clkPullPwm.h"
#include <util/delay.h>

#include <string.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>

#include "df4iah_fw_main.h"
#include "df4iah_fw_memory.h"

#include "df4iah_fw_ringbuffer.h"


#define min(a,b) ((a) < (b) ?  (a) : (b))

#ifndef false
# define false					0
#endif
#ifndef true
# define true					1
#endif


extern uint8_t usbRingBufferSendPushIdx;
extern uint8_t usbRingBufferSendPullIdx;
extern uint8_t usbRingBufferRcvPushIdx;
extern uint8_t usbRingBufferRcvPullIdx;
extern uint8_t usbRingBufferSendSemaphore;
extern uint8_t usbRingBufferRcvSemaphore;

extern uchar usbRingBufferSend[RINGBUFFER_SEND_SIZE];
extern uchar usbRingBufferRcv[RINGBUFFER_RCV_SIZE];


uint8_t ringbuffer_fw_getSemaphore(uint8_t isSend)
{
	uint8_t isLocked;
	uint8_t* semPtr = (isSend ?  &usbRingBufferSendSemaphore : &usbRingBufferRcvSemaphore);

#if 0
	uint8_t sreg;
	asm volatile
	(
		"ldi r19, 0x01\n\t"
		"in	 %1, 0x3f\n\t"
		"cli \n\t"
		"ld	%0, Z\n\t"
		"st	Z, r19\n\t"
		"out 0x3f, %1\n\t"
		: "=r" (isLocked),
		  "=r" (sreg)
		: "p" (semPtr)
		: "r19"
	);
#else
	uint8_t sreg = SREG;
	cli();
	isLocked = *semPtr;
	*semPtr = true;
	SREG = sreg;
#endif
	return !isLocked;
}

void ringbuffer_fw_freeSemaphore(uint8_t isSend)
{
	/* free semaphore */
	{
		uint8_t* semPtr = (isSend ?  &usbRingBufferSendSemaphore : &usbRingBufferRcvSemaphore);
		uint8_t sreg = SREG;
		cli();
		*semPtr = false;
		SREG = sreg;
	}
}

uint8_t ringbuffer_fw_ringBufferPush(uint8_t isSend, uint8_t isPgm, const uchar inData[], uint8_t len)
{
	uint8_t retLen = 0;
	uint8_t bufferSize = (isSend ?  RINGBUFFER_SEND_SIZE : RINGBUFFER_RCV_SIZE);

	uint8_t sreg = SREG;
	cli();
	uint8_t pushIdx = (isSend ?  usbRingBufferSendPushIdx : usbRingBufferRcvPushIdx);
	uint8_t pullIdx = (isSend ?  usbRingBufferSendPullIdx : usbRingBufferRcvPullIdx);
	SREG = sreg;

	if (!(((pushIdx + 1) == pullIdx) || (((pushIdx + 1) == bufferSize) && !pullIdx))) {
		uchar* ringBuffer = (isSend ?  usbRingBufferSend : usbRingBufferRcv);
		uint8_t lenTop = min((pullIdx > pushIdx ?  (pullIdx - pushIdx - 1) : bufferSize - pushIdx - (!pullIdx ?  1 : 0)), len);
		uint8_t lenBot = min((((pullIdx > pushIdx) || !pullIdx) ?  0 : pullIdx - 1), len - lenTop);

		if (lenTop) {
			memory_fw_copyBuffer(isPgm, &(ringBuffer[pushIdx]), inData, lenTop);
			retLen += lenTop;
		}

		if (lenBot) {
			memory_fw_copyBuffer(isPgm, &(ringBuffer[0]), &(inData[lenTop]), lenBot);
			retLen += lenBot;
		}

		// advance the index
		if (isSend) {
			uint8_t sreg = SREG;
			cli();
			usbRingBufferSendPushIdx += retLen;
			usbRingBufferSendPushIdx %= bufferSize;
			SREG = sreg;

		} else {
			uint8_t sreg = SREG;
			cli();
			usbRingBufferRcvPushIdx += retLen;
			usbRingBufferRcvPushIdx %= bufferSize;
			SREG = sreg;
		}
	}
	return retLen;
}

uint8_t ringbuffer_fw_ringBufferPull(uint8_t isSend, uchar outData[], uint8_t size)
{
	uint8_t len = 0;

	uint8_t sreg = SREG;
	cli();
	uint8_t pushIdx = (isSend ?  usbRingBufferSendPushIdx : usbRingBufferRcvPushIdx);
	uint8_t pullIdx = (isSend ?  usbRingBufferSendPullIdx : usbRingBufferRcvPullIdx);
	SREG = sreg;

	if ((pushIdx != pullIdx) && (size > 1)) {
		uchar* ringBuffer = (isSend ?  usbRingBufferSend : usbRingBufferRcv);
		uint8_t bufferSize = (isSend ?  RINGBUFFER_SEND_SIZE : RINGBUFFER_RCV_SIZE);
		uint8_t lenTop = min((pushIdx > pullIdx ?  (pushIdx - pullIdx) : bufferSize - pullIdx), size - 1);
		uint8_t lenBot = min((pushIdx > pullIdx ?  0 : pushIdx), size - 1 - lenTop);

		if (lenTop) {
			memcpy(outData, &(ringBuffer[pullIdx]), lenTop);
			len += lenTop;
		}

		if (lenBot) {
			memcpy(&(outData[lenTop]), &(ringBuffer[0]), lenBot);
			len += lenBot;
		}

		outData[len] = 0;

		// advance the index
		if (isSend) {
			uint8_t sreg = SREG;
			cli();
			usbRingBufferSendPullIdx += len;
			usbRingBufferSendPullIdx %= bufferSize;
			SREG = sreg;

		} else {
			uint8_t sreg = SREG;
			cli();
			usbRingBufferRcvPullIdx += len;
			usbRingBufferRcvPullIdx %= bufferSize;
			SREG = sreg;
		}
	} else if (!size) {
		outData[0] = 0;
	}
	return len;
}

enum RINGBUFFER_MSG_STATUS_t ringbuffer_fw_getStatusNextMsg(uint8_t isSend)
{
	enum RINGBUFFER_MSG_STATUS_t status = 0;

	uint8_t sreg = SREG;
	cli();
	uint8_t pushIdx = (isSend ?  usbRingBufferSendPushIdx : usbRingBufferRcvPushIdx);
	uint8_t pullIdx = (isSend ?  usbRingBufferSendPullIdx : usbRingBufferRcvPullIdx);
	SREG = sreg;

	if (pullIdx != pushIdx) {
		status |= RINGBUFFER_MSG_STATUS_AVAIL;

		/* test for NMEA message */
		uchar* ringBuffer = (isSend ?  usbRingBufferSend : usbRingBufferRcv);
		if (*(ringBuffer + pullIdx) == MSG_PATTERN_NMEA) {	// first character identifies message type
			status |= RINGBUFFER_MSG_STATUS_IS_NMEA;
		}
	}
	return status;
}

static void ringbuffer_fw_ringBufferWaitFreeAndKeepSemaphore(uint8_t isSend)
{
	for (;;) {
		if (ringbuffer_fw_getSemaphore(isSend)) {
			uint8_t pushIdx;
			uint8_t pullIdx;

			if (isSend) {
				uint8_t sreg = SREG;
				cli();
				pushIdx = usbRingBufferSendPushIdx;
				pullIdx = usbRingBufferSendPullIdx;
				SREG = sreg;

			} else {
				uint8_t sreg = SREG;
				cli();
				pushIdx = usbRingBufferRcvPushIdx;
				pullIdx = usbRingBufferRcvPullIdx;
				SREG = sreg;
			}

			if (pullIdx == pushIdx) {
				// buffer is empty, break loop and hold semaphore
				break;
			}

			ringbuffer_fw_freeSemaphore(isSend);

			// give the CPU away for a moment to delay, do not use giveAway() that would make a loop
		    wdt_reset();
		    usbPoll();
		}

		// give the CPU away for a moment to delay, do not use giveAway() that would make a loop
	    wdt_reset();
	    usbPoll();
	}
}

uint8_t ringbuffer_fw_ringBufferWaitAppend(uint8_t isSend, uint8_t isPgm, const uchar inData[], uint8_t len)
{
	ringbuffer_fw_ringBufferWaitFreeAndKeepSemaphore(isSend);
    uint8_t retLen = ringbuffer_fw_ringBufferPush(isSend, isPgm, inData, len);
	ringbuffer_fw_freeSemaphore(isSend);
    return retLen;
}
