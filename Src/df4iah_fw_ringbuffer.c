/*
 * df4iah_fw_ringbuffer.c
 *
 *  Created on: 19.12.2014
 *      Author: DF4IAH, Ulrich Habel
 */


#include <string.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>

#include "main.h"
#include "df4iah_fw_ringbuffer.h"


#define min(a,b) ((a) < (b) ?  (a) : (b))


extern uint8_t usbRingBufferSendPushIdx;
extern uint8_t usbRingBufferSendPullIdx;
extern uint8_t usbRingBufferRcvPushIdx;
extern uint8_t usbRingBufferRcvPullIdx;
extern uint8_t usbRingBufferSendSemaphore;
extern uint8_t usbRingBufferRcvSemaphore;
extern uint8_t usbRingBufferHookLen;
extern uint8_t usbRingBufferHookIsSend;

extern uchar usbRingBufferSend[RINGBUFFER_SEND_SIZE];
extern uchar usbRingBufferRcv[RINGBUFFER_RCV_SIZE];
extern uchar usbRingBufferHook[RINGBUFFER_HOOK_SIZE];


#ifdef RELEASE
__attribute__((section(".df4iah_fw_memory"), aligned(2)))
#endif
void* memcpy_rb(uint8_t isPgm, void* destPtr, const void* srcPtr, size_t len)
{
	if (!isPgm) {
		return memcpy(destPtr, srcPtr, len);

	} else {
		for (int idx = 0; idx < len; ++idx) {
			*((uchar*) destPtr + idx) = pgm_read_byte_near(srcPtr + idx);
		}
		return destPtr;
	}
}

#ifdef RELEASE
__attribute__((section(".df4iah_fw_memory"), aligned(2)))
#endif
uint8_t getSemaphore(uint8_t isSend)
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

#ifdef RELEASE
__attribute__((section(".df4iah_fw_memory"), aligned(2)))
#endif
void freeSemaphore(uint8_t isSend)
{
	/* check if the hook has a job attached to it */
	if (usbRingBufferHookLen && (usbRingBufferHookIsSend == isSend)) {
		(void) ringBufferPush(usbRingBufferHookIsSend, false, usbRingBufferHook, usbRingBufferHookLen);
		usbRingBufferHookLen = 0;
	}

	/* free semaphore */
	{
		uint8_t* semPtr = (isSend ?  &usbRingBufferSendSemaphore : &usbRingBufferRcvSemaphore);
		uint8_t sreg = SREG;
		cli();
		*semPtr = false;
		SREG = sreg;
	}
}

#ifdef RELEASE
__attribute__((section(".df4iah_fw_memory"), aligned(2)))
#endif
uint8_t ringBufferPush(uint8_t isSend, uint8_t isPgm, const uchar inData[], uint8_t len)
{
	uint8_t retLen = 0;
	uint8_t bufferSize = (isSend ?  RINGBUFFER_SEND_SIZE : RINGBUFFER_RCV_SIZE);
	uint8_t pushIdx = (isSend ?  usbRingBufferSendPushIdx : usbRingBufferRcvPushIdx);
	uint8_t pullIdx = (isSend ?  usbRingBufferSendPullIdx : usbRingBufferRcvPullIdx);

	if (!(((pushIdx + 1) == pullIdx) || (((pushIdx + 1) == bufferSize) && !pullIdx))) {
		uchar* ringBuffer = (isSend ?  usbRingBufferSend : usbRingBufferRcv);
		uint8_t lenTop = min((pullIdx > pushIdx ?  (pullIdx - pushIdx - 1) : bufferSize - pushIdx - (!pullIdx ?  1 : 0)), len);
		uint8_t lenBot = min((((pullIdx > pushIdx) || !pullIdx) ?  0 : pullIdx - 1), len - lenTop);

		if (lenTop) {
			memcpy_rb(isPgm, &(ringBuffer[pushIdx]), inData, lenTop);
			retLen += lenTop;
		}

		if (lenBot) {
			memcpy_rb(isPgm, &(ringBuffer[0]), &(inData[lenTop]), lenBot);
			retLen += lenBot;
		}

		// advance the index
		if (isSend) {
			usbRingBufferSendPushIdx += retLen;
			usbRingBufferSendPushIdx %= bufferSize;
		} else {
			usbRingBufferRcvPushIdx += retLen;
			usbRingBufferRcvPushIdx %= bufferSize;
		}
	}
	return retLen;
}

#ifdef RELEASE
__attribute__((section(".df4iah_fw_memory"), aligned(2)))
#endif
void ringBufferPushAddHook(uint8_t isSend, uint8_t isPgm, const uchar inData[], uint8_t len)
{
	/* copy data for the hooked job - hook needs to be unassigned before */
	if (!usbRingBufferHookLen) {
		usbRingBufferHookIsSend = isSend;
		memcpy_rb(isPgm, usbRingBufferHook, inData, len);
		usbRingBufferHookLen = len;							// this assignment last - since now ready to process
	}														// else: dismiss data - should not be the case anyway
}

#ifdef RELEASE
__attribute__((section(".df4iah_fw_memory"), aligned(2)))
#endif
uint8_t ringBufferPull(uint8_t isSend, uchar outData[], uint8_t size)
{
	uint8_t len = 0;
	uint8_t pushIdx = (isSend ?  usbRingBufferSendPushIdx : usbRingBufferRcvPushIdx);
	uint8_t pullIdx = (isSend ?  usbRingBufferSendPullIdx : usbRingBufferRcvPullIdx);

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
			usbRingBufferSendPullIdx += len;
			usbRingBufferSendPullIdx %= bufferSize;
		} else {
			usbRingBufferRcvPullIdx += len;
			usbRingBufferRcvPullIdx %= bufferSize;
		}
	} else if (!size) {
		outData[0] = 0;
	}
	return len;
}

#ifdef RELEASE
__attribute__((section(".df4iah_fw_memory"), aligned(2)))
#endif
enum RINGBUFFER_MSG_STATUS_t getStatusNextMsg(uint8_t isSend)
{
	enum RINGBUFFER_MSG_STATUS_t status = 0;
	uint8_t pushIdx = (isSend ?  usbRingBufferSendPushIdx : usbRingBufferRcvPushIdx);
	uint8_t pullIdx = (isSend ?  usbRingBufferSendPullIdx : usbRingBufferRcvPullIdx);

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

#ifdef RELEASE
__attribute__((section(".df4iah_fw_memory"), aligned(2)))
#endif
void ringBufferWaitFree(uint8_t isSend)
{
	for (;;) {
		if (getSemaphore(isSend)) {
			uint8_t pushIdx = (isSend ?  usbRingBufferSendPushIdx : usbRingBufferRcvPushIdx);
			uint8_t pullIdx = (isSend ?  usbRingBufferSendPullIdx : usbRingBufferRcvPullIdx);

			if (pullIdx == pushIdx) {
				freeSemaphore(isSend);
				break;
			}
			freeSemaphore(isSend);
		}

		// give the CPU away for a moment to delay
		give_away();
	};
}

#ifdef RELEASE
__attribute__((section(".df4iah_fw_memory"), aligned(2)))
#endif
uint8_t ringBufferAppend(uint8_t isSend, uint8_t isPgm, const uchar inData[], uint8_t len)
{
	uint8_t retLen = 0;

	if (getSemaphore(isSend)) {
		retLen = ringBufferPush(isSend, isPgm, inData, len);
		freeSemaphore(isSend);
	} else {
		ringBufferPushAddHook(isSend, isPgm, inData, len);
	}

	return retLen;
}

#ifdef RELEASE
__attribute__((section(".df4iah_fw_memory"), aligned(2)))
#endif
uint8_t ringBufferWaitAppend(uint8_t isSend, uint8_t isPgm, const uchar inData[], uint8_t len)
{
	ringBufferWaitFree(isSend);
	return ringBufferAppend(isSend, isPgm, inData, len);
}
