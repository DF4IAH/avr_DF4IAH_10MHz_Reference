/*
 * df4iah_fw_ringbuffer.c
 *
 *  Created on: 19.12.2014
 *      Author: DF4IAH, Ulrich Habel
 */


#include <string.h>

#include "df4iah_fw_ringbuffer.h"


#define min(a,b) ((a) < (b) ?  (a) : (b))


extern uchar usbRingBufferSend[RINGBUFFER_SEND_SIZE];
extern uchar usbRingBufferRcv[RINGBUFFER_RCV_SIZE];
extern uint8_t usbRingBufferSendPushIdx;
extern uint8_t usbRingBufferSendPullIdx;
extern uint8_t usbRingBufferRcvPushIdx;
extern uint8_t usbRingBufferRcvPullIdx;


#ifdef RELEASE
__attribute__((section(".df4iah_fw_memory"), aligned(2)))
#endif
uint8_t ringBufferPush(uint8_t isSend, uchar inData[], uint8_t len)
{
	uint8_t retLen = 0;
	uint8_t bufferSize = (isSend ?  RINGBUFFER_SEND_SIZE : RINGBUFFER_RCV_SIZE);
	uint8_t pushIdx = (isSend ?  usbRingBufferSendPushIdx : usbRingBufferRcvPushIdx);
	uint8_t pullIdx = (isSend ?  usbRingBufferSendPullIdx : usbRingBufferRcvPullIdx);

	if (!(((pushIdx + 1) == pullIdx) || (((pushIdx + 1) == bufferSize) && !pullIdx))) {
		uchar* ringBuffer = (isSend ?  usbRingBufferSend : usbRingBufferRcv);
		uint8_t lenTop = min((pullIdx > pushIdx ?  (pullIdx - pushIdx - 1) : bufferSize - pushIdx - (!pullIdx ?  1 : 0)), len);
		uint8_t lenBot = min((pullIdx > pushIdx ?  0 : pullIdx - 1), len - lenTop);

		if (lenTop > 0) {
			memcpy(&(ringBuffer[pushIdx]), inData, lenTop);
			retLen += lenTop;
		}

		if (lenBot > 0) {
			memcpy(&(ringBuffer[0]), &(inData[lenTop]), lenBot);
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
uint8_t ringBufferPull(uint8_t isSend, uchar outData[], uint8_t size)
{
	uint8_t len = 0;
	uint8_t pushIdx = (isSend ?  usbRingBufferSendPushIdx : usbRingBufferRcvPushIdx);
	uint8_t pullIdx = (isSend ?  usbRingBufferSendPullIdx : usbRingBufferRcvPullIdx);

	if (pushIdx != pullIdx) {
		uchar* ringBuffer = (isSend ?  usbRingBufferSend : usbRingBufferRcv);
		uint8_t bufferSize = (isSend ?  RINGBUFFER_SEND_SIZE : RINGBUFFER_RCV_SIZE);
		uint8_t lenTop = min((pushIdx > pullIdx ?  (pushIdx - pullIdx) : bufferSize - pullIdx), size);
		uint8_t lenBot = min((pushIdx > pullIdx ?  0 : pushIdx), size - lenTop);

		if (lenTop > 0) {
			memcpy(outData, &(ringBuffer[pullIdx]), lenTop);
			len += lenTop;
		}

		if (lenBot > 0) {
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
	} else {
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
