/*
 * df4iah_fw_ringbuffer.c
 *
 *  Created on: 19.12.2014
 *      Author: DF4IAH, Ulrich Habel
 */


#include <string.h>

#include "df4iah_fw_ringbuffer.h"


#define min(a,b) ((a) < (b) ?  (a) : (b))


static uchar usbRingBufferSend[RINGBUFFER_SEND_SIZE] = { 0 };
static uchar usbRingBufferRcv[RINGBUFFER_RCV_SIZE] = { 0 };
static uint8_t usbRingBufferSendPushIdx = 0;
static uint8_t usbRingBufferSendPullIdx = 0;
static uint8_t usbRingBufferRcvPushIdx = 0;
static uint8_t usbRingBufferRcvPullIdx = 0;


uint8_t ringBufferPush(uchar isSend, uchar inData[], uint8_t len)
{
	int retLen = 0;
	int bufferSize = (isSend ?  RINGBUFFER_SEND_SIZE : RINGBUFFER_RCV_SIZE);
	int pushIdx = (isSend ?  usbRingBufferSendPushIdx : usbRingBufferRcvPushIdx);
	int pullIdx = (isSend ?  usbRingBufferSendPullIdx : usbRingBufferRcvPullIdx);

	if (!(((pushIdx + 1) == pullIdx) || (((pushIdx + 1) == bufferSize) && !pullIdx))) {
		uchar* ringBuffer = (isSend ?  usbRingBufferSend : usbRingBufferRcv);
		int lenTop = min((pullIdx > pushIdx ?  (pullIdx - pushIdx - 1) : bufferSize - pushIdx - (!pullIdx ?  1 : 0)), len);
		int lenBot = min((pullIdx > pushIdx ?  0 : pullIdx - 1), len - lenTop);

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

uint8_t ringBufferPull(uchar isSend, uchar outData[], uint8_t size)
{
	int len = 0;
	int pushIdx = (isSend ?  usbRingBufferSendPushIdx : usbRingBufferRcvPushIdx);
	int pullIdx = (isSend ?  usbRingBufferSendPullIdx : usbRingBufferRcvPullIdx);

	if (pushIdx != pullIdx) {
		uchar* ringBuffer = (isSend ?  usbRingBufferSend : usbRingBufferRcv);
		int bufferSize = (isSend ?  RINGBUFFER_SEND_SIZE : RINGBUFFER_RCV_SIZE);
		int lenTop = min((pushIdx > pullIdx ?  (pushIdx - pullIdx) : bufferSize - pullIdx), size);
		int lenBot = min((pushIdx > pullIdx ?  0 : pushIdx), size - lenTop);

		if (lenTop > 0) {
			memcpy(outData, &(ringBuffer[pullIdx]), lenTop);
			len += lenTop;
		}

		if (lenBot > 0) {
			memcpy(&(outData[lenTop]), &(ringBuffer[0]), lenBot);
			len += lenBot;
		}

		// advance the index
		if (isSend) {
			usbRingBufferSendPullIdx += len;
			usbRingBufferSendPullIdx %= bufferSize;
		} else {
			usbRingBufferRcvPullIdx += len;
			usbRingBufferRcvPullIdx %= bufferSize;
		}
	}
	return len;
}
