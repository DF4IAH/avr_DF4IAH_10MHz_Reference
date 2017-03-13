/*
 * serout.c
 *
 *  Created on: 10.01.2016
 *      Author: espero
 */


#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <sys/time.h>
#include <time.h>
#include <string.h>
#include <libusb.h>											/* this is libusb */

#include "firmware/df4iah_fw_usb_requests.h"				/* custom request numbers */
#include "firmware/usbconfig.h"								/* USB_CFG_INTR_POLL_INTERVAL */

#include "terminal.h"
#include "main.h"

#include "serout.h"


#define min(a,b) ((a) < (b) ?  (a) : (b))


#ifdef DEBUG
uchar usbRingBufferHook[RINGBUFFER_HOOK_SIZE] = { 0 };
uint8_t usbRingBufferSendSemaphore = 0;
uint8_t usbRingBufferRcvSemaphore = 0;
uint8_t usbRingBufferHookLen = 0;
uint8_t usbRingBufferHookIsSend = 0;
#endif


extern libusb_device_handle* lu_handle;


/* -- 8< --  USB */

static void usb_buffer_controlOut(uchar inLine[], int len)
{
	ringbuffer_fw_ringBufferPush(true, inLine, len);
}

static int usb_buffer_controlIn(uchar outLine[], int size)
{
	return ringBufferPull(false, outLine, size);
}


/* -- 8< --  SEROUT - LOOP */

int switchMode(int mode)
{
	char setModeBuffer[32];

	if (mode == 0) {
		strcpy(setModeBuffer, "INFO");
	} else if (mode == 1) {
		strcpy(setModeBuffer, "SERON");
	}

	/* initial command to be sent */
	usb_buffer_controlOut((uchar*) setModeBuffer, strlen(setModeBuffer));
	return 1;  // one line added
}

void serout(int mode)
{
	struct timeval nowTime;
	uchar inLine[MSGBUFFER_SIZE] = { 0 };
	int inLineCnt = 0;
	int outLineCnt = 0;
	long loopCtr = 0L;
	FILE* fh = fopen("10MHz-Ref-Osc_dump.txt", "w");

	/* timing init */
	gettimeofday(&nowTime, NULL);
	long long nextTime = nowTime.tv_sec * 1000000 + nowTime.tv_usec + ((USB_CFG_INTR_POLL_INTERVAL * CLOCKS_PER_SEC) / 1000);

	char loop = 1;
	do {
		/* transfer field */
		inLineCnt = usb_buffer_controlIn(inLine, sizeof(inLine));
		if (inLineCnt) {
			printf("%s", (char*) &(inLine[0]));
			fprintf(fh, "%s", (char*) &(inLine[0]));
		}

		/* spare time for USB jobs to be done */
		usb_do_transfers();

		if (!lu_handle) {
			sleep(1);
			openDevice(true);
		}

		/* timer */
		gettimeofday(&nowTime, NULL);
		time_t deltaTime = (time_t) (nextTime - nowTime.tv_sec * 1000000 - nowTime.tv_usec);

		if (deltaTime > 0) {
			usleep(deltaTime);

		} else if (deltaTime < -1000) {
			/* adjust to now time */
			nextTime += -deltaTime;
		}

		nextTime += (USB_CFG_INTR_POLL_INTERVAL * CLOCKS_PER_SEC) / 1000;

		if (++loopCtr == 300) {
			outLineCnt += switchMode(mode);
		}
	} while (loop);

	fclose(fh);
}
