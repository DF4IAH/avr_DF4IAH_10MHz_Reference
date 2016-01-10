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
#include <usb.h>											/* this is libusb */

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


extern usb_dev_handle* handle;


#if 0
static int usbRingBufferRcvPushIdx = 0;
static int usbRingBufferRcvPullIdx = 0;
static int usbRingBufferSendPushIdx = 0;
static int usbRingBufferSendPullIdx = 0;
static uchar usbRingBufferSend[RINGBUFFER_SEND_SIZE];
static uchar usbRingBufferRcv[RINGBUFFER_RCV_SIZE];
static uchar usbMsg[MSGBUFFER_SIZE];
#endif

#ifdef TEST_DATATRANSFER_USB
static int errLine = 0;
#endif


/* -- 8< --  USB */

#if 0
static void usb_buffer_controlOut(uchar inLine[], int len)
{
	ringbuffer_fw_ringBufferPush(true, inLine, len);
}
#endif

static int usb_buffer_controlIn(uchar outLine[], int size)
{
	return ringBufferPull(false, outLine, size);
}


/* -- 8< --  SEROUT - LOOP */

void serout()
{
	struct timeval nowTime;
	uchar inLine[MSGBUFFER_SIZE] = { 0 };
//	uchar outLine[MSGBUFFER_SIZE] = { 0 };
	int inLineCnt = 0;
//	int outLineCnt = 0;
#ifdef TEST_DATATRANSFER_PANEL
	int idleCnt = 0;
#endif

	/* timing init */
	gettimeofday(&nowTime, NULL);
	long long nextTime = nowTime.tv_sec * 1000000 + nowTime.tv_usec + ((USB_CFG_INTR_POLL_INTERVAL * CLOCKS_PER_SEC) / 1000);

	char loop = 1;
	do {
		/* transfer field */
#ifdef TEST_DATATRANSFER_PANEL
		if (++idleCnt > 5) {
			static char toggle = 0;
			toggle = !toggle;
			strcpy(inLine, toggle ?  "test" : "$test");
			inLineCnt = strlen(inLine);
			idleCnt = 0;
		} else {
			inLineCnt = 0;
		}
#else
		inLineCnt = usb_buffer_controlIn(inLine, sizeof(inLine));
# ifdef TEST_DATATRANSFER_USB_TEST2
		if (inLineCnt) {
			char debugBuffer[MSGBUFFER_SIZE] = { 0 };
			sprintf(debugBuffer, " usb_controlIn:  inLineCnt=%03d \n", inLineCnt);
			ncurses_rx_print(&win_rx, debugBuffer, E_COLOR_PAIR_DEBUGGING_IN, 0);
			sprintf(debugBuffer, "%s\n", inLine);
			ncurses_rx_print(&win_rx, debugBuffer, E_COLOR_PAIR_RCV_MAIN, 0);
			inLineCnt = 0;   // TODO: remove me!
			ncurses_update(win_rxborder, win_rx, win_tx);  // TODO: remove me!
		}
# endif
#endif
		if (inLineCnt) {
			printf((char*) &(inLine[0]));
		}

		/* spare time for USB jobs to be done */
		usb_do_transfers();

		if (!handle) {
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

#ifdef TEST_DATATRANSFER_SLOW
		nextTime += 250000;
#else
		nextTime += (USB_CFG_INTR_POLL_INTERVAL * CLOCKS_PER_SEC) / 1000;
#endif
	} while (loop);

}
