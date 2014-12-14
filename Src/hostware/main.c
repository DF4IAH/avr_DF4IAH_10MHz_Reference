/* Name: hostware to DF4IAH_10MHz_Reference
 * Author: Christian Starkjohann, Ulrich Habel
 * Creation Date: 2014-12-14
 * Tabsize: 4
 * Copyright: (c) 2008 by OBJECTIVE DEVELOPMENT Software GmbH
 * License: GNU GPL v2 (see License.txt), GNU GPL v3 or proprietary (CommercialLicense.txt)
 */

/*
General Description:
This is the host-side driver for the custom-class example device. It searches
the USB for the DF4IAH_10MHz_Reference device and sends the requests understood by this
device.
This program must be linked with libusb on Unix and libusb-win32 on Windows.
See http://libusb.sourceforge.net/ or http://libusb-win32.sourceforge.net/
respectively.
*/

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include <string.h>
#include <usb.h>											/* this is libusb */
#include "opendevice.h"										/* common code moved to separate module */

#include "firmware/df4iah_fw_usb_requests.h"				/* custom request numbers */
#include "firmware/usbconfig.h"								/* device's VID/PID and names */


#define ENABLE_TEST 1

#define TERM_TOP_ROW		1
#define TERM_RX_FIRST_ROW	1
#define TERM_RX_LAST_ROW	30
#define	TERM_TX_EDIT_ROW	35
#define TERM_BOTTOM_ROW		37


static int stdinHasChar()
{
	struct timeval tv;
	fd_set fds;
	tv.tv_sec = 0;
	tv.tv_usec = 0;
	FD_ZERO(&fds);

	FD_SET(STDIN_FILENO, &fds);
	select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);
	return (FD_ISSET(0, &fds));
}

static void clearTermInLine()
{
	printf("%c[%d;%df", 0x1b, TERM_TX_EDIT_ROW, 1);			// set cursor to TERM_TX_EDIT_ROW
	printf("%c[2K", 0x1b);									// delete line
}

static void printTermInLine(char inLine[])
{
	printf("%c[%d;%df", 0x1b, TERM_TX_EDIT_ROW, 1);			// set cursor to TERM_TX_EDIT_ROW
	printf("%s", inLine);
//	printf("%s", "TEST");
}

static void printUpdate()
{
	printf("\n%c[1A", 0x1b);
}

static void sendInLine(char inLine[])
{

}

static void usage(char *name)
{
    fprintf(stderr, "usage:\n");
#if 0
    fprintf(stderr, "  %s on ....... turn on LED\n", name);
    fprintf(stderr, "  %s off ...... turn off LED\n", name);
    fprintf(stderr, "  %s status ... ask current status of LED\n", name);
#endif
    fprintf(stderr, "  %s terminal.. activates terminal transfer\n", name);
#if ENABLE_TEST
    fprintf(stderr, "  %s test ..... run driver reliability test\n", name);
#endif /* ENABLE_TEST */
}

int main(int argc, char **argv)
{
	usb_dev_handle* handle 			= NULL;
	const unsigned char rawVid[2] 	= { USB_CFG_VENDOR_ID };
	const unsigned char rawPid[2]	= { USB_CFG_DEVICE_ID };
	char vendor[] 					= { USB_CFG_VENDOR_NAME, 0 };
	char product[] 					= { USB_CFG_DEVICE_NAME, 0 };
	char buffer[4];
	int cnt, vid, pid;
//	int isOn;
//	int showWarnings 				= 1;

    if (argc < 2) {											// we need at least one argument
        usage(argv[0]);
        exit(1);
    }

    usb_init();

    /* compute VID/PID from usbconfig.h so that there is a central source of information */
    vid = rawVid[0] | (rawVid[1] << 8);
    pid = rawPid[0] | (rawPid[1] << 8);

    /* The following function is in opendevice.c: */
    if (usbOpenDevice(&handle, vid, vendor, pid, product, NULL, NULL, NULL) != 0) {
        fprintf(stderr, "Could not find USB device \"%s\" with vid=0x%x pid=0x%x\n", product, vid, pid);
        exit(1);
    }

    /* Since we use only control endpoint 0, we don't need to choose a
     * configuration and interface. Reading the device descriptor and setting a
     * configuration and interface is done through endpoint 0 after all.
     * However, newer versions of Linux require that we claim an interface
     * even for endpoint 0. Enable the following code if your operating system
     * needs it: */
#if 0
    int retries = 1, usbConfiguration = 1, usbInterface = 0;

    if (usb_set_configuration(handle, usbConfiguration) && showWarnings) {
        fprintf(stderr, "Warning: could not set configuration: %s\n", usb_strerror());
    }

    /* now try to claim the interface and detach the kernel HID driver on
     * Linux and other operating systems which support the call. */
    while ((len = usb_claim_interface(handle, usbInterface)) != 0 && retries-- > 0) {
#ifdef LIBUSB_HAS_DETACH_KERNEL_DRIVER_NP
        if (usb_detach_kernel_driver_np(handle, 0) < 0 && showWarnings) {
            fprintf(stderr, "Warning: could not detach kernel driver: %s\n", usb_strerror());
        }
#endif
    }
#endif

#if 0
    if (strcasecmp(argv[1], "status") == 0) {
        cnt = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, CUSTOM_RQ_GET_STATUS, 0, 0, buffer, sizeof(buffer), 5000);
        if (cnt < 1) {
            if (cnt < 0) {
                fprintf(stderr, "USB error: %s\n", usb_strerror());
            } else {
                fprintf(stderr, "only %d bytes received.\n", cnt);
            }
        } else {
            printf("LED is %s\n", buffer[0] ? "on" : "off");
        }
    } else if ((isOn = (strcasecmp(argv[1], "on") == 0)) || strcasecmp(argv[1], "off") == 0) {
        cnt = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_OUT, CUSTOM_RQ_SET_STATUS, isOn, 0, buffer, 0, 5000);
        if (cnt < 0) {
            fprintf(stderr, "USB error: %s\n", usb_strerror());
        }
#else  // TODO DF4IAH below
   if (strcasecmp(argv[1], "terminal") == 0) {
	   char inLine[1024] = { 0 };
	   int inLineCnt = 0;
	   struct timeval nowTime;
	   gettimeofday(&nowTime, NULL);
	   long long nextTime = nowTime.tv_sec * 1000000 + nowTime.tv_usec + ((USB_CFG_INTR_POLL_INTERVAL * CLOCKS_PER_SEC) / 1000);
	   int line = 0, cnt = 0, idleCnt = 0;

	   /* clear screen */
	   printf("%cc", 0x1b);
	   printf("%c[%d;%dr", 0x1b, TERM_TOP_ROW, TERM_BOTTOM_ROW);	// set top and bottom line of the window
	   //printf("%c[2J", 0x1b);										// clear entire screen
	   //printf("%c[H", 0x1b);										// move cursor to upper left corner (home)
	   line = TERM_RX_FIRST_ROW;

	   for (;;) {
		   /* output field */
		   if (++idleCnt > 5) {
			   idleCnt = 0;

			   printf("%c[%d;%df", 0x1b, line, 1);					// set cursor to position
			   printf("cnt=%d", ++cnt);
			   if (++line <= TERM_RX_LAST_ROW) {
				   clearTermInLine();
				   printTermInLine(inLine);
			   } else {
				   --line;
				   clearTermInLine();
				   printf("%c[%d;%df", 0x1b, TERM_BOTTOM_ROW, 1);	// set cursor to bottom
				   printf("%cD", 0x1b);								// scroll window one line up
				   printTermInLine(inLine);
			   }
		   }

		   /* input field */
		   if (stdinHasChar()) {
			   printf("*");
			   int c = getchar();
			   switch (c) {
			   case 0x0d:
				   continue;

			   case 0x0a:
				   sendInLine(inLine);
				   inLineCnt = 0;
				   inLine[inLineCnt] = 0;
				   //clearTermInLine();
				   break;

			   default:
				   if ((inLineCnt + 1) < sizeof(inLine)) {
					   inLine[inLineCnt++] = (char) (c & 0xff);
					   inLine[inLineCnt] = 0;
					   //printTermInLine(inLine);
				   }
			   }
		   }

		   /* update terminal window */
		   printUpdate();


		   /* timer */
		   gettimeofday(&nowTime, NULL);
		   time_t deltaTime = (time_t) (nextTime - nowTime.tv_sec * 1000000 - nowTime.tv_usec);
#ifdef __APPLE_CC__
		   if (deltaTime > 0) {
			   usleep(deltaTime);
		   }
#else
		   if (deltaTime > 0) {
			   unsigned int usSleepTime = deltaTime * (1000000 / CLOCKS_PER_SEC);
			   nanosleep(usSleepTime * 1000);
		   }
#endif
		   nextTime += (USB_CFG_INTR_POLL_INTERVAL * CLOCKS_PER_SEC) / 1000;
	   }
#endif
#if ENABLE_TEST
    } else if (strcasecmp(argv[1], "test") == 0) {
        srandomdev();
        for (int i = 0; i < 50000; i++) {
            int value = random() & 0xffff, index = random() & 0xffff;
            int rxValue, rxIndex;

            if ((i+1) % 100 == 0) {
                fprintf(stderr, "\r%05d", i+1);
                fflush(stderr);
            }

            cnt = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, USBCUSTOMRQ_ECHO, value, index, buffer, sizeof(buffer), 5000);
            if (cnt < 0) {
                fprintf(stderr, "\nUSB error in iteration %d: %s\n", i, usb_strerror());
                break;
            } else if (cnt != 4) {
                fprintf(stderr, "\nerror in iteration %d: %d bytes received instead of 4\n", i, cnt);
                break;
            }
            rxValue = (buffer[0] & 0xff) | (((int) buffer[1] & 0xff) << 8);
            rxIndex = (buffer[2] & 0xff) | (((int) buffer[3] & 0xff) << 8);
            if (rxValue != value || rxIndex != index) {
                fprintf(stderr, "\ndata error in iteration %d:\n", i);
                fprintf(stderr, "rxValue = 0x%04x value = 0x%04x\n", rxValue, value);
                fprintf(stderr, "rxIndex = 0x%04x index = 0x%04x\n", rxIndex, index);
            }
        }
        fprintf(stderr, "\nTest completed.\n");
#endif /* ENABLE_TEST */
    } else {
        usage(argv[0]);
        exit (1);
    }

    usb_close(handle);
    return 0;
}
