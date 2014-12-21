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

#include <strings.h>
#include <usb.h>											/* this is libusb */
#include "opendevice.h"										/* common code moved to separate module */

#include "terminal.h"										/* the terminal mimic is inside */
#include "firmware/df4iah_fw_usb_requests.h"				/* custom request numbers */
#include "firmware/usbconfig.h"								/* device's VID/PID and names */


#define ENABLE_TEST 1


usb_dev_handle* handle 			= NULL;


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
#ifdef DEBUG
	{
		const uint8_t isSend = false;
		const uchar bufferTestIn[3] = { '1', '2', '3' };
		uchar bufferTestOut[3] = { 0 };

		for (;;) {
			/* pull data */
			if (fw_getSemaphore(isSend)) {
				uint8_t retLen = fw_ringBufferPull(isSend, bufferTestOut, sizeof(bufferTestOut));
				fw_freeSemaphore(isSend);
			}

			/* push data */
			if (fw_getSemaphore(isSend)) {
				uint8_t retLen = fw_ringBufferPush(isSend, bufferTestIn, sizeof(bufferTestIn));
				fw_freeSemaphore(isSend);
			}
		}
	}
	return 0;
#endif

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
		fprintf(stderr, "\nERROR: Could not find USB device \"%s\" with vid=0x%x pid=0x%x\n\n", product, vid, pid);
		exit(1);
	}

	/* Since we use only control endpoint 0, we don't need to choose a
	 * configuration and interface. Reading the device descriptor and setting a
	 * configuration and interface is done through endpoint 0 after all.
	 * However, newer versions of Linux require that we claim an interface
	 * even for endpoint 0. Enable the following code if your operating system
	 * needs it: */
#if 0  /* SPECIAL USB HANDLING */
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
#endif  /* SPECIAL USB HANDLING */

	if (strcasecmp(argv[1], "terminal") == 0) {
		terminal();

#if ENABLE_TEST
	} else if (strcasecmp(argv[1], "test1") == 0) {  /* testing USB messaging */
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

	} else if (strcasecmp(argv[1], "test2") == 0) {  /* testing ringbuffers */
		for (;;) {
			char c[8] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };
			char d[8] = { 0 };

			ringBufferPush(false, (char*) &c, 3);
			ringBufferPull(false, (char*) &d, 2);
		}

#endif /* ENABLE_TEST */
	} else {
		usage(argv[0]);
		exit (1);
	}

	usb_close(handle);
	return 0;
}
