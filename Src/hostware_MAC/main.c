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

#include <unistd.h>
#include <stdlib.h>
#include <strings.h>
#include <libusb-1.0/libusb.h>

#include "serout.h"										    /* the serout feature is coded here */
#include "terminal.h"										/* the terminal mimic is inside */
#include "firmware/df4iah_fw_usb_requests.h"				/* custom request numbers */
#include "firmware/usbconfig.h"								/* device's VID/PID and names */

#include "main.h"

#define ENABLE_TEST											// activate on request


libusb_context* lu_context			= NULL;
libusb_device_handle *lu_handle		= NULL;


void openDevice(bool isReopening)
{
	/* fire up the USB engine */
	libusb_init(&lu_context);
	libusb_set_debug(lu_context, 3);
	if (isReopening && lu_handle) {
		/* refuse before handle is nulled */
		return;
	}

	// discover devices
	libusb_device **list;
	libusb_device *found = NULL;
	struct libusb_device_descriptor desc;
	ssize_t cnt = libusb_get_device_list(lu_context, &list);
	ssize_t i = 0;
	int err = 0;

	if (cnt >= 0) {
		for (i = 0; i < cnt; i++) {
			libusb_device *device = list[i];
			int r = libusb_get_device_descriptor(device, &desc);
			if (r < 0) {
				fprintf(stderr, "failed to get device descriptor");
				break;
			}
			//printf("USB Vendor/Product: 0x%04x/0x%04x\n", desc.idVendor, desc.idProduct);

			if (desc.idVendor == USB_CFG_VENDOR_ID__INT &&
				desc.idProduct == USB_CFG_DEVICE_ID__INT) {
				found = device;
				break;
			}
		}

		if (found) {
			//printf("+++ 10 MHz-Ref-Osc. found!\n");
			err = libusb_open(found, &lu_handle);
			if (err) {
				// error
				found = NULL;
				lu_handle = NULL;
			}
		} else {
			//printf("--- 10 MHz-Ref-Osc. NOT found!\n");
			lu_handle = NULL;
		}

	} else {
		// error
		found = NULL;
		lu_handle = NULL;
	}

	libusb_free_device_list(list, 1);
	return;
}

void closeDevice()
{
	libusb_close(lu_handle);
	lu_handle = NULL;

	libusb_exit(lu_context);
	lu_context = NULL;
}

static void usage(char *name)
{
    fprintf(stderr, "usage:\n");
    fprintf(stderr, "  %s -terminal.. activates terminal transfer\n", name);
    fprintf(stderr, "  %s -gpsout.. dumps NMEA0183 messages\n", name);
    fprintf(stderr, "  %s -infoout.. dumps Ref-Clk INFO messages\n", name);
#ifdef ENABLE_TEST
    fprintf(stderr, "  %s -test ..... run driver reliability test\n", name);
#endif /* ENABLE_TEST */
}

int main(int argc, char **argv)
{
	/*
	 * the main application starts here
	 */
    if (argc < 2) {											// we need at least one argument
        usage(argv[0]);
        exit(1);
    }

    /* open the USB device */
    openDevice(false);

	if (strcasecmp(argv[1], "-infoout") == 0) {
		serout(0);
	}

	else if (strcasecmp(argv[1], "-gpsout") == 0) {
		serout(1);
	}

	else if (strcasecmp(argv[1], "-terminal") == 0) {
	terminal();

#ifdef ENABLE_TEST
	} else if (strcasecmp(argv[1], "-test") == 0) {
		/* testing USB messaging */
#ifdef __APPLE_CC__
		srandomdev();
#endif
		for (int i = 0; i < 50000; i++) {
			int value = random() & 0xffff, index = random() & 0xffff;
			int rxValue, rxIndex;
			char buffer[4];

			if ((i+1) % 100 == 0) {
				fprintf(stderr, "\r%05d", i+1);
				fflush(stderr);
			}

			int cnt = libusb_control_transfer(lu_handle, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE | LIBUSB_ENDPOINT_IN, USBCUSTOMRQ_ECHO, value, index, (unsigned char*) buffer, sizeof(buffer), 5000);
			if (cnt < 0) {
				fprintf(stderr, "\nUSB error in iteration %d: %s\n", i, libusb_strerror(cnt));
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

	/* shut down the USB device */
	closeDevice();

	return 0;
}
