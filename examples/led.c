#include <stdio.h>
#include <string.h>
#include <libusb.h>
#include "libfreenect.h"


int main(int argc, char **argv)
{
	int res;
	freenect_context *f_ctx;
	freenect_device *f_dev;

	if (freenect_init(&f_ctx, NULL) < 0) {
		printf("freenect_init() failed\n");
		return 1;
	}

	if (freenect_open_device(f_ctx, &f_dev, 0) < 0) {
		printf("Could not open device\n");
		return 1;
	}

    if (freenect_set_led(f_dev, 6) < 0) {
        printf("Trouble setting LED.");
        return 1;
    }

    return 0;
}
