#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <usb.h>
#include "libfreenect.h"
#include "cameras.h"


//as per last post on - http://libusb.6.n5.nabble.com/isochronous-transfer-td3250228.html#a3250228

//always have 64 transfers going?
#define NUM_XFERS 64

//each transfer is made up of 32 Packets each 1920
#define PKTS_PER_XFER 32
#define PKT_SIZE_RGB 1920
#define PKT_SIZE_DEPTH 1760

#define XFER_SIZE_RGB (PKTS_PER_XFER*PKT_SIZE_RGB)
#define XFER_SIZE_DEPTH (PKTS_PER_XFER*PKT_SIZE_DEPTH)

//ignore these for now
#define DEPTH_LEN PKT_SIZE_DEPTH
#define RGB_LEN XFER_SIZE_RGB

//use new bayer code which doesn't overun the buffer
#define NEW_BAYER

static void *rgb_bufs[NUM_XFERS];
static void *depth_bufs[NUM_XFERS];

static depthcb depth_cb;
static rgbcb rgb_cb;

struct frame_hdr {
uint8_t magic[2];
uint8_t pad;
uint8_t flag;
uint8_t unk1;
uint8_t seq;
uint8_t unk2;
uint8_t unk3;
uint32_t timestamp;
};

uint8_t depth_buf[2*422400];
uint16_t depth_frame[640*480];
int depth_pos = 0;

//why is rgb buf 2*307200 = 2*640*480 ??????  // [From Majority] : because it's 640*480 (Bayer) but we allocate something bigger coz it could overrun
uint8_t rgb_buf[2*307200];
uint8_t rgb_frame[640*480*3];

int rgb_pos = 0;

extern const struct caminit inits[];
extern const int num_inits;

static void depth_process(uint8_t *buf, size_t len)
{
    if (len == 0)
    return;

	int i;
	struct frame_hdr *hdr = (void*)buf;
	uint8_t *data = buf + sizeof(*hdr);
	int datalen = len - sizeof(*hdr);
	int bitshift = 0;

	//printf("%02x %x\n", hdr->flag, depth_pos);
	switch (hdr->flag) {
		case 0x71:
			depth_pos = 0;
		case 0x72:
		case 0x75:
			memcpy(&depth_buf[depth_pos], data, datalen);
			depth_pos += datalen;
		break;
	}

	if (hdr->flag != 0x75){
		return;
	}

	//make sure we really are at the end of the image before updating.
    if( depth_pos < 640*480 ){
        return;
    }

	printf("GOT DEPTH FRAME, %d bytes\n", depth_pos);

	for (i=0; i<(640*480); i++) {
		int idx = (i*11)/8;
		uint32_t word = (depth_buf[idx]<<16) | (depth_buf[idx+1]<<8) | depth_buf[idx+2];
		depth_frame[i] = ((word >> (13-bitshift)) & 0x7ff);
		bitshift = (bitshift + 11) % 8;
	}

	if(depth_cb){
		depth_cb(depth_frame, 640, 480);
	}
}

static void bayer_to_rgb(const unsigned char* bayer, unsigned char* rgb, int w, int h);


static void rgb_process(uint8_t *buf, size_t len)
{

	int x,y,i,j;
	unsigned char newint = 0;
	struct frame_hdr *hdr = (void*)buf;
	uint8_t *data = buf+ sizeof(*hdr);
	int datalen = len - sizeof(*hdr);

	if (len < sizeof(struct frame_hdr))
		return;

	//printf("%02x %x\n", hdr->flag, depth_pos);
	switch (hdr->flag) {
		case 0x81:
            rgb_pos = 0;
		case 0x82:
		case 0x85:
			memcpy(&rgb_buf[rgb_pos], data, datalen);
			rgb_pos += datalen;
		break;
	}

    //printf("hdr flag is %02x\n", hdr->flag);
	if (hdr->flag != 0x85 ){
		return;
	}
	//make sure we really are at the end of the image before updating.
    if( rgb_pos < 640*480 ){
        return;
    }
	printf("GOT RGB FRAME, %d bytes\n", rgb_pos);

    /*
    //see the image as bayer grayscale
	for(k = 0,j=0; k < 640*480; k++){
        rgb_frame[j] = rgb_buf[k];
        rgb_frame[j+1] = rgb_buf[k];
        rgb_frame[j+2] = rgb_buf[k];
        j+= 3;
	}
	*/

    #ifdef NEW_BAYER
        //uses openCV bayer at the bottom of this file
        bayer_to_rgb(rgb_buf, rgb_frame, 640, 480);

    #else
        #warning this code overuns the buffer!
        #warning this code needs fixing
        //keeping this here because it needs fixing!!!!

		for (y=0; y<480; y++) {
            for (x=0; x<640; x++) {
                i = y*640+x;
                if (x&1) {
                    if (y&1) {
                        rgb_frame[3*i+1] = rgb_buf[i];
                        rgb_frame[3*i+4] = rgb_buf[i];
                    } else {
                        rgb_frame[3*i] = rgb_buf[i];
                        rgb_frame[3*i+3] = rgb_buf[i];
                        rgb_frame[3*(i-640)] = rgb_buf[i];
                        rgb_frame[3*(i-640)+3] = rgb_buf[i];
                    }
                } else {
                    if (y&1) {
                        rgb_frame[3*i+2] = rgb_buf[i];
                        rgb_frame[3*i-1] = rgb_buf[i];
                        rgb_frame[3*(i+640)+2] = rgb_buf[i];
                        rgb_frame[3*(i+640)-1] = rgb_buf[i];
                    } else {
                        rgb_frame[3*i+1] = rgb_buf[i];
                        rgb_frame[3*i-2] = rgb_buf[i];
                    }
                }
            }
        }
    #endif

	if(rgb_cb){
		rgb_cb(rgb_frame, 640, 480);
	}
}

//NOTE - There is something off about this approach
//in libusb_ we know the actual length of the packet - with usb we know the length we wanted our packets to be, the transfer size and the num packets
//this doesn't allow for the logic u see in the commented out one below.
static void rgb_callback(char * buf, int pktLen)
{
	int i;
	for (i=0; i<PKTS_PER_XFER; i++) {
		rgb_process(buf, pktLen);
		buf += pktLen;
	}

}

static void depth_callback(char * buf, int pktLen)
{
	 int i;
	 for (i=0; i<PKTS_PER_XFER; i++) {
		 //printf("DCB %p %d\n", buf, xfer->iso_packet_desc[i].actual_length);
		 depth_process(buf, pktLen);
		 buf += pktLen;
	 }

}


static usb_dev_handle* motorHandle = NULL;
static usb_dev_handle* cameraHandle = NULL;

enum LIBFREENECT_RETURN_CODE init_camera_device(){

	struct usb_bus *busses;
	struct usb_bus *bus;
    int c, i, a;

    usb_init();
    usb_find_busses();
    usb_find_devices();

    busses = usb_get_busses();

	if(NULL != cameraHandle) return FREENECT_DEVICE_ALREADY_OPEN;

	for (bus = busses; bus; bus = bus->next) {

		struct usb_device *dev;

		for (dev = bus->devices; dev; dev = dev->next) {

			if (dev->descriptor.idProduct == 0x02ae && dev->descriptor.idVendor == 0x045E ) {

				cameraHandle = usb_open(dev);

				if(NULL == cameraHandle) return FREENECT_ERROR_DEVICE_OPEN_FAILED;
				break;

			}
		}

		if(NULL == cameraHandle) break;

	}

	if(NULL == cameraHandle) return FREENECT_ERROR_DEVICE_NOT_FOUND;

	// TODO : do something with the return code
	if(usb_set_configuration(cameraHandle, 1) < 0) return FREENECT_ERROR_DEVICE_OPEN_FAILED;
	// TODO : do something with the return code
	if(usb_claim_interface(cameraHandle, 0) < 0) return FREENECT_ERROR_DEVICE_OPEN_FAILED;
	//if(usb_set_altinterface(cameraHandle, 0) < 0) return FREENECT_ERROR_DEVICE_OPEN_FAILED;

	return FREENECT_OK;

}

struct cam_hdr {
uint8_t magic[2];
uint16_t len;
uint16_t cmd;
uint16_t tag;
};

extern const struct caminit inits[];
extern const int num_inits;

void start_camera_device(){

    uint8_t obuf[0x2000];
    uint8_t ibuf[0x2000];
    struct cam_hdr *chdr = (void*)obuf;
    struct cam_hdr *rhdr = (void*)ibuf;
    int ret;
    int i, j;
    printf("INIT CAMERA\n");

    ret = usb_control_msg(cameraHandle, 0x80, 0x06, 0x3ee, 0, ibuf, 0x12, 160);
    printf("First xfer: %d\n", ret);

    chdr->magic[0] = 0x47;
    chdr->magic[1] = 0x4d;

    for (i=0; i<num_inits; i++) {
        const struct caminit *ip = &inits[i];
        chdr->cmd = ip->command;
        chdr->tag = ip->tag;
        chdr->len = ip->cmdlen / 2;
        memcpy(obuf+sizeof(*chdr), ip->cmddata, ip->cmdlen);

        ret = usb_control_msg(cameraHandle, 0x40, 0, 0, 0, obuf, ip->cmdlen + sizeof(*chdr), 160);

        printf("CTL CMD %04x %04x = %d\n", chdr->cmd, chdr->tag, ret);
        //do {
		ret = usb_control_msg(cameraHandle, 0xc0, 0, 0, 0, ibuf, 0x200, 160);
        //} while (ret == 0);

        printf("CTL RES = %d\n", ret);
        if (rhdr->magic[0] != 0x52 || rhdr->magic[1] != 0x42) {
            printf("Bad magic %02x %02x\n", rhdr->magic[0], rhdr->magic[1]);
            continue;
        }
        if (rhdr->cmd != chdr->cmd) {
            printf("Bad cmd %02x != %02x\n", rhdr->cmd, chdr->cmd);
            continue;
        }
        if (rhdr->tag != chdr->tag) {
            printf("Bad tag %04x != %04x\n", rhdr->tag, chdr->tag);
            continue;
        }
        if (rhdr->len != (ret-sizeof(*rhdr))/2) {
            printf("Bad len %04x != %04x\n", rhdr->len, (int)(ret-sizeof(*rhdr))/2);
            continue;
        }
        if (rhdr->len != (ip->replylen/2) || memcmp(ibuf+sizeof(*rhdr), ip->replydata, ip->replylen)) {
            printf("Expected: ");
            for (j=0; j<ip->replylen; j++) {
                printf("%02x ", ip->replydata[j]);
            }
            printf("\nGot: ");
            for (j=0; j<(rhdr->len*2); j++) {
                printf("%02x ", ibuf[j+sizeof(*rhdr)]);
            }
            printf("\n");
        }
    }

}

void * rgb_contexts[NUM_XFERS];
void * depth_contexts[NUM_XFERS];

static int count = 0;

//HERE WE START THE QUEUE
//from -- http://libusb.6.n5.nabble.com/isochronous-transfer-td3250228.html#a3250228
void setup_isochronous_async(usb_dev_handle *dev){

	int k;
	count = 0;

	for( k = 0; k < NUM_XFERS; k++ ){
		rgb_contexts[k] = NULL;
		depth_contexts[k] = NULL;

		rgb_bufs[k] = malloc(XFER_SIZE_RGB);
		depth_bufs[k] = malloc(XFER_SIZE_RGB);

		usb_isochronous_setup_async(cameraHandle, &rgb_contexts[k], 0x81, PKT_SIZE_RGB);
		usb_isochronous_setup_async(cameraHandle, &depth_contexts[k], 0x82, PKT_SIZE_DEPTH);
	}

	for( k = 0; k < NUM_XFERS; k++ ){
		int ret = usb_submit_async(rgb_contexts[k], (char*)rgb_bufs[k], XFER_SIZE_RGB);
		if( ret < 0 )
		{
			printf("error: %s\n", usb_strerror());
		}
		ret = usb_submit_async(depth_contexts[k], (char*)depth_bufs[k], PKTS_PER_XFER*PKT_SIZE_DEPTH);
		if( ret < 0 )
		{
			printf("error: %s\n", usb_strerror());
		}
	}
}

//HERE WE UPDATE IT FROM glView.c
//TODO: seperate updates for depth and rgb?
void update_isochronous_async(){

    //not sure if this is the right order for keeping the queue full.
    // but the logic came from - http://libusb.6.n5.nabble.com/isochronous-transfer-td3250228.html#a3250228
    // really good read!!!!!

	int read, ret;

	read = usb_reap_async_nocancel(depth_contexts[count], 5000);
    printf("read %d bytes\n", read);
    if( read < 0 ){
        //TODO: don't cancel for -116 (timeout)
        //for -116 just don't advanced the count or queue a new transfer
        printf("error: %s\n", usb_strerror());
        usb_cancel_async(depth_contexts[count]);
    }else if(read > 0){
        //send them to our rgb_callback - note the cb is modified as we don't have the same amount of packet info as libusb
        printf("depth_callback!\n");
        depth_callback(depth_bufs[count], PKT_SIZE_DEPTH);
    }


	ZeroMemory(depth_bufs[count], XFER_SIZE_DEPTH);
    ret = usb_submit_async(depth_contexts[count], (char*)depth_bufs[count], XFER_SIZE_DEPTH);
    if( ret < 0 ){
        printf("error: %s\n", usb_strerror());
        usb_cancel_async(depth_contexts[count]);
    }

    //reap any queued bytes ( ie read them in )
    read = usb_reap_async_nocancel(rgb_contexts[count], 5000);
    printf("read %d bytes\n", read);
    if( read < 0 ){
         //TODO: don't cancel for -116 (timeout)
        //for -116 just don't advanced the count or queue a new transfer
        printf("error: %s\n", usb_strerror());
        usb_cancel_async(rgb_contexts[count]);
    }else if(read > 0){
        //send them to our rgb_callback - note the cb is modified as we don't have the same amount of packet info as libusb
        printf("rgb_callback!\n");
        rgb_callback(rgb_bufs[count], PKT_SIZE_RGB);
    }

    //once we are done - we submit a new request!
	ZeroMemory(rgb_bufs[count], XFER_SIZE_RGB);
    ret = usb_submit_async(rgb_contexts[count], (char*)rgb_bufs[count], XFER_SIZE_RGB);
    if( ret < 0 ){
        printf("error: %s\n", usb_strerror());
        usb_cancel_async(rgb_contexts[count]);
    }

    //increase our count - ie we process one request a loop
    count++;
    if( count >= NUM_XFERS ){
        int k;
		count = 0;
    }

}


void prep_iso_transfers(depthcb dcb, rgbcb rcb){

    depth_cb = dcb;
    rgb_cb = rcb;

    setup_isochronous_async(cameraHandle);
}


enum LIBFREENECT_RETURN_CODE init_motor_device()
{
    struct usb_bus *busses;
    struct usb_bus *bus;
    int c, i, a;

    usb_init();
    usb_find_busses();
    usb_find_devices();

    busses = usb_get_busses();

    if(NULL != motorHandle) return FREENECT_DEVICE_ALREADY_OPEN;

    for (bus = busses; bus; bus = bus->next) {

        struct usb_device *dev;

        for (dev = bus->devices; dev; dev = dev->next) {

            if (dev->descriptor.idProduct == 0x02B0 && dev->descriptor.idVendor == 0x045E) {

                motorHandle = usb_open(dev);

                if(NULL == motorHandle) return FREENECT_ERROR_DEVICE_OPEN_FAILED;
                break;

            }
        }

        if(NULL == motorHandle) break;

    }

    if(NULL == motorHandle) return FREENECT_ERROR_DEVICE_NOT_FOUND;

    // TODO : do something with the return code
    if(usb_set_configuration(motorHandle, 1) < 0) return FREENECT_ERROR_DEVICE_OPEN_FAILED;
    // TODO : do something with the return code
    if(usb_claim_interface(motorHandle, 0) < 0) return FREENECT_ERROR_DEVICE_OPEN_FAILED;

    return FREENECT_OK;

}

enum LIBFREENECT_RETURN_CODE close_motor_device()
{
    if(NULL == motorHandle) return FREENECT_DEVICE_NOT_OPEN;

    if(usb_release_interface(motorHandle, 0) < 0) return FREENECT_ERROR_DEVICE_CLOSE_FAILED;

    if(usb_close(motorHandle) < 0) return FREENECT_ERROR_DEVICE_CLOSE_FAILED;

    motorHandle = NULL;

    return FREENECT_OK;
}

enum LIBFREENECT_RETURN_CODE set_led(enum KinectLEDStatus status)
{
    uint8_t bytes = 0;

    if(NULL == motorHandle) return FREENECT_DEVICE_NOT_OPEN;

    if(usb_control_msg(motorHandle, 0x40, 0x06, (uint16_t)status, 0, &bytes, 0, 0) < 0)
    return FREENECT_ERROR_TRANSFER;

    return FREENECT_OK;
}

enum LIBFREENECT_RETURN_CODE set_motor_tilt(uint8_t tiltValue)
{
    uint8_t bytes = 0;
    uint16_t mappedValue = 0;

    if(NULL == motorHandle) return FREENECT_DEVICE_NOT_OPEN;

    mappedValue = (uint8_t)(0xffd0 + tiltValue / 5);

    if(usb_control_msg(motorHandle, 0x40, 0x31, mappedValue, 0, &bytes, 0, 0) < 0)
    return FREENECT_ERROR_TRANSFER;

    return FREENECT_OK;
}



/*
 *  bayer.h
 *
 *
 *  Ripped unceremoniously from OpenCV -- cvcolor.cpp and constants.h
 *
 */


/*M///////////////////////////////////////////////////////////////////////////////////////
 //  ORIGINAL OPENCV COPYRIGHT NOTICE:
 //  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
 //
 //  By downloading, copying, installing or using the software you agree to this license.
 //  If you do not agree to this license, do not download, install,
 //  copy or use the software.
 //
 //                        Intel License Agreement
 //                For Open Source Computer Vision Library
 //
 // Copyright (C) 2000, Intel Corporation, all rights reserved.
 // Third party copyrights are property of their respective owners.
 //
 // Redistribution and use in source and binary forms, with or without modification,
 // are permitted provided that the following conditions are met:
 //
 //   * Redistribution's of source code must retain the above copyright notice,
 //     this list of conditions and the following disclaimer.
 //
 //   * Redistribution's in binary form must reproduce the above copyright notice,
 //     this list of conditions and the following disclaimer in the documentation
 //     and/or other materials provided with the distribution.
 //
 //   * The name of Intel Corporation may not be used to endorse or promote products
 //     derived from this software without specific prior written permission.
 //
 // This software is provided by the copyright holders and contributors "as is" and
 // any express or implied warranties, including, but not limited to, the implied
 // warranties of merchantability and fitness for a particular purpose are disclaimed.
 // In no event shall the Intel Corporation or contributors be liable for any direct,
 // indirect, incidental, special, exemplary, or consequential damages
 // (including, but not limited to, procurement of substitute goods or services;
 // loss of use, data, or profits; or business interruption) however caused
 // and on any theory of liability, whether in contract, strict liability,
 // or tort (including negligence or otherwise) arising in any way out of
 // the use of this software, even if advised of the possibility of such damage.
 //
 //M*/


/********************************* COPYRIGHT NOTICE *******************************\
 //  ORIGINAL BAYER CODE COPYRIGHT NOTICE:
 Original code for Bayer->BGR/RGB conversion is provided by Dirk Schaefer
 from MD-Mathematische Dienste GmbH. Below is the copyright notice:

 IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
 By downloading, copying, installing or using the software you agree
 to this license. If you do not agree to this license, do not download,
 install, copy or use the software.

 Contributors License Agreement:

 Copyright (c) 2002,
 MD-Mathematische Dienste GmbH
 Im Defdahl 5-10
 44141 Dortmund
 Germany
 www.md-it.de

 Redistribution and use in source and binary forms,
 with or without modification, are permitted provided
 that the following conditions are met:

 Redistributions of source code must retain
 the above copyright notice, this list of conditions and the following disclaimer.
 Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.
 The name of Contributor may not be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 THE POSSIBILITY OF SUCH DAMAGE.
 \**********************************************************************************/


/* Ripped from OpenCV constants.h */
#ifndef CV_BayerBG2BGR
    #define  CV_BayerBG2BGR 46
#endif
#ifndef CV_BayerGB2BGR
    #define  CV_BayerGB2BGR 47
#endif
#ifndef CV_BayerRG2BGR
    #define  CV_BayerRG2BGR 48
#endif
#ifndef CV_BayerGR2BGR
    #define  CV_BayerGR2BGR 49
#endif
#ifndef CV_BayerBG2RGB
    #define  CV_BayerBG2RGB CV_BayerRG2BGR
#endif
#ifndef CV_BayerGB2RGB
    #define  CV_BayerGB2RGB CV_BayerGR2BGR
#endif
#ifndef CV_BayerRG2RGB
    #define  CV_BayerRG2RGB CV_BayerBG2BGR
#endif
#ifndef CV_BayerGR2RGB
    #define  CV_BayerGR2RGB CV_BayerGB2BGR
#endif


/****************************************************************************************\
 *                            Bayer Pattern -> RGB conversion
 \****************************************************************************************/


static void bayer2BGR_8u_C1C3R(    const unsigned char* bayer0,
                            int bayer_step,
                            unsigned char* dst0, int dst_step,
                            int img_width, int img_height, int code );


static void bayer_to_rgb( const unsigned char* bayer,  unsigned char* rgb, int w, int h){
    bayer2BGR_8u_C1C3R(bayer, w, rgb, w*3, w, h, CV_BayerGB2RGB);
}

static void bayer2BGR_8u_C1C3R(    const unsigned char* bayer0,
                            int bayer_step,
                            unsigned char* dst0, int dst_step,
                            int img_width, int img_height, int code )
{
    int blue = code == CV_BayerBG2BGR || code == CV_BayerGB2BGR ? -1 : 1;
    int start_with_green = code == CV_BayerGB2BGR || code == CV_BayerGR2BGR;

    memset( dst0, 0, img_width*3*sizeof(dst0[0]) );
    memset( dst0 + (img_height - 1)*dst_step, 0, img_width*3*sizeof(dst0[0]) );
    dst0 += dst_step + 3 + 1;
    img_height -= 2;
    img_width -= 2;

    int bayer_step2 = bayer_step*2; // small optimization.

    for( ; img_height-- > 0; bayer0 += bayer_step, dst0 += dst_step )
    {
        int t0, t1;
        const unsigned char* bayer = bayer0;
        unsigned char* dst = dst0;
        const unsigned char* bayer_end = bayer + img_width;

        dst[-4] = dst[-3] = dst[-2] = dst[img_width*3-1] =
        dst[img_width*3] = dst[img_width*3+1] = 0;

        if( img_width <= 0 )
            continue;

        if( start_with_green )
        {
            t0 = (bayer[1] + bayer[bayer_step2+1] + 1) >> 1;
            t1 = (bayer[bayer_step] + bayer[bayer_step+2] + 1) >> 1;
            dst[-blue] = (unsigned char)t0;
            dst[0] = bayer[bayer_step+1];
            dst[blue] = (unsigned char)t1;
            bayer++;
            dst += 3;
        }

        if( blue > 0 )
        {
            for( ; bayer <= bayer_end - 2; bayer += 2, dst += 6 )
            {
                t0 = (bayer[0] + bayer[2] + bayer[bayer_step2] +
                      bayer[bayer_step2+2] + 2) >> 2;
                t1 = (bayer[1] + bayer[bayer_step] +
                      bayer[bayer_step+2] + bayer[bayer_step2+1]+2) >> 2;
                dst[-1] = (unsigned char)t0;
                dst[0]  = (unsigned char)t1;
                dst[1]  = bayer[bayer_step+1];

                t0 = (bayer[2] + bayer[bayer_step2+2] + 1) >> 1;
                t1 = (bayer[bayer_step+1] + bayer[bayer_step+3] + 1) >> 1;
                dst[2] = (unsigned char)t0;
                dst[3] = bayer[bayer_step+2];
                dst[4] = (unsigned char)t1;
            }
        }
        else
        {
            for( ; bayer <= bayer_end - 2; bayer += 2, dst += 6 )
            {
                t0 = (bayer[0] + bayer[2] + bayer[bayer_step2] +
                      bayer[bayer_step2+2] + 2) >> 2;
                t1 = (bayer[1] + bayer[bayer_step] +
                      bayer[bayer_step+2] + bayer[bayer_step2+1]+2) >> 2;
                dst[1]  = (unsigned char)t0;
                dst[0]  = (unsigned char)t1;
                dst[-1] = bayer[bayer_step+1];

                t0 = (bayer[2] + bayer[bayer_step2+2] + 1) >> 1;
                t1 = (bayer[bayer_step+1] + bayer[bayer_step+3] + 1) >> 1;
                dst[4] = (unsigned char)t0;
                dst[3] = bayer[bayer_step+2];
                dst[2] = (unsigned char)t1;
            }
        }

        if( bayer < bayer_end )
        {
            t0 = (bayer[0] + bayer[2] + bayer[bayer_step2] +
                  bayer[bayer_step2+2] + 2) >> 2;
            t1 = (bayer[1] + bayer[bayer_step] +
                  bayer[bayer_step+2] + bayer[bayer_step2+1]+2) >> 2;
            dst[-blue] = (unsigned char)t0;
            dst[0] = (unsigned char)t1;
            dst[blue] = bayer[bayer_step+1];
            bayer++;
            dst += 3;
        }

        blue = -blue;
        start_with_green = !start_with_green;
    }

    return;
}
