/*
 * Copyright (C) MAVgroup13
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/detect_avoid_mav13/detect_avoid_mav13.c"
 * @author MAVgroup13
 * Detect a specific colour and avoid it or track it
 */
 
 
 
// Remark for sending data: uint32 is atomic --> can be read within one clock tic
 

// own header file
#include "modules/detect_avoid_mav13/detect_avoid_mav13.h"
#include "stdio.h"
#include "string.h"

//#include "navigation.h"


////////////////////////////////////////////////////////////////////////

// obstacle detection and safe heading estimation based on color filtering
#include "detect_obstacle.h"

// Color filter settings - YCbCr color space
#include "color_mod.h"

// orange: Cr 1 & Cb -1
// light green: Cr -1 & Cb -1
// magenta: Cr 1 & Cb 1
// cyan: Cr -1 & Cb 1


// Intensity -1 --> 1
uint8_t color_lum_min = 105; // 105
uint8_t color_lum_max = 200; // 205
// Cb -1 --> 1
uint8_t color_cb_min  = 50; // 52
uint8_t color_cb_max  = 128; // 140
// Cr -1 --> 1
uint8_t color_cr_min  = 160; // 180
uint8_t color_cr_max  = 200; // 255

int color_count;

int color_detected = 0;
int color_tresh = 1200;

int safe_heading = 0;

/////////////////////////////////////////////////////////////////////////
// COMPUTER VISION THREAD

// Video
#include "modules/computer_vision/lib/v4l/video.h"
#include "modules/computer_vision/cv/resize.h"

#include "modules/computer_vision/cv/encoding/jpeg.h"
#include "modules/computer_vision/cv/encoding/rtp.h"

// UDP RTP Images
#include "modules/computer_vision/lib/udp/socket.h"

// Threaded computer vision
#include <pthread.h>

pthread_t computervision_thread;
volatile uint8_t computervision_thread_status = 0;
volatile uint8_t computer_vision_thread_command = 0;
void *computervision_thread_main(void* data);
void *computervision_thread_main(void* data)
{
  // Video Input
  struct vid_struct vid;
  vid.device = (char*)"/dev/video1";
  vid.w=1280;
  vid.h=720;
  vid.n_buffers = 4;
  if (video_init(&vid)<0) {
    printf("Error initialising video\n");
    computervision_thread_status = -1;
    return 0;
  }

  // Video Grabbing
  struct img_struct* img_new = video_create_image(&vid);

  // Video Resizing
  #define DOWNSIZE_FACTOR   4
  uint8_t quality_factor = 75; // From 0 to 99 (99=high)
  uint8_t dri_jpeg_header = 0;
  int millisleep = 250;

  struct img_struct small;
  small.w = vid.w / DOWNSIZE_FACTOR;
  small.h = vid.h / DOWNSIZE_FACTOR;
  small.buf = (uint8_t*)malloc(small.w*small.h*2);
  
  // Video Compression
  uint8_t* jpegbuf = (uint8_t*)malloc(vid.h*vid.w*2);

  // Network Transmit
  struct UdpSocket* vsock;
  vsock = udp_socket("192.168.1.255", 5000, 5001, FMS_BROADCAST);

  while (computer_vision_thread_command > 0)
  {
    usleep(1000* millisleep);
    video_grab_image(&vid, img_new);

    // Resize: device by 4
    resize_uyuv(img_new, &small, DOWNSIZE_FACTOR);
    
    
    ////////////////////////////////////////////////////////////////////////
    // Obstacle detection by color filtering

    // filter selected colors and sort the filtered pixels in n segments    
    
    // int n_segments=5; // only 5 is working now !!! (otherwise changes required below, in color_mod.h and in detect_obstacle.h)
    int result[5] = { 0 }; 
    
    color_count = colorfilt_uyvy_mod(&small,&small,
        color_lum_min,color_lum_max,
        color_cb_min,color_cb_max,
        color_cr_min,color_cr_max,
        &result, 5);

    //printf("ColorDetected L --> R = %d %d %d %d %d \n", result[0], result[1], result[2], result[3], result[4]);

    
    // compute safe heading based on above data    
    safe_heading=detectobst(color_count, &result, color_tresh, 5);

    //printf("Safe heading = %d \n", safe_heading);
    
    //////////////////////////////////////////////////////////////////////
    
    // Video transmission
    
    
    
    // JPEG encode the image:
    uint32_t image_format = FOUR_TWO_TWO;  // format (in jpeg.h)
    uint8_t* end = encode_image (small.buf, jpegbuf, quality_factor, image_format, small.w, small.h, dri_jpeg_header);
    uint32_t size = end-(jpegbuf);

    //printf("Sending an image ...%u\n",size);

    send_rtp_frame(
        vsock,            // UDP
        jpegbuf,size,     // JPEG
        small.w, small.h, // Img Size
        0,                // Format 422
        quality_factor,               // Jpeg-Quality
        dri_jpeg_header,                // DRI Header
        0              // 90kHz time increment
     );
    
     
  }
  printf("Thread Closed\n");
  video_close(&vid);
  computervision_thread_status = -100;
  return 0;
}

////////////////////////////////////////////////////////////////


void detect_avoid_init(void) {}

void detect_avoid_run(void) {}

void detect_avoid_start(void)
{
  computer_vision_thread_command = 1;
  int rc = pthread_create(&computervision_thread, NULL, computervision_thread_main, NULL);
  if(rc) {
    printf("ctl_Init: Return code from pthread_create(mot_thread) is %d\n", rc);
  }
}

void detect_avoid_stop(void)
{
  computer_vision_thread_command = 0;
}


//void detect_avoid_callback(void) {}

/*///////////////////////////////// Nav functions   ///////////////////////////

#include "generated/airframe.h"
#include <time.h>
#include <stdlib.h>

#include "messages.h"
#include "mcu_periph/uart.h"
#include "subsystems/datalink/downlink.h"
#include "generated/flight_plan.h" 
#include "math/pprz_algebra_int.h"


bool_t NavSetWaypointTowardsHeading(uint8_t curr, uint8_t dist, uint8_t next){
  int32_t s_heading, c_heading;
  int16_t offset_heading;
  
  offset_heading = INT32_RAD_OF_DEG(safe_heading << (INT32_ANGLE_FRAC));
  printf("nav_heading= %d \n", nav_heading);
  printf("offset_heading= %d \n", offset_heading);
  PPRZ_ITRIG_SIN(s_heading, nav_heading+offset_heading);
  PPRZ_ITRIG_COS(c_heading, nav_heading+offset_heading);
  waypoints[next].x = waypoints[curr].x + INT_MULT_RSHIFT(dist,s_heading,INT32_TRIG_FRAC-INT32_POS_FRAC);
  waypoints[next].y = waypoints[curr].y + INT_MULT_RSHIFT(dist,c_heading,INT32_TRIG_FRAC-INT32_POS_FRAC);

  printf("heading error= %d \n", safe_heading);
  return FALSE;
}
*/


