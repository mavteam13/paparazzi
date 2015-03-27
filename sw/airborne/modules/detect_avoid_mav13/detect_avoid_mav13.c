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
#include "unistd.h"


////////////////////////////////////////////////////////////////////////
// GENERAL VARIABLES (independent on the chosen vision algorithm)

int obstac[5] = { 0 }; 
int safe_heading = 0;
int obs_2sect_front = 0;


// vision algorithm switch:
int vision_switch = 2; // 1 - color detection
// 2 - stereo vision
// Should we define this as "extern" to be able to switch from the GUI?


////////////////////////////////////////////////////////////////////////
// COLOR DETECTION INCLUDES AND VARIABLES

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
int result[5] = { 0 }; 


////////////////////////////////////////////////////////////////////////
// STEREO VISION INCLUDES AND VARIABLES



// Stereo vision variables and includes to be pasted here

#include "modules/mavteam13/nav_team13.h"
#include "modules/computer_vision/edgefilter.h"


int intern_nav_status=0;



/////////////////////////////////////////////////////////////////////////
// COMPUTER VISION THREAD - independent on vision algorithm

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

    // Define image structures
    struct img_struct small;
    small.w = vid.w / DOWNSIZE_FACTOR;
    small.h = vid.h / DOWNSIZE_FACTOR;
    small.buf = (uint8_t*)malloc(small.w*small.h*2);

        struct img_struct small_edge;
    small_edge.w = vid.w / DOWNSIZE_FACTOR;
    small_edge.h = vid.h / DOWNSIZE_FACTOR;
    small_edge.buf = (uint8_t *)malloc(small_edge.w *small_edge.h * 2);

    struct img_struct small_blur1;
    small_blur1.w = vid.w / DOWNSIZE_FACTOR;
    small_blur1.h = vid.h / DOWNSIZE_FACTOR;
    small_blur1.buf = (uint8_t *)malloc(small_blur1.w *small_blur1.h * 2);


    struct img_struct small_blur2;
    small_blur2.w = vid.w / DOWNSIZE_FACTOR;
    small_blur2.h = vid.h / DOWNSIZE_FACTOR;
    small_blur2.buf = (uint8_t *)malloc(small_blur2.w *small_blur2.h * 2);


    struct img_struct small_frame1;
    small_frame1.w = vid.w / DOWNSIZE_FACTOR;
    small_frame1.h = vid.h / DOWNSIZE_FACTOR;
    small_frame1.buf = (uint8_t *)malloc(small_frame1.w *small_frame1.h * 2);


    struct img_struct small_frame2;
    small_frame2.w = vid.w / DOWNSIZE_FACTOR;
    small_frame2.h = vid.h / DOWNSIZE_FACTOR;
    small_frame2.buf = (uint8_t *)malloc(small_frame2.w *small_frame2.h * 2);

    struct img_struct small_diff;
    small_diff.w = vid.w / DOWNSIZE_FACTOR;
    small_diff.h = vid.h / DOWNSIZE_FACTOR;
    small_diff.buf = (uint8_t *)malloc(small_diff.w *small_diff.h * 2);


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


        switch (vision_switch)
        {
        ////////////////////////////////////////////////////////////////////////
        // DETECTION BY COLOR FILTERING
        case 1 :

            // reset the result array to zero
            for (int i=0; i<5; i++) { result[i] = 0; }

            // filter selected colors and sort the filtered pixels in 5 segments
            color_count = colorfilt_uyvy_mod(&small,&small,
                                             color_lum_min,color_lum_max,
                                             color_cb_min,color_cb_max,
                                             color_cr_min,color_cr_max,
                                             &result, 5);

           // printf("Color Detected L --> R = %d %d %d %d %d \n", result[0], result[1], result[2], result[3], result[4]);

            // compute safe heading based on above data
            safe_heading=detectobst(color_count, &result, &obstac, color_tresh, 5);

            break;

            ////////////////////////////////////////////////////////////////////////
            // DETECTION BY STEREO VISION
        case 2 :


            if(stereo_nav_status==1&&intern_nav_status==0){
                // blur_filter(&small,&small_blur,Gsize,sigma);
                printf("Taking first image\n");

                memcpy(small_frame1.buf,small.buf,small.h*small.w*2);

                printf("Saving first image\n");

                stereo_vision_status=1;
                printf("Flag send for first image\n");

                intern_nav_status++;
            }

            if(stereo_nav_status==2&&intern_nav_status==1){

                printf("Taking second image\n");
                //blur_filter(&small,&small_blur,Gsize,sigma);

                memcpy(small_frame2.buf,small.buf,small.h*small.w*2);
                //sobel_edge_filter(&small_frame1, &small_sobel);
                //blur_filter(&small_frame2,&small_blur)

                stereo_vision_status=2;

                intern_nav_status++;


            }

            if(stereo_nav_status==2&&intern_nav_status==2){
                printf("Processing stereo image\n");

                blur_filter(&small_frame1,&small_blur1);
                blur_filter(&small_frame2,&small_blur2);

                image_difference(&small_blur1,&small_blur2,&small_diff);




                uint8_t pxlcnt_lines[small.w];
                uint8_t pxlcnt_lines_bin[small.w];

                int safe_head;

                safe_head=detect_vertical_lines(&small_diff,&small_edge,&pxlcnt_lines,&pxlcnt_lines_bin);



               //  uint8_t pxcnt_size=5;
              //  uint32_t pxcnt[5]={0};

              //  int pxcnt_tot;
               //  pxcnt_tot=pixelcount(&small_edge,&pxcnt,pxcnt_size);

               // int color_tresh=600;


                //uint32_t obst[5]={0};
                  //safe_head=detectobst(pxcnt_tot, pxcnt,obst, color_tresh, 5);

                 //printf("Obstacles L --> R = %d %d %d %d %d \n", obst[0], obst[1], obst[2], obst[3], obst[4]);
                 printf("Safe heading = %d \n", safe_head);

                 // intern_nav_status=0;


            }

            //for tuning
            if(stereo_nav_status==0&&intern_nav_status==2) {
                intern_nav_status=0;
                stereo_vision_status=0;

                printf("Stereo vision navigation off\n");
            }


            // the following variables should store the results:
            // safe_heading
            // obstac[5]

            break;
        }

        //////////////////////////////////////////////////////////////////////
        // Common code independent on vision algorithm

      //  printf("Obstacles Detected L --> R = %d %d %d %d %d \n", obstac[0], obstac[1], obstac[2], obstac[3], obstac[4]);
       // printf("Safe heading = %d \n", safe_heading);

        //compute if obstacle occupies 2 sections including forward section
        // we are approximately within 1 meter of a pole if this is true. - Jaime
        if (safe_heading==0){obs_2sect_front = 0;}
        else if (obstac[2]==1 && obstac[3]==1){obs_2sect_front = 1;}
        else if (obstac[2]==1 && obstac[1]==1){obs_2sect_front = 1;}
        else {obs_2sect_front = 0;}

      //  printf("2 obstacles in front = %d \n", obs_2sect_front);


        /////////////////////////////////////////////////////////////////////
        // Video transmission - can be commented if video stream is not needed

        // JPEG encode the image:
        uint32_t image_format = FOUR_TWO_TWO;  // format (in jpeg.h)
        uint8_t* end = encode_image (small_edge.buf, jpegbuf, quality_factor, image_format, small.w, small.h, dri_jpeg_header);
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
