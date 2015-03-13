/*
 * Copyright (C) 2012-2014 The Paparazzi Community
 *               2015 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file modules/computer_vision/viewvideo.c
 *
 * Get live images from a RTP/UDP stream
 * and save pictures on internal memory
 *
 * Works on Linux platforms
 */

// Own header
#include "modules/computer_vision/viewvideo.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <math.h>
#include "mcu_periph/udp.h"
#include <errno.h>

// Video
#include "lib/v4l/v4l2.h"
#include "cv/resize.h"
#include "cv/encoding/jpeg.h"
#include "cv/encoding/rtp.h"

// Threaded computer vision
#include <pthread.h>

// The video device
#ifndef VIEWVIDEO_DEVICE
#define VIEWVIDEO_DEVICE /dev/video1
#endif
PRINT_CONFIG_VAR(VIEWVIDEO_DEVICE);

// The video device size (width, height)
#ifndef VIEWVIDEO_DEVICE_SIZE
#define VIEWVIDEO_DEVICE_SIZE 1280,720
#endif
#define __SIZE_HELPER(x, y) #x", "#y
#define _SIZE_HELPER(x) __SIZE_HELPER(x)
PRINT_CONFIG_MSG("VIEWVIDEO_DEVICE_SIZE = " _SIZE_HELPER(VIEWVIDEO_DEVICE_SIZE));

// The video device buffers (the amount of V4L2 buffers)
#ifndef VIEWVIDEO_DEVICE_BUFFERS
#define VIEWVIDEO_DEVICE_BUFFERS 10
#endif
PRINT_CONFIG_VAR(VIEWVIDEO_DEVICE_BUFFERS);

// Downsize factor for video stream
#ifndef VIEWVIDEO_DOWNSIZE_FACTOR
#define VIEWVIDEO_DOWNSIZE_FACTOR 4
#endif
PRINT_CONFIG_VAR(VIEWVIDEO_DOWNSIZE_FACTOR);

// From 0 to 99 (99=high)
#ifndef VIEWVIDEO_QUALITY_FACTOR
#define VIEWVIDEO_QUALITY_FACTOR 50
#endif
PRINT_CONFIG_VAR(VIEWVIDEO_QUALITY_FACTOR);

// Frames Per Seconds
#ifndef VIEWVIDEO_FPS
#define VIEWVIDEO_FPS 4
#endif
PRINT_CONFIG_VAR(VIEWVIDEO_FPS);

// The place where the shots are saved (without slash on the end)
#ifndef VIEWVIDEO_SHOT_PATH
#define VIEWVIDEO_SHOT_PATH "/data/video/images"
#endif
PRINT_CONFIG_VAR(VIEWVIDEO_SHOT_PATH);

// Check if we are using netcat instead of RTP/UDP
#ifndef VIEWVIDEO_USE_NETCAT
#define VIEWVIDEO_USE_NETCAT FALSE
#endif
#if VIEWVIDEO_USE_NETCAT
PRINT_CONFIG_MSG("[viewvideo] Using netcat.");
#else
PRINT_CONFIG_MSG("[viewvideo] Using RTP/UDP stream.");
PRINT_CONFIG_VAR(VIEWVIDEO_DEV);
#endif

/* These are defined with configure */
PRINT_CONFIG_VAR(VIEWVIDEO_HOST);
PRINT_CONFIG_VAR(VIEWVIDEO_PORT_OUT);

// Main thread
static void *viewvideo_thread(void *data);
void viewvideo_periodic(void) { }

// Initialize the viewvideo structure with the defaults
struct viewvideo_t viewvideo = {
  .is_streaming = FALSE,
  .downsize_factor = VIEWVIDEO_DOWNSIZE_FACTOR,
  .quality_factor = VIEWVIDEO_QUALITY_FACTOR,
  .fps = VIEWVIDEO_FPS,
  .take_shot = FALSE,
  .shot_number = 0
};


/**
 * Handles all the video streaming and saving of the image shots
 * This is a sepereate thread, so it needs to be thread safe!
 */
static void *viewvideo_thread(void *data __attribute__((unused)))
{
  // Start the streaming of the V4L2 device
  if (!v4l2_start_capture(viewvideo.dev)) {
    printf("[viewvideo-thread] Could not start capture of %s.\n", viewvideo.dev->name);
    return 0;
  }

  // Resize image if needed
  struct img_struct small;
  small.w = viewvideo.dev->w / viewvideo.downsize_factor;
  small.h = viewvideo.dev->h / viewvideo.downsize_factor;
  if (viewvideo.downsize_factor != 1) {
    small.buf = (uint8_t *)malloc(small.w * small.h * 2);
  } else {
    small.buf = NULL;
  }

  // JPEG compression (8.25 bits are required for a 100% quality image, margin of ~0.55)
  uint8_t *jpegbuf = (uint8_t *)malloc(ceil(small.w * small.h * 1.1));

  // time
  uint32_t microsleep = (uint32_t)(1000000. / (float)viewvideo.fps);
  struct timeval last_time;
  gettimeofday(&last_time, NULL);

#if VIEWVIDEO_USE_NETCAT
  char nc_cmd[64];
  sprintf(nc_cmd, "nc 192.168.1.3 5000 2>/dev/null");
#endif

  // Start streaming
  viewvideo.is_streaming = TRUE;
  while (viewvideo.is_streaming) {
    // compute usleep to have a more stable frame rate
    struct timeval vision_thread_sleep_time;
    gettimeofday(&vision_thread_sleep_time, NULL);
    int dt = (int)(vision_thread_sleep_time.tv_sec - last_time.tv_sec) * 1000000 + (int)(vision_thread_sleep_time.tv_usec - last_time.tv_usec);
    if (dt < microsleep) { usleep(microsleep - dt); }
    last_time = vision_thread_sleep_time;

    // Wait for a new frame (blocking)
    struct v4l2_img_buf *img = v4l2_image_get(viewvideo.dev);

    // Check if we need to take a shot
    if (viewvideo.take_shot) {
      // Create a high quality image (99% JPEG encoded)
      uint8_t *jpegbuf_hr = (uint8_t *)malloc(ceil(viewvideo.dev->w * viewvideo.dev->h * 1.1));
      uint8_t *end = jpeg_encode_image(img->buf, jpegbuf_hr, 99, FOUR_TWO_TWO, viewvideo.dev->w, viewvideo.dev->h, TRUE);
      uint32_t size = end - (jpegbuf_hr);

      // Search for a file where we can write to
      char save_name[128];
      for (; viewvideo.shot_number < 99999; viewvideo.shot_number++) {
        sprintf(save_name, "%s/img_%05d.jpg", STRINGIFY(VIEWVIDEO_SHOT_PATH), viewvideo.shot_number);
        // Check if file exists or not
        if (access(save_name, F_OK) == -1) {
          FILE *fp = fopen(save_name, "w");
          if (fp == NULL) {
            printf("[viewvideo-thread] Could not write shot %s.\n", save_name);
          } else {
            // Save it to the file and close it
            fwrite(jpegbuf_hr, sizeof(uint8_t), size, fp);
            fclose(fp);
          }

          // We don't need to seek for a next index anymore
          break;
        }
      }

      // We finished the shot
      free(jpegbuf_hr);
      viewvideo.take_shot = FALSE;
    }

    // Only resize when needed
    if (viewvideo.downsize_factor != 1) {
      struct img_struct input;
      input.buf = img->buf;
      input.w = viewvideo.dev->w;
      input.h = viewvideo.dev->h;
      resize_uyuv(&input, &small, viewvideo.downsize_factor);
    } else {
      small.buf = img->buf;
    }

    // JPEG encode the image:
    uint8_t *end = jpeg_encode_image(small.buf, jpegbuf, VIEWVIDEO_QUALITY_FACTOR, FOUR_TWO_TWO, small.w, small.h, VIEWVIDEO_USE_NETCAT);
    uint32_t size = end - (jpegbuf);

#if VIEWVIDEO_USE_NETCAT
    // Open process to send using netcat (in a fork because sometimes kills itself???)
    pid_t pid = fork();

    if(pid < 0) {
      printf("[viewvideo] Could not create netcat fork.\n");
    }
    else if(pid ==0) {
      // We are the child and want to send the image
      FILE *netcat = popen("nc 192.168.1.3 5000 2>/dev/null", "w");
      if (netcat != NULL) {
        fwrite(jpegbuf, sizeof(uint8_t), size, netcat);
        pclose(netcat); // Ignore output, because it is too much when not connected
      } else {
        printf("[viewvideo] Failed to open netcat process. %d: %s\n", errno, strerror(errno));
      }

      // Exit the program since we don't want to continue after transmitting
      exit(0);
    }
    else {
      // We want to wait until the child is finished
      wait(NULL);
    }
#else
    // Send image with RTP
    rtp_frame_send(
      &VIEWVIDEO_DEV,           // UDP device
      jpegbuf, size,            // JPEG
      small.w, small.h,         // Img Size
      0,                        // Format 422
      VIEWVIDEO_QUALITY_FACTOR, // Jpeg-Quality
      0,                        // DRI Header
      0                         // 90kHz time increment
    );
    // Extra note: when the time increment is set to 0,
    // it is automaticaly calculated by the send_rtp_frame function
    // based on gettimeofday value. This seems to introduce some lag or jitter.
    // An other way is to compute the time increment and set the correct value.
    // It seems that a lower value is also working (when the frame is received
    // the timestamp is always "late" so the frame is displayed immediately).
    // Here, we set the time increment to the lowest possible value
    // (1 = 1/90000 s) which is probably stupid but is actually working.
#endif

    // Free the image
    v4l2_image_free(viewvideo.dev, img);
  }

  // Free all buffers
  free(jpegbuf);
  if (viewvideo.downsize_factor != 1)
    free(small.buf);
  return 0;
}

/**
 * Initialize the view video
 */
void viewvideo_init(void)
{
#ifdef VIEWVIDEO_SUBDEV
  PRINT_CONFIG_MSG("[viewvideo] Configuring a subdevice!");
  PRINT_CONFIG_VAR(VIEWVIDEO_SUBDEV);

  // Initialize the V4L2 subdevice (TODO: fix hardcoded path, which and code)
  if (!v4l2_init_subdev(STRINGIFY(VIEWVIDEO_SUBDEV), 0, 1, V4L2_MBUS_FMT_UYVY8_2X8, VIEWVIDEO_DEVICE_SIZE)) {
    printf("[viewvideo] Could not initialize the %s subdevice.\n", STRINGIFY(VIEWVIDEO_SUBDEV));
    return;
  }
#endif

  // Initialize the V4L2 device
  viewvideo.dev = v4l2_init(STRINGIFY(VIEWVIDEO_DEVICE), VIEWVIDEO_DEVICE_SIZE, VIEWVIDEO_DEVICE_BUFFERS);
  if (viewvideo.dev == NULL) {
    printf("[viewvideo] Could not initialize the %s V4L2 device.\n", STRINGIFY(VIEWVIDEO_DEVICE));
    return;
  }

  // Create the shot directory
  char save_name[128];
  sprintf(save_name, "mkdir -p %s", STRINGIFY(VIEWVIDEO_SHOT_PATH));
  if (system(save_name) != 0) {
    printf("[viewvideo] Could not create shot directory %s.\n", STRINGIFY(VIEWVIDEO_SHOT_PATH));
    return;
  }

#if VIEWVIDEO_USE_NETCAT
  // Create an Netcat receiver file for the streaming
  sprintf(save_name, "/data/video/images/netcat-recv.sh");
  FILE *fp = fopen(save_name, "w");
  if (fp != NULL) {
    fprintf(fp, "i=0\n");
    fprintf(fp, "while true\n");
    fprintf(fp, "do\n");
    fprintf(fp, "\tn=$(printf \"%%04d\" $i)\n");
    fprintf(fp, "\tnc -l 0.0.0.0 %d > img_${n}.jpg\n", (int)(VIEWVIDEO_PORT_OUT));
    fprintf(fp, "\ti=$((i+1))\n");
    fprintf(fp, "done\n");
    fclose(fp);
  } else {
  printf("failed %d: %s\n", errno, strerror(errno));
}
#else
  // Create an SDP file for the streaming
  sprintf(save_name, "%s/stream.sdp", STRINGIFY(VIEWVIDEO_SHOT_PATH));
  FILE *fp = fopen(save_name, "w+");
  if (fp != NULL) {
    fprintf(fp, "v=0\n");
    fprintf(fp, "m=video %d RTP/AVP 26\n", (int)(VIEWVIDEO_PORT_OUT));
    fprintf(fp, "c=IN IP4 0.0.0.0\n");
    fclose(fp);
  }
#endif
}

/**
 * Start with streaming
 */
void viewvideo_start(void)
{
  // Check if we are already running
  if (viewvideo.is_streaming) {
    return;
  }

  // Start the streaming thread
  pthread_t tid;
  if (pthread_create(&tid, NULL, viewvideo_thread, NULL) != 0) {
    printf("[vievideo] Could not create streaming thread.\n");
    return;
  }
}

/**
 * Stops the streaming
 * This could take some time, because the thread is stopped asynchronous.
 */
void viewvideo_stop(void)
{
  // Check if not already stopped streaming
  if (!viewvideo.is_streaming) {
    return;
  }

  // Stop the streaming thread
  viewvideo.is_streaming = FALSE;

  // Stop the capturing
  if (!v4l2_stop_capture(viewvideo.dev)) {
    printf("[viewvideo] Could not stop capture of %s.\n", viewvideo.dev->name);
    return;
  }

  // TODO: wait for the thread to finish to be able to start the thread again!
}

/**
 * Take a shot and save it
 * This will only work when the streaming is enabled
 */
void viewvideo_take_shot(bool_t take)
{
  viewvideo.take_shot = take;
}
