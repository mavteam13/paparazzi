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
 * @file "modules/detect_avoid_mav13/detect_avoid_mav13.h"
 * @author MAVgroup13
 * Detect a specific colour and avoid it or track it
 */

#ifndef DETECT_AVOID_MAV13_H
#define DETECT_AVOID_MAV13_H

#include <stdint.h>

extern void detect_avoid_init(void);
extern void detect_avoid_start(void);
extern void detect_avoid_run(void);
extern void detect_avoid_stop(void);
//extern void detect_avoid_message(void);
//extern void detect_avoid_callback(void);

extern uint8_t color_lum_min;
extern uint8_t color_lum_max;

extern uint8_t color_cb_min;
extern uint8_t color_cb_max;

extern uint8_t color_cr_min;
extern uint8_t color_cr_max;

extern int color_count;
extern int color_detected;

extern int obstacle_heading;
extern int safe_heading;

#endif

