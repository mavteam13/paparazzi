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
 * @file "modules/navteam13/nav_team13.h"
 * @author MAVgroup13
 * Set waypoint to safe area within field of view
 */

#ifndef NAV_TEAM13_H
#define NAV_TEAM13_H


#include "std.h"
#include "firmwares/rotorcraft/navigation.h"

// variable declarations


// Macros
#define ObstacleInPath() obstacle_in_path()
#define ObstacleNearby() obstacle_nearby()


//******* FUNCTIONS *******//
void nav_team13_init(void);
extern bool_t NavSetWaypointTowardsHeading(uint8_t curr, uint8_t dist, uint8_t next);
extern bool_t NavSetWaypointTowardsHeadingNew(uint8_t curr, uint8_t dist, uint8_t next, uint8_t heading, uint8_t global, uint8_t global_prev);
extern bool_t NavSetWaypointAvoidInBounds(uint8_t curr, uint8_t dist, uint8_t next);
extern bool_t offset_wp_cm(uint8_t wp1, uint8_t wp2, uint8_t d);
extern bool_t flag_wp1(void);
extern bool_t flag_wp2(void);
extern bool_t stereo_init(uint8_t wpfoto);
extern bool_t stereo_loop(uint8_t wpfoto);

bool_t move_global_wp(uint8_t glob,uint8_t fz1,uint8_t fz2,uint8_t fz3,uint8_t fz4,uint8_t nxt,uint8_t curr);
bool_t move_global_wp_new(uint8_t glob,uint8_t fz1,uint8_t fz2,uint8_t fz3,uint8_t fz4,uint8_t nxt,uint8_t curr,uint8_t heading, uint8_t glob_prev);
bool_t obstacle_in_path(void);
bool_t obstacle_nearby(void);

extern int stereo_nav_status;
extern int wait_time;


#endif

