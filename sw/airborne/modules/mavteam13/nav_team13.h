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

//******* FUNCTIONS *******//
extern bool_t NavSetWaypointTowardsHeading(uint8_t curr, uint8_t dist, uint8_t next);
extern bool_t offset_wp_cm(uint8_t wp1, uint8_t wp2, uint8_t d);
bool_t obstacle_in_path(void);



#endif

