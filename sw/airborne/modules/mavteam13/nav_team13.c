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
 * @file "modules/navteam13/nav_team13.c"
 * @author MAVgroup13
 * Set waypoint to safe area within field of view
 */
 
 
#include "nav_team13.h"
#include "generated/airframe.h"
#include <time.h>
#include <stdlib.h>

#include "messages.h"
#include "mcu_periph/uart.h"
#include "subsystems/datalink/downlink.h"
#include "generated/flight_plan.h" 
#include "math/pprz_algebra_int.h"

//****** Declare variables ******//
  int32_t s_heading, c_heading;
  int16_t offset_heading;
  int safe_heading;

//****** Functions ******//

void nav_team13_init(void) {
	printf("init1\n");
	srand(time(NULL));
	
}

bool_t NavSetWaypointTowardsHeading(uint8_t curr, uint8_t dist, uint8_t next){
// distance in cm's

// random heading (angle) -32,-16,0,16,32 degrees
//  safe_heading = ((rand() % 5) * 16) - 32;
// safe_heading = 45;  //hack for sim testing

  offset_heading = INT32_RAD_OF_DEG(safe_heading << (INT32_ANGLE_FRAC));
  
  printf("nav_heading= %d \n", nav_heading);
  printf("offset_heading= %d \n", offset_heading);
  PPRZ_ITRIG_SIN(s_heading, nav_heading+offset_heading);
  PPRZ_ITRIG_COS(c_heading, nav_heading+offset_heading);
  waypoints[next].x = waypoints[curr].x + INT_MULT_RSHIFT(dist,s_heading,INT32_TRIG_FRAC-INT32_POS_FRAC) / 100;
  waypoints[next].y = waypoints[curr].y + INT_MULT_RSHIFT(dist,c_heading,INT32_TRIG_FRAC-INT32_POS_FRAC) / 100;

  printf("heading error= %d \n", safe_heading);
  return FALSE;
}


