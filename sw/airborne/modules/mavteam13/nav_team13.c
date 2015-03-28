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

bool_t NavSetWaypointTowardsHeading(uint8_t curr, uint8_t dist, uint8_t next)
{
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
	
	/*if (safe_heading==90 || safe_heading==-90)
	{
		waypoints[next].x=waypoints[curr].x;
		waypoints[next].y=waypoints[curr].y;
	}*/
 	 printf("heading error= %d \n", safe_heading);
 	 return FALSE;
}

bool_t move_global_wp(uint8_t glob,uint8_t fz1,uint8_t fz2,uint8_t fz3,uint8_t fz4,uint8_t nxt,uint8_t curr)
{
	printf("in proximity of: %i \n", nav_approaching_from(&waypoints[glob],NULL,0));
	if (!InsideFlight_Area((float)INT_MULT_RSHIFT(1,waypoints[nxt].x,INT32_POS_FRAC),(float)INT_MULT_RSHIFT(1,waypoints[nxt].y,INT32_POS_FRAC)) || nav_approaching_from(&waypoints[glob],NULL,0))
// || waypoints[nxt].x<waypoints[glob].x*1.1 && waypoints[nxt].x>waypoints[glob].x*0.9 && waypoints[nxt].x<waypoints[glob].x*1.1 && waypoints[nxt].x>waypoints[glob].x*0.9 
	//if (!InsideFlight_Area(GetPosX(),GetPosY()))
	{
		printf("out of bound triggered\n");
		printf("getposx %f \n",GetPosX());
		printf("getposy %f \n",GetPosY());
		printf("wp x %f \n",(float)waypoints[nxt].x);
		printf("wp y %f \n",(float)waypoints[nxt].y);
		if (waypoints[glob].x==waypoints[fz1].x) {
			waypoints[glob].x=waypoints[fz2].x;
			waypoints[glob].y=waypoints[fz2].y;
		}
		else if (waypoints[glob].x==waypoints[fz2].x) {
			waypoints[glob].x=waypoints[fz3].x;
			waypoints[glob].y=waypoints[fz3].y;
		}
		else if (waypoints[glob].x==waypoints[fz3].x) {
			waypoints[glob].x=waypoints[fz4].x;
			waypoints[glob].y=waypoints[fz4].y;
		}
		else if (waypoints[glob].x==waypoints[fz4].x) {
			waypoints[glob].x=waypoints[fz1].x;
			waypoints[glob].y=waypoints[fz1].y;
		}
		nav_set_heading_towards_waypoint(glob);
		NavSetWaypointTowardsHeading(curr,55,nxt);
	}
	return FALSE;
}



