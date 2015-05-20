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
#include <stdio.h>
#include "messages.h"
#include "mcu_periph/uart.h"
#include "subsystems/datalink/downlink.h"
#include "generated/flight_plan.h" 
#include "math/pprz_algebra_int.h"
#include "navigation.h" 

//****** Declare variables ******//

int16_t offset_heading;
int16_t next_heading;
int safe_heading;
int wait_time = 2;
int obs_2sect_front;
int stereo_nav_status = 0; 
int stereo_vision_status;


//****** Functions ******//

void nav_team13_init(void)
{
  //printf("init1\n");
  srand(time(NULL));
}

bool_t NavSetWaypointTowardsHeading(uint8_t curr, uint8_t dist, uint8_t next)
{
  // distance in cm's
  int32_t s_heading, c_heading;

  // random heading (angle) -32,-16,0,16,32 degrees
  // safe_heading = ((rand() % 5) * 16) - 32;
  // safe_heading = 45;  //hack for sim testing
  printf("0 \n");
  offset_heading = 0;
  PPRZ_ITRIG_SIN(s_heading, nav_heading+offset_heading);
  PPRZ_ITRIG_COS(c_heading, nav_heading+offset_heading);
  while (safe_heading != 0)
  {
    printf("1 \n");
    offset_heading = offset_heading+INT32_RAD_OF_DEG(safe_heading << (INT32_ANGLE_FRAC));
  
    printf("nav_heading= %d \n", nav_heading);
    printf("offset_heading= %d \n", offset_heading);
    PPRZ_ITRIG_SIN(s_heading, nav_heading+offset_heading);
    PPRZ_ITRIG_COS(c_heading, nav_heading+offset_heading);
    next_heading=nav_heading+offset_heading; 	

    waypoints[next].x = waypoints[curr].x + INT_MULT_RSHIFT(dist,s_heading,INT32_TRIG_FRAC-INT32_POS_FRAC) / 100;
    waypoints[next].y = waypoints[curr].y + INT_MULT_RSHIFT(dist,c_heading,INT32_TRIG_FRAC-INT32_POS_FRAC) / 100;

    nav_set_heading_towards_waypoint(next);
    while ( (nav_heading - next_heading)>-20 && (nav_heading - next_heading)<20 )
    {
      printf("2 \n");
    }
    printf("heading error= %d \n", safe_heading);
    return FALSE;
  }
  waypoints[next].x = waypoints[curr].x + INT_MULT_RSHIFT(dist,s_heading,INT32_TRIG_FRAC-INT32_POS_FRAC) / 100;
  waypoints[next].y = waypoints[curr].y + INT_MULT_RSHIFT(dist,c_heading,INT32_TRIG_FRAC-INT32_POS_FRAC) / 100;
  printf("3 \n");

  //printf("heading error= %d \n", safe_heading);
  return FALSE;
}

bool_t move_global_wp(uint8_t glob,uint8_t fz1,uint8_t fz2,uint8_t fz3,uint8_t fz4,uint8_t nxt,uint8_t curr)
{
  if (!InsideFlight_Area((float)INT_MULT_RSHIFT(1,waypoints[nxt].x,INT32_POS_FRAC),(float)INT_MULT_RSHIFT(1,waypoints[nxt].y,INT32_POS_FRAC)) || nav_approaching_from(&waypoints[glob],NULL,0))
  //if (!InsideFlight_Area(GetPosX(),GetPosY()))
  {
    printf("out of bound triggered\n");
    if (waypoints[glob].x==waypoints[fz1].x)
    {
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

//////////////////////////////////////////////////////
// New functions added by Matej

bool_t NavSetWaypointTowardsHeadingNew(uint8_t curr, uint8_t dist, uint8_t next, uint8_t heading, uint8_t global, uint8_t global_prev)
    {
    int32_t s_heading, c_heading;
    // distance in cm's
    // random heading (angle) -32,-16,0,16,32 degrees
    // safe_heading = ((rand() % 5) * 16) - 32;
    // safe_heading = 45; //hack for sim testing
    offset_heading = INT32_RAD_OF_DEG(safe_heading << (INT32_ANGLE_FRAC));
    printf("nav_heading= %d \n", nav_heading);
    printf("offset_heading= %d \n", offset_heading);
    PPRZ_ITRIG_SIN(s_heading, nav_heading+offset_heading);
    PPRZ_ITRIG_COS(c_heading, nav_heading+offset_heading);
    waypoints[heading].x = waypoints[curr].x + INT_MULT_RSHIFT(dist,s_heading,INT32_TRIG_FRAC-INT32_POS_FRAC) / 100;
    waypoints[heading].y = waypoints[curr].y + INT_MULT_RSHIFT(dist,c_heading,INT32_TRIG_FRAC-INT32_POS_FRAC) / 100;
    if (safe_heading==90 || safe_heading==-90)
    {
        waypoints[next].x=waypoints[curr].x;
        waypoints[next].y=waypoints[curr].y;
        // waypoints[global_prev].x=waypoints[global].x;
        // waypoints[global_prev].y=waypoints[global].y;
        // waypoints[global].x=waypoints[heading].x;
        // waypoints[global].y=waypoints[heading].y;
        printf("90 deg heading detected");
        
    }
    else
    {
        waypoints[next].x=waypoints[heading].x;
        waypoints[next].y=waypoints[heading].y;
        // waypoints[global].x=waypoints[global_prev].x;
        // waypoints[global].y=waypoints[global_prev].y;
    }
    printf("heading error= %d \n", safe_heading);
    return FALSE;
}


bool_t move_global_wp_new(uint8_t glob,uint8_t fz1,uint8_t fz2,uint8_t fz3,uint8_t fz4,uint8_t nxt,uint8_t curr,uint8_t heading, uint8_t glob_prev)
{
  if (!InsideFlight_Area((float)INT_MULT_RSHIFT(1,waypoints[nxt].x,INT32_POS_FRAC),(float)INT_MULT_RSHIFT(1,waypoints[nxt].y,INT32_POS_FRAC)) || nav_approaching_from(&waypoints[glob],NULL,0))
  //if (!InsideFlight_Area(GetPosX(),GetPosY()))
  {
    printf("out of bound triggered\n");
    if (waypoints[glob].x==waypoints[fz1].x)
    {
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
    // waypoints[glob_prev].x=waypoints[glob].x;
    // waypoints[glob_prev].y=waypoints[glob].y;
    waypoints[heading].x=waypoints[glob].x;
    waypoints[heading].y=waypoints[glob].y;
    waypoints[nxt].x=waypoints[curr].x;
    waypoints[nxt].y=waypoints[curr].y;
    wait_time=0;
  }
  else
  {
    wait_time=2;
  }
  return FALSE;
}

bool_t NavSetNewWayPoint(uint8_t curr, uint8_t dist, uint8_t next, uint8_t heading, uint8_t glob, uint8_t look,uint8_t fz1,uint8_t fz2,uint8_t fz3,uint8_t fz4)
    {
    int32_t s_heading, c_heading;
    // distance in cm's
    // random heading (angle) -32,-16,0,16,32 degrees
    // safe_heading = ((rand() % 5) * 16) - 32;
    // safe_heading = 45; //hack for sim testing
    offset_heading = INT32_RAD_OF_DEG(safe_heading << (INT32_ANGLE_FRAC));
    printf("nav_heading= %d \n", nav_heading);
    printf("offset_heading= %d \n", offset_heading);
    PPRZ_ITRIG_SIN(s_heading, nav_heading+offset_heading);
    PPRZ_ITRIG_COS(c_heading, nav_heading+offset_heading);
    waypoints[heading].x = waypoints[curr].x + INT_MULT_RSHIFT(dist,s_heading,INT32_TRIG_FRAC-INT32_POS_FRAC) / 100;
    waypoints[heading].y = waypoints[curr].y + INT_MULT_RSHIFT(dist,c_heading,INT32_TRIG_FRAC-INT32_POS_FRAC) / 100;

    printf("heading error= %d \n", safe_heading);
    if (safe_heading>=45 || safe_heading<=-45)
    {
      waypoints[next].x=waypoints[curr].x;
      waypoints[next].y=waypoints[curr].y;

      printf("45 deg heading detected");  
    }
    else
    {
      waypoints[next].x=waypoints[heading].x;
      waypoints[next].y=waypoints[heading].y;
	printf("1\n");
    }
  

  if (!InsideFlight_Area((float)INT_MULT_RSHIFT(1,waypoints[next].x,INT32_POS_FRAC),(float)INT_MULT_RSHIFT(1,waypoints[next].y,INT32_POS_FRAC)) || nav_approaching_from(&waypoints[glob],NULL,0))
  
  {
    printf("out of bound triggered\n");
    if (waypoints[glob].x==waypoints[fz1].x)
    {
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

    waypoints[heading].x=waypoints[glob].x;
    waypoints[heading].y=waypoints[glob].y;
    waypoints[next].x=waypoints[curr].x;
    waypoints[next].y=waypoints[curr].y;
    wait_time=0;
  }
  else
  {
    wait_time=2;
    printf("4\n");
  }
  if (safe_heading>=45 || safe_heading<=-45)
  {
    waypoints[look].x=waypoints[heading].x;
    waypoints[look].y=waypoints[heading].y;

    wait_time=0;

    printf("45 deg heading detected 2"); 
  }
  else
  {
    waypoints[look].x=waypoints[glob].x;
    waypoints[look].y=waypoints[glob].y;

    printf("3\n");
  }
  return FALSE;
}

//////////////////////////////////////////////////////////////////////////////////

bool_t NavSetWaypointAvoidInBounds(uint8_t curr, uint8_t dist, uint8_t next)
{
  // distance in cm's

  // declare variables
  int32_t s_heading, c_heading;
  int8_t InBoundAndSafe = 0;   // flag to determine if next heading is inbounds and safe
  int8_t offsetsign;
  int16_t turncount = 1;
  // safe_heading = 45;  //hack for sim testing

//determine sign of safe_heading,
  if (safe_heading==90){offsetsign=1;} //no safe heading - arbitrarily turn cw
  else if (safe_heading>0){offsetsign=1;} 
  else {offsetsign=-1;}  
  
  while (InBoundAndSafe == 0)
  {

    // If there is no safe heading turn cw 37 deg and set offsetsign=1 so that it will turn the other way it goes out of bounds
    // vehicle will continue turning cw in 37 deg increments until there is a safe_heading
    if (safe_heading == 90){      
    offset_heading = INT32_RAD_OF_DEG(37 << (INT32_ANGLE_FRAC));
    nav_heading = nav_heading + offset_heading;
     offsetsign = 1;
    turncount++;
     continue;
    }

    // calculate location for next waypoint
    offset_heading = INT32_RAD_OF_DEG(safe_heading << (INT32_ANGLE_FRAC));

    PPRZ_ITRIG_SIN(s_heading, nav_heading+offset_heading);
    PPRZ_ITRIG_COS(c_heading, nav_heading+offset_heading);
    waypoints[next].x = waypoints[curr].x + INT_MULT_RSHIFT(dist,s_heading,INT32_TRIG_FRAC-INT32_POS_FRAC) / 100;
    waypoints[next].y = waypoints[curr].y + INT_MULT_RSHIFT(dist,c_heading,INT32_TRIG_FRAC-INT32_POS_FRAC) / 100;
    
    if (!InsideFlight_Region((float)INT_MULT_RSHIFT(1,waypoints[next].x,INT32_POS_FRAC),(float)INT_MULT_RSHIFT(1,waypoints[next].y,INT32_POS_FRAC)))
      {  
        // if the new wp is not within the boundary of Flight_area, turn the opposite direction to find a new safe heading.
	offset_heading = INT32_RAD_OF_DEG((-offsetsign * 42 * turncount) << (INT32_ANGLE_FRAC));
        nav_heading = nav_heading + offset_heading;
        // will safe_heading be updating???  I think so.
	continue;
      }
    else { InBoundAndSafe=1;}
  }
  //printf("heading error= %d \n", safe_heading);
  return FALSE;
}

bool_t offset_wp_cm(uint8_t wp1, uint8_t wp2, uint8_t d)
{
  /*int32_t nh = stateGetNedToBodyEulers_i() ->psi; */
  int32_t s_heading, c_heading;
	
  PPRZ_ITRIG_SIN(s_heading, nav_heading);
  PPRZ_ITRIG_COS(c_heading, nav_heading);
      
  waypoints[wp2].x = waypoints[wp1].x + INT_MULT_RSHIFT(d,c_heading,INT32_TRIG_FRAC-INT32_POS_FRAC)/100;
  waypoints[wp2].y = waypoints[wp1].y + INT_MULT_RSHIFT(d,-s_heading,INT32_TRIG_FRAC-INT32_POS_FRAC)/100;
    
  return FALSE;
}

bool_t stereo_init(uint8_t wpfoto){
        // Initialize
        NavSetWaypointHere(wpfoto);
        stereo_nav_status=-1;
        return FALSE;
        }

bool_t stereo_loop(uint8_t wpfoto){
    switch(stereo_nav_status)
        {
        case -1 :
        if (nav_approaching_from(&waypoints[wpfoto], NULL, 0)) { stereo_nav_status = 1; printf("Wp1 reached");}
        break;
        
        case 1 :
        // Waiting for a response from vision
        if (stereo_vision_status==1) 
            {
            offset_wp_cm(wpfoto,wpfoto,10);
            stereo_nav_status = -2;
            }
        break;
        
        case -2 :
        if (nav_approaching_from(&waypoints[wpfoto], NULL, 0)) { stereo_nav_status = 2; printf("Wp2 reached"); }
        break;
        
        case 2 :
        // Waiting for a response from vision
        if (stereo_vision_status==2) 
            {
            // set a new wp based on vision info - for now I use current position
            NavSetWaypointHere(wpfoto);
            stereo_nav_status = -1;
            }
        break;  
        } 
    return FALSE;
}

bool_t flag_wp1(void)
{
  printf("In wp1\n");
  // Set flag: position ready for first photo
  stereo_nav_status = 1;        
  return FALSE;
}

bool_t flag_wp2()
{
  printf("In wp2\n");
  // Set flag: position ready for second photo
  stereo_nav_status = 2; 
  return FALSE;
}

// Is the safe heading not ==0?
bool_t obstacle_in_path(void)
{
  //int safe_heading = 0;  //sim hack
  //if ((rand() % 1000)==8){   // sim hack
  //safe_heading = 45;
  //printf("safe_heading = %d\n", safe_heading); 
  //return TRUE;}
  
  if (safe_heading==0) { return FALSE; }
  return TRUE;
}

// Is the obstacle in front of me and nearby?
bool_t obstacle_nearby(void)
{
  if (safe_heading==0){return FALSE;}
  else if (obs_2sect_front){
  return TRUE; 
  printf("obstacle nearby\n");
  }
  else {return FALSE;}
}

// Is the vehicle still enough in yaw rate?
bool_t yawStill(void)
{
//stateGetNedToBodyEulers_i() ->psi;
//stateGetBodyRates_f() -> p;
//stateGetBodyRates_f() -> q;
float yawr, yawpsi, diff;
yawr=stateGetBodyRates_f() -> r;
yawpsi = stateGetNedToBodyEulers_f() ->psi;  //radians
diff = yawpsi-ANGLE_FLOAT_OF_BFP(nav_heading);  //diff in radians
//printf("diff = %f\n", diff);
//printf("psi (rad)= %f\n", yawpsi);
//printf("nav_heading_floatofbfp = %f\n", ANGLE_FLOAT_OF_BFP(nav_heading));

  if (diff<0.06 && diff>-0.06){ return TRUE; }
  //if (yawr<(1) && yawr>(-1)){return TRUE;}
  else {return FALSE;}
}


