/*
 * Copyright (C) 2012-2013
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
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */


#include <stdint.h>

inline int detectobst(int cnt_tot, int* counter, int* obstacle, int clr_tresh, int N);
inline int detectobst(int cnt_tot, int* counter, int* obstacle, int clr_tresh, int N)
{
  int *cnt = counter;
  int *obst = obstacle;
  int obst_head=0;
  int Nobst=0;
  int Nhalf= (int) N/2;
  int safe_head=180;
    
  // int obstacle[5];
  for (int i=0; i<N; i++) 
      {
      obst_head+= (int) cnt[i]*(i-Nhalf)*90/N;
      if (cnt[i] > clr_tresh)
          { 
          obst[i] = 1;
          Nobst++;
          }
      else
          { obst[i] = 0; }
      };
  int obst_heading=(int) obst_head/(cnt_tot+1);
  
  // following code only works for N=5 segments
  if (Nobst > 3)
    { 
    if ( obst_heading > 0 )
        { safe_head = 90; }
    else
        { safe_head = 90; }
    }
    
  // 3 obstacles
  else if (Nobst == 3)
    {
    if ( (obst[0] == 0) && (obst[1] == 0) )
        { safe_head = -36; }
    else if ( (obst[3] == 0) && (obst[4] == 0) )
        { safe_head = 36; }
    else
        {
        if ( obst_heading > 0 )
            { safe_head = 90; }
        else
            { safe_head = 90; }
        }
        
    }
  
  // 2 obstacles  
  else if (Nobst == 2)
    {
    if (obst[2] == 1) // obst in the middle
        {
        if ( (obst[0] == 0) && (obst[1] == 0) )
            { safe_head = -36; }
        else 
            { safe_head = 36; }
        }
    else // no obstacle in the middle
        {
        if ( (obst[1] == 0) && (obst[2] == 0) && (obst[3] == 0) )
            { safe_head = 0; }
        else if ( (obst[0] == 0) && (obst[1] == 0) && (obst[2] == 0) )
            { safe_head = -18; }
        else if ( (obst[2] == 0) && (obst[3] == 0) && (obst[4] == 0) )
            { safe_head = +18; }
        else
            {
            if ( obst_heading > 0 )
                { safe_head = 90; }
            else
                { safe_head = 90; }
            }    
        }
    }
  
  // 1 obstacle
  else if (Nobst == 1)
    {
    if ( (obst[0] == 1) || (obst[4] == 1) ) // obst on the side
        { safe_head = 0; }
    else if (obst[1] == 1)
        { safe_head = 18; }
    else if (obst[3] == 1)
        { safe_head = -18; }
    else // obstacle in the middle
        {
        if ( obst_heading > 0 )
            { safe_head = -36; }
        else
            { safe_head = 36; }
        }

    }
  
  // no obstacle  
  else
    { safe_head = 0;}
  //printf("Obstacles L --> R = %d %d %d %d %d \n", obst[0], obst[1], obst[2], obst[3], obst[4]);
  //printf("Obstacle heading = %d \n", obst_heading);
  //printf("Safe heading = %d \n", safe_head);
    
  return safe_head;
  
}


