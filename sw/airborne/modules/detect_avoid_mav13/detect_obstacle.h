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

inline int detectobst(int cnt_tot, int* counter, int clr_tresh, int N);
inline int detectobst(int cnt_tot, int* counter, int clr_tresh, int N)
{
  int *cnt = counter;
  int obst_head=0;
  int Nobst=0;
  int Nhalf= (int) N/2;
  int safe_head=180;
    
  int obstacle[5];
  for (int i=0; i<N; i++) 
      {
      obst_head+= (int) cnt[i]*(i-Nhalf)*90/N;
      if (cnt[i] > clr_tresh)
          { 
          obstacle[i] = 1;
          Nobst++;
          }
      else
          { obstacle[i] = 0; }
      };
  int obst_heading=(int) obst_head/(cnt_tot+1);
  
  // following code only works for N=5 segments
  if (Nobst > 3)
    { 
    if ( obst_heading > 0 )
        { safe_head = -90; }
    else
        { safe_head = 90; }
    }
    
  // 3 obstacles
  else if (Nobst == 3)
    {
    if ( (obstacle[0] == 0) && (obstacle[1] == 0) )
        { safe_head = -36; }
    else if ( (obstacle[3] == 0) && (obstacle[4] == 0) )
        { safe_head = 36; }
    else
        {
        if ( obst_heading > 0 )
            { safe_head = -90; }
        else
            { safe_head = 90; }
        }
        
    }
  
  // 2 obstacles  
  else if (Nobst == 2)
    {
    if (obstacle[2] == 1) // obstacle in the middle
        {
        if ( (obstacle[0] == 0) && (obstacle[1] == 0) )
            { safe_head = -36; }
        else 
            { safe_head = 36; }
        }
    else // no obstacle in the middle
        {
        if ( (obstacle[1] == 0) && (obstacle[2] == 0) && (obstacle[3] == 0) )
            { safe_head = 0; }
        else if ( (obstacle[0] == 0) && (obstacle[1] == 0) && (obstacle[2] == 0) )
            { safe_head = -18; }
        else if ( (obstacle[2] == 0) && (obstacle[3] == 0) && (obstacle[4] == 0) )
            { safe_head = +18; }
        else
            {
            if ( obst_heading > 0 )
                { safe_head = -90; }
            else
                { safe_head = 90; }
            }    
        }
    }
  
  // 1 obstacle
  else if (Nobst == 1)
    {
    if ( (obstacle[0] == 1) || (obstacle[4] == 1) ) // obstacle on the side
        { safe_head = 0; }
    else if (obstacle[1] == 1)
        { safe_head = 18; }
    else if (obstacle[3] == 1)
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
   printf("Obstacles L --> R = %d %d %d %d %d \n", obstacle[0], obstacle[1], obstacle[2], obstacle[3], obstacle[4]);
  //printf("Obstacle heading = %d \n", obst_heading);
  printf("Safe heading = %d \n", safe_head);
    
  return safe_head;
  
}


