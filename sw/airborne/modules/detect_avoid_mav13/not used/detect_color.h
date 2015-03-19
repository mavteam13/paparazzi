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
#include "modules/computer_vision/cv/image.h"


inline int detectobst_color(struct img_struct* input, struct img_struct* output, uint8_t y_m, uint8_t y_M, uint8_t u_m, uint8_t u_M, uint8_t v_m, uint8_t v_M, int color_tresh, int N);
inline int detectobst_color(struct img_struct* input, struct img_struct* output, uint8_t y_m, uint8_t y_M, uint8_t u_m, uint8_t u_M, uint8_t v_m, uint8_t v_M, int color_tresh, int N)
{
  
  uint8_t *source = input->buf;
  uint8_t *dest = output->buf;
  int cnt[10] = { 0 };
  int cnt_total = 0;
  
  // Color filtering - YCbCr color space (from color.h)
  
  for (int y=0;y<output->h;y++)
  {
    for (int x=0;x<output->w;x+=2)
    {
      // Color Check:
      if (
          // Light
               (dest[1] >= y_m)
            && (dest[1] <= y_M)
            && (dest[0] >= u_m)
            && (dest[0] <= u_M)
            && (dest[2] >= v_m)
            && (dest[2] <= v_M)
         )// && (dest[2] > 128))
      {
        
        cnt_total++;
        
        for ( int j=0; j<N; j++ )
            {
            if ( x < output->w*(j+1)/5 )
                {
                cnt[j]=cnt[j]+1; 
                break; 
                }
            }
        
        /*        
        if (x<output->w/5)
        { cnt[0]=cnt[0]+1;}
        else if (x<output->w*2/5) 
        { cnt[1]=cnt[1]+1;}
        else if (x<output->w*3/5) 
        { cnt[2]=cnt[2]+1;}
        else if (x<output->w*4/5) 
        { cnt[3]=cnt[3]+1;}
        else
        { cnt[4]=cnt[4]+1;}
        */
        
        
        // UYVY
        dest[0] = 64;        // U
        dest[1] = source[1];  // Y
        dest[2] = 255;        // V
        dest[3] = source[3];  // Y
      }
      else
      {
        // UYVY
        char u = source[0]-127;
        u/=4;
        dest[0] = 127;        // U
        dest[1] = source[1];  // Y
        u = source[2]-127;
        u/=4;
        dest[2] = 127;        // V
        dest[3] = source[3];  // Y
      }

      dest+=4;
      source+=4;
    }
  }
  
  int obstacle[5];
  for (int ii=0; ii<5; ii++) 
      { 
      if (cnt[ii] > color_tresh)
          { obstacle[ii] = 1; }
      else
          { obstacle[ii] = 0; }
      };
    
  int obst_heading=(int) (cnt[0]*(-36)+cnt[1]*(-18)+cnt[3]*(18)+cnt[4]*(36))/(cnt_total+1);
    
  printf("ColorDetected L --> R = %d %d %d %d %d \n", cnt[0], cnt[1], cnt[2], cnt[3], cnt[4]);
  //printf("Obstacle heading = %d \n", obst_heading);
    

  return obst_heading;
  
}


