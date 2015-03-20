#include "edgefilter.h"
#include <image.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>



void sobel_edge_filter(struct img_struct *input,struct img_struct *output)
{


    uint32_t  Sobel[3][3] = { {-1, 0, 1}, {-2, 0, 2}, {-1, 0, 1} };
    int8_t r, c;
    uint32_t  sobel;
    uint8_t *source = input->buf;
    uint8_t *dest = output->buf;


    for(uint16_t y = 1; y < input->h-1; y++) {
        for(uint16_t x = 1; x < input->w-1; x++) {
            uint32_t idx = input->w*y*2 + (x)*2;

            //Convolution
            sobel=0;
            for(r = -1; r <=1; r++)
            {
                for(c = -1; c <= 1; c++)
                {
                    uint32_t idx_filter = input->w*(y+r)*2 + (x+c)*2;
                    sobel += Sobel[r+1][c+1] * (source[idx_filter]);
                }
            }
            sobel=abs(sobel);
            dest[idx+1] = sobel;//abs(-1*source[idx_left+1]+source[idx_right+1]);
            dest[idx]=127;
        }
    }
}
void blur_filter(struct img_struct *input,struct img_struct *output)
{


    double  Gaussian[5][5] = {{0.0232,0.0338,0.0383,0.0338,0.0232},{0.0338 ,0.0492,0.0558 ,0.0492,0.0338},{0.0383 ,0.0558 ,0.0632,0.0558 ,0.0383},{0.0338,0.0492,0.0558,0.0492,0.0338}, {0.0232,0.0338,0.0383,0.0338 ,0.0232}};
    int8_t r, c;
    uint32_t  gaussian;
    uint8_t *source = input->buf;
    uint8_t *dest = output->buf;


    for(uint16_t y = 0; y < input->h; y++) {
        for(uint16_t x = 0; x < input->w; x++) {
            uint32_t idx = input->w*y*2 + (x)*2;

            //Convolution
            if(y>2&&y<input->h-2&&x>2&&x<input->w-2)
            {
                gaussian=0;
                for(r = -2; r <=2; r++)
                {
                    for(c = -2; c <= 2; c++)
                    {
                        uint32_t idx_filter = input->w*(y+r)*2 + (x+c)*2;
                        gaussian += (uint32_t)(Gaussian[r+2][c+2] * (source[idx_filter+1]));
                    }
                }
                gaussian=abs(gaussian);
                dest[idx+1] = gaussian;//abs(-1*source[idx_left+1]+source[idx_right+1]);
                dest[idx]=127;
            }else{
                dest[idx]=127;

            }

        }
    }
}

void image_difference(struct img_struct *input,struct img_struct *input_prev,struct img_struct *output,int count)
{


    uint32_t pixelcount[5]={0};
    uint8_t idx_count;
    uint8_t *source = input->buf;
    uint8_t *source_prev = input_prev->buf;
    uint8_t *dest = output->buf;
    uint8_t value=0;

    for(uint16_t y = 0; y < input->h; y++) {
        idx_count=0;
        for(uint16_t x = 0; x < input->w; x++) {
            uint32_t idx = input->w*y*2 + (x)*2;
            value= abs(source_prev[idx+1]-source[idx+1]);

            if(count){
                if (x%(input->w/6)==0)
                idx_count++;

            if(value>70){
                pixelcount[idx_count]=pixelcount[idx_count]+1;
            }}else{

            dest[idx+1]=value;

            dest[idx]=127;}




        }
    }

    printf("pixel count is=%d, %d, %d, %d, %d,\n",
           pixelcount[1],pixelcount[2],pixelcount[3],
            pixelcount[4],pixelcount[5],pixelcount[6]);
}
