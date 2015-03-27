#include "edgefilter.h"
#include <image.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

int Gsize=3;
double sigma=1.0;
int thres=30;
int stereo_nav_status=0;
int thres_verticalcount=80;

void sobel_edge_filter(struct img_struct *input,struct img_struct *output)
{

    uint32_t  Sobel[3] = {-1, 0, 1};
    int8_t r, c;
    uint32_t  sobel;
    uint8_t *source = input->buf;
    uint8_t *dest = output->buf;



    for(uint16_t y = 1; y < input->h-1; y++) {
        for(uint16_t x = 1; x < input->w-1; x++) {
            uint32_t idx = input->w*y*2 + (x)*2;
            sobel=0;
            //Convolution
            if(y>1&&y<input->h-1&&x>1&&x<input->w-1)
            {
                for(r = 0; r <=1; r++)
                {
                    for(c = -1; c <= 1; c++)
                    {
                        uint32_t idx_filter = input->w*(y+r)*2 + (x+c)*2;
                        sobel += Sobel[c+1] * (source[idx_filter]);
                    }
                }}
            sobel=abs(sobel);
            /* if(sobel>40)
            dest[idx+1] = 255;
            else dest[idx+1] = 0;*/
            dest[idx+1]=sobel;

            dest[idx]=127;
        }
    }
}
void blur_filter(struct img_struct *input,struct img_struct *output)
{
    double  Gaussian[Gsize][Gsize];
    int G_hsize=(Gsize-1)/2;
    double radius;
    // sum is for normalization
    double sum = 0.0;
    // generate  kernel
    for (int k = -G_hsize; k <= G_hsize; k++)
    {
        for(int m = -G_hsize; m <= G_hsize; m++)
        {

            radius = sqrt(k*k + m*m);
            Gaussian[k + G_hsize][m + G_hsize] = exp(-(radius*radius)/(2*sigma*sigma));
            sum += Gaussian[k + G_hsize][m + G_hsize];
        }    //printf("%d",(size-1)/2);

    }

    // normalize the Kernel
    for(int i = 0; i < Gsize; ++i){
        for(int j = 0; j < Gsize; ++j){
            Gaussian[i][j] /= sum;
        }
    }
    //double  Gaussian[5][5] = {{0.0232,0.0338,0.0383,0.0338,0.0232},{0.0338 ,0.0492,0.0558 ,0.0492,0.0338},{0.0383 ,0.0558 ,0.0632,0.0558 ,0.0383},{0.0338,0.0492,0.0558,0.0492,0.0338}, {0.0232,0.0338,0.0383,0.0338 ,0.0232}};
    int8_t r, c;
    uint32_t  gaussian;
    uint8_t *source = input->buf;
    uint8_t *dest = output->buf;


    for(uint16_t y = 0; y < input->h; y++) {
        for(uint16_t x = 0; x < input->w; x++) {
            uint32_t idx = input->w*y*2 + (x)*2;

            //Convolution
            if(y>G_hsize&&y<input->h-G_hsize&&x>G_hsize&&x<input->w-G_hsize)
            {
                gaussian=0;
                for(r = -G_hsize; r <=G_hsize; r++)
                {
                    for(c = -G_hsize; c <= G_hsize; c++)
                    {
                        uint32_t idx_filter = input->w*(y+r)*2 + (x+c)*2;
                        gaussian += (uint32_t)(Gaussian[r+G_hsize][c+G_hsize] * (source[idx_filter+1]));
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

void image_difference(struct img_struct *input,struct img_struct *input_prev,struct img_struct *output)
{


    uint8_t *source = input->buf;
    uint8_t *source_prev = input_prev->buf;
    uint8_t *dest = output->buf;
    uint8_t value=0;

    for(uint16_t y = 0; y < input->h; y++) {
        for(uint16_t x = 0; x < input->w; x++) {
            uint32_t idx = input->w*y*2 + (x)*2;
            value= abs(source_prev[idx+1]-source[idx+1]);

            //dest[idx+1]=value;
            //dest[idx]=127;
            if(value>thres){
                dest[idx+1]=255;

                dest[idx]=127;
            }else{

                dest[idx+1]=0;

                dest[idx]=127;}

        }


    }
}

void detect_vertical_lines(struct img_struct *input, struct img_struct *output ,uint8_t *pxlcnt_lines){

    uint8_t *source = input->buf;
    uint8_t *dest = output->buf;


    uint8_t pxlcnt_temp;

    for(uint16_t x = 0; x < input->w; x++) {
        pxlcnt_temp=0;
        for(uint16_t y = 0; y < input->h; y++) {
            uint32_t idx = input->w*y*2 + (x)*2;

            if(source[idx+1]==255)
            {
                pxlcnt_temp++;

            }


        }
        if(pxlcnt_temp>thres_verticalcount){
           // printf("increment pxlcnt");
            pxlcnt_lines[x]=pxlcnt_lines[x]+1;
            // pxlcnt_lines[x-1]=0;

        }else pxlcnt_lines[x]=0;
    }


    for(uint16_t y = 0; y < input->h; y++) {
        for(uint16_t x = 0; x < input->w; x++) {
            uint32_t idx = input->w*y*2 + (x)*2;

            if(pxlcnt_lines[x]>1){
                dest[idx]=0;
            }else     dest[idx]=127;

            dest[idx+1]=source[idx+1];
        }

    }



}
void image_flow(struct img_struct *input,struct img_struct *input_prev,struct img_struct *output,double increment_value)
{


    uint8_t *source = input->buf;
    uint8_t *source_prev = input_prev->buf;
    uint8_t *dest = output->buf;
    uint8_t value=0;

    for(uint16_t y = 0; y < input->h; y++) {
        for(uint16_t x = 0; x < input->w; x++) {
            uint32_t idx = input->w*y*2 + (x)*2;
            if (source[idx+1]>10)
                value=(uint8_t)(increment_value*source_prev[idx+1]+5*source[idx+1]);
            else value=(increment_value*source_prev[idx+1]+0);
            if( value>254) value= 255;
            if( value<0) value= 0;
            //else dest[idx+1]=value;
            dest[idx+1]=value;
            dest[idx]=127;
        }
    }
}






int pixelcount(struct img_struct* input, uint32_t* pxcnt, uint8_t pxcnt_size)
{

    //uint32_t pixelcount[5]={0};
    uint8_t idx_count;
    uint8_t *source = input->buf;
    uint8_t value=0;
    int pxcnt_tot=0;



    for(uint16_t y = 0; y < input->h; y++) {
        idx_count=0;
        for(uint16_t x = 0; x < input->w; x++) {
            uint32_t idx = input->w*y*2 + (x)*2;
            value=source[idx+1];

            if (x%(input->w/(pxcnt_size))==0){
                idx_count++;
            }
            // printf("%d",idx_count);

            if(value>200){
                pxcnt[idx_count-1]=pxcnt[idx_count-1]+1;
                pxcnt_tot++;
            }




        }
    }
    return pxcnt_tot;
}

int pixelratio(struct img_struct* input, uint32_t* pxcnt, uint8_t pxcnt_size)
{

    //uint32_t pixelcount[5]={0};
    uint8_t idx_count;
    uint8_t *source = input->buf;
    uint8_t value=0;
    int pxcnt_tot=0;



    for(uint16_t y = 0; y < input->h; y++) {
        idx_count=0;
        for(uint16_t x = 0; x < input->w; x++) {
            uint32_t idx = input->w*y*2 + (x)*2;
            value=source[idx+1];

            if (x%(input->w/(pxcnt_size))==0){
                idx_count++;
            }
            //printf("%d",idx_count);

            pxcnt[idx_count-1]=pxcnt[idx_count-1]+value;
            pxcnt_tot=pxcnt_tot+value;




        }
    }
    return pxcnt_tot;
}
