
#include <image.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

void sobel_edge_filter(struct img_struct *input,struct img_struct *output);
void blur_filter(struct img_struct *input,struct img_struct *output,int size, double sigma);
void image_difference(struct img_struct *input,struct img_struct *input_prev,struct img_struct *output,int thres);
int pixelcount(struct img_struct* input, uint32_t* pxcnt, uint8_t pxcnt_size);
void image_flow(struct img_struct *input,struct img_struct *input_prev,struct img_struct *output,double increment_value);
int pixelratio(struct img_struct* input, uint32_t* pxcnt, uint8_t pxcnt_size);
