
#include <image.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>


int extern thres_verticalcount;
extern int Gsize;
extern double sigma;
extern  int thres;
//extern int stereo_nav_status;
extern int thres_disparity;

void sobel_edge_filter(struct img_struct *input,struct img_struct *output);
void blur_filter(struct img_struct *input,struct img_struct *output);
void image_difference(struct img_struct *input,struct img_struct *input_prev,struct img_struct *output);
int pixelcount(struct img_struct* input, uint32_t* pxcnt, uint8_t pxcnt_size);
void image_flow(struct img_struct *input,struct img_struct *input_prev,struct img_struct *output,double increment_value);
int pixelratio(struct img_struct* input, uint32_t* pxcnt, uint8_t pxcnt_size);
void detect_vertical_lines(struct img_struct *input, struct img_struct *output ,uint8_t *pxlcnt_lines,uint8_t *pxlcnt_lines_bin);
