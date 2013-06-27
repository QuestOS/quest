/*
  FAST-EDGE
  Copyright (c) 2009 Benjamin C. Haynor

  Permission is hereby granted, free of charge, to any person
  obtaining a copy of this software and associated documentation
  files (the "Software"), to deal in the Software without
  restriction, including without limitation the rights to use,
  copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the
  Software is furnished to do so, subject to the following
  conditions:

  The above copyright notice and this permission notice shall be
  included in all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
  OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
  OTHER DEALINGS IN THE SOFTWARE.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "frame.h"
#include "qcv_types.h"
#include "canny.h"


#define LOW_THRESHOLD_PERCENTAGE 0.8 // percentage of the high threshold value that the low threshold shall be set at
#define HIGH_THRESHOLD_PERCENTAGE 0.10 // percentage of pixels that meet the high threshold - for example 0.15 will ensure that at least 15% of edge pixels are considered to meet the high threshold


void canny_edge_detect(qcv_frame_t * img_in, qcv_frame_t * img_out);
void gaussian_noise_reduce(qcv_frame_t * img_in, qcv_frame_t * img_out);
void calc_gradient_sobel(qcv_frame_t * img_in, int g[], int dir[]);
void calc_gradient_scharr(qcv_frame_t * img_in, int g_x[], int g_y[], int g[], int dir[]);
void non_max_suppression(qcv_frame_t * img, int g[], int dir[]);
void estimate_threshold(qcv_frame_t * img, int * high, int * low);
void hysteresis (int high, int low, qcv_frame_t * img_in, qcv_frame_t * img_out);
int trace (int x, int y, int low, qcv_frame_t * img_in, qcv_frame_t * img_out);
int range (qcv_frame_t * img, int x, int y);
void dilate_1d_h(qcv_frame_t * img, qcv_frame_t * img_out);
void dilate_1d_v(qcv_frame_t * img, qcv_frame_t * img_out);
void erode_1d_h(qcv_frame_t * img, qcv_frame_t * img_out);
void erode_1d_v(qcv_frame_t * img, qcv_frame_t * img_out);
void erode(qcv_frame_t * img_in, qcv_frame_t * img_scratch, qcv_frame_t * img_out);
void dilate(qcv_frame_t * img_in, qcv_frame_t * img_scratch, qcv_frame_t * img_out);
void morph_open(qcv_frame_t * img_in, qcv_frame_t * img_scratch, qcv_frame_t * img_scratch2, qcv_frame_t * img_out);
void morph_close(qcv_frame_t * img_in, qcv_frame_t * img_scratch, qcv_frame_t * img_scratch2, qcv_frame_t * img_out);


/*
  CANNY EDGE DETECT
  DOES NOT PERFORM NOISE REDUCTION - PERFORM NOISE REDUCTION PRIOR TO USE
  Noise reduction omitted, as some applications benefit from morphological operations such as opening or closing as opposed to Gaussian noise reduction
  If your application always takes the same size input image, uncomment the definitions of WIDTH and HEIGHT in the header file and define them to the size of your input image,
  otherwise the required intermediate arrays will be dynamically allocated.
  If WIDTH and HEIGHT are defined, the arrays will be allocated in the compiler directive that follows:
*/

int qcv_canny(qcv_frame_t * img_in, qcv_canny_params_t* params, qcv_frame_t * img_out) {
  qcv_frame_t img_scratch;
  int high, low;
  int* g;
  int* dir;
  unsigned char* img_scratch_data;

  if(qcv_frame_type(img_in) != QCV_FRAME_TYPE_1BYTE_GREY) return -1;

  g = malloc(qcv_frame_width(img_in) * qcv_frame_height(img_in) * sizeof(int));
  dir = malloc(qcv_frame_width(img_in) * qcv_frame_height(img_in) * sizeof(int));
  img_scratch_data = malloc(qcv_frame_width(img_in) * qcv_frame_height(img_in) * sizeof(unsigned char));
  
  qcv_frame_width(&img_scratch) = qcv_frame_width(img_in);
  qcv_frame_height(&img_scratch) = qcv_frame_height(img_in);
  qcv_frame_buf(&img_scratch) = img_scratch_data;
  qcv_create_frame(img_out, qcv_frame_width(img_in), qcv_frame_height(img_in), QCV_FRAME_TYPE_1BYTE_GREY);
  calc_gradient_sobel(img_in, g, dir);
  printf("*** performing non-maximum suppression ***\n");
  non_max_suppression(&img_scratch, g, dir);
  estimate_threshold(&img_scratch, &high, &low);
  hysteresis(high, low, &img_scratch, img_out);
#ifndef WIDTH
  free(g);
  free(dir);
  free(img_scratch_data);
#endif
  return 0;
}

/*
  GAUSSIAN_NOISE_ REDUCE
  apply 5x5 Gaussian convolution filter, shrinks the image by 4 pixels in each direction, using Gaussian filter found here:
  http://en.wikipedia.org/wiki/Canny_edge_detector
*/
void gaussian_noise_reduce(qcv_frame_t * img_in, qcv_frame_t * img_out)
{
#ifdef CLOCK
  clock_t start = clock();
#endif
  int w, h, x, y, max_x, max_y;
  w = qcv_frame_width(img_in);
  h = qcv_frame_height(img_in);
  qcv_frame_width(img_out) = w;
  qcv_frame_height(img_out) = h;
  max_x = w - 2;
  max_y = w * (h - 2);
  for (y = w * 2; y < max_y; y += w) {
    for (x = 2; x < max_x; x++) {
      qcv_frame_buf(img_out)[x + y] = (2 * qcv_frame_buf(img_in)[x + y - 2 - w - w] + 
				       4 * qcv_frame_buf(img_in)[x + y - 1 - w - w] + 
				       5 * qcv_frame_buf(img_in)[x + y - w - w] + 
				       4 * qcv_frame_buf(img_in)[x + y + 1 - w - w] + 
				       2 * qcv_frame_buf(img_in)[x + y + 2 - w - w] + 
				       4 * qcv_frame_buf(img_in)[x + y - 2 - w] + 
				       9 * qcv_frame_buf(img_in)[x + y - 1 - w] + 
				       12 * qcv_frame_buf(img_in)[x + y - w] + 
				       9 * qcv_frame_buf(img_in)[x + y + 1 - w] + 
				       4 * qcv_frame_buf(img_in)[x + y + 2 - w] + 
				       5 * qcv_frame_buf(img_in)[x + y - 2] + 
				       12 * qcv_frame_buf(img_in)[x + y - 1] + 
				       15 * qcv_frame_buf(img_in)[x + y] + 
				       12 * qcv_frame_buf(img_in)[x + y + 1] + 
				       5 * qcv_frame_buf(img_in)[x + y + 2] + 
				       4 * qcv_frame_buf(img_in)[x + y - 2 + w] + 
				       9 * qcv_frame_buf(img_in)[x + y - 1 + w] + 
				       12 * qcv_frame_buf(img_in)[x + y + w] + 
				       9 * qcv_frame_buf(img_in)[x + y + 1 + w] + 
				       4 * qcv_frame_buf(img_in)[x + y + 2 + w] + 
				       2 * qcv_frame_buf(img_in)[x + y - 2 + w + w] + 
				       4 * qcv_frame_buf(img_in)[x + y - 1 + w + w] + 
				       5 * qcv_frame_buf(img_in)[x + y + w + w] + 
				       4 * qcv_frame_buf(img_in)[x + y + 1 + w + w] + 
				       2 * qcv_frame_buf(img_in)[x + y + 2 + w + w]) / 159;
    }
  }
#ifdef CLOCK
  printf("Gaussian noise reduction - time elapsed: %f\n", ((double)clock() - start) / CLOCKS_PER_SEC);
#endif
}

/*
  CALC_GRADIENT_SOBEL
  calculates the result of the Sobel operator - http://en.wikipedia.org/wiki/Sobel_operator - and estimates edge direction angle
*/
/*void calc_gradient_sobel(qcv_frame_t * img_in, int g_x[], int g_y[], int g[], int dir[]) {//float theta[]) {*/
void calc_gradient_sobel(qcv_frame_t * img_in, int g[], int dir[]) {
#ifdef CLOCK
  clock_t start = clock();
#endif
  int w, h, x, y, max_x, max_y, g_x, g_y;
  float g_div;
  w = qcv_frame_width(img_in);
  h = qcv_frame_height(img_in);
  max_x = w - 3;
  max_y = w * (h - 3);
  for (y = w * 3; y < max_y; y += w) {
    for (x = 3; x < max_x; x++) {
      g_x = (2 * qcv_frame_buf(img_in)[x + y + 1] 
	     + qcv_frame_buf(img_in)[x + y - w + 1]
	     + qcv_frame_buf(img_in)[x + y + w + 1]
	     - 2 * qcv_frame_buf(img_in)[x + y - 1] 
	     - qcv_frame_buf(img_in)[x + y - w - 1]
	     - qcv_frame_buf(img_in)[x + y + w - 1]);
      g_y = 2 * qcv_frame_buf(img_in)[x + y - w] 
	+ qcv_frame_buf(img_in)[x + y - w + 1]
	+ qcv_frame_buf(img_in)[x + y - w - 1]
	- 2 * qcv_frame_buf(img_in)[x + y + w] 
	- qcv_frame_buf(img_in)[x + y + w + 1]
	- qcv_frame_buf(img_in)[x + y + w - 1];
#ifndef ABS_APPROX
      g[x + y] = sqrt(g_x * g_x + g_y * g_y);
#endif
#ifdef ABS_APPROX
      g[x + y] = abs(g_x[x + y]) + abs(g_y[x + y]);
#endif
      if (g_x == 0) {
	dir[x + y] = 2;
      } else {
	g_div = g_y / (float) g_x;
	/* the following commented-out code is slightly faster than the code that follows, but is a slightly worse approximation for determining the edge direction angle
	   if (g_div < 0) {
	   if (g_div < -1) {
	   dir[n] = 0;
	   } else {
	   dir[n] = 1;
	   }
	   } else {
	   if (g_div > 1) {
	   dir[n] = 0;
	   } else {
	   dir[n] = 3;
	   }
	   }
	*/
	if (g_div < 0) {
	  if (g_div < -2.41421356237) {
	    dir[x + y] = 0;
	  } else {
	    if (g_div < -0.414213562373) {
	      dir[x + y] = 1;
	    } else {
	      dir[x + y] = 2;
	    }
	  }
	} else {
	  if (g_div > 2.41421356237) {
	    dir[x + y] = 0;
	  } else {
	    if (g_div > 0.414213562373) {
	      dir[x + y] = 3;
	    } else {
	      dir[x + y] = 2;
	    }
	  }
	}
      }
    }
		
  }	
#ifdef CLOCK
  printf("Calculate gradient Sobel - time elapsed: %f\n", ((double)clock() - start) / CLOCKS_PER_SEC);
#endif
}

/*
  CALC_GRADIENT_SCHARR
  calculates the result of the Scharr version of the Sobel operator - http://en.wikipedia.org/wiki/Sobel_operator - and estimates edge direction angle
  may have better rotational symmetry
*/
void calc_gradient_scharr(qcv_frame_t * img_in, int g_x[], int g_y[], int g[], int dir[]) {//float theta[]) {
#ifdef CLOCK
  clock_t start = clock();
#endif
  int w, h, x, y, max_x, max_y, n;
  float g_div;
  w = qcv_frame_width(img_in);
  h = qcv_frame_height(img_in);
  max_x = w - 1;
  max_y = w * (h - 1);
  n = 0;
  for (y = w; y < max_y; y += w) {
    for (x = 1; x < max_x; x++) {
      g_x[n] = (10 * qcv_frame_buf(img_in)[x + y + 1] 
		+ 3 * qcv_frame_buf(img_in)[x + y - w + 1]
		+ 3 * qcv_frame_buf(img_in)[x + y + w + 1]
		- 10 * qcv_frame_buf(img_in)[x + y - 1] 
		- 3 * qcv_frame_buf(img_in)[x + y - w - 1]
		- 3 * qcv_frame_buf(img_in)[x + y + w - 1]);
      g_y[n] = 10 * qcv_frame_buf(img_in)[x + y - w] 
	+ 3 * qcv_frame_buf(img_in)[x + y - w + 1]
	+ 3 * qcv_frame_buf(img_in)[x + y - w - 1]
	- 10 * qcv_frame_buf(img_in)[x + y + w] 
	- 3 * qcv_frame_buf(img_in)[x + y + w + 1]
	- 3 * qcv_frame_buf(img_in)[x + y + w - 1];
#ifndef ABS_APPROX
      g[n] = sqrt(g_x[n] * g_x[n] + g_y[n] * g_y[n]);
#endif
#ifdef ABS_APPROX
      g[n] = abs(g_x[n]) + abs(g_y[n]);
#endif
      if (g_x[n] == 0) {
	dir[n] = 2;
      } else {
	g_div = g_y[n] / (float) g_x[n];
	if (g_div < 0) {
	  if (g_div < -2.41421356237) {
	    dir[n] = 0;
	  } else {
	    if (g_div < -0.414213562373) {
	      dir[n] = 1;
	    } else {
	      dir[n] = 2;
	    }
	  }
	} else {
	  if (g_div > 2.41421356237) {
	    dir[n] = 0;
	  } else {
	    if (g_div > 0.414213562373) {
	      dir[n] = 3;
	    } else {
	      dir[n] = 2;
	    }
	  }
	}
      }
      n++;
    }
  }	
#ifdef CLOCK
  printf("Calculate gradient Scharr - time elapsed: %f\n", ((double)clock() - start) / CLOCKS_PER_SEC);
#endif
}
/*
  NON_MAX_SUPPRESSION
  using the estimates of the Gx and Gy image gradients and the edge direction angle determines whether the magnitude of the gradient assumes a local  maximum in the gradient direction
  if the rounded edge direction angle is 0 degrees, checks the north and south directions
  if the rounded edge direction angle is 45 degrees, checks the northwest and southeast directions
  if the rounded edge direction angle is 90 degrees, checks the east and west directions
  if the rounded edge direction angle is 135 degrees, checks the northeast and southwest directions
*/
void non_max_suppression(qcv_frame_t * img, int g[], int dir[]) {//float theta[]) {
#ifdef CLOCK
  clock_t start = clock();
#endif
  int w, h, x, y, max_x, max_y;
  w = qcv_frame_width(img);
  h = qcv_frame_height(img);
  max_x = w;
  max_y = w * h;
  for (y = 0; y < max_y; y += w) {
    for (x = 0; x < max_x; x++) {
      switch (dir[x + y]) {
      case 0:
	if (g[x + y] > g[x + y - w] && g[x + y] > g[x + y + w]) {
	  if (g[x + y] > 255) {
	    qcv_frame_buf(img)[x + y] = 0xFF;
	  } else {
	    qcv_frame_buf(img)[x + y] = g[x + y];
	  }
	} else {
	  qcv_frame_buf(img)[x + y] = 0x00;
	}
	break;
      case 1:
	if (g[x + y] > g[x + y - w - 1] && g[x + y] > g[x + y + w + 1]) {
	  if (g[x + y] > 255) {
	    qcv_frame_buf(img)[x + y] = 0xFF;
	  } else {
	    qcv_frame_buf(img)[x + y] = g[x + y];
	  }
	} else {
	  qcv_frame_buf(img)[x + y] = 0x00;
	}
	break;
      case 2:
	if (g[x + y] > g[x + y - 1] && g[x + y] > g[x + y + 1]) {
	  if (g[x + y] > 255) {
	    qcv_frame_buf(img)[x + y] = 0xFF;
	  } else {
	    qcv_frame_buf(img)[x + y] = g[x + y];
	  }
	} else { 
	  qcv_frame_buf(img)[x + y] = 0x00;
	}
	break;
      case 3:
	if (g[x + y] > g[x + y - w + 1] && g[x + y] > g[x + y + w - 1]) {
	  if (g[x + y] > 255) {
	    qcv_frame_buf(img)[x + y] = 0xFF;
	  } else {
	    qcv_frame_buf(img)[x + y] = g[x + y];
	  }
	} else {
	  qcv_frame_buf(img)[x + y] = 0x00;
	}
	break;
      default:
	printf("ERROR - direction outside range 0 to 3");
	break;
      }
    }
  }
#ifdef CLOCK
  printf("Non-maximum suppression - time elapsed: %f\n", ((double)clock() - start) / CLOCKS_PER_SEC);
#endif
}
/*
  ESTIMATE_THRESHOLD
  estimates hysteresis threshold, assuming that the top X% (as defined by the HIGH_THRESHOLD_PERCENTAGE) of edge pixels with the greatest intesity are true edges
  and that the low threshold is equal to the quantity of the high threshold plus the total number of 0s at the low end of the histogram divided by 2
*/
void estimate_threshold(qcv_frame_t * img, int * high, int * low) {
#ifdef CLOCK
  clock_t start = clock();
#endif
  int i, max, pixels, high_cutoff;
  int histogram[256];
  max = qcv_frame_width(img) * qcv_frame_height(img);
  for (i = 0; i < 256; i++) {
    histogram[i] = 0;
  }
  for (i = 0; i < max; i++) {
    histogram[qcv_frame_buf(img)[i]]++;
  }
  pixels = (max - histogram[0]) * HIGH_THRESHOLD_PERCENTAGE;
  high_cutoff = 0;
  i = 255;
  while (high_cutoff < pixels) {
    high_cutoff += histogram[i];
    i--;
  }
  *high = i;
  i = 1;
  while (histogram[i] == 0) {
    i++;
  }
  *low = (*high + i) * LOW_THRESHOLD_PERCENTAGE;
#ifdef PRINT_HISTOGRAM
  for (i = 0; i < 256; i++) {
    printf("i %d count %d\n", i, histogram[i]);
  }
#endif
	
#ifdef CLOCK
  printf("Estimate threshold - time elapsed: %f\n", ((double)clock() - start) / CLOCKS_PER_SEC);
#endif
}

void hysteresis (int high, int low, qcv_frame_t * img_in, qcv_frame_t * img_out)
{
#ifdef CLOCK
  clock_t start = clock();
#endif
  int x, y, n, max;
  max = qcv_frame_width(img_in) * qcv_frame_height(img_in);
  for (n = 0; n < max; n++) {
    qcv_frame_buf(img_out)[n] = 0x00;
  }
  for (y=0; y < qcv_frame_height(img_out); y++) {
    for (x=0; x < qcv_frame_width(img_out); x++) {
      if (qcv_frame_buf(img_in)[y * qcv_frame_width(img_out) + x] >= high) {
	trace (x, y, low, img_in, img_out);
      }
    }
  }
#ifdef CLOCK
  printf("Hysteresis - time elapsed: %f\n", ((double)clock() - start) / CLOCKS_PER_SEC);
#endif
}

int trace(int x, int y, int low, qcv_frame_t * img_in, qcv_frame_t * img_out)
{
  int y_off, x_off;//, flag;
  if (qcv_frame_buf(img_out)[y * qcv_frame_width(img_out) + x] == 0)
    {
      qcv_frame_buf(img_out)[y * qcv_frame_width(img_out) + x] = 0xFF;
      for (y_off = -1; y_off <=1; y_off++)
	{
	  for(x_off = -1; x_off <= 1; x_off++)
	    {
	      if (!(y == 0 && x_off == 0) && range(img_in, x + x_off, y + y_off) && qcv_frame_buf(img_in)[(y + y_off) * qcv_frame_width(img_out) + x + x_off] >= low) {
		if (trace(x + x_off, y + y_off, low, img_in, img_out))
		  {
		    return(1);
		  }
	      }
	    }
	}
      return(1);
    }
  return(0);
}

int range(qcv_frame_t * img, int x, int y)
{
  if ((x < 0) || (x >= qcv_frame_width(img))) {
    return(0);
  }
  if ((y < 0) || (y >= qcv_frame_height(img))) {
    return(0);
  }
  return(1);
}

void dilate_1d_h(qcv_frame_t * img, qcv_frame_t * img_out) {
  int x, y, offset, y_max;
  y_max = qcv_frame_height(img) * (qcv_frame_width(img) - 2);
  for (y = 2 * qcv_frame_width(img); y < y_max; y += qcv_frame_width(img)) {
    for (x = 2; x < qcv_frame_width(img) - 2; x++) {
      offset = x + y;
      qcv_frame_buf(img_out)[offset] = max(max(max(max(qcv_frame_buf(img)[offset-2], qcv_frame_buf(img)[offset-1]), qcv_frame_buf(img)[offset]), qcv_frame_buf(img)[offset+1]), qcv_frame_buf(img)[offset+2]);	
    }
  }
}

void dilate_1d_v(qcv_frame_t * img, qcv_frame_t * img_out) {
  int x, y, offset, y_max;
  y_max = qcv_frame_height(img) * (qcv_frame_width(img) - 2);
  for (y = 2 * qcv_frame_width(img); y < y_max; y += qcv_frame_width(img)) {
    for (x = 2; x < qcv_frame_width(img) - 2; x++) {
      offset = x + y;
      qcv_frame_buf(img_out)[offset] = max(max(max(max(qcv_frame_buf(img)[offset-2 * qcv_frame_width(img)], qcv_frame_buf(img)[offset-qcv_frame_width(img)]), qcv_frame_buf(img)[offset]), qcv_frame_buf(img)[offset+qcv_frame_width(img)]), qcv_frame_buf(img)[offset+2*qcv_frame_width(img)]);	
    }
  }
}

void erode_1d_h(qcv_frame_t * img, qcv_frame_t * img_out) {
  int x, y, offset, y_max;
  y_max = qcv_frame_height(img) * (qcv_frame_width(img) - 2);
  for (y = 2 * qcv_frame_width(img); y < y_max; y += qcv_frame_width(img)) {
    for (x = 2; x < qcv_frame_width(img) - 2; x++) {
      offset = x + y;
      qcv_frame_buf(img_out)[offset] = min(min(min(min(qcv_frame_buf(img)[offset-2], qcv_frame_buf(img)[offset-1]), qcv_frame_buf(img)[offset]), qcv_frame_buf(img)[offset+1]), qcv_frame_buf(img)[offset+2]);	
    }
  }
}

void erode_1d_v(qcv_frame_t * img, qcv_frame_t * img_out) {
  int x, y, offset, y_max;
  y_max = qcv_frame_height(img) * (qcv_frame_width(img) - 2);
  for (y = 2 * qcv_frame_width(img); y < y_max; y += qcv_frame_width(img)) {
    for (x = 2; x < qcv_frame_width(img) - 2; x++) {
      offset = x + y;
      qcv_frame_buf(img_out)[offset] = min(min(min(min(qcv_frame_buf(img)[offset-2 * qcv_frame_width(img)], qcv_frame_buf(img)[offset-qcv_frame_width(img)]), qcv_frame_buf(img)[offset]), qcv_frame_buf(img)[offset+qcv_frame_width(img)]), qcv_frame_buf(img)[offset+2*qcv_frame_width(img)]);	
    }
  }
}

void erode(qcv_frame_t * img_in, qcv_frame_t * img_scratch, qcv_frame_t * img_out) {
#ifdef CLOCK
  clock_t start = clock();
#endif
  erode_1d_h(img_in, img_scratch);
  erode_1d_v(img_scratch, img_out);
#ifdef CLOCK
  printf("Erosion - time elapsed: %f\n", ((double)clock() - start) / CLOCKS_PER_SEC);
#endif
}

void dilate(qcv_frame_t * img_in, qcv_frame_t * img_scratch, qcv_frame_t * img_out) {
#ifdef CLOCK
  clock_t start = clock();
#endif
  dilate_1d_h(img_in, img_scratch);
  dilate_1d_v(img_scratch, img_out);
#ifdef CLOCK
  printf("Dilation - time elapsed: %f\n", ((double)clock() - start) / CLOCKS_PER_SEC);
#endif
}

void morph_open(qcv_frame_t * img_in, qcv_frame_t * img_scratch, qcv_frame_t * img_scratch2, qcv_frame_t * img_out) {
#ifdef CLOCK
  clock_t start = clock();
#endif
  erode(img_in, img_scratch, img_scratch2);
  dilate(img_scratch2, img_scratch, img_out);
#ifdef CLOCK
  printf("Morphological opening - time elapsed: %f\n", ((double)clock() - start) / CLOCKS_PER_SEC);
#endif
}

void morph_close(qcv_frame_t * img_in, qcv_frame_t * img_scratch, qcv_frame_t * img_scratch2, qcv_frame_t * img_out) {
#ifdef CLOCK
  clock_t start = clock();
#endif
  dilate(img_in, img_scratch, img_scratch2);
  erode(img_scratch2, img_scratch, img_out);
#ifdef CLOCK
  printf("Morphological closing - time elapsed: %f\n", ((double)clock() - start) / CLOCKS_PER_SEC);
#endif
}
