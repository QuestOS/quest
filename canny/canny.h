#ifndef _CANNY_H_
#define _CANNY_H_

/* 
 * Author: Tom Gibara
 * create the detector
 * CannyEdgeDetector detector = new CannyEdgeDetector();
 * adjust its parameters as desired
 * detector.setLowThreshold(0.5f);
 * detector.setHighThreshold(1f);
 * apply it to an image
 * detector.setSourceImage(frame);
 * detector.process();
 * BufferedImage edges = detector.getEdgesImage();
 */

#define BOOL unsigned char
#define TRUE 1
#define FALSE 0

//# define M_PI 3.14159265358979323846
#define PI M_PI

//# define M_E 2.7182818284590452354

#define TYPE_INT_RGB 1
#define TYPE_INT_ARGB 2
#define TYPE_BYTE_GRAY 3
#define TYPE_USHORT_GRAY 4
#define TYPE_3BYTE_BGR 5

#define WIDTH   30
#define HEIGHT  30

/* image origin is in the upper-left corner */
typedef struct
{
  int type, width, height;
  unsigned char *data;
}
Image;

void
I_init(Image *img, int width, int height, int type);

void
I_deinit(Image *img);

void 
I_setData(Image *img, int *pixels);

#define GAUSSIAN_CUT_OFF (float)0.005f
#define MAGNITUDE_SCALE (float)100
#define MAGNITUDE_LIMIT (float)1000
#define MAGNITUDE_MAX (int)(MAGNITUDE_SCALE * MAGNITUDE_LIMIT)


typedef struct
{
  int height, width, picsize;
  int *data, *magnitude;
  int data_len, mag_len;
  Image *sourceImage, *edgesImage;
  float gaussianKernelRadius;
  float lowThreshold, highThreshold;
  int gaussianKernelWidth;
  BOOL contrastNormalized;
  float *xConv, *yConv, *xGradient, *yGradient;
  int xC_len, yC_len, xG_len, yG_len;
}
CEDetector;

void 
C_init(CEDetector *detect);

void 
C_deinit(CEDetector *detect);

void
C_process(CEDetector *detect);

void
C_initArrays(CEDetector *detect);

void
C_computeGradients(CEDetector *detect, float kernelRadius, int kernelWidth);


float
gaussian(float x, float sigma);

void
C_performHysteresis(CEDetector *detect, int low, int high);

void
C_follow(CEDetector *detect, int x1, int y1, int i1, int threshold);

void
C_thresholdEdges(CEDetector *detect);

int
luminance(float r, float g, float b);

void
C_readLuminance(CEDetector *detect);

void
C_normalizeContrast(CEDetector *detect);

void
C_writeEdges(CEDetector *detect, int *pixels);




#endif

/* vi: set et sw=2 sts=2: */
