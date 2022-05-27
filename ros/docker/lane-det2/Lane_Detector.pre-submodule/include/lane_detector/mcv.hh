/***
 * \file mcv.hh
 * \author Mohamed Aly <malaa@caltech.edu>
 * \date 11/29/2006
 */

#ifndef MCV_HH_
#define MCV_HH_

#include <stdio.h>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>

using namespace std;

namespace LaneDetector
{

//constant definitions
#define FLOAT_MAT_TYPE CV_32FC1
#define FLOAT_MAT_ELEM_TYPE float

#define INT_MAT_TYPE CV_8UC1
#define INT_MAT_ELEM_TYPE unsigned char

#define FLOAT_IMAGE_TYPE IPL_DEPTH_32F
#define FLOAT_IMAGE_ELEM_TYPE float

#define INT_IMAGE_TYPE IPL_DEPTH_8U
#define INT_IMAGE_ELEM_TYPE unsigned char

#define FLOAT_POINT2D CvPoint2D32f
#define FLOAT_POINT2D_F cvPoint2D632f

#define FLOAT float
#define INT int
#define SHORT_INT unsigned char

//some helper functions for debugging
void SHOW_MAT(const CvMat *pmat, char str[]="Matrix");

void SHOT_MAT_TYPE(const CvMat *pmat);

void SHOW_IMAGE(const CvMat *pmat, const char str[]="Window", int wait=0);
void SHOW_IMAGE(const IplImage *pmat, char str[]="Window");

void SHOW_POINT(const FLOAT_POINT2D pt, char str[]="Point:");

void SHOW_RECT(const CvRect rect, char str[]="Rect:");

/**
 * This function returns the original color
 * image and the first (red) channel scaled to [0 .. 1] with float type. The
 * images are allocated inside the function, so you will need to deallocate
 * them
 *
 * \param clrImage the raw input image
 * \param channelImage the first channel
 */
void mcvLoadImage(CvMat **clrImage, CvMat** channelImage);

void mcvLoadImage2(const char *filename, CvMat **clrImage, CvMat** channelImage);


/**
 * This function scales the input image to have values 0->1
 *
 * \param inImage the input image
 * \param outImage hte output iamge
 */
void mcvScaleMat(const CvMat *inImage, CvMat *outMat);

/**
 * This function creates a double matrix from an input vector
 *
 * \param vec the input vector
 * \param mat the output matrix (column vector)
 *
 */
template <class T>
CvMat* mcvVector2Mat(const vector<T> &vec);

} // namespace LaneDetector

#endif /*MCV_HH_*/


