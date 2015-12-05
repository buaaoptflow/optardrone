/*
*  FILE optutil.h
*  AUTHOR Sarah
*  DATE 2015/08/14 22:25
*  TODO: 光流计算方法处理光流的公共方�? 
*/
#pragma once

#include "stdafx.h"
#include <cv.h>

using namespace cv;
using namespace std;

#ifndef util_FOE_TTC_
#define util_FOE_TTC_

void tagSafeAreaByTTC(int cols, float* ttc, float ttcAvg , float k,int *tagSafe);

float compareTag(int *tagOrigin, int *tagSafe, int cols, int *tags);

float balanceControlLR(bool isBig, int leftSumFlow, int rightSumFlow, float k);

float turnLRScale(float leftSumFlow, float rightSumFlow, float k);

Mat calibrate(Mat img);

void calibrate(IplImage* &iplimg);

void writeFile(const char* lineStr);

void drawOrientation(Vec2i leftSumFlow, Vec2i rightSumFlow, int px, int py, float result, IplImage* imgdst);

void drawFlowWithoutZero(CvPoint2D32f* cornersprev, CvPoint2D32f* cornerscurr, IplImage* imgdst, bool isleft);

void drawFlowWithoutZero(CvMat* velx, CvMat* vely, IplImage* imgdst);

void drawFlowWithoutZero(Mat flow, Mat &framedst);

void drawFlowForFeatureCvPoint(CvPoint2D32f* cornersprev_11, CvPoint2D32f* cornerscurr_11, IplImage* imgdst, bool isleft = true);

void drawFlowForDenseCvMat(CvMat* velx, CvMat* vely, IplImage* imgdst);

void drawFlowForDenseMat(Mat flow, Mat &framedst);

void drawArrow(CvPoint p, CvPoint q, IplImage* imgdst, CvScalar rgb = CV_RGB(0,0,255) , int thickness = 1);

void drawArrow(CvPoint p, CvPoint q, Mat &framedst, CvScalar rgb = CV_RGB(0,0,255) , int thickness = 1);


#endif /* util_FOE_TTC_ */
