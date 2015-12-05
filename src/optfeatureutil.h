/*
*  FILE optfeatureutil.h
*  AUTHOR Sarah
*  DATE 2015/08/15 10:42
*  TODO: PyrLK �?��光流公共方法如左右光流平衡�?画光流�?求FOE、TTC�?
*/
#pragma once

#include "stdafx.h"
#include <cv.h>

#include "optutil.h"
#include "optcvmatutil.h"

using namespace cv;
using namespace std;

#ifndef util_OPT_FEATURE_CVMAT_
#define util_OPT_FEATURE_CVMAT_

/*
*  Method:     balanceForFeatureCvPoint
*  Description: 左右光流平衡算法. 
*  Returns:    int. 控制指令. 1 - �? 2 - �? 3 - �? 4 - 停止. 
*  CvPoint2D32f * cornersprev: Required = true. X方向光流. 
*  CvPoint2D32f * cornerscurr: Required = true. Y方向光流. 
*  IplImage * imgdst: Required = true. 目标图像�?
*  int threshold: Required = true. 阈�?，当光流大于此阈值时，返回控制指�?.
*  float k: Required = false. 左右光流平衡的权�? 当左右差距在权�?范围内时，返�?.
*  int px: Required = false. 左右光流的分界线.
*  int py: Required = false. 上下光流的分界线.
*/
float balanceForFeatureCvPoint(CvPoint2D32f* cornersprev_11, CvPoint2D32f* cornerscurr_11, CvPoint2D32f* cornersprev_12, CvPoint2D32f* cornerscurr_12, IplImage* imgdst, float k, int px = WIDTH/2, int py = HEIGHT/2);

Vec2i foeForFeatureCvPoint(CvPoint2D32f* cornersprev, CvPoint2D32f* cornerscurr);

float ttcForFeatureCvPoint(CvPoint2D32f* cornersprev, CvPoint2D32f* cornerscurr, int foeY, float *ttc);

void getSpeedFromFlow(CvPoint2D32f* cornersprev_11, CvPoint2D32f* cornerscurr_11, CvPoint2D32f* cornersprev_12, CvPoint2D32f* cornerscurr_12, IplImage* imgdst);

Vec2d getSpeed(CvPoint2D32f* cornersprev, CvPoint2D32f* cornerscurr, IplImage* imgdst, bool isleft = true, float kangle=0);

#endif