/*
*  FILE common.h
*  AUTHOR Sarah
*  DATE 2015/08/15 10:43
*  TODO: 重置图像大小和对图像灰度化处�?
*/
#pragma once

#include "stdafx.h"
#include <cv.h>
#include "optutil.h"

using namespace cv;
using namespace std;

#ifndef util_OPTFLOW_
#define util_OPTFLOW_

/*
*  Method:     imgResize
*  Description: 调整图像大小和对图像灰度化处理，宽高�?define WIDTH and HEIGHT(in stdafx.h)定义.
*               参数类型为IplImage.
*  Returns:    IplImage*. resize和灰度化处理后的图像
*  IplImage * img: Required = true. 源图像帧.
*/
IplImage* imgResize(IplImage* img);

/*
*  Method:     Lucaskanade 
*  Description: 稠密光流方法（参数为IplImage�?- cvCalcOpticalFlowLK.
*  Returns:    int. 返回阈�?，当光流大于此阈值时uav停止, 传�?给optcvmatutil.h.
*  IplImage * imgprev: Required = true. 第一帧图�?
*  IplImage * imgcurr: Required = true. 第二帧图�?
*  CvMat * velx: Required = true. X方向的光�?程序结束后被赋�?).
*  CvMat * vely: Required = true. Y方向的光�?程序结束后被赋�?).
*/
float Lucaskanade(IplImage* imgprev, IplImage* imgcurr, CvMat* velx, CvMat* vely);

/*
*  Method:     HornSchunck
*  Description: 稠密光流方法（参数为IplImage�?- cvCalcOpticalFlowHS.
*  Returns:    int. 返回阈�?，当光流大于此阈值时uav停止, 传�?给optcvmatutil.h.
*  IplImage * imgprev: Required = true. 第一帧图�?
*  IplImage * imgcurr: Required = true. 第二帧图�?
*  CvMat * velx: Required = true. X方向的光�?程序结束后被赋�?).
*  CvMat * vely: Required = true. Y方向的光�?程序结束后被赋�?).
*/
float HornSchunck(IplImage* imgprev, IplImage* imgcurr, CvMat* velx, CvMat* vely);

/*
*  Method:     BlockMatch
*  Description: 稠密光流方法（参数为IplImage�? cvCalcOpticalFlowBM.
*  Returns:    int. 返回阈�?，当光流大于此阈值时uav停止, 传�?给optcvmatutil.h.
*  IplImage * imgprev: Required = true. 第一帧图�?
*  IplImage * imgcurr: Required = true. 第二帧图�?
*  CvMat * velx: Required = true. X方向的光�?程序结束后被赋�?).
*  CvMat * vely: Required = true. Y方向的光�?程序结束后被赋�?).
*/
float BlockMatch(IplImage* imgprev, IplImage* imgcurr, CvMat* velx, CvMat* vely);

/*
*  Method:     PyrLK
*  Description: �?��光流方法（参数为IplImage�?- cvCalcOpticalFlowBM.
*  Returns:    int. 返回阈�?，当光流大于此阈值时uav停止, 传�?给optmatutil.h.
*  IplImage * imgprev: Required = true. 第一帧图�?
*  IplImage * imgcurr: Required = true. 第二帧图�?
*  CvPoint2D32f * cornersprev: Required = true. X方向的光�?程序结束后被赋�?).
*  CvPoint2D32f * cornerscurr: Required = true. Y方向的光�?程序结束后被赋�?).
*/
float PyrLK(IplImage* imgprev, IplImage* imgcurr,CvPoint2D32f* cornersprev,CvPoint2D32f* cornerscurr, CvRect rect);

/*
*  Method:     matResize
*  Description: 调整图像大小和对图像灰度化处理，宽高�?define WIDTH and HEIGHT(in stdafx.h)定义. 
*               参数类型�?cv::Mat.
*  Returns:    cv::Mat. resize和灰度化处理后的图像.
*  Mat frame: Required = true. 源图像帧.
*/
Mat matResize(Mat frame);

/*
*  Method:     matColorResize
*  Description: 调整图像大小，宽高在#define WIDTH and HEIGHT(in stdafx.h)定义. 
*               参数类型�?cv::Mat.
*  Returns:    cv::Mat. resize后的图像.
*  Mat frame: Required = true. 源图像帧.
*/
Mat matColorResize(Mat frame);

/*
*  Method:     SimpleFlow
*  Description: 光流方法 - calcOpticalFlowSF. 
*  Returns:    cv::Mat. 光流.
*  Mat frameprev: Required = true. 第一帧图�? 
*  Mat framecurr: Required = true. 第二帧图�?
*  Mat flow: Required = true. 得到光流(程序结束后被赋�?).
*/
Mat SimpleFlow(Mat frameprev, Mat framecurr, Mat flow);

/*
*  Method:     FarneBack
*  Description: 光流方法 - calcOpticalFlowFarneback. 
*  Returns:    cv::Mat. 返回光流.
*  Mat frameprev: Required = true. 第一帧图�? 
*  Mat framecurr: Required = true. 第二帧图�?
*  Mat flow: Required = true. 得到光流(程序结束后被赋�?).
*/
Mat FarneBack(Mat frameprev, Mat framecurr, Mat flow);

#endif /* util_OPTFLOW_ */