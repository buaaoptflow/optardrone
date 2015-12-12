#include "stdafx.h"
#include <cv.h>
#include <highgui.h>
#include "optutil.h"
#include "opticalflow.h"
#include "navigation.h"
#include "optcvmatutil.h"
#include "optfeatureutil.h"
#include "optmatutil.h"
#include "motioncolor.h"
//#include "optdrawflow.h"

using namespace cv;
using namespace std;

//利用左右光流平衡返回左1右2前3停止4，为金字塔光流算法
float imgFeatureStrategic(ImgFeatureFunType funtype,IplImage* imgprev, IplImage* imgcurr, IplImage* imgdst, Mat &color, int strategic){
	//1.计算光流
	IplImage* imgprev_1 = imgResize(imgprev);
	IplImage* imgcurr_1 = imgResize(imgcurr);
	CvPoint2D32f* cornersprev_11 = new CvPoint2D32f[ MAX_CORNERS ];
	CvPoint2D32f* cornerscurr_11 = new CvPoint2D32f[ MAX_CORNERS ];
	float k = funtype(imgprev_1, imgcurr_1, cornersprev_11, cornerscurr_11,cvRect(0,0,WIDTH/2,HEIGHT));
	CvPoint2D32f* cornersprev_12 = new CvPoint2D32f[ MAX_CORNERS ];
	CvPoint2D32f* cornerscurr_12 = new CvPoint2D32f[ MAX_CORNERS ];
	funtype(imgprev_1, imgcurr_1, cornersprev_12, cornerscurr_12,cvRect(WIDTH/2,0,WIDTH/2,HEIGHT));

	float result  = 0;
	if ((strategic >> 0 & 1) == 1) //balance
	{
		result = balanceForFeatureCvPoint(cornersprev_11, cornerscurr_11, cornersprev_12, cornerscurr_12, imgdst, k);
		printf("balance result : %lf\n", result);
	}    
	if ((strategic >> 1 & 1) == 1) //draw optflow with grap
	{
		drawFlowForFeatureCvPoint(cornersprev_11,cornerscurr_11, imgdst);
		drawFlowForFeatureCvPoint(cornersprev_12,cornerscurr_12, imgdst, false);
	}
	if ((strategic >> 2 & 1) == 1) //mation to color
	{
		motionToColor(cornersprev_11, cornerscurr_11, cornersprev_12, cornerscurr_12, color);
	}
	if ((strategic >>3 & 1) == 1)//get speed
	{ 
		getSpeedFromFlow(cornersprev_11, cornerscurr_11, cornersprev_12, cornerscurr_12, imgdst);
	}
	if ((strategic >>4 & 1) ==1 ) //draw flow without zero
	{
		drawFlowWithoutZero(cornersprev_11, cornerscurr_11, imgdst, true);
		drawFlowWithoutZero(cornersprev_12, cornerscurr_12, imgdst, false);
	}
	cvReleaseImage(&imgprev_1);
	cvReleaseImage(&imgcurr_1);
	free(cornerscurr_11);
	free(cornersprev_11);
	free(cornerscurr_12);
	free(cornersprev_12);
	return result;
}

float imgStrategic(ImgFunType funtype, IplImage* imgprev, IplImage* imgcurr, IplImage* imgdst, Mat &color, int strategic){
	//计算光流初始化变量
	IplImage* imgprev_1 = imgResize(imgprev);
	IplImage* imgcurr_1 = imgResize(imgcurr);
	CvMat *velx, *vely;
	velx = cvCreateMat(HEIGHT, WIDTH, CV_32FC1);
	vely = cvCreateMat(HEIGHT, WIDTH, CV_32FC1);
	cvSetZero(velx);
	cvSetZero(vely);
	//计算光流
	float k = funtype(imgprev_1,imgcurr_1,velx,vely);

	float result  = 0;
	if ((strategic >> 0 & 1) == 1) //balance
	{
		result = balanceForDenseCvMat(velx, vely, imgdst, k);
		//result = safeAreaForDenseCvMat(velx, vely, imgdst, k);
		printf("balance result : %lf\n", result);
	}    
	if ((strategic >> 1 & 1) == 1) //draw optflow with grap
	{
		drawFlowForDenseCvMat(velx, vely, imgdst);
	}
	if ((strategic >> 2 & 1) == 1) //mation to color
	{
		motionToColor(velx, vely, color);
	}
	if ((strategic>> 3 & 1) == 1) //get speed
	{ 
		getSpeedFromFlow(velx, vely, imgdst);
	}
	if ((strategic>> 4 & 1) == 1) //draw flow without zero
	{ 
		drawFlowWithoutZero(velx, vely, imgdst);
	}
	cvReleaseImage(&imgprev_1);
	cvReleaseImage(&imgcurr_1);
	cvReleaseMat(&velx);
	cvReleaseMat(&vely);

	return result;
}

float matStrategic(MatFunType funtype, Mat frameprev, Mat framecurr, Mat &framedst,Mat &color, int strategic,bool issf){
	Mat frameprev_1, framecurr_1, flow;
	if (issf)
	{
		frameprev_1 = matColorResize(frameprev);
		framecurr_1 = matColorResize(framecurr);
	}else{
		frameprev_1 = matResize(frameprev);
		framecurr_1 = matResize(framecurr);
	}

	flow = funtype(frameprev_1, framecurr_1, flow);
    float k = FB_K;
    if(issf){
        k = SF_K;
    }
    float result = 0;
	if ((strategic >> 0 & 1) == 1) //balance
	{
		result = balanceForDenseMat(flow, framedst, k);
		printf("balance result : %lf\n", result);
	}
	if ((strategic >> 1 & 1) == 1) //draw optflow with grap
	{
		drawFlowForDenseMat(flow, framedst);
	}
	if ((strategic >> 2 & 1) == 1) //mation to color
	{
		motionToColor(flow, color);
	}
	if ((strategic>> 3 & 1) == 1)//get speed
	{ 
		getSpeedFromFlow(flow, framedst);
	}
	if ((strategic>> 4 & 1) == 1) //draw flow without zero
	{ 
		drawFlowWithoutZero(flow, framedst);
	}
	framecurr_1.release();
	frameprev_1.release();
	flow.release();
	return result;
}

