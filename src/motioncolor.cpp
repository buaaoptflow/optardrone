/************************************************************************/
/* motioncolor.h  optflow function                                      */
/************************************************************************/

#include "stdafx.h"
#include <cv.h>
#include <float.h>
#include "optutil.h"
#include "motioncolor.h"

using namespace cv;
using namespace std;


static vector<Scalar> colorwheel; //Scalar r,g,b

//判断数是否合法
bool isNan(double x){
	return x != x || ((x == x) && (( x - x) != ( x - x ))) ;
}

void makeColorWheel(vector<Scalar> &colorwheel){
	int RY = 15;
	int YG = 6;
	int GC = 4;
	int CB = 11;
	int BM = 13;
	int MR = 6;
	int i;

	for (i = 0; i < RY; i++)
		colorwheel.push_back(Scalar(255, 255 * i / RY, 0));
	for (i = 0; i < YG; i++)
		colorwheel.push_back(Scalar(255 - 255 * i / YG, 255, 0));
	for (i = 0; i < GC; i++)
		colorwheel.push_back(Scalar(0, 255, 255 * i / GC));
	for (i = 0; i < CB; i++)
		colorwheel.push_back(Scalar(0, 255 - 255 * i / CB, 255));
	for (i = 0; i < BM; i++)
		colorwheel.push_back(Scalar(255 * i / BM, 0, 255));
	for (i = 0; i < MR; i++)
		colorwheel.push_back(Scalar(255, 0, 255 - 255 * i / MR));
}

void motionToColor(Mat flow, Mat &color){
	if (color.empty())
		color.create(HEIGHT, WIDTH, CV_8UC3);

	if (colorwheel.empty())
		makeColorWheel(colorwheel);

	// determine motion range:
	float maxrad = -1;

	// Find max flow to normalize fx and fy
	for (int i = 0; i < flow.rows; ++i) {
		for (int j = 0; j < flow.cols; ++j) {
			Vec2f flow_at_point = flow.at<Vec2f>(i, j);
			float fx = flow_at_point[0];
			float fy = flow_at_point[1];
			if ((fabs(fx) > UNKNOWN_FLOW_THRESH)
				|| (fabs(fy) > UNKNOWN_FLOW_THRESH)|| isNan(fx) || isNan(fy))
				continue;
			float rad = sqrt(fx * fx + fy * fy);
			maxrad = maxrad > rad ? maxrad : rad;
		}
	}

	for (int i = 0; i < flow.rows; ++i) {
		for (int j = 0; j < flow.cols; ++j) {
			uchar *data = color.data + color.step[0] * i + color.step[1] * j;
			Vec2f flow_at_point = flow.at<Vec2f>(i, j);

			float fx = flow_at_point[0] / maxrad;
			float fy = flow_at_point[1] / maxrad;
			if ((fabs(fx) > UNKNOWN_FLOW_THRESH)
				|| (fabs(fy) > UNKNOWN_FLOW_THRESH)|| isNan(fx) || isNan(fy)) {
					data[0] = data[1] = data[2] = 0;
					continue;
			}
			float rad = sqrt(fx * fx + fy * fy);

			float angle = atan2(-fy, -fx) / CV_PI;
			float fk = (angle + 1.0) / 2.0 * (colorwheel.size() - 1);
			int k0 = (int) fk;
			int k1 = (k0 + 1) % colorwheel.size();
			float f = fk - k0;
			//f = 0; // uncomment to see original color wheel

			for (int b = 0; b < 3; b++) {
				float col0 = colorwheel[k0][b] / 255.0;
				float col1 = colorwheel[k1][b] / 255.0;
				float col = (1 - f) * col0 + f * col1;
				if (rad <= 1)
					col = 1 - rad * (1 - col); // increase saturation with radius
				else
					col *= .75; // out of range
				data[2 - b] = (int) (255.0 * col);
			}
		}
	}
}

void motionToColor(CvMat* velx, CvMat* vely, Mat &color){
	if (color.empty())
		color.create(HEIGHT, WIDTH, CV_8UC3);

	if (colorwheel.empty())
		makeColorWheel(colorwheel);

	// determine motion range:
	float maxrad = -1;

	// Find max flow to normalize fx and fy
	for (int i = 0; i < HEIGHT; ++i) {
		for (int j = 0; j <WIDTH; ++j) {
			float fx = (float) cvGetReal2D(velx, i, j);
			float fy = (float) cvGetReal2D(vely, i, j);
			if ((fabs(fx) > UNKNOWN_FLOW_THRESH)
				|| (fabs(fy) > UNKNOWN_FLOW_THRESH || isNan(fx) || isNan(fy)))
				continue;
			float rad = sqrt(fx * fx + fy * fy);
			maxrad = maxrad > rad ? maxrad : rad;
		}
	}

	for (int i = 0; i < HEIGHT; ++i) {
		for (int j = 0; j < WIDTH; ++j) {
			uchar *data = color.data + color.step[0] * i + color.step[1] * j;

			float fx = ((float) cvGetReal2D(velx, i, j)) / maxrad;
			float fy = ((float) cvGetReal2D(velx, i, j)) / maxrad;
			if ((fabs(fx) > UNKNOWN_FLOW_THRESH)
				|| (fabs(fy) > UNKNOWN_FLOW_THRESH)|| isNan(fx) || isNan(fy)) {
					data[0] = data[1] = data[2] = 0;
					continue;
			}
			float rad = sqrt(fx * fx + fy * fy);

			float angle = atan2(-fy, -fx) / CV_PI;
			float fk = (angle + 1.0) / 2.0 * (colorwheel.size() - 1);
			int k0 = (int) fk;
			int k1 = (k0 + 1) % colorwheel.size();
			float f = fk - k0;
			//f = 0; // uncomment to see original color wheel

			for (int b = 0; b < 3; b++) {
				float col0 = colorwheel[k0][b] / 255.0;
				float col1 = colorwheel[k1][b] / 255.0;
				float col = (1 - f) * col0 + f * col1;
				if (rad <= 1)
					col = 1 - rad * (1 - col); // increase saturation with radius
				else
					col *= .75; // out of range
				data[2 - b] = (int) (255.0 * col);
			}
		}
	}
}

void motionToColor(CvPoint2D32f* cornersprev_11, CvPoint2D32f* cornerscurr_11, CvPoint2D32f* cornersprev_12, CvPoint2D32f* cornerscurr_12, Mat &color){
	if (color.empty())
		color.create(HEIGHT, WIDTH, CV_8UC3);

	if (colorwheel.empty())
		makeColorWheel(colorwheel);

	// determine motion range:
	float maxrad = -1;

	// Find max flow to normalize fx and fy
	for (int i =0; i < MAX_CORNERS; i++)
	{
		float fx = (float)(cornerscurr_11[i].x - cornersprev_11[i].x);
		float fy = (float)(cornerscurr_11[i].y - cornersprev_11[i].y);
		if ((fabs(fx) > UNKNOWN_FLOW_THRESH)
			|| (fabs(fy) > UNKNOWN_FLOW_THRESH)|| isNan(fx) || isNan(fy))
			continue;
		float rad = sqrt(fx * fx + fy * fy);
		maxrad = maxrad > rad ? maxrad : rad;
		fx = (float)(cornerscurr_12[i].x - cornersprev_12[i].x);
		fy = (float)(cornerscurr_12[i].y - cornersprev_12[i].y);
		if ((fabs(fx) > UNKNOWN_FLOW_THRESH)
			|| (fabs(fy) > UNKNOWN_FLOW_THRESH)|| isNan(fx) || isNan(fy))
			continue;
		rad = sqrt(fx * fx + fy * fy);
		maxrad = maxrad > rad ? maxrad : rad;
	}

	//draw
	int x,y;
	//调节范围，因为均为特征点，稀疏光流，计算效果不佳，所以置为1，才可用不同颜色大概显示出光流大小
	maxrad = 1;
	for (int i = 0; i < MAX_CORNERS; i++)
	{
		x = (int)cornersprev_11[i].x;
		y = (int)cornersprev_11[i].y;
		if (x < 0 || x > WIDTH )
		{
			break;
		}

		uchar *data = color.data + color.step[0] * y + color.step[1] * x;

		float fx = (float)(cornerscurr_11[i].x - cornersprev_11[i].x)/maxrad;
		float fy = (float)(cornerscurr_11[i].y - cornersprev_11[i].y)/maxrad;
		if ((fabs(fx) > UNKNOWN_FLOW_THRESH)
				|| (fabs(fy) > UNKNOWN_FLOW_THRESH)|| isNan(fx) || isNan(fy)) {
					data[0] = data[1] = data[2] = 0;
					continue;
		}
		float rad = sqrt(fx * fx + fy * fy);

		float angle = atan2(-fy, -fx) / CV_PI;
		float fk = (angle + 1.0) / 2.0 * (colorwheel.size() - 1);
		int k0 = (int) fk;
		int k1 = (k0 + 1) % colorwheel.size();
		float f = fk - k0;
		//f = 0; // uncomment to see original color wheel

		for (int b = 0; b < 3; b++) {
			float col0 = colorwheel[k0][b] / 255.0;
			float col1 = colorwheel[k1][b] / 255.0;
			float col = (1 - f) * col0 + f * col1;
			if (rad <= 1)
				col = 1 - rad * (1 - col); // increase saturation with radius
			else
				col *= .75; // out of range
			data[2 - b] = (int) (255.0 * col);
		}
	}
	for (int i = 0; i < MAX_CORNERS; i++)
	{
		x = (int)cornersprev_12[i].x + WIDTH/2;
		y = (int)cornersprev_12[i].y;
		if (x < 0 || x > WIDTH )
		{
			break;
		}

		uchar *data = color.data + color.step[0] * y + color.step[1] * x;

		float fx = (float)(cornerscurr_12[i].x - cornersprev_12[i].x)/maxrad;
		float fy = (float)(cornerscurr_12[i].y - cornersprev_12[i].y)/maxrad;
		if ((fabs(fx) > UNKNOWN_FLOW_THRESH)
			|| (fabs(fy) > UNKNOWN_FLOW_THRESH)|| isNan(fx) || isNan(fy)) {
				data[0] = data[1] = data[2] = 0;
				continue;
		}
		float rad = sqrt(fx * fx + fy * fy);

		float angle = atan2(-fy, -fx) / CV_PI;
		float fk = (angle + 1.0) / 2.0 * (colorwheel.size() - 1);
		int k0 = (int) fk;
		int k1 = (k0 + 1) % colorwheel.size();
		float f = fk - k0;
		//f = 0; // uncomment to see original color wheel

		for (int b = 0; b < 3; b++) {
			float col0 = colorwheel[k0][b] / 255.0;
			float col1 = colorwheel[k1][b] / 255.0;
			float col = (1 - f) * col0 + f * col1;
			if (rad <= 1)
				col = 1 - rad * (1 - col); // increase saturation with radius
			else
				col *= .75; // out of range
			data[2 - b] = (int) (255.0 * col);
		}
	}
}



