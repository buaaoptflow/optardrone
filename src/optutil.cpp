#include "stdafx.h"
#include "optutil.h"
#include <cv.h>

using namespace std;
using namespace cv;

/***************************************************************************************/
/*    根据TTC，标记是否为安全区域，暂定大于ttcAvg为安全1，反之，不安全0，然后再调整        
      cols 列数；ttc：计算的ttc数组；ttcAvg：ttc平均值；k：比例系数
      tagSafe : 通过比较规则获得是否安全，标记之                                       */
/***************************************************************************************/
void tagSafeAreaByTTC(int cols, float* ttc, float ttcAvg , float k,int *tagSafe){
	for (int i = 0; i < cols; i++)
	{
		if (ttc[i] > ttcAvg*k)
		{
			tagSafe[i] = 1;
		}
	}
}

/***************************************************************************************/
/*  判定标记是否准确。安全判定安全1，安全非安全2，非安全判定安全3，非安全判定非安全4    
  tagOrigin: 准确的tag标记； tagSafe：根据算法估计的tag标记；tags：根据上述原则判定结果
  函数返回：根据1-4结果，返回此光流方法避撞的评价指数,值越大性能越差                   */
/***************************************************************************************/
float compareTag(int *tagOrigin, int *tagSafe, int cols, int *tags){
	int tag_1 = 0, tag_2 = 0, tag_3 = 0, tag_4 = 0;
	for (int i = 0; i < cols; i++)
	{
		if (tagOrigin[i] == 1 && tagSafe[i] == 1)
		{
			tags[i] = 1;
			tag_1++;
		}else if (tagOrigin[i] == 1 && tagSafe[i] == 0)
		{
			tags[i] = 2;
			tag_2++;
		}else if (tagOrigin[i] == 0 && tagSafe[i] == 1)
		{
			tags[i] = 3;
			tag_3++;
		} 
		else if (tagOrigin[0] == 0 && tagSafe[i] == 0)
		{
			tags[i] = 4;
			tag_4++;
		}
	}
	return tag_1 + tag_2*1.5 + tag_3*2 + tag_4;//各比例项暂定1-1.5-2-1；
}

float balanceControlLR(bool isBig, int leftSumFlow, int rightSumFlow, float k){
    float result = -2;
    //if(leftSumFlow>rightSumFlow)
	//result*=-1;
    if(isBig){ //前方遇到同一色巨型障碍物，返回-2
	printf("BIG\n");
        return result*INT_FLOAT;
    }
    if(leftSumFlow == 0 || rightSumFlow == 0){
        return turnLRScale(leftSumFlow, rightSumFlow, k);
    }
    float gain = (rightSumFlow*INT_FLOAT)/(leftSumFlow*1.0);
    //return gain;
    if(gain < INT_FLOAT*k && gain > INT_FLOAT/k){
        return 0;
    }else{
        return turnLRScale(leftSumFlow, rightSumFlow, k);
    }
}
float turnLRScale(float leftSumFlow, float rightSumFlow, float k){
    if(leftSumFlow == rightSumFlow){
        return 0;
    }else{
        if(K_FLAG){
            k = 1;
        }
        if(leftSumFlow > rightSumFlow){
            return ((leftSumFlow - k*rightSumFlow)*INT_FLOAT)/(leftSumFlow + rightSumFlow);
        }else{
            return ((k*leftSumFlow - rightSumFlow)*INT_FLOAT)/(leftSumFlow + rightSumFlow);
        }
    }
}

Mat calibrate(Mat img)
{
	static double mtx[3][3]={562.89836209,0,314.41994795,0 ,552.27825968,160.37443571,0,0,1};
	static Mat matrix(Size(3,3),CV_64F,mtx);
	static Vec<float,5> dist(-5.25438307e-01,3.76324874e-01,-4.78114662e-04,3.51717002e-04,-1.37709825e-01); 
	static Mat newcameramtx = getOptimalNewCameraMatrix(matrix , dist ,Size(img.rows,img.cols), 0,Size(img.rows,img.cols));
	Mat img2;
	undistort(img,img2,matrix,dist,newcameramtx);
    //matrix.release();
    //img2.release();
    //newcameramtx.release();
	return img2;
}

void calibrate(IplImage* &iplimg)
{
	Mat img(iplimg,false);
	Mat img2 = calibrate(img);
	//iplimg = (IplImage *) cvClone(&IplImage(img2));
	IplImage img3 = IplImage(img2);
	iplimg = (IplImage *)cvClone(&img3);
    img.release();
    img2.release();
}

void writeFile(const char* lineStr){
	FILE *fp = fopen("../video/result.txt", "a+");
	if (fp == 0)
	{
		printf("can't open file\n");
		return;
	}
	fseek(fp, 0, SEEK_END);
	fwrite(lineStr, strlen(lineStr), 1, fp);
	fclose(fp);
}

void drawOrientation(Vec2i leftSumFlow, Vec2i rightSumFlow, int px, int py, float result, IplImage* imgdst){
	Vec2i diffFlow = Vec2i(leftSumFlow[0] - rightSumFlow[0], leftSumFlow[1] - rightSumFlow[1]);
	//printf("diffFlow: %d , %d \n", diffFlow[0], diffFlow[1]);
	//printf("left :%d  right:%d\n", leftSumFlow[0], rightSumFlow[0]);
	cvLine(imgdst, cvPoint(px, py), cvPoint(px+diffFlow[0], py), CV_RGB (0, 255, 0), 3, CV_AA, 0);

	CvFont font;
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, 1, 1, 0, 2);

	if (result == -2*INT_FLOAT)
	{
		cvPutText(imgdst, "S", cvPoint(20, 20), &font, CV_RGB(255, 0, 0));
	}else if (result  == 0)
	{
		cvPutText(imgdst, "F", cvPoint(20, 20), &font, CV_RGB(255, 0, 0));
	}else if (result < 0)
	{
		cvPutText(imgdst, "L", cvPoint(20, 20), &font, CV_RGB(255, 0, 0));
	}else if (result > 0)
	{
		cvPutText(imgdst, "R", cvPoint(20, 20), &font, CV_RGB(255, 0, 0));
	}

	char c[50];
	//itoa(result, c, 10);
	sprintf(c, "%d", result);
	cvPutText(imgdst, c, cvPoint(10, 50), &font, CV_RGB(255, 0, 0));
}

void drawFlowWithoutZero(CvPoint2D32f* cornersprev, CvPoint2D32f* cornerscurr, IplImage* imgdst, bool isleft){
	for (int i =0; i < MAX_CORNERS; i++)
	{
		if ((int)cornersprev[i].x < 0 || (int)cornersprev[i].x > WIDTH )
		{
			break;
		}
		if (((int)cornersprev[i].x - (int)cornerscurr[i].x) == 0 && ((int)cornersprev[i].y - (int)cornerscurr[i].y) == 0)
		{
			continue;
		}
		CvPoint p,q;
		p.x = isleft ? (int) cornersprev[i].x : ((int)cornersprev[i].x + WIDTH/2);
		p.y = (int) cornersprev[i].y;
		q.x = isleft ? (int) cornerscurr[i].x : ((int)cornerscurr[i].x + WIDTH/2);;
		q.y = (int) cornerscurr[i].y;
		drawArrow(p, q, imgdst);
	}
}

void drawFlowWithoutZero(CvMat* velx, CvMat* vely, IplImage* imgdst){
	for (int i = 0; i < HEIGHT; i += 1)
	{
		for(int j = 0; j < WIDTH; j += 1){
			if ((int) cvGetReal2D(velx, i, j) == 0 && (int) cvGetReal2D(vely, i, j) == 0 )
			{
				continue;
			}
			CvPoint p, q;
			q.x = (int) cvGetReal2D(velx, i, j) + j;
			q.y = (int) cvGetReal2D(vely, i, j) + i;
			p.x = j;
			p.y = i;
			drawArrow(p, q, imgdst);
		}
	}
}

void drawFlowWithoutZero(Mat flow, Mat &framedst){
	for (int i = 0; i < HEIGHT; i += 1)
	{
		for(int j = 0; j < WIDTH; j += 1){
			Vec2f flow_at_point = flow.at<Vec2i>(i, j);
			float fx = flow_at_point[0]/10e8;
			float fy = flow_at_point[1]/10e8;
			if (fabs(fx) > UNKNOWN_FLOW_THRESH || fabs(fy) > UNKNOWN_FLOW_THRESH || (fx == 0 && fy == 0))
			{
				continue;
			}
			CvPoint p, q;
			p.x = j;
			p.y = i;
			q.x = fx + j;
			q.y = fy + i;
			//printf("p: (%d,%d). q :(%d, %d)\n", p.x, p.y, flow.at<Vec2i>(i, j)[0], flow.at<Vec2i>(i, j)[1]);
			drawArrow(p, q, framedst);
		}
	}
}

void drawFlowForFeatureCvPoint(CvPoint2D32f* cornersprev, CvPoint2D32f* cornerscurr, IplImage* imgdst, bool isleft){
	for (int i =0; i < MAX_CORNERS; i++)
	{
		if ((int)cornersprev[i].x < 0 || (int)cornersprev[i].x > WIDTH )
		{
			break;
		}
		CvPoint p,q;
		p.x = isleft ? (int) cornersprev[i].x : ((int)cornersprev[i].x + WIDTH/2);
		p.y = (int) cornersprev[i].y;
		q.x = isleft ? (int) cornerscurr[i].x : ((int)cornerscurr[i].x + WIDTH/2);;
		q.y = (int) cornerscurr[i].y;
		drawArrow(p, q, imgdst);
	}
}

void drawFlowForDenseCvMat(CvMat* velx, CvMat* vely, IplImage* imgdst){
	for (int i = DRAWGAP; i < HEIGHT; i += DRAWGAP)
	{
		for(int j = DRAWGAP; j < WIDTH; j += DRAWGAP){
			CvPoint p, q;
			q.x = (int) cvGetReal2D(velx, i, j) + j;
			q.y = (int) cvGetReal2D(vely, i, j) + i;
			p.x = j;
			p.y = i;
			drawArrow(p, q, imgdst);
		}
	}
}

void drawFlowForDenseMat(Mat flow, Mat &framedst){
	for (int i = DRAWGAP; i < HEIGHT; i += DRAWGAP)
	{
		for(int j = DRAWGAP; j < WIDTH; j += DRAWGAP){
			Vec2f flow_at_point = flow.at<Vec2i>(i, j);
			float fx = flow_at_point[0]/10e8;
			float fy = flow_at_point[1]/10e8;
			if (fabs(fx) > UNKNOWN_FLOW_THRESH || fabs(fy) > UNKNOWN_FLOW_THRESH)
			{
				continue;
			}
			CvPoint p, q;
			p.x = j;
			p.y = i;
			q.x = fx + j;
			q.y = fy + i;
			//printf("p: (%d,%d). q :(%d, %d)\n", p.x, p.y, flow.at<Vec2i>(i, j)[0], flow.at<Vec2i>(i, j)[1]);
			drawArrow(p, q, framedst);
		}
	}
}

void drawArrow(CvPoint p, CvPoint q, IplImage* imgdst, CvScalar rgb , int thickness){
	double angle; 
	angle = atan2((double) p.y - q.y, (double) p.x - q.x);
	double hypotenuse; 
	hypotenuse = sqrt(((p.y - q.y)*(p.y - q.y) +(p.x - q.x)*(p.x - q.x))*1.0);

	q.x = (int) (p.x - 3 * hypotenuse * cos(angle));
	q.y = (int) (p.y - 3 * hypotenuse * sin(angle));
	cvLine(imgdst, p, q, rgb,thickness);

	p.x = (int) (q.x + 2 * hypotenuse * cos(angle + CV_PI / 4));
	p.y = (int) (q.y + 2 * hypotenuse * sin(angle + CV_PI / 4));
	cvLine(imgdst, p, q, rgb,thickness);
	p.x = (int) (q.x + 2 * hypotenuse * cos(angle - CV_PI / 4));
	p.y = (int) (q.y + 2 * hypotenuse * sin(angle - CV_PI / 4));
	cvLine(imgdst, p, q, rgb,thickness);
}

void drawArrow(CvPoint p, CvPoint q, Mat &framedst, CvScalar rgb , int thickness){
	double angle; 
	angle = atan2((double) p.y - q.y, (double) p.x - q.x);
	double hypotenuse; 
	hypotenuse = sqrt(((p.y - q.y)*(p.y - q.y) +(p.x - q.x)*(p.x - q.x))*1.0);

	q.x = (int) (p.x - 3 * hypotenuse * cos(angle));
	q.y = (int) (p.y - 3 * hypotenuse * sin(angle));
	line(framedst, p, q, rgb,thickness);

	p.x = (int) (q.x + 3 * cos(angle + CV_PI / 4));
	p.y = (int) (q.y + 3  * sin(angle + CV_PI / 4));
	line(framedst, p, q,rgb,thickness );

	p.x = (int) (q.x + 3 * cos(angle - CV_PI / 4));
	p.y = (int) (q.y + 3 * sin(angle - CV_PI / 4));
	line(framedst, p, q, rgb,thickness );
}










