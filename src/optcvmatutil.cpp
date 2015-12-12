#include "stdafx.h"
#include <cv.h>
#include <highgui.h>

//#include "optdrawflow.h"
#include "optutil.h"
#include "optcvmatutil.h"
#include <math.h>

using namespace cv;
using namespace std;

/************************************************************************/
/* 稠密光流，光流参数类型为CvMat的FOE（x，y）的计算
   velx：x方向光流；vely：y方向光流                                     */
/************************************************************************/
Vec2i foeForDenseCvMat1(CvMat* velx, CvMat* vely){
	int length = velx->height * velx->width;/*x方向与y方向的图像大小是一样的*/
	//计算H0 = +(ai0*bi);
	//计算H1 = +(ai0*ai0);
	//计算H2 = +(ai1*bi);
	//计算H3 = +(ai0*ai1);
	//计算H4 = +(ai1*ai1);
	//计算H5 = +(ai0**2 * ai1**2);
	double H0 = 0;
	double H00 = 0;
	double H1 = 0;
	double H11 = 0;
	double H2 = 0;
	double H22 = 0;
	double H3 = 0;
	double H33 = 0;
	double H4 = 0;
	double H44 = 0;
	double H5 = 0;
	double H55 = 0;
	for (int col = 0; col < velx->width; col += 200){
		for (int row = 0; row < velx->height; row += 200){
			float* vy = vely->data.fl + row * vely->step/4 + col;
			float* vx = velx->data.fl + row * velx->step/4 + col;
			float ai0 = *vy;
			float ai1 = -*vx;
			double bi = col * *vy - row * *vx;
			double h0 = ai0 * bi;
			double h1 = ai0 * ai0;
			double h2 = ai1 * bi;
			double h3 = ai0 * ai1;
			double h4 = ai1 * ai1;
			double h5 = ai0 * ai0 * ai1 * ai1;
			H00 += h0;
			H11 += h1;
			H22 += h2;
			H33 += h3;
			H44 += h4;
			H55 += h5;
		}
		H0 += H00;
		H1 += H11;
		H2 += H22;
		H3 += H33;
		H4 += H44;
		H5 += H55;
	}
	/*printf("H0->%f\n", H0);
	printf("H1->%f\n", H1);
	printf("H2->%f\n", H2);
	printf("H3->%f\n", H3);
	printf("H4->%f\n", H4);
	printf("H5->%f\n", H5);*/

	//计算foeX = (H0*H4 - H2*H3) / (H5 - H3**2)
	double q = (H5 - H3 * H3);
	//printf("q->%d\n", q);
	int foeX = (int) ((H0*H4 - H2*H3) / q);
	//printf("foeX->%d\n", foeX);
	//计算foeY = (H0*H3 - H2*H1) / (H5 - H3**2)
	int foeY = (int) ((-H0*H3 + H2*H1) / q);
	//printf("foeY->%d\n", foeY);
	return Vec2i(foeX, foeY);
}

Vec2i foeForDenseCvMat2(CvMat* velx, CvMat* vely){
	//calculate x of foe
	float cols[COLS];
	for (int col = 0; col < velx->width; col++)
	{
		float tmp = 0;
		for (int row = 0; row < velx->height; row++)
		{
			float* pData = velx->data.fl + row*velx->step/4;
			tmp += *(pData + col); 
		}
		cols[col] = tmp;
	}
	float colsLeft[COLS];//以col左侧的光流之和
	for (int i = 0; i < velx->width; i++)
	{
		colsLeft[i] = cols[i];
		if (i > 0)
		{
			colsLeft[i] += colsLeft[i-1];
		}
	}
	float colsRight[COLS];//以col右侧的光流之和
	for (int j = velx->width - 1; j >= 0; j--)
	{
		colsRight[j] = cols[j];
		if (j < velx->width -1)
		{
			colsRight[j] += colsRight[j+1];
		};
	}
	//找出x使其左右两边之差最小的为foe点的x
	float colsMin = abs(abs(colsLeft[0]) - abs(colsRight[0]));
	int foeX = 0;
	for (int i = 1; i < velx->width; i++)
	{
		float tmp = abs(abs(colsLeft[i]) - abs(colsRight[i]));
		if (tmp < colsMin)
		{
			colsMin = tmp;
			foeX = i;
		}
	}

	//calculate y of foe
	float rows[ROWS];
	for (int row = 0 ; row < vely->height; row++ ){
		float tmp = 0;
		float* pData = vely->data.fl + row*vely->step/4;
		for(int col = 0 ; col < vely->width; col++){
			tmp += *(pData + col);
		}
		rows[row] = tmp;
	}
	float rowsUp[ROWS];
	for (int i = 0; i < vely->height; i++)
	{
		rowsUp[i] = rows[i];
		if (i > 0)
		{
			rowsUp[i] += rowsUp[i-1];
		}
	}
	float rowsDown[ROWS];
	for (int j = vely->height - 1; j >= 0; j--)
	{
		rowsDown[j] = rows[j];
		if (j < vely->height-1)
		{
			rowsDown[j] += rowsDown[j+1];
		}
	}
	float rowsMin = abs(abs(rowsUp[0]) - abs(rowsDown[0]));
	int foeY = 0;
	for (int i = 1; i < vely->height; i++)
	{
		float tmp = abs(abs(rowsUp[i]) - abs(rowsDown[i]));
		if (tmp < rowsMin)
		{
			rowsMin = tmp;
			foeY = i;
		}
	}
	printf("FOE : [cols(x) = %d, rows(y) = %d]\n", foeX, foeY);

	return Vec2i(foeX, foeY);
}

/************************************************************************/
/* 稠密光流且光流参数类型为CvMat的TTC计算，返回平均TTC值
   vely：y方向光流； foeY：foe点y的坐标；ttc：计算获得的每列的ttc       */
/************************************************************************/
float ttcForDenseCvMat(CvMat* vely, int foeY, float *ttc){
	float sum  = 0;
	for (int col = 0; col < vely->width; col++)
	{
		float tmp = 0;
		for (int row = 0; row < vely->height; row++)
		{
			float* pData = vely->data.fl + row*vely->step/4 + col;
			if (*pData == 0)
			{
				tmp += abs((row - foeY)*10); //(row - foeY)/0.1,如果为0，则认为速度为0.1
			}else{
				tmp += abs((row - foeY)/(*pData));
			}
		}
		ttc[col] = tmp/(vely->height);
		sum += ttc[col];
	}
	return sum/(vely->width);
}
/********************************
/*利用障碍物间隙
*********************************/
float safeAreaForDenseCvMat(CvMat* velx, CvMat* vely, IplImage* imgdst, float k){
	float * ttc = (float*)malloc(COLS*sizeof(float));
	int * tagSafe = (int*)malloc(COLS*sizeof(float)); 
	Vec2i foe = foeForDenseCvMat2(velx, vely);
	float ttcAvg = ttcForDenseCvMat(vely, foe[1], ttc);
	tagSafeAreaByTTC(COLS, ttc, ttcAvg , k, tagSafe);
	float result = 0;
	
	int safel = -1;
	int safer = -1;
	int safem = -1;
	int safevalue = 0;
	int tempvalue = 0;
	int i = 0;
	int flag = 0;
	for(i = 0; i < COLS; i++){
		if(tagSafe[i] == 1){
			tempvalue++;
			safer = i;
			if(safel == -1){
				safer = i;
			}
			if(tempvalue > safevalue){
				safevalue = tempvalue;
				safem = (safer+safel)/2;
			}	
		}else{
			flag++;
			if(flag >= 3){
				safer = -1;
				safel = -1;
				tempvalue = 0;
			}
		}
	}
	bool isBig = isBigObstacle(imgdst, velx);
	if(isBig || safevalue < (COLS/3)){
		result = -200;
	}else{
		result = 2*safem/COLS - 1;
	}
	return result;
}

//利用左右光流平衡返回左1右2前3停止4
float balanceForDenseCvMat(CvMat* velx, CvMat* vely, IplImage* imgdst,  float k, int px, int py){
	Vec2i leftSumFlow = Vec2i(0, 0);
    float up = EDGE*HEIGHT;
    float down = (1-EDGE)*HEIGHT;
	for (int i = 0; i < px; i++)
	{
		for (int j = up; j < down; j++)
		{
			leftSumFlow[0] += (int) cvGetReal2D(velx, j, i);
			leftSumFlow[1] += (int) cvGetReal2D(vely, j, i);
		}
	}
	Vec2i rightSumFlow = Vec2i(0, 0);
	for (int i = px; i < WIDTH; i++)
	{
		for(int j = up; j < down; j++){
			rightSumFlow[0] += (int) cvGetReal2D(velx, j, i);
			rightSumFlow[1] += (int) cvGetReal2D(vely, j, i);
		}
	}
	//printf("pre left :%d,%d  right:%d,%d\n", leftSumFlow[0], leftSumFlow[1], rightSumFlow[0], rightSumFlow[1]);
	leftSumFlow[0] = abs(leftSumFlow[0] / px);
	leftSumFlow[1] = abs(leftSumFlow[1] / px);
	rightSumFlow[0] = abs(rightSumFlow[0] / (WIDTH - px));
	rightSumFlow[1] = abs(rightSumFlow[1] / (WIDTH - px));

	if(IS_FLOW_WRITE_FILE){
		char buffer[50];
		sprintf(buffer, "leftSumFlow	%d	rightSumFlow	%d\n", leftSumFlow[0], rightSumFlow[0]);
		writeFile(buffer);
	}

    bool isBig = isBigObstacle(imgdst, velx);
    float result  = balanceControlLR(isBig, leftSumFlow[0], rightSumFlow[0], k);
    
    drawOrientation(leftSumFlow, rightSumFlow, px, py, result, imgdst);
    return result;
}

bool isBigObstacle(IplImage* imgdst, CvMat* velx){
   uchar *data = (uchar*)imgdst->imageData;
	int step = imgdst->widthStep / sizeof(uchar);
	int channels = imgdst->nChannels;
	int sumR = 0, sumG = 0, sumB = 0;
	int count = 0;
	for(int i = EDGE_OBS*HEIGHT; i < (1-EDGE_OBS)*HEIGHT; i++){
		for(int j = EDGE_OBS*WIDTH; j < (1-EDGE_OBS)*WIDTH; j++ ){
			sumB += (int)data[i*step+j*channels+0];
			sumG += (int)data[i*step+j*channels+1];
			sumR += (int)data[i*step+j*channels+2];
			count ++ ;
		}
	}
	int avgB = sumB / count;
	int avgG = sumG / count;
	int avgR = sumR / count;
	int timers;
	int timerCount = 0;
	int flowZeroCount = 0;
	for(int i = EDGE_OBS*HEIGHT; i < (1-EDGE_OBS)*HEIGHT; i++){
		for(int j = EDGE_OBS*WIDTH; j < (1-EDGE_OBS)*WIDTH; j++ ){
			timers = 0;
			if(abs((int)data[i*step+j*channels+0] - avgB) < COLOR_SCALE){
				timers += 1;
			}
			if(abs((int)data[i*step+j*channels+1] - avgG) < COLOR_SCALE){
				timers += 1;
			}
			if(abs((int)data[i*step+j*channels+2] - avgR) < COLOR_SCALE){
				timers += 1;
			}
			if (timers == 3)
			{
				timerCount ++;
				if (abs(((int) cvGetReal2D(velx, i, j))) <= FLOW_ZERO)
				{
					flowZeroCount ++;
				}
			}
		}
	}
	float timerPro = (timerCount*1.0)/count;
	float flowZeroPro = (flowZeroCount*1.0)/timerCount;
	//printf("time -> %f flow -> %f\n", timerPro, flowZeroPro );
	if (timerPro > THRESHOLD_TIMER && flowZeroPro <= THRESHOLD_ZERO)
	{
		return true;
	}else{
		return false;
	}
}

void getSpeedFromFlow(CvMat* velx, CvMat* vely, IplImage *imgdst){
	static int midy = HEIGHT/2, midx = WIDTH/2;
	int count = 0, x, y;
	double w = 0, v = 0;
	for (int k = -10; k < -9; k+=5)
	{
		//double angle = TACHOGRAPH_ANGLE*CV_PI/180;
		double angle = k*CV_PI/180;
		double sina = sin(angle);
		double cosa = cos(angle);
		for (int i = HEIGHT*TACHOGRAPH_UPY; i < HEIGHT*TACHOGRAPH_DOWNY; i++ )
		{
			for (int j = WIDTH*TACHOGRAPH_UPX; j < WIDTH*TACHOGRAPH_DOWNX; j++)
			{
		/*for (int i = 80 ; i < HEIGHT*TACHOGRAPH_DOWNY; i++ )
		{
			for (int j = 175; j < WIDTH*TACHOGRAPH_DOWNX; j++)
			{*/
				//if (abs(0.6923*j - 41 - i)<10)
				{
					/*CvPoint a;
					a.x = j, a.y = i;
					cvCircle(imgdst, a, 0.1, CV_RGB(0,255,0));*/
					int px = (int) cvGetReal2D(velx, i, j);
					int py = (int) cvGetReal2D(vely, i, j);
					if (px != 0 && py > 0)
					{
						//py = abs(py);
						x = j - midx, y = i - midy;
						double tmp = TACHOGRAPH_FOCAL*sina + y*cosa;
						//if (abs(tmp) > 0.6 && abs(tmp) < 3.5)
						{
							double w1 = (TACHOGRAPH_FOCAL*TACHOGRAPH_HEIGHT*py)/(tmp*tmp*3.6);
							double v1 = (TACHOGRAPH_HEIGHT*x*py*cosa - TACHOGRAPH_HEIGHT*px*tmp)/(tmp*tmp*3.6);
							w += (TACHOGRAPH_FOCAL*TACHOGRAPH_HEIGHT*py)/(tmp*tmp*3.6);  //因为最后m/s换算成km/h要除3.6
							v += (TACHOGRAPH_HEIGHT*x*py - TACHOGRAPH_HEIGHT*px*tmp)/(tmp*tmp*3.6);
							count++;
						/*	char buffer[50];
							sprintf(buffer, "%d	%d	%d	%d	%lf	%lf	%lf\n", x, y, px, py, w1, v1, tmp);
							writeFile(buffer);*/
						}
					}
				}
			}
		}
		count = count == 0 ? 1 : count;
		char buffer[50];
		sprintf(buffer, "%d	%lf	%lf	", k, w/count, v/count);
		writeFile(buffer);

		CvPoint a, b;
		a.y = 40, a.x = WIDTH-30;
		b.y = 40-(w/count)/10, b.x = WIDTH-30+(v/count)/10;
		drawArrow(a,b, imgdst, CV_RGB(255,0,0),2);

		/*	CvPoint p, q;
		p.y = 80, p.x = 175;
		q.y = 125, q.x = 240;
		cvLine(imgdst, p, q, CV_RGB(255,0,0),1);*/
		

		CvFont font;
		cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, 1, 1, 0, 2);
		char c[50];
		//itoa(w/count, c, 10);
		sprintf(c, "%d", w/count);
		cvPutText(imgdst, c, cvPoint(WIDTH-50,70), &font, CV_RGB(255, 0, 0));
	}
	char buffer[50];
	sprintf(buffer, "\n");
	writeFile(buffer);
}


