/*
*  FILE stdafx.h
*  AUTHOR Sarah
*  DATE 2015/08/14 22:27
*  TODO: 标准系统包含文件的包含文件，或是经常使用但不常更改的特定于项目的包含文件
*/
#pragma once

#include <stdio.h>

/*
*  ROWS: 数组申请内存空间大小，大于等于HEIGHT.
*  COLS: 数组申请内存空间大小，大于等于WIDTH.
*  HEIGHT: 重置图像后的\E9\AB?
*  WIDTH: 重置图像后的\E5\AE?
*  MAX_CORNERS: PyrLK计算特征点数\E7\9B?
*  WINSIZE: 窗口大小.
*  DRAWGAP: 画光流的间隔.
*  UNKNOWN_FLOW_THRESH: 大于此\E5?得光流认为是error光流. 
*/
#define ROWS 350
#define COLS 350
//#define HEIGHT 190  //matlab
//#define WIDTH 288   //matlab
#define HEIGHT 180   //ARDrone
#define WIDTH 320   //ARDrone
#define MAX_CORNERS  200
#define WINSIZE 5
#define DRAWGAP 15
#define UNKNOWN_FLOW_THRESH 1e10
#define LK_K 6
#define HS_K 4.3
#define BM_K 1.4
#define FB_K 2
#define SF_K 3 //δ\B2\E2\CA\D4
#define PYRLK_K 3 //δ\B2\E2\CA\D4
#define INT_FLOAT 100.0 //matlab\D6\D0int\CF\F2\C9\CFfloatת\D0\CD\D3\D0\CE\CA\CC⣬\CB\F9\D2Խ\E1\B9\FB\B3\CB100\A3\AC\B4\A6\C0\EDʱ\D4ٳ\FD100.
#define K_FLAG true  //\D4ڱ\DCײƫ\D2\C6\C1\BF\D6У\ACfalse\B1\EDʾkȡ1\A3\AC\B6\F8\B7\C7\C9\CF\CA\F6*_K\B1\E4\C1\BF
#define EDGE 0 //0.1429
#define EDGE_OBS 0.33
#define COLOR_SCALE 20
#define THRESHOLD_TIMER 0.75 //ͬɫ\CB\F9ռ\B1\C8\C0\FD\B4\F3\D3\DA\D5\FB\B7\F9ͼ\CF\F1\B5\C475%\A3\AC\D4\F2\C8\CFΪ\CA\C7ǽ\A3\ACֹͣ\A1\A3
#define THRESHOLD_ZERO 0.5
#define FLOW_ZERO 0
#define FB_SCALE 10e8
#define IS_FLOW_WRITE_FILE false   //\CAǷ\F1\BD\AB\D7\F3\D3ҹ\E2\C1\F7\CA\FD\BE\DDд\C8\EB\CEļ\FE
#define IS_CALIBRATE false//\CAǷ\F1\BD\F8\D0\D0\C9\E3\CF\F1ͷ\B1궨
//#define TACHOGRAPH_UPX 0.25  //\B5\D8\C3\E6
//#define TACHOGRAPH_UPY 0.6
//#define TACHOGRAPH_DOWNX 0.75
//#define TACHOGRAPH_DOWNY 0.9
#define TACHOGRAPH_UPX 0
#define TACHOGRAPH_UPY 0
#define TACHOGRAPH_DOWNX 1
#define TACHOGRAPH_DOWNY 1
#define TACHOGRAPH_ANGLE 10
#define TACHOGRAPH_FOCAL 68 //\B5\A5λ190\CF\F1\CBأ\ACע\A3\BA1080p\BD\B9\BE\E0Ϊ1141\CF\F1\CB\D8
#define TACHOGRAPH_HEIGHT 1.2 // \B5\A5λ\C3\D7

