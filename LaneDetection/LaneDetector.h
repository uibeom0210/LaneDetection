#pragma once
#ifndef _LANE_DETECTOR_
#define _LANE_DETECTOR_

#ifdef _DEBUG
	#define IMSHOW_FRAME
	#define IMSHOW_TOP
	#define IMSHOW_FILTER
	//#define IMSHOW_EDGE
	#define DRAW_POINT_TOP
#endif // _DEBUG

#include "OpencvInit.h"
#include <vector>
using namespace std;

class LaneDetector
{
public:
	LaneDetector();
	~LaneDetector();

	Mat filterColors(Mat img_frame);
	Mat limitRegion(Mat img_edges);
	Mat makeTopView(Mat img_frame);
	Mat drawLine(Mat img_input, vector<Point> lane, string dir);

private:
	double img_size, img_center;
	double left_m, right_m;
	Point left_b, right_b;
	bool left_detect = false, right_detect = false;
	//관심 영역 범위 계산시 사용 
	double poly_bottom_width = 1;  //사다리꼴 아래쪽 너비 계산을 위한 백분율
	double poly_top_width = 0.16;  //사다리꼴 위쪽 너비 계산을 위한 백분율
	double poly_height = 0.35;     //사다리꼴 높이 계산을 위한 백분율
};


#endif // _LANE_DETECTOR_