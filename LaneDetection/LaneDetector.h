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
	//���� ���� ���� ���� ��� 
	double poly_bottom_width = 1;  //��ٸ��� �Ʒ��� �ʺ� ����� ���� �����
	double poly_top_width = 0.16;  //��ٸ��� ���� �ʺ� ����� ���� �����
	double poly_height = 0.35;     //��ٸ��� ���� ����� ���� �����
};


#endif // _LANE_DETECTOR_