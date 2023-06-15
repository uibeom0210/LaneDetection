#pragma once
#ifndef _LANE_DETECTOR_
#define _LANE_DETECTOR_

#define IMSHOW_FRAME
#ifdef _DEBUG
//#define IMSHOW_FRAME
#define IMSHOW_TOP
#define IMSHOW_FILTER
//#define IMSHOW_EDGE
#define IMSHOW_HISTO
//#define DRAW_POINT_TOP
//#define HSV_TRACK_BAR
#define IMSHOW_ROI
#define IMSHOW_LINE
#define IMSHOW_UNWARP
#endif // _DEBUG

#include "OpencvInit.h"

using namespace std;

class LaneDetector
{
public:
	typedef struct drawData
	{
		vector<Point> vPtFind_Left;
		vector<Point> vPtFind_Right;
		double radius_Left;
		double radius_Right;
		double center_Offset;

	} drawDataInfo;

	LaneDetector();
	~LaneDetector();

	Mat filterColors(Mat img_frame);
	Mat limitRegion(Mat img_edges);
	Mat makeTopView(Mat img_frame);
	void getPosition(Mat img);
	Mat makeROI(Mat img_filter, drawDataInfo& drawData);
	Mat drawLine(Mat img_draw_rois, drawDataInfo& drawData);
	//Mat unwarpImg(Mat img_drawline, const Mat& trans, const Mat& img_frame);
	Mat unwarpImg(const Mat& img_drawline, const Mat& img_frame);
	cv::Mat PolynomialFit_XY(std::vector<cv::Point>& pts, int order);

	cv::Mat PolynomialFit_YX(std::vector<cv::Point>& pts, int order);
	Mat getTrans();
	//Mat drawLine(Mat img_input, vector<Point> lane, string dir);
	//void DrawDashedLine(cv::Mat& img, cv::Point pt1, cv::Point pt2, cv::Scalar color, int thickness, std::string style, int gap);
	//bool IntersectPoint(const Point& pt11, const Point& pt12, const Point& pt21, const Point& pt22, Point* ptCross);
	//float GetOffsetDist();
private:
	Size img_Size;
	double img_size, img_center;
	double left_m, right_m;
	Point left_b, right_b;
	bool left_detect = false, right_detect = false;
	//관심 영역 범위 계산시 사용 
	double poly_bottom_width = 1;  //사다리꼴 아래쪽 너비 계산을 위한 백분율
	double poly_top_width = 0.16;  //사다리꼴 위쪽 너비 계산을 위한 백분율
	double poly_height = 0.35;     //사다리꼴 높이 계산을 위한 백분율
	int initial_pos[2] = { 50, 230 };
	static const int rois = 10;  //roi 개수
	cv::Rect l_roi[rois], r_roi[rois];
};


#endif // _LANE_DETECTOR_

