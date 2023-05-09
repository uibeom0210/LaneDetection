
#include "LaneDetector.h"

LaneDetector::LaneDetector()
	: img_size(0.)
	, img_center(0.)
	, left_m(0.)
	, right_m(0.)
	, left_b(Point(0, 0))
	, right_b(Point(0, 0))
	, left_detect(false)
	, right_detect(false)
	, poly_bottom_width(1)
	, poly_top_width(0.16)
	, poly_height(0.35)
	//, predict_x(0.)
{
}

LaneDetector::~LaneDetector()
{
}

Mat LaneDetector::filterColors(Mat img_frame)
{
	/*
		흰색/노란색 색상의 범위를 정해 해당되는 차선을 필터링한다.
	*/
	Mat output;
	UMat img_hsv;
	UMat white_mask, white_image;
	UMat yellow_mask, yellow_image;
	img_frame.copyTo(output);

	//차선 색깔 범위 
	Scalar lower_white = Scalar(200, 200, 200); //흰색 차선 (RGB)
	Scalar upper_white = Scalar(255, 255, 255);
	Scalar lower_yellow = Scalar(10, 100, 140); //노란색 차선 (HSV)
	Scalar upper_yellow = Scalar(40, 255, 255);

	//흰색 필터링
	inRange(output, lower_white, upper_white, white_mask);
	bitwise_and(output, output, white_image, white_mask);

	cvtColor(output, img_hsv, COLOR_BGR2HSV);

	//노란색 필터링
	inRange(img_hsv, lower_yellow, upper_yellow, yellow_mask);
	bitwise_and(output, output, yellow_image, yellow_mask);

	//두 영상을 합친다.
	addWeighted(white_image, 1.0, yellow_image, 1.0, 0.0, output);
	return output;
}

Mat LaneDetector::limitRegion(Mat img_edges)
{
	/*
		관심 영역의 가장자리만 감지되도록 마스킹한다.
		관심 영역의 가장자리만 표시되는 이진 영상을 반환한다.
	*/
	int width = img_edges.cols;
	int height = img_edges.rows;

	Mat output;
	Mat mask = Mat::zeros(height, width, CV_8UC1);

	//관심 영역 정점 계산
	Point points[4]{
		Point(width * (1 - poly_bottom_width) / 2, height),
		Point(width * (1 - poly_top_width) / 2, height - height * poly_height),
		Point(width - (width * (1 - poly_top_width)) / 2,
												height - height * poly_height),
		Point(width - (width * (1 - poly_bottom_width)) / 2, height)
	};

	//정점으로 정의된 다각형 내부의 색상을 채워 그린다.
	fillConvexPoly(mask, points, 4, Scalar(255, 0, 0));

	//결과를 얻기 위해 edges 이미지와 mask를 곱한다.
	bitwise_and(img_edges, mask, output);
	return output;
}

Mat LaneDetector::makeTopView(Mat img_frame)
{
	int width = img_frame.size().width;
	int height = img_frame.size().height;

	const int poly_pts = 4;
	vector<Point2f> points = {
		Point2f(width * (1-poly_top_width) / 2, height - height * poly_height),
		Point2f(width - (width * (1 - poly_top_width)) / 2,
												height - height * poly_height),
		Point2f(width * (1-poly_bottom_width) / 2, height),
		Point2f(width - (width * (1-poly_bottom_width)) / 2, height)
	};
	Size warp_size(width, height);
	Mat img_top(warp_size, img_frame.type());
	//Warping 후의 좌표
	vector<Point2f> warp_corners(4);
	warp_corners[0] = Point2f(0, 0);
	warp_corners[1] = Point2f(img_top.cols, 0);
	warp_corners[2] = Point2f(0, img_top.rows);
	warp_corners[3] = Point2f(img_top.cols, img_top.rows);
	//Transformation Matrix 구하기
	Mat trans = getPerspectiveTransform(points, warp_corners);
	//Warping
	warpPerspective(img_frame, img_top, trans, warp_size);
#ifdef DRAW_POINT_TOP
	for (int i = 0; i < points.size(); i++)
	{
		circle(img_frame, points[i], 3, Scalar(0, 255, 0), 3);
	}
#endif // DRAW_POINT_TOP
	return img_top;
}

Mat LaneDetector::makeHistogram(Mat img)
{
	int width = img.cols;
	int height = img.rows;
	vector<int> _vHistogram(width, 0);
	for (int col = 0; col < width; col++)
	{
		for (int row = 0; row < height; row++)
		{
			int index = row * width + col;
			_vHistogram[col] += img.data[index];
		}
	}
	Mat output = Mat::zeros(height, width, CV_8UC1);
	int y_pix = 0;
	int x_pix = 0;
	int lineType = LINE_8;
	int thickness = 1;
	for (int col = 0; col < width; col++)
	{
		line(output, Point(col, 0), Point(col, _vHistogram[col] / 200),
			Scalar(255), 1, lineType);
	}
	return output;
}

Mat LaneDetector::makeROI(Mat img_filter)
{
	
	const int rois = 10;  //roi 개수
	
	Mat img_gray, img_bin;
	cvtColor(img_filter, img_gray, COLOR_BGR2GRAY);  
	threshold(img_gray, img_bin, 100, 255, ThresholdTypes::THRESH_BINARY);
	cv::Rect l_roi[rois], r_roi[rois];
	float ratio_width = 0.3;
	int subHeight = img_filter.rows / rois;
	int subWidth = img_filter.cols * ratio_width; 

	int l_offset = 10;
	int r_offset = 230;

	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	for (size_t k = 0; k < rois; k++)
	{
		l_roi[k].x = 0+ l_offset; 
		r_roi[k].x = img_filter.cols/2 + r_offset; 
		l_roi[k].y = r_roi[k].y = k*subHeight;
		l_roi[k].width = r_roi[k].width = subWidth; 
		l_roi[k].height = r_roi[k].height = subHeight;

		rectangle(img_filter, l_roi[k], Scalar(255, 0, 0), 2); 
		rectangle(img_filter, r_roi[k], Scalar(0, 0, 255), 2);

		Mat subL = img_bin(l_roi[k]);
		findContours(subL, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
		for (size_t i = 0; i < contours.size(); i++)
		{
			RotatedRect rt = minAreaRect(contours[i]); //외접하는 최소 크기 외접 사각형 찾기
			
			double area = contourArea(contours[i]);
			if (area < 10)
				continue;

			Point ptCenter = rt.center;
			Point ptCross = Point(ptCenter.x + l_roi[k].x, ptCenter.y + l_roi[k].y);
			drawMarker(img_filter, ptCross, Scalar(255, 0, 255), MARKER_CROSS);
			
		}
		Mat subR = img_bin(r_roi[k]);
		findContours(subR, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
		for (size_t i = 0; i < contours.size(); i++)
		{
			RotatedRect rt = minAreaRect(contours[i]);

			double area = contourArea(contours[i]);
			if (area < 10)
				continue;
			Point ptCenter = rt.center;
			Point ptCross = Point(ptCenter.x + r_roi[k].x, ptCenter.y + r_roi[k].y);
			drawMarker(img_filter, ptCross, Scalar(255, 0, 255), MARKER_CROSS);
		}
	}
	return img_filter;
}

Mat LaneDetector::drawLine(Mat img_input, vector<Point> lane, string dir)
{
	return Mat();
}