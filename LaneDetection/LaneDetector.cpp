
#include "LaneDetector.h"
#include <algorithm>

Mat trans;
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
	img_Size = img_frame.size();

	int width = img_frame.cols;
	int height = img_frame.rows;

	const int poly_pts = 4;
	vector<Point2f> OriginAreaPts(poly_pts);
	OriginAreaPts[0] = Point2f(width * (1 - poly_top_width) / 2, height - height * poly_height);
	OriginAreaPts[1] = Point2f(width - (width * (1 - poly_top_width)) / 2, height - height * poly_height);
	OriginAreaPts[2] = Point2f(width * (1 - poly_bottom_width) / 2, height);
	OriginAreaPts[3] = Point2f(width - (width * (1 - poly_bottom_width)) / 2, height);

	//Warping 후의 좌표
	vector<Point2f> WarpAreaPts(poly_pts);
	WarpAreaPts[0] = Point2f(0, 0);
	WarpAreaPts[1] = Point2f(img_frame.cols - 1, 0);
	WarpAreaPts[2] = Point2f(0, img_frame.rows - 1);
	WarpAreaPts[3] = Point2f(img_frame.cols - 1, img_frame.rows - 1);

	//Transformation Matrix 구하기
	trans = getPerspectiveTransform(OriginAreaPts, WarpAreaPts);
	//Warping
	Mat img_topView;
	warpPerspective(img_frame, img_topView, trans, img_frame.size());




	//Mat img_unwarp;
	//Mat trans_inv = trans.inv();
	//warpPerspective(img_topView, img_unwarp, trans_inv, img_topView.size(), INTER_LINEAR);




#ifdef DRAW_POINT_TOP
	for (int i = 0; i < points.size(); i++)
	{
		circle(img_frame, points[i], 3, Scalar(0, 255, 0), 3);
	}
#endif // DRAW_POINT_TOP
	return img_topView;
}

void LaneDetector::getPosition(Mat img)
{
	const int left = 0, right = 1;
	int width = img.cols;
	int height = img.rows;
	float ratio_width = 0.20;
	int subWidth = width * ratio_width;
	int max_left = 0, max_right = 0;
	vector<int> v_histogram(width, 0);
	int v_pos[2] = { 0 };
	for (int col = 0; col < width; col++)
	{
		for (int row = 0; row < height; row++)
		{
			int index = row * width + col;   // 2차원 이미지의 픽셀을 1차원 배열의 인덱스로 변환하기 위해 공식 사용
			v_histogram[col] += img.data[index]; //벡터에 각 열의 히스토그램 값을 누적
		}
	}
#ifdef IMSHOW_HISTO
	Mat output = Mat::zeros(height, width, CV_8UC1);
	int y_pix = 0;
	int x_pix = 0;
	int lineType = LINE_8;
	int thickness = 1;
	for (int col = 0; col < width; col++)
	{
		line(output, Point(col, 0), Point(col, v_histogram[col] / 150),
			Scalar(255), 1, lineType);
	}
	imshow("img_histo", output);
#endif // IMSHOW_HISTO

	//	왼쪽 차선과 오른쪽 차선의 최대 히스토그램 값을 찾아 v_pos 배열에 해당 열의 위치를 저장
	for (int pos = 0; pos < (width / 2); pos++)
	{
		if (max_left < v_histogram[pos])
		{
			max_left = v_histogram[pos];
			v_pos[left] = pos;
		}
	}
	for (int pos = (width / 2); pos < width; pos++)
	{
		if (max_right < v_histogram[pos])
		{
			max_right = v_histogram[pos];
			v_pos[right] = pos;
		}
	}

	//	서브 영역의 너비(subWidth)를 기준으로 왼쪽과 오른쪽 차선의 위치를 조정
	if (v_pos[left] < (subWidth / 2))
	{
		v_pos[left] = 0;
	}
	else
	{
		v_pos[left] -= (subWidth / 2);
	}
	if (v_pos[right] > (width - (subWidth / 2)))
	{
		v_pos[right] = width - subWidth;
	}
	else
	{
		v_pos[right] -= (subWidth / 2);
	}

	//	초기 위치(initial_pos) 배열에 왼쪽과 오른쪽 차선의 위치를 저장
	initial_pos[left] = v_pos[left];
	initial_pos[right] = v_pos[right];
}

Mat LaneDetector::makeROI(Mat img_filter, drawDataInfo& drawData)
{

	drawData.vPtFind_Left.clear();
	drawData.vPtFind_Right.clear();


	const int left = 0, right = 1;

	Mat img_gray, img_bin;
	cvtColor(img_filter, img_gray, COLOR_BGR2GRAY);//주어진 img_filter를 grayscale로 변환

	//	이진화를 수행,차선 영역을 추출
	threshold(img_gray, img_bin, 100, 255, ThresholdTypes::THRESH_BINARY);

	cv::Rect l_roi_remain, r_roi_remain; //ROI의 하위 영역 너비와 높이를 계산
	int subHeight = img_filter.rows / rois;
	float ratio_width = 0.20;
	int subWidth = img_filter.cols * ratio_width;
	//imshow("img_bin",img_bin);
	getPosition(img_bin);  //getPosition() 함수를 호출하여 초기 차선 위치를 얻음
	int width = img_bin.cols;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	for (size_t k = 0; k < rois; k++) //왼쪽 차선과 오른쪽 차선의 ROI를 생성 후,영역 내에서 윤곽선을 찾음
	{
		l_roi[k].x = initial_pos[left];
		r_roi[k].x = initial_pos[right];
		if (l_roi_remain.x == 0)
		{
			l_roi_remain.x = initial_pos[left];
		}
		if (r_roi_remain.x == 0)
		{
			r_roi_remain.x = initial_pos[right];
		}
		l_roi[k].y = r_roi[k].y = k * subHeight;
		l_roi[k].width = r_roi[k].width = l_roi_remain.width = r_roi_remain.width = subWidth;
		l_roi[k].height = r_roi[k].height = l_roi_remain.height = r_roi_remain.height = subHeight;

		Mat subL = img_bin(l_roi[k]);
		findContours(subL, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
		vector<Point> vPtCross;
		vector<double> vArea;
		double max = 0.0;
		int max_index = 0;
		for (size_t i = 0; i < contours.size(); i++)
		{
			RotatedRect rt = minAreaRect(contours[i]); //외접하는 최소 크기 외접 사각형 찾기
			double area = contourArea(contours[i]);
			if (area < 10)
			{
				continue;
			}

			Point ptCenter = rt.center;
			Point ptCross = Point(ptCenter.x + l_roi[k].x, ptCenter.y + l_roi[k].y);
			vArea.push_back(area);
			vPtCross.push_back(ptCross);
		}

		//		가장 큰 윤곽선을 선택하여 해당 지점에 표식을 그림
		if (vArea.size() == 0) //차선이 검출되지 않은 경우에는 이전 프레임의 ROI를 사용하여 위치를 조정
		{
			l_roi_remain.y = l_roi_remain.y + subHeight;
			l_roi[k] = l_roi_remain;
			rectangle(img_filter, l_roi[k], Scalar(255, 0, 0), 2);
		}
		else
		{
			for (size_t i = 0; i < vArea.size(); i++)
			{
				if (vArea[i] > max)
				{
					max = vArea[i];
					max_index = i;
				}
			}
			drawMarker(img_filter, vPtCross[max_index], Scalar(255, 0, 255), MARKER_CROSS);
			l_roi[k].x = 0 + vPtCross[max_index].x * 0.5;
			l_roi_remain.x = l_roi[k].x;
			l_roi_remain.y = l_roi[k].y;
			rectangle(img_filter, l_roi[k], Scalar(255, 0, 0), 2);
			drawData.vPtFind_Left.push_back(vPtCross[max_index]); //표식 그린 지점을 구조체에 저장
		}
		vPtCross.clear();
		vArea.clear();
		max = 0.0;
		max_index = 0;

		Mat subR = img_bin(r_roi[k]);
		findContours(subR, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
		for (size_t i = 0; i < contours.size(); i++)
		{
			RotatedRect rt = minAreaRect(contours[i]);
			double area = contourArea(contours[i]);
			if (area < 10)
			{
				continue;
			}
			Point ptCenter = rt.center;
			Point ptCross = Point(ptCenter.x + r_roi[k].x, ptCenter.y + r_roi[k].y);
			vArea.push_back(area);
			vPtCross.push_back(ptCross);
		}
		if (vArea.size() == 0)
		{
			if (r_roi_remain.y == 0)
			{
				/*
				r_roi[k].x = r_roi[k].x - 50;
				rectangle(img_filter, r_roi[k], Scalar(0, 255, 255), 2);
				vPtCross[1] = Point((r_roi[k].x + width) / 2, (r_roi[k].y + subHeight) / 2);
				drawMarker(img_filter, vPtCross[1], Scalar(255, 0, 255), MARKER_CROSS);
				*/
				Point markerPoint(r_roi[k].x + 50, (r_roi[k].y + subHeight) / 2);
				drawMarker(img_filter, markerPoint, Scalar(0, 255, 255), MARKER_CROSS);
				drawData.vPtFind_Right.push_back(markerPoint);
				r_roi[k].x = r_roi[k].x - 40;
				rectangle(img_filter, r_roi[k], Scalar(0, 255, 255), 2);
			}
			else
			{
				rectangle(img_filter, r_roi_remain, Scalar(0, 0, 255), 2);
				r_roi_remain.y = r_roi_remain.y + subHeight;
			}
		}
		else
		{
			for (size_t i = 1; i < vArea.size(); i++)
			{
				if (vArea[i] > max)
				{
					max = vArea[i];
					max_index = i;
				}
			}
			drawMarker(img_filter, vPtCross[max_index], Scalar(255, 0, 255), MARKER_CROSS);
			r_roi[k].x = (img_filter.cols / 2) + vPtCross[max_index].x * 0.3;
			rectangle(img_filter, r_roi[k], Scalar(0, 0, 255), 2);
			r_roi_remain.x = r_roi[k].x;
			r_roi_remain.y = r_roi[k].y + subHeight;;
			drawData.vPtFind_Right.push_back(vPtCross[max_index]); //표식 그린 지점을 구조체에 저장
		}
	}
	return img_filter;
}

// img_draw_rois = 다각형이 그려질 이미지
// drawData = 다각형을 그리기 위한 정보를 담고 있는 구조체(표식 그린 지점을 저장한 구조체)
Mat LaneDetector::drawLine(Mat img_draw_rois, drawDataInfo& drawData)
{
	Mat img_drawline = Mat::zeros(img_draw_rois.size(), CV_8UC3);
	vector<Point> vPolyPts;
	vPolyPts.clear();

	vector<Point> vL, vR;
	vL.clear();
	vR.clear();
	if (drawData.vPtFind_Left.size()) //멤버 변수에 각각 점들이 있는 경우
	{
		vector<Point> v;
		v.clear();

		v.push_back(Point(0, drawData.vPtFind_Left[0].x));
		for (size_t i = 0; i < drawData.vPtFind_Left.size(); i++)
		{
			v.push_back(Point(drawData.vPtFind_Left[i].y, drawData.vPtFind_Left[i].x));
		}
		v.push_back(Point(img_drawline.rows - 1, drawData.vPtFind_Left[drawData.vPtFind_Left.size() - 1].x));
		int order = 2;

		cv::Mat K = PolynomialFit_XY(v, order);
		if (v.size() > 0) {
			for (int j = v.at(0).x; j < v.at(v.size() - 1).x; j++) {

				cv::Point2d point(j, 0);
				for (int k = 0; k < order + 1; k++) {
					point.y += K.at<double>(k, 0) * std::pow(j, k);
				}

				cv::Point2d point_yx(point.y, point.x);
				vL.push_back(point_yx);
				cv::circle(img_drawline, point_yx, 1, cv::Scalar(0, 255, 0), CV_FILLED, CV_AA);
			}
		}
	}
	if (drawData.vPtFind_Right.size())
	{
		vector<Point> v;
		v.clear();

		v.push_back(Point(0, drawData.vPtFind_Right[0].x));
		for (size_t i = 0; i < drawData.vPtFind_Right.size(); i++)
		{
			v.push_back(Point(drawData.vPtFind_Right[i].y, drawData.vPtFind_Right[i].x));
		}
		v.push_back(Point(img_drawline.rows - 1, drawData.vPtFind_Right[drawData.vPtFind_Right.size() - 1].x));
		int order = 2;

		cv::Mat K = PolynomialFit_XY(v, order);
		if (v.size() > 0) {
			for (int j = v.at(0).x; j < v.at(v.size() - 1).x; j++) {

				cv::Point2d point(j, 0);
				for (int k = 0; k < order + 1; k++) {
					point.y += K.at<double>(k, 0) * std::pow(j, k);
				}

				cv::Point2d point_yx(point.y, point.x);
				vR.push_back(point_yx);
				cv::circle(img_drawline, point_yx, 1, cv::Scalar(0, 255, 255), CV_FILLED, CV_AA);
			}
		}
	}

	if (vL.size())
	{
		size_t st = vL.size() - 1;
		for (int i = st; i >= 0; --i)
		{
			vPolyPts.push_back(vL[i]);
		}
	}
	if (vR.size())
	{
		for (size_t i = 0; i < vR.size(); i++)
		{
			vPolyPts.push_back(vR[i]);
		}
	}

	{
		if (vPolyPts.size() > 0)
			fillPoly(img_drawline, vPolyPts, Scalar(0, 0, 255), LINE_AA);
	}
	return img_drawline;
}

Mat LaneDetector::unwarpImg(const Mat& img_drawline, const Mat& img_frame)
{
	//Mat img_unwarp;
	//warpPerspective(img_drawline, img_unwarp, trans, Size(img_frame.cols, img_frame.rows), INTER_LINEAR);

	Mat img_unwarp;
	Mat trans_inv = trans.inv();
	warpPerspective(img_drawline, img_unwarp, trans_inv, img_drawline.size(), INTER_LINEAR);

	return img_unwarp;
}

//다항식으로 주어진 점들을 가장 잘 피팅하는 계수를 찾기 위한 함수
cv::Mat LaneDetector::PolynomialFit_XY(std::vector<cv::Point>& pts, int order = 3)
{
	cv::Mat U(pts.size(), (order + 1), CV_64F);
	cv::Mat Y(pts.size(), (1), CV_64F);

	for (size_t row = 0; row < U.rows; row++)
	{
		for (size_t col = 0; col < U.cols; col++)
		{
			U.at<double>(row, col) = pow(pts[row].x, col);
		}
	}
	for (size_t i = 0; i < Y.rows; i++)
	{
		Y.at<double>(i, 0) = pts[i].y;
	}

	cv::Mat K((order + 1), 1, CV_64F);

	if (U.data != nullptr)
	{
		K = (U.t() * U).inv() * U.t() * Y;
	}




	return K;
}


cv::Mat LaneDetector::PolynomialFit_YX(std::vector<cv::Point>& pts, int order = 3)
{
	cv::Mat U(pts.size(), (order + 1), CV_64F);
	cv::Mat Y(pts.size(), (1), CV_64F);

	for (size_t row = 0; row < U.rows; row++)
	{
		for (size_t col = 0; col < U.cols; col++)
		{
			U.at<double>(row, col) = pow(pts[row].y, col);
		}
	}
	for (size_t i = 0; i < Y.rows; i++)
	{
		Y.at<double>(i, 0) = pts[i].x;
	}

	cv::Mat K((order + 1), 1, CV_64F);

	if (U.data != nullptr)
	{
		K = (U.t() * U).inv() * U.t() * Y;
	}




	return K;
}

Mat LaneDetector::getTrans()
{
	return trans;
}

/*
Mat LaneDetector::drawLine(Mat img_input, vector<Point> lane, string dir) {
	
//		좌우 차선을 경계로 하는 내부 다각형을 투명하게 색을 채운다.
//		예측 진행 방향 텍스트를 영상에 출력한다.
//		좌우 차선을 영상에 선으로 그린다.


	vector<Point> poly_points;
	Mat output;
	img_input.copyTo(output);


	poly_points.push_back(lane[2]);
	poly_points.push_back(lane[0]);
	poly_points.push_back(lane[1]);
	poly_points.push_back(lane[3]);
	fillConvexPoly(output, poly_points, Scalar(0, 230, 30), LINE_AA, 0);  //다각형 색 채우기
	addWeighted(output, 0.3, img_input, 0.7, 0, img_input);  //영상 합하기

	//예측 진행 방향 텍스트를 영상에 출력
	putText(img_input, dir, Point(520, 100), FONT_HERSHEY_PLAIN, 3, Scalar(255, 255, 255), 3, LINE_AA);

	//좌우 차선 선 그리기
	line(img_input, lane[0], lane[1], Scalar(0, 255, 255), 5, LINE_AA);
	line(img_input, lane[2], lane[3], Scalar(0, 255, 255), 5, LINE_AA);



	//predict horizontal direction
	{
		cv::Point ptCross(0, 0);
		IntersectPoint(lane[0], lane[1], lane[2], lane[3], &ptCross);
		//cv::circle(img_input, ptCross, 20, Scalar(0, 0, 255), 1, LINE_AA);
		drawMarker(img_input, ptCross, Scalar(0, 0, 255), MARKER_TRIANGLE_DOWN);
		//
		//y = a*x+b
		Point p1, p2;
		Vec4d axis_line;
		vector<Point> axis_points;
		axis_points.push_back(Point(img_input.cols / 2 - 1, img_input.rows - 1));
		axis_points.push_back(ptCross);
		fitLine(axis_points, axis_line, DIST_L2, 0, 0.01, 0.01);
		double m = axis_line[1] / axis_line[0];  //기울기
		Point b = cv::Point(axis_line[2], axis_line[3]);

		int y1 = img_input.rows - 1;
		int y2 = 50;// 470;
		double x1 = ((y1 - b.y) / m) + b.x;
		double x2 = ((y2 - b.y) / m) + b.x;
		cv::arrowedLine(img_input, Point(x1, y1), Point(x2, y2), Scalar(255, 0, 255), 2);


		cv::arrowedLine(img_input, Point(img_input.cols / 2 - 1, ptCross.y), ptCross, Scalar(0, 255, 0), 5);

		y1 = 0;
		y2 = img_input.rows - 1;
		x1 = ((y1 - b.y) / m) + b.x;
		x2 = ((y2 - b.y) / m) + b.x;
		DrawDashedLine(img_input, Point(x1, y1), Point(x2, y2), Scalar(0, 255, 255), 2, "dotted", 10);


	}


	//reference center cross line
	DrawDashedLine(img_input, Point(img_input.cols / 2 - 1, 0), Point(img_input.cols / 2 - 1, img_input.rows - 1), Scalar(0, 0, 255), 1, "", 5);
	DrawDashedLine(img_input, Point(0, img_input.rows / 2 - 1), Point(img_input.cols - 1, img_input.rows / 2 - 1), Scalar(0, 0, 255), 1, "", 5);

	return img_input;
}


void LaneDetector::DrawDashedLine(cv::Mat& img, cv::Point pt1, cv::Point pt2,
	cv::Scalar color, int thickness, std::string style,
	int gap) {
	float dx = pt1.x - pt2.x;
	float dy = pt1.y - pt2.y;
	float dist = std::hypot(dx, dy);

	std::vector<cv::Point> pts;
	for (int i = 0; i < dist; i += gap) {
		float r = static_cast<float>(i / dist);
		int x = static_cast<int>((pt1.x * (1.0 - r) + pt2.x * r) + .5);
		int y = static_cast<int>((pt1.y * (1.0 - r) + pt2.y * r) + .5);
		pts.emplace_back(x, y);
	}

	int pts_size = pts.size();

	if (style == "dotted") {
		for (int i = 0; i < pts_size; ++i) {
			cv::circle(img, pts[i], thickness, color, FILLED);
		}
	}
	else {
		cv::Point s = pts[0];
		cv::Point e = pts[0];

		for (int i = 0; i < pts_size; ++i) {
			s = e;
			e = pts[i];
			if (i % 2 == 1) {
				cv::line(img, s, e, color, thickness);
			}
		}
	}
}

bool LaneDetector::IntersectPoint(const Point& pt11, const Point& pt12,
	const Point& pt21, const Point& pt22, Point* ptCross)
{
	double t;
	double s;
	double under = (pt22.y - pt21.y) * (pt12.x - pt11.x) - (pt22.x - pt21.x) * (pt12.y - pt11.y);
	if (under == 0) return false;

	double _t = (pt22.x - pt21.x) * (pt11.y - pt21.y) - (pt22.y - pt21.y) * (pt11.x - pt21.x);
	double _s = (pt12.x - pt11.x) * (pt11.y - pt21.y) - (pt12.y - pt11.y) * (pt11.x - pt21.x);

	t = _t / under;
	s = _s / under;

	if (t < 0.0 || t>1.0 || s < 0.0 || s>1.0) return false;
	if (_t == 0 && _s == 0) return false;

	ptCross->x = pt11.x + t * (double)(pt12.x - pt11.x);
	ptCross->y = pt11.y + t * (double)(pt12.y - pt11.y);

	return true;
}

float LaneDetector::GetOffsetDist()
{
	Mat trans_inv = trans.inv();
	double  x1 = 10, x2 = 90;

	//topview
	float  x = 650;//calc
	float  y = 180;//fixed
	double  M11 = trans.at<double>(0, 0);
	double  M12 = trans.at<double>(0, 1);
	double  M13 = trans.at<double>(0, 2);
	double  M21 = trans.at<double>(1, 0);
	double  M22 = trans.at<double>(1, 1);
	double  M23 = trans.at<double>(1, 2);
	double  M31 = trans.at<double>(2, 0);
	double  M32 = trans.at<double>(2, 1);
	double  M33 = trans.at<double>(2, 2);

	//frontview
	float newX = M11 * x + M12 * y + M13 / M31 * x + M32 * y + M33;
	float newY = M21 * x + M22 * y + M23 / M31 * x + M32 * y + M33;

	float oc_x = img_Size.width / 2 - 1;
	float diff_x = newX - oc_x;
	if (diff_x < 10)
		diff_x = 0;
	return diff_x;
}
*/