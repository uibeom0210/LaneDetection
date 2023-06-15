
#include <iostream>
#include "LaneDetector.h"

#ifdef HSV_TRACK_BAR
const int max_value_H = 360 / 2;
const int max_value = 255;
const String window_capture_name = "Video Capture";
const String window_detection_name = "Object Trackbar";
int low_H = 10, low_S = 100, low_V = 100;
int high_H = 40, high_S = 255, high_V = 255;
static void on_high_H_thresh_trackbar(int, void*);
static void on_low_H_thresh_trackbar(int, void*);
static void on_low_S_thresh_trackbar(int, void*);
static void on_high_S_thresh_trackbar(int, void*);
static void on_low_V_thresh_trackbar(int, void*);
static void on_high_V_thresh_trackbar(int, void*);
#endif // HSV_TRACK_BAR

int main()
{
	LaneDetector laneDetector;
	LaneDetector::drawDataInfo drawData;
	Mat img_frame, img_bilater, img_filter, img_edges, img_mask, img_top, img_histo, trans;
	// ���� �ҷ�����
	VideoCapture video("input.mp4");
	//VideoCapture video("input01.mp4");
	if (!video.isOpened())
	{
		cout << "������ ������ �� �� �����ϴ�. \n" << endl;
		getchar();
		return -1;
	}
	video.read(img_frame);
	if (img_frame.empty())
	{
		return -1;
	}
	int codec = VideoWriter::fourcc('X', 'V', 'I', 'D');  // ���ϴ� �ڵ� ����
	//double fps = 29.97;  // ������
	double fps = 25;

#ifdef HSV_TRACK_BAR
	namedWindow(window_capture_name);
	namedWindow(window_detection_name);
	// Trackbars to set thresholds for HSV values
	createTrackbar("Low H", window_detection_name, &low_H, max_value_H, on_low_H_thresh_trackbar);
	createTrackbar("High H", window_detection_name, &high_H, max_value_H, on_high_H_thresh_trackbar);
	createTrackbar("Low S", window_detection_name, &low_S, max_value, on_low_S_thresh_trackbar);
	createTrackbar("High S", window_detection_name, &high_S, max_value, on_high_S_thresh_trackbar);
	createTrackbar("Low V", window_detection_name, &low_V, max_value, on_low_V_thresh_trackbar);
	createTrackbar("High V", window_detection_name, &high_V, max_value, on_high_V_thresh_trackbar);
	Mat frame_HSV, frame_threshold;
#endif // HSV_TRACK_BAR

	while (1)
	{
		// 1. ���� ������ �о�´�. + bilateralFilter ���
		if (!video.read(img_frame))
		{
			break;
		}
		img_top = laneDetector.makeTopView(img_frame);
#ifdef IMSHOW_TOP
		imshow("img_top", img_top);
#endif // IMSHOW_TOP

		bilateralFilter(img_top, img_bilater, 10, 50, 50);
		// 2. ���, ����� ���� ���� �ִ� �͸� ���͸��Ͽ� ���� �ĺ��� �����Ѵ�.

#ifdef HSV_TRACK_BAR
		// Convert from BGR to HSV colorspace
		cvtColor(img_bilater, frame_HSV, COLOR_BGR2HSV);
		// Detect the object based on HSV Range Values
		inRange(frame_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), frame_threshold);
		// Show the frames
		imshow(window_capture_name, img_frame);
		imshow("object detect", frame_threshold);
		img_filter = frame_HSV;
#else
		img_filter = laneDetector.filterColors(img_bilater);
#endif // HSV_TRACK_BAR

#ifdef IMSHOW_FILTER
		imshow("img_filter", img_filter);
#endif // IMSHOW_FILTER

		// 3. ROI�� �簢�� ���� ����(histogram�� �̿�)
		Mat img_roitop = img_filter.clone();
		Mat img_draw_rois = laneDetector.makeROI(img_roitop, drawData);
#ifdef IMSHOW_ROI
		imshow("img_roitop", img_draw_rois);
#endif // IMSHOW_ROI

		// 4. ���� �׸���
		Mat img_drawline = laneDetector.drawLine(img_draw_rois, drawData);

		Mat draw_frame;
		addWeighted(img_bilater, 1.0, img_drawline, 0.3, 1.0, draw_frame);

//		float dist = laneDetector.GetOffsetDist();
//		cv::line(draw_frame, Point(draw_frame.cols/2-1, draw_frame.rows/2-1), 
//			Point(draw_frame.cols / 2 - 1 + dist, draw_frame.rows / 2 - 1), Scalar(0, 255, 255), 5, LINE_AA);
//		cv::arrowedLine(draw_frame, Point(draw_frame.cols / 2 - 1, draw_frame.rows / 2 - 1),
//			Point(draw_frame.cols / 2 - 1 + dist, draw_frame.rows / 2 - 1), Scalar(255, 0, 255), 5);

#ifdef IMSHOW_LINE
		imshow("img_drawline", draw_frame);
#endif // IMSHOW_LINE

		Mat img_unwarp = laneDetector.unwarpImg(img_drawline, img_frame);

#ifdef IMSHOW_UNWARP
		imshow("img_unwarp", img_unwarp);
#endif // IMSHOW_UNWARP

		addWeighted(img_frame, 1.0, img_unwarp, 0.3, 1.0, draw_frame);

		// 5. Canny Edge Detection���� ������ ����.
		// (���� ���Ÿ� ���� Gaussian ���͸��� ����)
		GaussianBlur(img_filter, img_filter, Size(9, 9), 0, 0);

#ifdef IMSHOW_EDGE
		Canny(img_filter, img_edges, 50, 150);
		imshow("img_edge", img_edges);
#endif // IMSHOW_EDGE

		// 6. ������� �ٴڿ� �����ϴ� �������� �����ϱ� ���� ���� ������ ����
		img_mask = laneDetector.limitRegion(img_edges);

#ifdef IMSHOW_FRAME
		// ��� ���� ���
		//imshow("img_frame", img_frame);
		imshow("img_frame", draw_frame);
#endif // IMSHOW_FRAME

		//esc Ű ����
		if (waitKey(1) == 27)
		{
			break;
		}
	}
	return 0;
}
#ifdef HSV_TRACK_BAR
static void on_low_H_thresh_trackbar(int, void*)
{
	low_H = min(high_H - 1, low_H);
	setTrackbarPos("Low H", window_detection_name, low_H);
}
static void on_high_H_thresh_trackbar(int, void*)
{
	high_H = max(high_H, low_H + 1);
	setTrackbarPos("High H", window_detection_name, high_H);
}
static void on_low_S_thresh_trackbar(int, void*)
{
	low_S = min(high_S - 1, low_S);
	setTrackbarPos("Low S", window_detection_name, low_S);
}
static void on_high_S_thresh_trackbar(int, void*)
{
	high_S = max(high_S, low_S + 1);
	setTrackbarPos("High S", window_detection_name, high_S);
}
static void on_low_V_thresh_trackbar(int, void*)
{
	low_V = min(high_V - 1, low_V);
	setTrackbarPos("Low V", window_detection_name, low_V);
}
static void on_high_V_thresh_trackbar(int, void*)
{
	high_V = max(high_V, low_V + 1);
	setTrackbarPos("High V", window_detection_name, high_V);
}
#endif // HSV_TRACK_BAR
