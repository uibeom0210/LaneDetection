
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
	Mat img_frame, img_bilater, img_filter, img_edges, img_mask, img_top, img_roitop;
	// 영상 불러오기
	VideoCapture video("input.mp4");  
	//VideoCapture video("input01.mp4");
	if (!video.isOpened())
	{
		cout << "동영상 파일을 열 수 없습니다. \n" << endl;
		getchar();
		return -1;
	}
	video.read(img_frame);
	if (img_frame.empty()) 
	{
		return -1;
	}
	int codec = VideoWriter::fourcc('X', 'V', 'I', 'D');  // 원하는 코덱 선택
	//double fps = 29.97;  // 프레임
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
		// 1. 원본 영상을 읽어온다. + bilateralFilter 사용
		if (!video.read(img_frame))
		{
			break;
		}
		img_top = laneDetector.makeTopView(img_frame);
		bilateralFilter(img_top, img_bilater, 10, 50, 50);
		// 2. 흰색, 노란색 범위 내에 있는 것만 필터링하여 차선 후보로 저장한다.




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

		// 3. ROI로 사각형 영역 설정
		Mat img_roitop = img_filter.clone();
		Mat img_draw_rois = laneDetector.makeROI(img_roitop);
#ifdef IMSHOW_ROI
		imshow("img_roitop", img_draw_rois);
#endif // IMSHOW_ROI

		// 4. 영상을 GrayScale 으로 변환한다.
		cvtColor(img_filter, img_filter, COLOR_BGR2GRAY);
#ifdef IMSHOW_FILTER
		imshow("img_filter", img_filter);
#endif // IMSHOW_FILTER

		// 5. Canny Edge Detection으로 에지를 추출.
		// (잡음 제거를 위한 Gaussian 필터링도 포함)
		GaussianBlur(img_filter, img_filter, Size(9, 9), 0, 0);
		Canny(img_filter, img_edges, 50, 150);
#ifdef IMSHOW_EDGE
		imshow("img_edge", img_edges);
#endif // IMSHOW_EDGE

		// 6. 진행방향 바닥에 존재하는 차선만을 검출하기 위한 관심 영역을 지정
		img_mask = laneDetector.limitRegion(img_edges);
#ifdef IMSHOW_TOP
		imshow("img_top", img_top);
#endif // IMSHOW_TOP
#ifdef IMSHOW_FRAME
		// 결과 영상 출력
		imshow("img_frame", img_frame);
#endif // IMSHOW_FRAME
		//esc 키 종료
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
