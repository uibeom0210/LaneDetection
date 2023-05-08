
#include <iostream>
#include "LaneDetector.h"

int main()
{
	LaneDetector laneDetector;
	Mat img_frame, img_bilater, img_filter, img_edges, img_mask, img_top;
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
		img_filter = laneDetector.filterColors(img_bilater);

		// 3. 영상을 GrayScale 으로 변환한다.
		cvtColor(img_filter, img_filter, COLOR_BGR2GRAY);
#ifdef IMSHOW_FILTER
		imshow("img_filter", img_filter);
#endif // IMSHOW_FILTER

		// 4. Canny Edge Detection으로 에지를 추출.
		// (잡음 제거를 위한 Gaussian 필터링도 포함)
		GaussianBlur(img_filter, img_filter, Size(9, 9), 0, 0);
		Canny(img_filter, img_edges, 50, 150);
#ifdef IMSHOW_EDGE
		imshow("img_edge", img_edges);
#endif // IMSHOW_EDGE

		// 5. 진행방향 바닥에 존재하는 차선만을 검출하기 위한 관심 영역을 지정
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