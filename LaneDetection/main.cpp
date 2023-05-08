
#include <iostream>
#include "LaneDetector.h"

int main()
{
	LaneDetector laneDetector;
	Mat img_frame, img_bilater, img_filter, img_edges, img_mask, img_top;
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
	while (1)
	{
		// 1. ���� ������ �о�´�. + bilateralFilter ���
		if (!video.read(img_frame))
		{
			break;
		}
		img_top = laneDetector.makeTopView(img_frame);
		bilateralFilter(img_top, img_bilater, 10, 50, 50);
		// 2. ���, ����� ���� ���� �ִ� �͸� ���͸��Ͽ� ���� �ĺ��� �����Ѵ�.
		img_filter = laneDetector.filterColors(img_bilater);

		// 3. ������ GrayScale ���� ��ȯ�Ѵ�.
		cvtColor(img_filter, img_filter, COLOR_BGR2GRAY);
#ifdef IMSHOW_FILTER
		imshow("img_filter", img_filter);
#endif // IMSHOW_FILTER

		// 4. Canny Edge Detection���� ������ ����.
		// (���� ���Ÿ� ���� Gaussian ���͸��� ����)
		GaussianBlur(img_filter, img_filter, Size(9, 9), 0, 0);
		Canny(img_filter, img_edges, 50, 150);
#ifdef IMSHOW_EDGE
		imshow("img_edge", img_edges);
#endif // IMSHOW_EDGE

		// 5. ������� �ٴڿ� �����ϴ� �������� �����ϱ� ���� ���� ������ ����
		img_mask = laneDetector.limitRegion(img_edges);
#ifdef IMSHOW_TOP
		imshow("img_top", img_top);
#endif // IMSHOW_TOP
#ifdef IMSHOW_FRAME
		// ��� ���� ���
		imshow("img_frame", img_frame);
#endif // IMSHOW_FRAME
		//esc Ű ����
		if (waitKey(1) == 27)
		{
			break;
		}
	}
	return 0;
}