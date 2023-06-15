#pragma once

#ifndef _OPENCV_INIT_
#define _OPENCV_INIT_

#define OPENCV_470
#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>

#ifdef OPENCV_470
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/imgproc/types_c.h>
#endif // OPENCV_470

using namespace cv;

#ifdef _DEBUG
#pragma comment(lib,"opencv_world470d.lib")
#else
#pragma comment(lib,"opencv_world470.lib")
#endif


#endif // _OPENCV_INIT_


#include <vector>
#include <string>
#include <memory>
#include <map>
#include <algorithm>