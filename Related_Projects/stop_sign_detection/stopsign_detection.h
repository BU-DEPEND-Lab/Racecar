#include "radl__stopsign_detection.h"

// cpp includes
#include <iostream>
#include <vector>
#include <deque>
#include <sstream>

// opencv includes
#include <opencv2/core/core.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/gpu/gpu.hpp>

const int QUESIZE = 10;

using namespace std;
using namespace cv;
using namespace cv::gpu;

class StopsignDetection {
public:
	StopsignDetection();
	void step(const radl_in_t*, const radl_in_flags_t*, radl_out_t*, radl_out_flags_t*);
private:
	string cascadeName;
	int QuePicker;
	deque<int> widthQue;
	deque<int> xQue;
	deque<int> yQue;
	CascadeClassifier_GPU cascade_gpu;
	VideoCapture capture;
	
	int gpuCnt;
	
	Mat frame;
	long frmCnt;
	double totalT;
};