#include "radl__stopsign_detection.h"

// cpp includes
#include <iostream>
#include <vector>
#include <deque>
#include <sstream>
#include <opencv/cv.h>
#include <opencv2/core/cuda.hpp>

using namespace std;
using namespace cv;
using namespace cv::cuda;

// opencv includes
#include <opencv2/core/core.hpp>
#include <opencv2/cudaobjdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/opencv.hpp>

const int QUESIZE = 10;

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
	  cv::Ptr<cv::cuda::CascadeClassifier> cascade_gpu;
	  VideoCapture capture;
	
	  int gpuCnt;
	
	  Mat frame;
	  long frmCnt;
	  double totalT;
};
