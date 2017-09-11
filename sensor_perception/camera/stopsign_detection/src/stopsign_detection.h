#include "radl__stopsign_detection.h"

class StopsignDetection {
public:
	StopsignDetection();
	void step(const radl_in_t*, const radl_in_flags_t*, radl_out_t*, radl_out_flags_t*);
private:
	string cascadeName;
	int QueSize = 10;
	int QuePicker = QueSize >> 1;
	deque<int> widthQue(QueSize, 0);
	deque<int> xQue(QueSize, 0);
	deque<int> yQue(QueSize, 0);
	CascadeClassifier_GPU cascade_gpu;
	VideoCapture capture(0);
	
	int gpuCnt = getCudaEnabledDeviceCount();
	
	Mat frame;
	long frmCnt = 0;
	double totalT = 0.0;
};