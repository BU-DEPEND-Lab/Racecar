#include "stopsign_detection.h"

#include <opencv2/core/cuda.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/cudaobjdetect.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <vector>

using namespace std;
using namespace cv;
using namespace cv::cuda;

const int minSize = 40;

int disEstimate(int width)
{
	if ( width == minSize )
	{
		return 520;
	}
	else if ( width < 350 )
	{
		return 21189/width - 20;
	}
	else
	{
		return 50;
	}
}

StopsignDetection::StopsignDetection() 
{
	QuePicker = QUESIZE >> 1;
	cascadeName = *RADL_THIS->cascade_filename; 
	gpuCnt = getCudaEnabledDeviceCount();
	widthQue = deque<int> (QUESIZE, 0);
	xQue = deque<int> (QUESIZE, 0);
	yQue = deque<int> (QUESIZE, 0);
	capture = VideoCapture(1);
	if(!capture.isOpened()) {std::cout << "camera open failed" << std::endl;}
	cascade_gpu = cuda::CascadeClassifier::create(cascadeName);
	if(gpuCnt==0) {std::cout << "gpu error" << endl;}
	cout << "stopsign_detection initialized" << endl;
	frmCnt = 0;
	totalT = 0.0;
}

void StopsignDetection::step(const radl_in_t * in, const radl_in_flags_t* inflags, radl_out_t * out, radl_out_flags_t* outflags)
{
	capture >> frame;   // grab current frame from the camera
	double t = (double)getTickCount();
 
	Mat    y;
  GpuMat tmp, x, x1;

  tmp.upload(frame);
  cv::cuda::cvtColor(tmp, x1, cv::COLOR_RGB2BGR);
  cv::cuda::cvtColor(x1, x, cv::COLOR_BGR2GRAY);
  x.download(y);

  GpuMat gray_gpu(y);
	GpuMat objbuf;
	cascade_gpu->detectMultiScale(gray_gpu, objbuf);
	std::vector<Rect> faces;
	cascade_gpu->convert(objbuf, faces);
	int detect_num = faces.size();

	Mat obj_host;
	objbuf.colRange(0, detect_num).download(obj_host);

	Rect *cfaces = obj_host.ptr<Rect>();
	t=((double)getTickCount()-t)/getTickFrequency();  // check how long did it take to detect face
	totalT += t;
	frmCnt++;
 
	if (detect_num)
	{
		Point pt = cfaces[0].tl();
		Size sz = cfaces[0].size();
			
		// width queue update
		widthQue.push_back ((int)sz.width);
		widthQue.pop_front();
		// x queue update
		xQue.push_back ((int)pt.x);
		xQue.pop_front();
		// y queue update
		yQue.push_back ((int)pt.y);
		yQue.pop_front();
	}
	else
	{
		widthQue.push_back (0);
		widthQue.pop_front();
		xQue.push_back (0);
		xQue.pop_front();
		yQue.push_back (0);
		yQue.pop_front();
	}

	// get width median value
	std::vector<int> sortedWidth( widthQue.begin(), widthQue.end() );
	sort( sortedWidth.begin(), sortedWidth.end() );
	std::vector<int>::iterator itW = sortedWidth.begin();	
	int midWidth = *(itW + QuePicker);
		
	// get width median value
	std::vector<int> sortedX( xQue.begin(), xQue.end() );
	sort( sortedX.begin(), sortedX.end() );
	std::vector<int>::iterator itX = sortedX.begin();	
	int midX = *(itX + QuePicker);
		
	// get width median value
	std::vector<int> sortedY( yQue.begin(), yQue.end() );
	sort( sortedY.begin(), sortedY.end() );
	std::vector<int>::iterator itY = sortedY.begin();	
	int midY = *(itY + QuePicker);
		
	// std_msgs::Int32 distanceMsg;
	int stopDis;
	if ( midWidth ) stopDis = disEstimate(midWidth);
	else stopDis = -1;
	out->stop_sign_distance->data = stopDis;
	
	if ( stopDis != -1 )
	{
		cout << "Distance is : " << stopDis << endl;
	}
		
	// display
	if ( *RADL_THIS->display_camera )
	{
		for (int i = 0; i < faces.size(); i++) 
			rectangle(frame, faces[i], Scalar(255));
	    
		imshow("stopSigns", frame);
		if(waitKey(10)==27) 
			return;
	}

	cout << "fps: " << 1.0/(totalT/(double)frmCnt) << endl;
}
