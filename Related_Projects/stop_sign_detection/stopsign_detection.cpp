#include "stopsign_detection.h"

using namespace std;
using namespace cv;
using namespace cv::gpu;

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

StopsignDetection::StopsignDetection() {
	QuePicker = QUESIZE >> 1;
	cascadeName = "/home/bu/race_ws/Racecar/sensor_perception/camera/stopsign_detection/data/stopsign_detector.xml";
	gpuCnt = getCudaEnabledDeviceCount();
	widthQue = deque<int> (QUESIZE, 0);
	xQue = deque<int> (QUESIZE, 0);
	yQue = deque<int> (QUESIZE, 0);
	capture = VideoCapture(0);
	if(!capture.isOpened()) {std::cout << "camera open failed" << std::endl;}
	if(!cascade_gpu.load(cascadeName))
		{cout << "failed to load cascade file " << endl;}  // failed to load cascade file, quit
	if(gpuCnt==0) {std::cout << "gpu error" << endl;}
	cout << "stopsign_detection initialized" << endl;
	frmCnt = 0;
	totalT = 0.0;
}

void StopsignDetection::step(const radl_in_t * in, const radl_in_flags_t* inflags,
	radl_out_t * out, radl_out_flags_t* outflags){

	capture >> frame;   // grab current frame from the camera
		double t = (double)getTickCount();
 
		GpuMat faces;
		Mat frame_gray;
		cvtColor(frame, frame_gray, CV_BGR2GRAY);  // convert to gray image as face detection do NOT use color info
		GpuMat gray_gpu(frame_gray);  // copy the gray image to GPU memory
		equalizeHist(frame_gray,frame_gray);
 
		int detect_num = cascade_gpu.detectMultiScale(
							gray_gpu, faces,
							1.2, 4, Size(minSize, minSize) );  // call face detection routine
		Mat obj_host;
		faces.colRange(0, detect_num).download(obj_host);  // retrieve results from GPU
 
		Rect* cfaces = obj_host.ptr<Rect>();  // results are now in "obj_host"
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

		// for(int i=0;i<detect_num;++i)
		// {
		//	Point pt1 = cfaces[i].tl();
		//	Size sz = cfaces[i].size();
		//	Point pt2(pt1.x+sz.width, pt1.y+sz.height);
		//	rectangle(frame, pt1, pt2, Scalar(255));
		//	int pp = 0;
		//}  // retrieve all detected faces and draw rectangles for visualization
		
		// get width median value
		vector<int> sortedWidth( widthQue.begin(), widthQue.end() );
		sort( sortedWidth.begin(), sortedWidth.end() );
		vector<int>::iterator itW = sortedWidth.begin();	
		int midWidth = *(itW + QuePicker);
		
		// get width median value
		vector<int> sortedX( xQue.begin(), xQue.end() );
		sort( sortedX.begin(), sortedX.end() );
		vector<int>::iterator itX = sortedX.begin();	
		int midX = *(itX + QuePicker);
		
		// get width median value
		vector<int> sortedY( yQue.begin(), yQue.end() );
		sort( sortedY.begin(), sortedY.end() );
		vector<int>::iterator itY = sortedY.begin();	
		int midY = *(itY + QuePicker);


		// geometry_msgs::Point msg;
		// msg.x = midX;
		// msg.y = midY;
		// msg.z = midWidth;
		// bd_box_pub.publish(msg);

		
		// std_msgs::Int32 distanceMsg;
		int stopDis;
		if ( midWidth ) stopDis = disEstimate(midWidth);
		else stopDis = -1;
		out->stop_sign_distance->data = stopDis;
		
		
		if ( midWidth )
		{
			cout << "Distance is : " << disEstimate(midWidth) << endl;
		}
		
		// display
		if ( 1 )
		{
		    Point pt1;
			pt1.x = midX;
			pt1.y = midY;
			Point pt2(midX+midWidth, midY+midWidth);
			rectangle(frame, pt1, pt2, Scalar(255, 0, 0), 2);
		    
		    
			imshow("stopSigns", frame);
			if(waitKey(10)==27) 
				return;
		}
		cout << "fps: " << 1.0/(totalT/(double)frmCnt) << endl;

}