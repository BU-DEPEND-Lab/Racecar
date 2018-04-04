// cpp includes
#include <iostream>
#include <vector>
#include <deque>

// opencv includes
#include <opencv2/core/core.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/gpu/gpu.hpp>


// ROS includes

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Point.h"
#include <sstream>

using namespace std;
using namespace cv;
using namespace cv::gpu;

#define minSize 40

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




int main(int argc, char **argv)
{
	ros::init(argc, argv, "stopsign_detection");
	ros::NodeHandle n;
	ros::Publisher bd_box_pub = n.advertise<geometry_msgs::Point>("/stop_sign_bd_box", 1000);
	ros::Publisher stop_sign_dis = n.advertise<std_msgs::Int32>("/stop_sign_distance", 1000);

	ros::Rate loop_rate(25);

	string cascadeName = argv[1];

	string display;
	if ( argc > 2 ) 
	{
		display = argv[2];	
	}


	int QueSize = 10;
	int QuePicker = QueSize >> 1;
	deque<int> widthQue(QueSize, 0);
	deque<int> xQue(QueSize, 0);
	deque<int> yQue(QueSize, 0);

	CascadeClassifier_GPU cascade_gpu;
	VideoCapture capture(0);
	if(!capture.isOpened()) return -1;
 
	int gpuCnt = getCudaEnabledDeviceCount();   // gpuCnt >0 if CUDA device detected
	if(gpuCnt==0) return -1;  // no CUDA device found, quit
 
	if(!cascade_gpu.load(cascadeName))
		return -1;  // failed to load cascade file, quit

		 

	Mat frame;
	long frmCnt = 0;
	double totalT = 0.0;

	while(ros::ok())
	{
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


		geometry_msgs::Point msg;
		msg.x = midX;
		msg.y = midY;
		msg.z = midWidth;
		bd_box_pub.publish(msg);

		
		std_msgs::Int32 distanceMsg;
		int stopDis;
		if ( midWidth ) stopDis = disEstimate(midWidth);
		else stopDis = -1;
		distanceMsg.data = stopDis;
		stop_sign_dis.publish(distanceMsg);


		ros::spinOnce();
		loop_rate.sleep();
		
		if ( midWidth )
		{
			cout << "Distance is : " << disEstimate(midWidth) << endl;
		}
		
		
		if ( "1" == display )
		{
		    Point pt1;
			pt1.x = midX;
			pt1.y = midY;
			Point pt2(midX+midWidth, midY+midWidth);
			rectangle(frame, pt1, pt2, Scalar(255, 0, 0), 2);
		    
		    
			imshow("stopSigns", frame);
			if(waitKey(10)==27) 
				break;
		}
		cout << "fps: " << 1.0/(totalT/(double)frmCnt) << endl;
	}
 
   
}
