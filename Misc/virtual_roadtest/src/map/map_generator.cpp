#include <ctype.h>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <fstream>
#include <iostream>
#include <string>
#include <stdlib.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <math.h>
#include <cmath>
#include <utility>
#include <algorithm>
#include <time.h>
#include <string>
#include <vector>


using namespace std;
using namespace cv;

/*Global Variables*/
Mat Map = Mat::zeros(1048,1048, CV_8UC1);

void readInputs(){}
vector<Point> joint_coord;

float dist(Point p1, Point p2){
	float dist = sqrt(abs((p2.y - p1.y)*(p2.y - p1.y) + (p2.x - p1.x)*(p2.x - p1.x)));
	return dist;
}

bool validCheck(Point p, float length){
	for (int i = 0; i < joint_coord.size() ; i++){
		if(dist(p, joint_coord[i]) >= length*0.5)
			continue;
		else
			return false;
	}
	return true;
}

/*
draw line
return next point
*/
Point process_line(float length, float width, float theta, Point crcl_center, float radius, float prev_theta){
	int lineType = 8;
	Point next;

	//rectangle starting at center of circle
	Point temp_next = Point(crcl_center.x , crcl_center.y);

	//shift along y axis by radius (no theta)
	temp_next.y += (radius+length);

	//change translation w.r.t origin
	int x = temp_next.x - crcl_center.x ;
	int y = temp_next.y - crcl_center.y ;

	//multiply by rotation matrix
	int newx = cos(theta)*x - sin(theta)*y;
	int newy = sin(theta)*x + cos(theta)*y;

	//add translation w.r.t circle center again --> assign to next
	next.x = newx + crcl_center.x;
	next.y = newy + crcl_center.y;

	int color = 255;

	if(validCheck(next, length) == false){
		//change translation w.r.t origin
		x = temp_next.x - crcl_center.x ;
		y = temp_next.y - crcl_center.y ;

		//multiply by rotation matrix
		newx = cos(prev_theta)*x - sin(prev_theta)*y;
		newy = sin(prev_theta)*x + cos(prev_theta)*y;

		//add translation w.r.t circle center again --> assign to next
		next.x = newx + crcl_center.x;
		next.y = newy + crcl_center.y;
	}

	line(Map, crcl_center, next, color, int(width), lineType, 0); //shift = 0 

	return next;
} // end of line()


/*
Draw circle
*/
void circle(Point center, float radius){
 int thickness = -1; // -1 fills the circle in
 int lineType = 8; // line thickness
 int color = 255;

 circle( Map, center, radius, color,thickness, lineType);
}



/*
avoid clashes
return end point
*/
void createPNG(){

}



void createOutputFile(){
	/*
	myfile.open(outpath.c_str());
	for (int i = 0; i<speeds.size(); i++) {
        Path.push_back({((LF1[i][0] - 1024.0)/scale) ,((LF1[i][1] - 1024.0)/scale)});
       	//cout << speeds[i] << " " << Path[i][0] << " " << Path[i][1] << endl;
       	myfile << Path[i][0] << "\n" << Path[i][1] << "\n" << speeds[i] << "\n";
    }
    */
}




int main(int argc, char** argv) {

	float line_l = atof(argv[1]);
	float line_w = atof(argv[2]);
	float radius = atof(argv[3]);
	int joints_count = atoi(argv[4]); // number of circular joints

	//array with theta values
	float thetas[10] = {2, 2, -2, -2, 4, 2, -2, 3, 1.5, 1.5};



	//initial starting point at spawn time
	Point start = Point(500,500);
	joint_coord.push_back(start);

	Point next = process_line(line_l, line_w, 0, start, radius, 0);
	joint_coord.push_back(next);

	// plot each joint(circle), separated by a line segment
	float prev_theta = 0;;
	for (int i=0; i < joints_count ; i++){
		//get turning angle (theta) for current joint
		float theta = thetas[i];

		//plot joint
		circle(next, radius); 
		joint_coord.push_back(next);


		//plot line segment and update next point
		next = process_line(line_l, line_w, theta, next, radius, prev_theta);

		prev_theta = theta;
	}

	imshow("Map", Map);
 	imwrite( "map_demo.jpg", Map );
 	waitKey(0);

} // end of main


