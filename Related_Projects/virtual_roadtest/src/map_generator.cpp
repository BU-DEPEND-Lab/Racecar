/*
Author: Muhammad Zuhayr Raghib  12/09/2017

-----Map generator for Genetic Algorithm-------

Arguments are line_l, line_w, joint_radius, num_joints, 15 joint angles
Currently, include all 15 joint angles, even if num_joint < 15

#e.g: 
./map_generator  40 20 10 14 2 2 -2 2 2.5 -2.1 1 1.5 1.0 1.3 2.3 2.2 2.1 -1.5 -1.5

Inputs : 
1) argiments
2) text files for generating results.world file. File locations could vary

Outputs :
 1) results.world file to be executed in Gazebo
 2) goal.txt for the global path planner

*/



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
#include <vector>
#include <stdio.h>


using namespace std;
using namespace cv;

/*Global Variables*/
Mat Map = Mat::zeros(2048,2048, CV_8UC1);
int boxn = 0; //number of boxes in world file
Point goal;

void readInputs(){}
vector<Point> joint_coord;
vector<Point> boxes; //coordinates of boxes for .world file

//find boundary
vector<vector<Point>> contours;
Mat hierarchy;


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



/*Populate vector of box coordinates*/
void add_box_coord(Point start, Point next, float w, float theta){
	float l = dist(start, next)/2;
	
	// point on the left and right side of line segment if turning angle is 0
	Point left = Point(start.x - w, l + start.y); 
	Point right = Point(start.x + w, l + start.y);

	//change translation w.r.t origin
	int x1 = left.x - start.x ;
	int y1 = left.y - start.y ;

	int x2 = right.x - start.x ;
	int y2 = right.y - start.y ;

	//multiply by rotation matrix
	int newx1 = cos(theta)*x1 - sin(theta)*y1;
	int newy1 = sin(theta)*x1 + cos(theta)*y1;

	//multiply by rotation matrix
	int newx2 = cos(theta)*x2 - sin(theta)*y2;
	int newy2 = sin(theta)*x2 + cos(theta)*y2;

	//add translation w.r.t circle center again --> assign to next
	left.x = newx1 + start.x;
	left.y = newy1 + start.y;
	boxes.push_back(left);

	right.x = newx2 + start.x;
	right.y = newy2 + start.y;
	boxes.push_back(right);


}
/* draw line and return next point */
Point process_line(float length, float width, float theta, Point crcl_center, float radius, float prev_theta){
	int lineType = 8;
	Point next;

	//rectangle starting at center of circle
	Point temp_next = Point(crcl_center.x , crcl_center.y);

	//shift along y axis by radius (no theta) so rectangle shifts to edge of circle
	temp_next.y += (length);// cc.y + r + l

	//change translation w.r.t origin
	int x = temp_next.x - crcl_center.x ;// 0
	int y = temp_next.y - crcl_center.y ;// r + l

	//multiply by rotation matrix
	int newx = cos(theta)*x - sin(theta)*y;// -sin(theta) * (r + l)
	int newy = sin(theta)*x + cos(theta)*y;// cos(theta) * (r + l)

	//add translation w.r.t circle center again --> assign to next
	next.x = newx + crcl_center.x;// sin(theta) * (r + l) + cc.x
	next.y = newy + crcl_center.y;// cos(theta) * (r + l) + cc.y

	int color = 255;

	if(validCheck(next, length) == false){
		//change translation w.r.t origin
		x = temp_next.x - crcl_center.x ;//0
		y = temp_next.y - crcl_center.y ;//r+l

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
void circle(Point center, float radius, int color){
 int thickness = -1; // -1 fills the circle in
 int lineType = 8; // line thickness

 goal = center;
 circle( Map, center, radius, color,thickness, lineType);
}




string addLines(int unit_number, float side_length ){
	string x= "	<model name='unit_box_" + to_string(unit_number) + "'>\n";
	x +=      "		<pose>0 -1 0.5 0 -0 0</pose>\n";
	x +=      "		<link name='link'>\n";
	x +=      "			<inertial>\n";
	x +=      "			<mass>1</mass>\n";
	x +=      "				<inertia>\n";
	x +=      "					<ixx>1</ixx>\n";
	x +=      "					<ixy>0</ixy>\n";
	x +=      "					<ixz>0</ixz>\n";
	x +=      "					<iyy>1</iyy>\n";
	x +=  	  "					<iyz>0</iyz>\n";
	x +=      "					<izz>2</izz>\n";
	x +=      "				</inertia>\n";
	x +=  	  "			</inertial>\n";
	x +=  	  "			<collision name='collision'>\n";
	x += 	  "				<geometry>\n";
	x += 	  "				<box>\n";
	x += 	  "					<size> "+ to_string(side_length)+" "+to_string(side_length)+" "+to_string(2)+" </size>\n";
	x +=      "				</box>\n";
	x +=      "				</geometry>\n";
	x +=      "				<max_contacts>10</max_contacts>\n";
	x +=   	  "				<surface>\n";
	x +=      "					<bounce/>\n";
	x +=      "					<friction>\n";
	x +=      "						<ode/>\n";
	x +=      "					</friction>\n";
	x +=      "					<contact>\n";
	x +=      "						<ode/>\n";
	x +=      "					</contact>\n";
	x +=      "				</surface>\n";
	x +=      "			</collision>\n";
	x +=      "			<visual name='visual'>\n";
	x +=      "				<geometry>\n";
	x +=      "					<box>\n";
	x +=      "						<size> "+ to_string(side_length)+" "+to_string(side_length)+" "+to_string(2)+" </size>\n";
	x +=      "					</box>\n";
	x +=      "				</geometry>\n";
	x +=      "				<material>\n";
	x +=      "					<script>\n";
	x +=      "						<uri>file://media/materials/scripts/gazebo.material</uri>\n";
	x +=      "						<name>Gazebo/Grey</name>\n";
	x +=      "					</script>\n";
	x +=      "				</material>\n";
	x +=      "			</visual>\n";
	x +=      "			<velocity_decay>\n";
	x +=      "				<linear>0</linear>\n";
	x +=      "				<angular>0</angular>\n";
	x +=      "			</velocity_decay>\n";
	x +=      "			<self_collide>0</self_collide>\n";
	x +=      " 		<kinematic>0</kinematic>\n";
	x +=      "			<gravity>1</gravity>\n";
	x +=      " 	</link>\n";
	x +=      " 	<static>true</static>\n";
	x +=      "	</model>\n";

	return x;
}

void FindBinaryLargeObjects(const Mat &binary, vector <vector<Point2i>> &blobs)
{
	//clear blobs vector
	blobs.clear();

	//labelled image of type CV_32SC1
	Mat label_image;
	binary.convertTo(label_image, CV_32SC1);

	//label count starts at 2
	int label_count = 2;

	//iterate over all pixels until a pixel with a 1-value is encountered
	for (int y = 0; y < label_image.rows; y++) {
		int *row = (int*)label_image.ptr(y);
		for (int x = 0; x < label_image.cols; x++) {
			if (row[x] != 1) {
				continue;
			}
			//cout << x<<endl;
			//floodFill the connected component with the label count
			//floodFill documentation: http://docs.opencv.org/modules/imgproc/doc/miscellaneous_transformations.html#floodfill
			Rect rect;
			floodFill(label_image, Point(x, y), label_count, &rect, 0, 0, 4);

			//store all 2D co-ordinates in a vector of 2d points called blob
			vector <Point2i> blob;
			for (int i = rect.y; i < (rect.y + rect.height); i++) {
				int *row2 = (int*)label_image.ptr(i);
				for (int j = rect.x; j < (rect.x + rect.width); j++) {
					if (row2[j] != label_count) {
						continue;
					}
					blob.push_back(Point2i(j, i));
				}
			}
			//store the blob in the vector of blobs
			blobs.push_back(blob);

			//increment counter
			label_count++;
			cout << "number of blobs" << blobs.size() << endl;

			
		}
	}
	//cout << "The number of blobs in the image is: " << label_count;
	//Code derived from: http://nghiaho.com/
}


int main(int argc, char** argv) {

	float line_l = atof(argv[1]);
	float line_w = atof(argv[2]);
	float radius = atof(argv[3]);
	int joints_count = atoi(argv[4]); // number of circular joints
	float j1 = atof(argv[5]); // number of circular joints
	float j2= atof(argv[6]); // number of circular joints
	float j3 = atof(argv[7]); // number of circular joints
	float j4 = atof(argv[8]); // number of circular joints
	float j5 = atof(argv[9]); // number of circular joints
	float j6 = atof(argv[10]); // number of circular joints
	float j7 = atof(argv[11]); // number of circular joints
	float j8 = atof(argv[12]); // number of circular joints
	float j9 = atof(argv[13]); // number of circular joints
	float j10 = atof(argv[14]); // number of circular joints
	float j11 = atof(argv[15]); // number of circular joints
	float j12 = atof(argv[16]); // number of circular joints
	float j13 = atof(argv[17]); // number of circular joints
	float j14 = atof(argv[18]); // number of circular joints
	float j15 = atof(argv[19]); // number of circular joints



	//array with theta values
	//float thetas[15] = {2, 2, -2, -2, 4, 2, -2, 3, 1.5, 1.5};
	float thetas[15] = {j1,j2,j3,j4,j5,j6,j7,j8,j9,j10,j11,j12,j13,j14,j15} ; 


	//initial starting point at spawn time
	Point start = Point(1024,1024);
	circle(start, radius, 255); 

	joint_coord.push_back(start);

	Point next = process_line(line_l, line_w, 0, start, radius, 0);
	joint_coord.push_back(next);

	//box coordinate
	add_box_coord(start, next, line_w/2, 0);

	// plot each joint(circle), separated by a line segment
	float prev_theta = 0;
	for (int i=0; i < joints_count ; i++){
		//get turning angle (theta) for current joint
		float theta = thetas[i];

		//plot joint
		circle(next, radius, 255); 
		joint_coord.push_back(next);

		//for box coordinate
		Point temp = next;

		//plot line segment and update next point
		next = process_line(line_l, line_w, theta, next, radius, prev_theta);

		//box coordinate
		add_box_coord(temp, next, line_w/2, theta);

		prev_theta = theta;
	}

	// write goal coordinates to a .txt file for path planner
	ofstream goal_file; 

	goal_file.open("/home/f1/catkin_ws/src/virtual_racetrack/virtual_roadtest/src/map/ref/goal.txt");

	string g = "start_point: 0, 0\nend_point(Gazebo): 0.0, 0.0\n"; //adding string to be compatable with current path planner
	g += "end_point(2048): ";
	g += to_string(goal.x-1024) ;
	g += ", " ;
	g += to_string(goal.y-1024) ;

	goal_file << g;
	goal_file.close();





	/*start - box points*/
	////////////////////////////////////////////////
	//CALCULATING COORDINATES FOR BOXES FOR GAZEBO WORLD

	/*
	//print Map DOES NOT WORK
	for(int i = 0 ; i < boxes.size() ; i++){
		circle(boxes.at(i), 2, 120); 
	}
	//imshow("Map", Map);
	*/


	waitKey(30);

	//Mat dst = Map.clone();
	Mat dst = Map.clone();


	threshold(Map, dst, 0, 255, THRESH_BINARY_INV);

	waitKey(5);

	// print entra points in case of crossings
	
	//initialize vector to store all blobs (area inside a circular loop of track caused by crossings)
	Mat binary;
	threshold(dst, binary, 0.0, 1.0, THRESH_BINARY);

	vector <vector<Point2i>> blobs;

	//call function that detects blobs and modifies the input binary image to store blob info
	FindBinaryLargeObjects(binary, blobs);
	//display the output
	Mat output = Mat::zeros(Map.size(), CV_8UC1);

	// Randomly color the blobs
	for (size_t i = 0; i < blobs.size(); i++) {

		unsigned char r = 100 * (rand() / (1.0 + RAND_MAX));
		cout << (int)r << endl;

		for (size_t j = 0; j < blobs[i].size(); j++) {
			int x = blobs[i][j].x;
			int y = blobs[i][j].y;
			output.at<uchar>(y,x) = (int)r;

		}
	}


	//print boundry
	findContours(output, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
	// cout << "contours size " << contours.size() << endl;

	// drawContours --> These are all possible gazebo box coordinates

	// There will only be one contour in this case
	for(int i = 0 ; i < contours.size() ; i++){
		for(int j = 0; j < contours.at(i).size() ; j += 7){
			//if(j % 2 == 0)   
				circle(contours[i][j], 2, 120); 
			// else
				// continue;

		}
	}


	waitKey(30);

 	imwrite( "final.png", Map );


	////////////////////////////////////////////////
	/*end - box points*/





	//GENERATING THE WORLD FILE - START
	ifstream top, bottom;
	ofstream world_file; 

	top.open("/home/f1/catkin_ws/src/virtual_racetrack/virtual_roadtest/src/map/ref/topheader.txt");
	bottom.open("/home/f1/catkin_ws/src/virtual_racetrack/virtual_roadtest/src/map/ref/bottomheader.txt");
	world_file.open("/home/f1/catkin_ws/src/racecar_simulator/racecar_gazebo/worlds/result.world");


	//1) top.txt information

	while(top.eof()==0){
		string line;
		getline(top,line);
		world_file<<line;
	}


	//2) add box state info //////////////////////////////////////////////////////


	for(int i = 0; i < boxes .size(); i++) {
			int xpos = (int)boxes.at(i).x;
			int ypos = (int)boxes.at(i).y;
			string line;
			int ii = 1;

			string l1 = "	<model name='unit_box_" + to_string(i)+ "'>\n";
			l1 += "			<pose>-0 -1 0.5 0 -0 0</pose>\n";
			l1 +=  "			<link name='link'>\n";
			l1 +=  "				<pose> "+to_string(((float)(xpos-1024)/9.96985090825054))+" "+to_string(((float)(ypos-1024)/9.96985090825054))+" "+to_string(0)+" 0 -0 0</pose>\n";
			l1 +=  "				<velocity>0 0 0 0 -0 0</velocity>\n";
			l1 +=  "				<acceleration>0 0 0 0 -0 0</acceleration>\n";
			l1 +=  "				<wrench>0 0 0 0 -0 0</wrench>\n";
			l1 +=  "			</link>\n";
			l1 +=  "	</model>\n";

			world_file<< l1;

	}
	//add line to end state information

	string line1 = "    </state>\n";
	world_file << line1;

	//add box model info //////////////////////////////////////////////////////

	for(int i = 0; i < boxes.size(); i++) {
		float side_length = 1; // dimensions of 'box' in gazebo
		if (world_file.is_open()) {
			string line = addLines(i, side_length); // add box info to world file
			world_file << line;
		}
	}

	//3) bottom.txt imformation
	while(bottom.eof()==0){
		string line;
		getline(bottom,line);
		world_file<<line;
	}
	
	//close all files
	top.close(); bottom.close(); world_file.close(); 



	return 0 ;
} // end of main


