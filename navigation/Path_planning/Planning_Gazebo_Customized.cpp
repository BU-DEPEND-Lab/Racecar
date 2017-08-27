#include <ctype.h>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <fstream>
#include <iostream>
#include <string>
#include <stdlib.h>
#include <opencv2/highgui/highgui.hpp>
#define HEIGHT 2048
#define LENGTH 2048

using namespace std;
using namespace cv;
struct passwd *pw = getpwuid(getuid());

const string homedir = pw->pw_dir;
string coordpath = homedir + "/catkin_ws/src/gzbo2_generator/output/coord.txt";
int start [2];
int stop [2];
int map [2048][2048];
int x = 0;
int y = 0;
double scale = 19.93970181650108;
int mvp = 255;
int nmvp = 0;
int obs = 0;
int wal = 150;
int walhigh = 3;
int wallow = 2;
Mat img;
string mappath = homedir + "/catkin_ws/src/gzbo2_generator/output/map.png";

//////////////////////////////////////////////////////////////////////////////////
// Name : read_coord	                                                        //
// Return type : void	                                                        //
// Parameters : int &, int &						                            //
// Function : Reads the coordinates from the file coord.txt					 	//
// Return : N/A  																//
//////////////////////////////////////////////////////////////////////////////////	
void read_coord(int &x, int &y) {
	string line;
	int i = 0;
	int neg_flag = 0;
	ifstream myfile (coordpath.c_str());
	if (myfile.is_open())
	{
		getline (myfile,line);
		getline (myfile,line);
		getline (myfile,line);
	}
	line.erase(line.begin() + 0, line.begin() + 17);
	while(line[i] != ','){
		if(line[i] == '-'){
			neg_flag = -1;
		}
		else{
			x = (x*10)+(line[i] - 48);
		}
		i++;
	}
	if(neg_flag)
		x *= -1;
	i+=2;
	while(line[i]){
		if(line[i] == '-'){
			neg_flag = -1;
		}
		else{
			y = (y*10)+line[i] - 48;
		}
		i++;
	}
	if(neg_flag)
		y *= -1;
	myfile.close();
}

//////////////////////////////////////////////////////////////////////////////////
// Name : read_map		                                                        //
// Return type : void	                                                        //
// Parameters : int &						            		                //
// Function : Reads the map from the file map.png							 	//
// Return : N/A  																//
//////////////////////////////////////////////////////////////////////////////////	
void read_map() {
	img = imread(mappath.c_str(),CV_LOAD_IMAGE_COLOR);
    imshow("opencvtest",img);
    waitKey(0);
  	}
  	
int main() {

	read_coord(x,y);
	read_map();
	cout << "Goal X = " << x << endl << "Goal Y = " << y << endl;
	stop[0] = (x+1024);
	stop[1] = (y+1024);
	cout << "Goal X = " << stop[0] << endl << "Goal Y = " << stop[1] << endl;
	cout << map[1024][1024] << endl;
	return 0;
}


