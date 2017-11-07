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
#include <numeric>
#include <math.h>
#include <cmath>
#include <utility>
#include <map>
#include <algorithm>
#include <unordered_map>
#include <time.h>
#include <math.h>

#define HEIGHT 2048
#define LENGTH 2048

using namespace std;
using namespace cv;
struct passwd *pw = getpwuid(getuid());
const string homedir = pw->pw_dir;
string coordpath = homedir + "/Desktop/Path_planning/Generator/LooseFiles/Maps/coord";
string outpath = homedir + "/Desktop/Path_planning/Generator/LooseFiles/Paths_RRT/path";
vector<int> start = {(1024), (1024)};
vector<int> stop = {(0), (0)};
uint8_t m [2048][2048];
int x = 0;
int y = 0;
int X = 2048;
int Y = 2048;
double scale = 19.93970181650108;
int mvp = 255;
int nmvp = 0;
int obs = 0;
int wal = 150;
int walhigh = 3;
int wallow = 2;
int delta = 20;
Mat img;
vector<vector<int>> nei;
string mappath = homedir + "/Desktop/Path_planning/Generator/LooseFiles/Maps/map";

struct idmap
{
	int pos;
	double Value;
};

////////////////////////////////////////////////////////////////////////////////
// Name : read_coord	                                                        
// Return type : void	                                                        
// Parameters : int &, int &						                            
// Function : Reads the coordinates from the file coord.txt					 	
// Return : N/A  																
////////////////////////////////////////////////////////////////////////////////	
void read_coord(int &x, int &y) {
	string line;
	int i = 0;
	int neg_flag = 0;
	ifstream myfile (coordpath.c_str());
	if (myfile.is_open()) {
		getline (myfile,line);
		getline (myfile,line);
		getline (myfile,line);
	}
	line.erase(line.begin() + 0, line.begin() + 17);
	while(line[i] != ',') {
		if(line[i] == '-') {
			neg_flag = -1;
		}
		else {
			x = (x*10)+(line[i] - 48);
		}
		i++;
	}
	if(neg_flag)
		x *= -1;
	i+=2;
	neg_flag = 0;
	while(line[i]) {
		if(line[i] == '-') {
			neg_flag = -1;
		}
		else {
			y = (y*10)+line[i] - 48;
		}
		i++;
	}
	if(neg_flag)
		y *= -1;
	myfile.close();
}


/**  @function Erosion  */
void Erosion(Mat& image)
{ 
  int erosion_elem = 0;
  int erosion_size = 10;
  int erosion_type;
  if( erosion_elem == 0 ){ erosion_type = MORPH_RECT; }
  else if( erosion_elem == 1 ){ erosion_type = MORPH_CROSS; }
  else if( erosion_elem == 2) { erosion_type = MORPH_ELLIPSE; }

  Mat element = getStructuringElement( erosion_type,
                                       Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                       Point( erosion_size, erosion_size ) );

  /// Apply the erosion operation
  erode( image, image, element );
  //imshow( "Erosion Demo", image );
  //waitKey(-1); 


}

////////////////////////////////////////////////////////////////////////////////
// Name : read_map                                                              
// Return type : void                                                           
// Parameters : void                                                            
// Function : Reads the map from the file map.png                               
// Return : N/A                                                                 
////////////////////////////////////////////////////////////////////////////////    
void read_map() {
    img = imread(mappath.c_str(), 0);
    Erosion(img);
    img.convertTo(img, CV_8U);
    for(int i = 0; i<2048;i++){
        for(int j = 0; j<2048; j++){
            m[i][j] = (int)img.at<uint8_t>(i,j);
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////
//                          Function Definition
//   Name  - DisplayPath
//   Function  - used to diaply path if necessary
//   Arguments  - vector<vector<int>> Line , int val
/////////////////////////////////////////////////////////////////////////////////

void DisplayPath(vector<vector<int>> path , int val) {
		for (int i = 0; i < path.size(); i ++){
			m[path[i][0]][path[i][1]] = val;
		}
    }



////////////////////////////////////////////////////////////////////////////////
// Name : rot90		                                                        
// Return type : void	                                                        
// Parameters : int num (number of rotations)						            		                
// Function : Rotates the map							 	
// Return : N/A  																
////////////////////////////////////////////////////////////////////////////////	
void rot90(int num) {
	int temp;
	for (int i = 0; i<num;i++) {
		for (int n = 0; n< (2048 - 2); n++){
			for(int l = n+1; l<(2048 - 1); l++){
				temp = m[n][l];
				m[n][l] = m[l][n];
				m[l][n] = temp;
			}
		}
	}
}
  	
////////////////////////////////////////////////////////////////////////////////
// Name : fliplr		                                                        
// Return type : void	                                                        
// Parameters : N/A						            		                
// Function : flips the map horizontlly							 	
// Return : N/A  																
////////////////////////////////////////////////////////////////////////////////	
void fliplr() {
	int temp;
	for (int n = 0; n< (2048 ); n++){
		for(int l = 0; l<(2048/2); l++){
			temp = m[n][l];
			m[n][l] = m[n][2048 - l];
			m[n][2048 - l] = temp;
		}
	}
}
  

////////////////////////////////////////////////////////////////////////////////
//                          Function Definition									
//   Name  - Neighbors                                                          
//   Function  - Finds the neighboring points around a given point in a grid.	
//               Depending on the requirement, it is used for finding all the 	
//               points between the high and low x, y distance. And it is also 	
//               used to find a points for the search step. For 3, returns		
//               every 10th neighbor to prevent over-crowding of branches.		
//   Arguments  - int ch -> 1(low 3 and high 4) - Check if Valid move,				
//                     2(low 1 and high 2) - Check for noise					
//                     3(low 20 and high 21) - Only consider perimeter for jump	
//                int ptx, int pty -> point of reference								
////////////////////////////////////////////////////////////////////////////////

	
int Neighbors(int ch , int ptx, int pty) {
	nei.clear();
	int low, high;
    if (ch == 1) {
	    low = 1;
    	high = 2;
    }
    if (ch == 2) {
        low = 10;
        high = 10;
    }

    if (ch == 3) {
    	low = 15;
        high = 16;
        int thres = 13;
        for (int dx=-low; dx<=high; dx++)
	  		for (int dy=-low; dy<=high; dy++) 
			    if (dx || dy){
			        int x = ptx+dx, y=pty+dy;
	   		        if ((x >= 0) && (x < X) && (y >= 0) && (y < Y) && 
	   		        	((abs(x-ptx)>thres) || (abs(y-pty)>thres))) {
			            nei.push_back({x,y});
		        }
		    }
	return 0;
	}
    if (ch == 4) {
        low = wallow;
        high = walhigh;
	}

	for (int dx=-low; dx<=high; dx++)
  		for (int dy=-low; dy<=high; dy++) 
		    if (dx || dy){
		        int x = ptx+dx, y=pty+dy;
   		        if (x >= 0 && x < X && y >= 0 && y < Y){
		            nei.push_back({x,y});

	        }
	    }
	return 0;
}

/////////////////////////////////////////////////////////////////////////////////
//                          Function Definition
//   Name  - Dist
//   Function  - Returns Distance between two coordinates 
//   Arguments  - int x1, int x2, int y1, int y2
/////////////////////////////////////////////////////////////////////////////////

double Dist(int x1, int x2, int y1, int y2) {

    return(sqrt(((x1-x2)*(x1-x2)) + ((y1-y2)*(y1-y2))));
}
////////////////////////////////////////////////////////////////////////////////
//                          Function Definition									
//   Name  - ExtendStarStopt													
//   Function  - Extend the walls in the map to give enough moving room for the 
//               Search algorithm. Done because Gazebo gives coordinates of		
//               Start and Stop right next to the walls 						
//   Arguments  - N/A 															
////////////////////////////////////////////////////////////////////////////////

void ExtendStartStop() {
    //cout << "Extending start ..." << endl;
    Neighbors(2,start[0],start[1]);
    for( int i = 0; i<nei.size(); i++)
       	m[nei[i][0]][nei[i][1]] = mvp;
    //cout << "Extending stop ..." << endl;
 	Neighbors(2,stop[0],stop[1]);
    for( int i = 0; i<nei.size(); i++)
       	m[nei[i][0]][nei[i][1]] = mvp;
}


////////////////////////////////////////////////////////////////////////////////
//                          Function Definition									
//   Name  - ang 																
//   Function  - Used to find angle between two lines 							
//   Arguments  - vector<vector<int>> lineA, vector<vector<int>> lineB 													
////////////////////////////////////////////////////////////////////////////////

double ang(vector<vector<int>> lineA, vector<vector<int>> lineB) {
    
    // Get nicer vector form
    int vA[] = {(lineA[0][0]-lineA[1][0]), (lineA[0][1]-lineA[1][1])};
    int vB[] = {(lineB[0][0]-lineB[1][0]), (lineB[0][1]-lineB[1][1])};
    
    // Get dot prod
    double dot_prod = inner_product(begin(vA), end(vA), begin(vB), 0.0);

    // Get magnitudes
    double magA = pow(inner_product(begin(vA), end(vA), begin(vA), 0.0),0.5);
    double magB = pow(inner_product(begin(vB), end(vB), begin(vB), 0.0),0.5);
    double angle;
    try {

        // Get angle in radians and then convert to degrees
         angle = acos(dot_prod/magB/magA);
    }
    catch(...) {
         angle = 0;
    }

    // Basically doing angle <- angle mod 360
    double ang_deg = fmod(((180.0/3.14)*(angle)),360.0);
    
    if (std::isnan(ang_deg)) {
        return 180;
    }
    return ang_deg;
}

////////////////////////////////////////////////////////////////////////////////	
//                          Function Definition 								
//   Name  - get_line															
//   Function  - Returns all the points in a grid that fall between two given	
//               referrence points. 											
//   Arguments  - int x1, int y1 -> (pt1), int x2, int y2 -> (pt2) 								
////////////////////////////////////////////////////////////////////////////////

vector<vector<int>> get_line(int x1, int y1, int x2, int y2) {
    vector<vector<int>> points;
    bool issteep;
    if(abs(y2-y1) > abs(x2-x1))
    	issteep = true;
    else
    	issteep = false;
    if (issteep){
        int temp;
        temp = x1;
        x1 = y1;
        y1 = temp;
        temp = x2;
        x2 = y2;
        y2 = temp;
    }
    bool rev = false;
    if (x1 > x2){
    	int temp;
        temp = x1;
        x1 = x2;
        x2 = temp;
        temp = y1;
        y1 = y2;
        y2 = temp;
        rev = true;
    }
    int deltax = x2 - x1;
    int deltay = abs(y2-y1);
    int error = (int)(deltax / 2);
    int y = y1;
    int ystep = 0;
    if (y1 < y2)
        ystep = 1;
    else
        ystep = -1;
    int count = 0;
    for (int x = x1; x<x2+1 ; x++){
        if (issteep && (count%4 == 0)){
            points.push_back({y, x});
        }
        else if (count%4 == 0){
            points.push_back({x, y});
		}
        error -= deltay;
        if (error < 0){
            y += ystep;
            error += deltax;
        }
        count++;
    }

    // Reverse the list if the coordinates were reversed
    if (rev)
        reverse(points.begin(), points.end());
    return points;
}


////////////////////////////////////////////////////////////////////////////////
//                          Function Definition									
//   Name  - CheckValid															
//   Function  - Checks if a move is valid between two given referrence points.	
//               The condition is that the two points should not be non movable 
//               points and every point between the two should have a buffer 	
//               from the wall. The buffer if found using Neighbors with x = 1.	
//               Returns 1 for valid move, and 0 for invalid move.				
//   Arguments  - vector<int> pt1, vector<int> pt2														
////////////////////////////////////////////////////////////////////////////////
int CheckValid(vector<int> pt1, vector<int> pt2) {
	vector<vector<int>> gline;
	gline = get_line(pt1[0],pt1[1],pt2[0],pt2[1]);
    for(int i = 0;i<gline.size();i++){
        if ((int)m[gline[i][0]][gline[i][1]] == nmvp){
            return 0;}
        Neighbors(1,gline[i][0],gline[i][1]);
        for(int j = 0;j<nei.size(); j++){
            if ((int)m[nei[j][0]][nei[j][1]] == nmvp){
                return 0;
            }
        }
    }
    return 1;
}

////////////////////////////////////////////////////////////////////////////////
//                          Function Definition                                 
//   Name  - CheckGoalRRT                                                         
//   Function  - Checks if point sees tree              
//   Arguments  - vector<int> pt1, vector<vector<vector<int>>> Tree                                                      
////////////////////////////////////////////////////////////////////////////////
vector<int> CheckGoalRRT(vector<int> pt1, vector<vector<vector<int>>> Tree) {
    vector<vector<int>> gline;
    vector<int> pt2, ret;
    int flag = 0;
    for(int ii = 0; ii<Tree.size(); ii++){
        for(int jj = 0; jj<Tree[ii].size(); jj++){
            flag = 0;
            pt2.clear();
            pt2.push_back(Tree[ii][jj][0]);
            pt2.push_back(Tree[ii][jj][1]);
            if(Dist(pt1[0],pt1[1],pt2[0],pt2[1]) > 20){
                if(!CheckValid(pt1, pt2)){
                    flag = 1;
                }
            }
            if(flag == 0){
                ret.push_back(ii);
                ret.push_back(jj);
                return ret;
            }
        }
    }
    return ret;
}

////////////////////////////////////////////////////////////////////////////////
//                          Function Definition									
//   Name  - CutGrid														
//   Function  - Returns Valid movable points from Grid   					
//   Arguments  - pair<int,int> pt(point of referrence), map<pair<int,int>, 
//				  vector<vector<int>>> sD(Dictionary of the branches of the other tree.), 
//				  pair<int,int> ap(Previos point for reference)												
////////////////////////////////////////////////////////////////////////////////
    
vector<vector<int>> CutGrid(vector<vector<int>> grid, vector<vector<int>> sD) {
    vector<vector<int>> nv;
    int i;
    for (i = 0; i < grid.size(); i++) {
		int f = grid[i][0];
		int s = grid[i][1];
		int it = -1;
		for (int k = 0; k<sD.size(); k++){
			if((sD[k][0] == f) && (sD[k][1] == s)){
				it = k;
				break;
			}
		}
        if ((it == -1))
            nv.push_back({grid[i][0], grid[i][1]}); 
    }
    return nv;
}

/////////////////////////////////////////////////////////////////////////////////
//                          Function Definition
//   Name  - LineComp
//   Function  - This Function adds all intermediate points to the Line for 
//               easier movement control by the car
//   Arguments  - vector<vector<int>> Line -> Line with scattered points instead of being continuous
/////////////////////////////////////////////////////////////////////////////////

vector<vector<int>> LineComp(vector<vector<int>> Line) {
    vector<vector<int>> NewLine, gline; 
    for (int i = 0; i<Line.size()-1; i++){
    	gline = get_line(Line[i][0],Line[i][1],Line[i+1][0],Line[i+1][1]);
    	for (int j = 0; j<gline.size()-1; j++){
        	NewLine.push_back({gline[j][0],gline[j][1]});
        }
    }
    return NewLine;
}

////////////////////////////////////////////////////////////////////////////////
//                          Function Definition
//   Name  - LineOptimization
//   Function  - This function optimizes the line by removing redundant points. 
//               It does this by checking every point on the line with all the
//               points after it to check for line of sight. If found, it 
//               removes all the intermediate points.   
//   Arguments  - vector<vector<int>> Line1 -> Final Path
/////////////////////////////////////////////////////////////////////////////////

vector<vector<int>> LineOptimization(vector<vector<int>> Line1) {
    
    // Variable declaration for line optimization //
    vector<vector<int>> Line2;
    int pt1 = 0;
    int pt2 = Line1.size()-1;
    cout <<"Length of path Before = " << Line1.size() << endl;
    
    // Line Optimization //
    while (pt1<(Line1.size()-1) && pt2>pt1) {
        while ((CheckValid(Line1[pt1],Line1[pt2]) != 1) && 
        (pt2 >pt1)){
        	pt2 -=1;
        }
        Line2.push_back(Line1[pt1]);
        pt1 = pt2+1;
        pt2 = Line1.size()-1;
    }
    Line2.push_back(Line1[Line1.size()-1]);
    cout << "Useful points = " << Line2.size() << endl;
    Line2 = LineComp(Line2);
    cout << "Length of path After = " << Line2.size() << endl;
    vector<vector<int>> Line3;
    pt2 = 0;
    pt1 = Line2.size()-1;
    cout <<"Length of path Before(reverse) = " << Line2.size() << endl;
    
    // Line Optimization //
    while ((pt1 > 0) && (pt1>pt2)) {
        while ((CheckValid(Line2[pt2],Line2[pt1]) != 1) && 
        (pt1 > pt2)){
        	//cout << (CheckValid(Line2[pt2],Line2[pt1])) << " " << Line2[pt2][0] << " " << Line2[pt2][1]<< " " << Line2[pt1][0]<< " " << Line2[pt1][1]<< endl; 
        	pt2 +=1;
        }
        //cout << "\n"<< Line2[pt1][0] << " " <<Line2[pt1][1] << "\n\n"; 
        Line3.push_back(Line2[pt1]);
        pt1 = pt2-1;
        pt2 = 0;
    }
    Line3.push_back(Line2[0]);
    cout << "Useful points(reverse) = " << Line3.size() << endl;
    Line3 = LineComp(Line3);
    cout << "Length of path After(reverse) = " << Line3.size() << endl;
    return Line3;
}

////////////////////////////////////////////////////////////////////////////////
//                          Function Definition
//   Name  - dot
//   Function  - Used to find dot product between two vectors
//   Arguments  - vector<int> vA -> Vector A, vector<int> vB -> Vector B
////////////////////////////////////////////////////////////////////////////////

int dot(vector<int> vA, vector<int> vB){
    return vA[0]*vB[0]+vA[1]*vB[1];
}

/////////////////////////////////////////////////////////////////////////////////
//                          Function Definition
//   Name  - AngleDef
//   Function  - Used to create a list of all the angular changes in the entire
//               path. It does this by finding the angular difference at each 
//               position in the map w.r.t the next position
//   Arguments  - vector<vector<int>> Line
/////////////////////////////////////////////////////////////////////////////////

vector<idmap> AngleDef(vector<vector<int>> Line){
    idmap temp2;
    vector<idmap> anglist ;
    vector<vector<int>> AF = LineComp(Line);
    vector<int> temp;
    for (int i = 1; i<Line.size() - 1; i++){
    	vector<vector<int>> lineA = {{Line[i-1][0], Line[i-1][1]},{Line[i][0], Line[i][1]}};
		vector<vector<int>> lineB = {{Line[i][0],Line[i][1]},{Line[i+1][0], Line[i+1][1]}};
    	temp = {Line[i][0], Line[i][1]};
    	int it = distance(AF.begin(), find(AF.begin(), AF.end(), temp));
        temp2.pos = it;
        temp2.Value = ang({Line[i-1],Line[i]},{Line[i],Line[i+1]});
        anglist.push_back(temp2);
    }
    return anglist;
}

/////////////////////////////////////////////////////////////////////////////////
//                          Function Definition
//   Name  - SpeedList
//   Function  - Calculates the speeds at which the car has to travel
//   Arguments  - vector<vector<int>> Line, map<int, double> angleList
/////////////////////////////////////////////////////////////////////////////////


vector<double> SpeedList(vector<vector<int>> Line, vector<idmap> angleList) {
    vector<double> speedlist;
    double speed;
    int i;
    for (i = 0; i < Line.size() - 20; i++) {
        int flag = 0;
        for (int j = 0; j< angleList.size(); j++){
            double minspeed = 100.0 - int((angleList[j].Value/45.0) * 70.0);
            
            // Change ranges depending on use case //
            if ((angleList[j].pos - i) == 0) {
                speed = minspeed;
                speedlist.push_back(speed);
                flag = 1;                
                break;
            }
            else if (((angleList[j].pos - i) < 20) && ((angleList[j].pos - i) > 0)) {
                speed = 100.0 - ((100.0 - minspeed)*(1 - ((angleList[j].pos - i)/20.0)));
                speedlist.push_back(speed);
                flag = 1;
                break;
            }
            else if (((angleList[j].pos - i)> -10) && ((angleList[j].pos - i) < 0)) {
                speed = 100.0 - ((100.0 - minspeed)*(1 + ((angleList[j].pos - i)/10.0)));
                speedlist.push_back(speed);
                flag = 1;
                break;
            }
        }
        if (flag == 0) 
            speedlist.push_back(100.0);
        
    }
    for(int k = 0; k<19; k++)
        speedlist.push_back((100.0 - (100.0*(k/19.0))));
       
    return speedlist;
}

/////////////////////////////////////////////////////////////////////////////////
//                          Function Definition
//   Name  - RRTGridspace
//   Function  - Returns a uniformly distributed gridspace for RRT
//   Arguments  - N/A
/////////////////////////////////////////////////////////////////////////////////

vector<vector<int>> RRTGridspace() {
    vector<vector<int>> GridCoord;
    for(int x = 0; x < 2048; x += delta){
        for(int y = 0;y < 2048; y += delta){
            if((int)m[x][y] == mvp){
                GridCoord.push_back({x,y});
            }
        }
    }
    return GridCoord;
}



/////////////////////////////////////////////////////////////////////////////////
//                          Function Definition
//   Name  - DisplayTree
//   Function  - used to diaply path if necessary
//   Arguments  - vector<vector<vector<int>>> Tree
/////////////////////////////////////////////////////////////////////////////////

void DisplayTree(vector<vector<vector<int>>> Tree, int val) {
        for(int i = 0; i<Tree.size(); i++){
            DisplayPath(LineComp(Tree[i]) , val);
            }
        Mat dataMatrix1(2048,2048,CV_8UC1, m);
        imshow( "Display window", dataMatrix1 );   // Show our image inside it.
        waitKey(10); 
       }

int main(int argc, char** argv) {
    string tempst = argv[1];
    coordpath += (tempst+".txt");
    outpath += (tempst+".txt");
    mappath += (tempst+".png");
	//cout << "Starting" <<coordpath<<endl<<outpath<<endl<<mappath<<endl;
	vector<vector<int>> gline, GridCoord;
	read_coord(x,y);
	stop[0] = (x+1024);
	stop[1] = (y+1024);
	//cout<<"Coordinates Read "<<stop[0] << " " << stop[1] <<" "<< start[0]<<" "<<start[1] <<endl;
	read_map();
	//cout<<"Map Read"<<endl;
	rot90(3);
	//cout<<"Map Orientation set"<<endl;
	ExtendStartStop();
    GridCoord = RRTGridspace();
    int Goal = 0;
    int rtemp, flag = 0, flag2 = 0;
    double minDist, d;
    vector<int> choice, branch, choice2, branch2;
    vector<vector<int>> setDismissed;
    vector<vector<int>> pushvec, pushvec2, LineFinal;
    vector<vector<vector<int>>> startTree, stopTree;
    //cout << "Starting "<< endl;
    pushvec.push_back({start[0], start[1]});
    startTree.push_back(pushvec);
    pushvec.clear();
    pushvec.push_back({stop[0], stop[1]});
    stopTree.push_back(pushvec);
    clock_t search_start;
    search_start = clock();
    while(Goal != 1){
	if(((double)(clock() - search_start)/1000000) > 60000){
		return 0;	
	}
        //cout <<"RRT : - "<< "[" << (double)(clock() - search_start)/1000000<<"] : " << "Searching ... " <<endl; // To enable time taken
        srand(time(NULL));
        rtemp = rand() % GridCoord.size();
        choice= GridCoord[rtemp];
        minDist = 100000;
        branch.push_back(0);
        branch.push_back(0); 
        flag = 0; 
        for(int ii = 0; ii<startTree.size(); ii++){
            for(int jj = 0; jj<startTree[ii].size(); jj++){
                if(CheckValid(choice, startTree[ii][jj]) == 1) {
                    d = Dist(choice[0], startTree[ii][jj][0], choice[1], startTree[ii][jj][1]);
                    if((d < minDist)){
                    branch[0] = ii;
                    branch[1] = jj;
                    flag = 1;
                    minDist = d;
                    }
               }
            }
        }
        pushvec.clear();
        if((flag == 1) && (minDist != 0)){
            setDismissed.push_back({choice[0], choice[1]});
            GridCoord = CutGrid(GridCoord, setDismissed);
            if(branch[1] != startTree[branch[0]].size() - 1){
                for(int ii = 0; ii<=branch[1]; ii++){
                    pushvec.push_back(startTree[branch[0]][ii]);
                }

                pushvec.push_back(choice);
                startTree.push_back(LineComp(pushvec));
            }
            else{
                pushvec.push_back(startTree[branch[0]][branch[1]]);
                pushvec.push_back(choice);
                pushvec = LineComp(pushvec);
                for(int i = 0; i<pushvec.size(); i++){
                    startTree[branch[0]].push_back(pushvec[i]);
                }
            }
        }
        if((flag ==1) && (CheckGoalRRT(choice, stopTree).size() != 0)){
            Goal = 1;
            //cout << "Goal Reached Start" << endl;
            pushvec.clear();
            pushvec.push_back(CheckGoalRRT(choice, stopTree));
            for(int ii = 0; ii<startTree[branch[0]].size(); ii++){
                LineFinal.push_back(startTree[branch[0]][ii]);
                //cout<<LineFinal[ii][0] << " " <<LineFinal[ii][1] << endl;
            }
            pushvec2 = get_line(choice[0], choice[1], 
                stopTree[pushvec[0][0]][pushvec[0][1]][0],
                stopTree[pushvec[0][0]][pushvec[0][1]][1]);
            for(int ii = 0; ii<pushvec2.size(); ii++){
                LineFinal.push_back(pushvec2[ii]);
                //cout<<LineFinal[ii][0] << " " <<LineFinal[ii][1] << endl;
            } 
            pushvec2.clear();
            for(int ii = 0; ii<pushvec[0][1]; ii++){
                pushvec2.push_back(stopTree[pushvec[0][0]][ii]);
            }
            reverse(pushvec2.begin(), pushvec2.end());
            for(int ii = 0; ii<pushvec2.size(); ii++){
                LineFinal.push_back(pushvec2[ii]);
                //cout<<LineFinal[ii][0] << " " <<LineFinal[ii][1] << endl;
            } 
            //DisplayPath(LineComp(LineFinal), 0);
            //Mat dataMatrix1(2048,2048,CV_8UC1, m);
            //imshow( "Display window", dataMatrix1 );
            //waitKey(-1);

        }
        //DisplayTree(startTree, 150);
        srand(time(NULL));
        rtemp = rand() % GridCoord.size();
        choice= GridCoord[rtemp];
        minDist = 100000;
        branch.push_back(0);
        branch.push_back(0); 
        flag = 0; 
        for(int ii = 0; ii<stopTree.size(); ii++){
            for(int jj = 0; jj<stopTree[ii].size(); jj++){
                if(CheckValid(choice, stopTree[ii][jj]) == 1) {
                    d = Dist(choice[0], stopTree[ii][jj][0], choice[1], stopTree[ii][jj][1]);
                    if((d < minDist)){
                    branch[0] = ii;
                    branch[1] = jj;
                    flag = 1;
                    minDist = d;
                    }
               }
            }
        }
        pushvec.clear();
        if((flag == 1) && (minDist != 0)){
            setDismissed.push_back({choice[0], choice[1]});
            GridCoord = CutGrid(GridCoord, setDismissed);
            if(branch[1] != stopTree[branch[0]].size() - 1){
                for(int ii = 0; ii<=branch[1]; ii++){
                    pushvec.push_back(stopTree[branch[0]][ii]);
                }

                pushvec.push_back(choice);
                stopTree.push_back(LineComp(pushvec));
            }
            else{
                pushvec.push_back(stopTree[branch[0]][branch[1]]);
                pushvec.push_back(choice);
                pushvec = LineComp(pushvec);
                for(int i = 0; i<pushvec.size(); i++){
                    stopTree[branch[0]].push_back(pushvec[i]);
                }
            }
            }
        if((flag ==1) && (CheckGoalRRT(choice, startTree).size() != 0)){
            Goal = 1;
            //cout << "Goal Reached Stop" << endl;
            pushvec.clear();
            pushvec.push_back(CheckGoalRRT(choice, startTree));
            for(int ii = 0; ii<stopTree[branch[0]].size(); ii++){
                LineFinal.push_back(stopTree[branch[0]][ii]);
                //cout<<LineFinal[ii][0] << " " <<LineFinal[ii][1] << endl;
            }
            pushvec2 = get_line(choice[0], choice[1], 
                startTree[pushvec[0][0]][pushvec[0][1]][0],
                startTree[pushvec[0][0]][pushvec[0][1]][1]);
            for(int ii = 0; ii<pushvec2.size(); ii++){
                LineFinal.push_back(pushvec2[ii]);
                //cout<<LineFinal[ii][0] << " " <<LineFinal[ii][1] << endl;
            } 
            pushvec2.clear();
            for(int ii = 0; ii<pushvec[0][1]; ii++){
                pushvec2.push_back(startTree[pushvec[0][0]][ii]);
            }
            reverse(pushvec2.begin(), pushvec2.end());
            for(int ii = 0; ii<pushvec2.size(); ii++){
                LineFinal.push_back(pushvec2[ii]);
                //cout<<LineFinal[ii][0] << " " <<LineFinal[ii][1] << endl;
            } 
            //DisplayPath(LineComp(LineFinal), 0);
            //Mat dataMatrix1(2048,2048,CV_8UC1, m);
            //imshow( "Display window", dataMatrix1 );
            //waitKey(-1);

        }
    //DisplayTree(stopTree, 150);
    }
    // Try processing multiple times to see different results //
	vector<vector<int>>LF1 = LineComp(LineFinal);
	vector<vector<int>> LF;
	//cout << "Starting Optimization" << endl;   
	//cout << "Map - without optimization" << endl;
	//LF1 = LineOptimization(LF);
    //cout << "Time taken:- " <<(double)(clock() - search_start)/1000000 << endl;
    //DisplayPath(LF1, 0);
    //Mat dataMatrix1(2048,2048,CV_8UC1, m);
    //imshow( "Display window", dataMatrix1 );
    //waitKey(-1);
	vector<idmap> angs = AngleDef(LF1);
	vector<vector<double>> Path;
	vector<double> speeds = SpeedList(LF1,angs);
    ofstream myfile;
    myfile.open(outpath.c_str());
	for (int i = 0; i<speeds.size(); i++) {
        Path.push_back({((LF1[i][0] - 1024.0)/scale) ,((LF1[i][1] - 1024.0)/scale)});
       	//cout << speeds[i] << " " << Path[i][0] << " " << Path[i][1] << endl;
       	myfile << Path[i][0] << "\n" << Path[i][1] << "\n" << speeds[i] << "\n";
    }
    myfile.close();
    cout<<LF1.size();
    return LF1.size();
}


