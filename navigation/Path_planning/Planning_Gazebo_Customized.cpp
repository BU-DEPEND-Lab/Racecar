#include <ctype.h>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <fstream>
#include <iostream>
#include <string>
#include <stdlib.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <numeric>
#include <math.h>
#include <cmath>
#include <utility>
#include <map>
#include <algorithm>
#include <unordered_map>
#include <time.h>

#define HEIGHT 2048
#define LENGTH 2048

using namespace std;
using namespace cv;
struct passwd *pw = getpwuid(getuid());
const string homedir = pw->pw_dir;
string coordpath = homedir + "/catkin_ws/src/gzbo2_generator/output/coord.txt";
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
Mat img;
vector<vector<int>> nei;
string mappath = homedir + "/catkin_ws/src/gzbo2_generator/output/map.png";

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

////////////////////////////////////////////////////////////////////////////////
// Name : read_map		                                                        
// Return type : void	                                                        
// Parameters : void						            		                
// Function : Reads the map from the file map.png							 	
// Return : N/A  																
////////////////////////////////////////////////////////////////////////////////	
void read_map() {
	img = imread(mappath.c_str(), 0);
	//imshow( "Display window", img );   // Show our image inside it.
    //waitKey(0);
	img.convertTo(img, CV_8U);
    for(int i = 0; i<2048;i++){
    	for(int j = 0; j<2048; j++){
    		m[i][j] = (int)img.at<uint8_t>(i,j);
    	}
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
	int low, high;
    if (ch == 1) {
	    low = 4;
    	high = 5;
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

////////////////////////////////////////////////////////////////////////////////
//                          Function Definition									
//   Name  - ExtendStarStopt													
//   Function  - Extend the walls in the map to give enough moving room for the 
//               Search algorithm. Done because Gazebo gives coordinates of		
//               Start and Stop right next to the walls 						
//   Arguments  - N/A 															
////////////////////////////////////////////////////////////////////////////////

void ExtendStartStop() {
    cout << "Extending start ..." << endl;
    if(Neighbors(2,start[0],start[1]))
    	for( int i = 0; i<nei.size(); i++)
        	m[nei[i][0]][nei[i][1]] = mvp;

 	if(Neighbors(2,stop[0],stop[1]))
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

	// m = rise / run
	float m = 0.0;
	vector<vector<int>> line;
	if(x2 == x1) {
		if(y2>y1)
			for(int y = y1; y<=y2; ++y) {
				line.push_back({x1,y});
			}
		
		else {
			for(int y = y2; y<=y1; ++y) {
				line.push_back({x1,y});
			}
		
		}
	}
	

	else if(x2>x1) {
		int run = x2 - x1;
		int rise = y2 - y1;
		m = ((float) rise) / ((float) run);
		if((run == 0) || (rise == 0)) {
			m = 0.;
		}
		// solve for b
		// (start with y = mx + b, subtract mx from both sides)
		float b = y1 - (m * x1);

		// note: assumes x2 > x1
		for (int x = x1; x <= x2; ++x)
		{
		    // solve for y
		    float y = (m * x) + b;

		    // round to nearest int
		    int rounded = (y > 0.0) ? floor(y + 0.5) : ceil(y - 0.5);

		    // convert int result back to float, compare
		    if ((float) rounded == y)
				line.push_back({x,rounded});
	
		}

	}
	else {
		int run = x1 - x2;
		int rise = y1 - y2;
		m = ((float) rise) / ((float) run);
		if((run == 0) || (rise == 0)) {
			m = 0.;
		}
		// solve for b
		// (start with y = mx + b, subtract mx from both sides)
		float b = y1 - (m * x1);

		// note: assumes x2 > x1
		for (int x = x2; x <= x1; ++x)
		{
		    // solve for y
		    float y = (m * x) + b;

		    // round to nearest int
		    int rounded = (y > 0.0) ? floor(y + 0.5) : ceil(y - 0.5);

		    // convert int result back to float, compare
		    if ((float) rounded == y)
		    	line.push_back({x,rounded});
		}
	}
	return line;
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
	//cout << gline.size()<<endl;
    for(int i = 0;i<gline.size();i++){
        if (m[gline[i][0]][gline[i][1]] != mvp)
            return 0;
        Neighbors(1,gline[i][0],gline[i][1]);
        for(int j = 0;j<nei.size(); j++){
            if (m[nei[j][0]][nei[j][1]] != mvp){
                return 0;
            }
        }
    }
    return 1;
}

////////////////////////////////////////////////////////////////////////////////
//                          Function Definition									
//   Name  - CheckDirect														
//   Function  - This function check if a point from one tree has a direct line 
//               of sight move to any branch of the other tree. This is done to 
//               curb run time, and prevent wastage of search if a direct path 	
//               is already visible.   											
//   Arguments  - vector<int> pt(point of referrence), 
//				  map<pair<int,int>, vector<vector<int>>> dict(Dictionary of the branches of	
//                the other tree.)												
////////////////////////////////////////////////////////////////////////////////
    
int CheckDirect(vector<int> pt, map<pair<int,int>, vector<vector<int>>> dict) {
	for (std::map<pair<int,int>, vector<vector<int>>>::iterator it=dict.begin(); it!=dict.end(); ++it)        
		if(CheckValid(pt,{{(it->first).first,(it->first).second }}))
            return 1;
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
//                          Function Definition									
//   Name  - CutNeighbors														
//   Function  - Returns Valid movable points from neighbors   					
//   Arguments  - pair<int,int> pt(point of referrence), map<pair<int,int>, 
//				  vector<vector<int>>> sD(Dictionary of the branches of the other tree.), 
//				  pair<int,int> ap(Previos point for reference)												
////////////////////////////////////////////////////////////////////////////////
    
vector<vector<int>> CutNeighbors(pair<int,int> pos, vector<pair<int,int>> sD, pair<int,int> ap) {
    vector<vector<int>> nv;
    int i;
    for (i = 0; i < nei.size(); i++) {
    	vector<vector<int>> lineA = {{ap.first, ap.second},{pos.first, pos.second}};
		vector<vector<int>> lineB = {{pos.first, pos.second},{nei[i][0],nei[i][1]}};
		int f = nei[i][0];
		int s = nei[i][1];
		auto it = std::find_if( sD.begin(), sD.end(),
	    ([s,f](const pair<int, int>& element)
	    {return ((element.first == f) && (element.second == s));}));
	    int temp = (CheckValid({pos.first, pos.second},nei[i])) ;
	    double temp2 = ang(lineA,lineB);
	    cout << temp << " " << temp2 << " " << (it == sD.end()) << endl;
        if ((CheckValid({pos.first, pos.second},nei[i]) == 1) && 
        (ang(lineA,lineB) <=90 )&&
        (it != sD.end()))
        	cout << nei[i][0] << " "<< nei[i][1];
           nv.push_back({nei[i][0], nei[i][1]}); 
   }
    return nv;
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
    int pt2 = 1;
    cout <<"Length of path Before = " << Line1.size() << endl;
    // Line Optimization //
    while (pt1<(Line1.size()-1) && pt2<(Line1.size()-1)) {
        while ((CheckValid(Line1[pt1],Line1[pt2]) == 1) and 
        (pt2 <(Line1.size()-1)))
            pt2 +=1;
        Line2.push_back(Line1[pt1]);
        pt1 = pt2;
        pt2 = pt1+1;
    }
    Line2.push_back(Line1[Line1.size()-1]);
    cout << "Length of path After = " << Line2.size() << endl;
    return Line2;
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

struct idmap
{
	int pos;
	double Value;
};

vector<idmap> AngleDef(vector<vector<int>> Line){
    idmap temp2;
    vector<idmap> anglist ;
    vector<vector<int>> AF = LineComp(Line);
    vector<int> temp;
    for (int i = 1; i<Line.size() - 1; i++){
    	vector<vector<int>> lineA = {{Line[i-1][0], Line[i-1][1]},{Line[i][0], Line[i][1]}};
		vector<vector<int>> lineB = {{Line[i][0],Line[i][1]},{Line[i+1][0], Line[i+1][1]}};
    	temp = {Line[i][0], Line[i][0]};
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


vector<int> SpeedList(vector<vector<int>> Line, vector<idmap> angleList) {
    vector<int> speedlist;
    double speed;
    int i;
    for (i = 0; i < Line.size() - 20; i++) {
        int flag = 0;
        for (int j = 0; j< angleList.size(); j++){
            double minspeed = 100.0 - int((angleList[j].pos/45.0) * 70.0);
            // Change ranges depending on use case //
            if ((angleList[j].pos - i) == 0) {
                speed = minspeed;
                speedlist[i] = (int)speed;
                flag = 1;                
                break;
            }
            else if (((angleList[j].pos - i) < 20) && ((angleList[j].pos - i) > 0)) {
                speed = 100 - ((100.0 - minspeed)*(1 - ((angleList[j].pos - i)/20.0)));
                speedlist[i] = (int)speed;
                flag = 1;
                break;
            }
            else if (((angleList[j].pos - i)> -10) && ((angleList[j].pos - i) < 0)) {
                speed = 100 - ((100.0 - minspeed)*(1 + ((angleList[j].pos - i)/10.0)));
                speedlist[i] = (int)speed;
                flag = 1;
                break;
            }
            else {
                flag = 1;
                speedlist[i] = 100;
            }
        }
        if (flag == 0) {
            speedlist[i] = 100;
        }
    }
    for(int k = 0; k<20; k++){
        speedlist[k+i] = int(100 - (100.0*(k/19.0)));
    }
    return speedlist;
}


int main() {
	vector<int> nei;
	vector<vector<int>> gline;
	read_coord(x,y);
	cout<<"Coordinates Read "<<x << " " << y <<endl;
	stop[0] = (x+1024);
	stop[1] = (y+1024);
	read_map();
	cout<<"Map Read"<<endl;
	rot90(3);
	fliplr();
	cout<<"Map Orientation set"<<endl;
	ExtendStartStop();
	int ErrorFree = 0;
	while (ErrorFree == 0) {
		//try {
			// Variable declerations for the Search tree that will begin from START
			map <pair<int,int>, vector<vector<int>>> SetTrees;
			SetTrees[{start[0],start[1]}].push_back({start[0],start[1]});
			pair<int,int> pos = {start[0],start[1]};
			vector<pair<int,int>> SetDismissed;
			pair<int,int> AnglePrevRef = {start[0]-1, start[1]-1};
			pair<int,int> prev;
			Neighbors(3,start[0],start[1]);
			// Variable declerations for the Search tree that will begin from STOP     
			map <pair<int,int>, vector<vector<int>>> SetTreesopp;
			SetTreesopp[{stop[0],stop[1]}].push_back({stop[0],stop[1]});
			pair<int,int> posopp = {stop[0],stop[1]};
			pair<int,int> AnglePrevRefopp = {stop[0]-1, stop[1]-1};
			pair<int,int> prepre = {start[0]-1, start[1]-1};

			// Variable Declaration for The final path 
			vector<vector<int>> LineFinal;
			vector<vector<int>> LineFinal2; 
			int GoalFlag = 0 ;
			LineFinal.push_back({start[0],start[1]});
			LineFinal2.push_back({stop[0],stop[1]}); 
			cout << "Variables set "<<endl;

			// Check if there is a direct path from START to STOP 
			int st_line = 0;
			if (CheckValid(start,stop) == 1){
			    GoalFlag = 1;   
			    st_line = 1;
			    cout<<"Direct Path Found" << endl;
			
			}
			cout<<"Direct Path Checked" << endl;
			clock_t search_start = clock();
			Neighbors(3,start[0],start[1]);
			cout<<"Starting Search" << endl;
			while (GoalFlag == 0) {
			    int finStep = 0;
			    while (finStep == 0){
			        vector<vector<int>> nV = CutNeighbors(pos,SetDismissed, AnglePrevRef); // nV(neighborsValid)
			        cout<<"Found valid Neighbors" << endl;
			        int nF, rtemp; // nF(notFound)
			        pair<int,int> nC; // nC(neighborChoice)
			        cout<<nV.size()<<endl;
			        if (nV.size() == 0){
			            nF = 1;
			        }
			        else{
			            nF = 0;
			            rtemp = rand() % nV.size();
			            nC = {nV[rtemp][0], nV[rtemp][1]};
			        }
			        if (nF == 1){
			            pos = {SetTrees[{AnglePrevRef}][SetTrees[{AnglePrevRef}].size()-2][0], SetTrees[{AnglePrevRef}][SetTrees[{AnglePrevRef}].size()-2][1]};
			            AnglePrevRef = {SetTrees[AnglePrevRef][SetTrees[AnglePrevRef].size()-3][0], SetTrees[AnglePrevRef][SetTrees[AnglePrevRef].size()-3][1]};
			        }
			        else{
			            cout << "[" << clock() - search_start<<"] : " << "Searching ... " << endl; // To enable time taken
			            prepre = AnglePrevRef;
			            SetDismissed.push_back(nC);
			            vector<int> nCtemp = {nC.first, nC.second};
			            int ret = CheckDirect(nCtemp , SetTreesopp); // Check for direct line of sight
			            AnglePrevRef = pos;        
			            if (ret == 1){ // There is direct line of sight
			                cout << "Found Goal" << endl;
			                GoalFlag = 1;
			                finStep = 1;

			                // Add point and line of sight branch to tree 
			                SetTrees[nC] = SetTrees[pos];
			                SetTrees[nC].push_back({nCtemp[0],nCtemp[1]});

			                // copy final tree from START 
			                for (int ii = 0; ii<SetTrees[nC].size(); ii++){
			                    LineFinal.push_back(SetTrees[nC][ii]);
			                }
			            }

			            else{ // There is no direct line of sight
			                // Add point to as a branch to START tree 
			                SetTrees[nC] =SetTrees[pos];
			                SetTrees[nC].push_back(nCtemp);
			                // Choose next point to search from 
			                pos = nC;
			     		    Neighbors(3,pos.first,pos.second);
			            }    
		    		}
		    	}
		    }
		ErrorFree = 1;
		//}

	   // catch(...){
	    //    cout<< "Caught Index Error"<<endl;
	    //    ErrorFree = 0 ;
	   // }
	}
/*
	""" Function calls for Final-Processing of Final Path """

	""" Convert the two branches from START and STOP into one """
	LineFinal2.reverse() # Reverse the final path from STOP tree
	LineFinal+=LineFinal2 # Combine into one path

	""" Try processing multiple times to see different results """
	start_time5 = time.time()
	LF1 = LineComp(LineFinal)
	if not st_line: 
	    print " Starting Optimization"    
	    print "Map - without optimization"
	    LF1 = LineComp(LF1)
	    LF2 = LineOptimization(LF1)
	    print "Map - first optimization"
	    LF2 = LineComp(LF2)
	    LF = LineOptimization(LF2)

	else:
	    LF = LF1
	stop_time5 = time.time()
	time5 = stop_time5 - start_time5

	angs = AngleDef(LF)
	SF = np.array(LineComp(LF))
	SF = SF.astype(float)
	#print "SSSSSSFFFFFF = ", SF
	speeds = SpeedList(SF,angs)
	#print "SPEEEEEEDDDDDSSSS = ",speeds
	arr = Float64MultiArray()
	setsend = [[0,0,100]for ii in range(len(SF))]
	for i in range(len(SF)-1):
	    for k in range(2):
	        SF[i][k] -= 1024.0
	        SF[i][k] /= scale
	for i in range(len(SF)-1):
	    setsend[i] = [SF[i][0],SF[i][1],speeds[str(i)]]
	arr.data = np.ndarray.flatten(np.array(setsend)).tolist() 
	print "Path Sent"

	#print "ARRRRRRRRRRRRRRRRR = ", arr
	pub.publish(arr)
*/
	return 0;
}


