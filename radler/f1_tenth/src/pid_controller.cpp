#include "pid_controller.h"

// include cpp libraries
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <math.h>
#include <algorithm>
#include <stdlib.h>
#include <iterator>
using namespace std;

#define	TRUE	1
#define	FALSE	0

struct quaternion{
    double x, y, z, w;
};

struct pose{
    double x, y, roll, pitch, yaw;
};

struct Msg{
      float velocity, angle; 
};

struct robot{
    double x, y, yaw, speed;
};

robot myrobot;

double look_ahead_dist, goalRadius, speed;
double prev_steer = 0;
double prev_speed = 0;
int STOP_Dist = 200;
int planner_coord;


int minIndex(vector<double>::iterator start, vector<double>::iterator end) {

    if (start == end) return 0;

    vector<double>::iterator first, minimum = start;
    int counter = 0;

    while ( start != end ) {
        // cout << "start is : " << *start << "  min is : " << *minimum << endl;
        if (*start < *minimum) {
          // cout << "min in " << endl;
          minimum = start;
          counter ++;
        }
        start ++;
    }
    cout << counter << endl;
    return counter;

}

pose poseUpdate(const radl_in_t * in)
{
    // orientation data
    quaternion q;
    pose e;
    q.x = in->slam_out_pose->orientation_x;
    q.y = in->slam_out_pose->orientation_y;
    q.z = in->slam_out_pose->orientation_z;
    q.w = in->slam_out_pose->orientation_w;

    //cout << "q.x is :::::::: " << q.x << endl;
	  // roll (x-axis rotation)
	  double sinr = +2.0 * (q.w * q.x + q.y * q.z);
	  double cosr = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
	  e.roll = atan2(sinr, cosr);

	  // pitch (y-axis rotation)
	  double sinp = +2.0 * (q.w * q.y - q.z * q.x);
    if (fabs(sinp) >= 1)
      e.pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
	    e.pitch = asin(sinp);

	  // yaw (z-axis rotation)
	  double siny = +2.0 * (q.w * q.z + q.x * q.y);
	  double cosy = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);  
	  e.yaw = atan2(siny, cosy);
	  
	  // pose data
	  e.x = in->slam_out_pose->position_x;
	  e.y = in->slam_out_pose->position_y;
	  
	  return e;
}

double dist(double x1, double x2, double y1, double y2){
  double d = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
  return d;
}

bool goalCheck(double goal_x, double goal_y, double curr_x, double curr_y){
  double goalRadius = dist(goal_x, curr_x, goal_y, curr_y);
  cout << "goalRadius " << goalRadius << " " << goal_x << " " << goal_y <<" " << curr_x << " " << curr_y << endl;
  if ( goalRadius < 1.0 )
    return TRUE;
  else
    return FALSE;
}


// Set Look ahead distance
double look_ahead(double curr_speed){
  if ( curr_speed < 2 )
    look_ahead_dist = 0.5;
  if ( curr_speed > 2 && curr_speed < 10 )
    look_ahead_dist = 0.5;
  if ( curr_speed > 10 )
    look_ahead_dist = 0.5;
  
  return look_ahead_dist;
}

// calculate desired steering angle
double calculate_desired_steer(double carrot_x, double carrot_y, double curr_x, double curr_y, double curr_yaw){
  ////double delta_x = carrot_y - curr_y;
  double delta_y = carrot_y - curr_y;
  // l = look ahead distance from robot to carrot
  cout << "curr_x" << curr_x << " curr_y " << curr_y << " carrot_x " << carrot_x << " carrot_y " << carrot_y << endl;
  double l = sqrt(((carrot_x - curr_x) * (carrot_x - curr_x)) + ((carrot_y - curr_y) * (carrot_y - curr_y)));
  cout << "l = " << l << " " << delta_y<< endl;
  
  double theta = asin( delta_y / l ) ; //gazebo
  if(curr_x > carrot_x && theta > 0){
    theta = 3.14 - theta;
  }
  else if(curr_x > carrot_x && theta < 0){
    theta = - 3.14 - theta;
  }
  cout << " theta raw = " << theta << endl;
  //if (theta > 3.14 && theta < (3.14+1.57))
  //  theta = theta - 3.14;
  //if (theta < -3.14)
  //  theta = 3.14 + (theta + 3.14) + 1.57;
  //cout << " theta = " << theta << endl;
    double ds = curr_yaw - theta;

  cout << "curr_yaw = " << curr_yaw << endl;
  cout <<"desired raw = "<< ds<< endl;
  if (ds >= 3.14){
    ds = -(6.28 - ds);
   }
  if (ds < -3.14){
    ds = (6.28 + ds);
  }
  return ds;
}


// PD controller for steering
double PD_controller(double desired_steer, double diff_error){
  double kp = 1.0;
  double kd = 0.09;
  
  double steerOutput = -(kp * desired_steer) - (kd * diff_error);
  if( (steerOutput < 0.1) && (steerOutput > 0.0) )
    steerOutput = 0.1;
  else if( (steerOutput > -0.1) && (steerOutput < 0.0) )
  	steerOutput = -0.1;
  else if( steerOutput < -0.7 )
  	steerOutput = -0.7;
  else if( steerOutput > 0.7 )
  	steerOutput = 0.7;
  
  return steerOutput;
}

// open loop controller for speed
double speedControl(double speed_percentage, double prev_speed){
	double max_speed = 1.0;
	double max_speed_reverse = -1.0;
	double stop = 0;
	
	double desired_speed = speed_percentage * max_speed;
	
	double speed_tmp = 0.9 * prev_speed + 0.1 * desired_speed;
	
	if( (speed_tmp < -max_speed_reverse) && (speed_tmp < stop) )
		speed_tmp = -max_speed_reverse;
	else if( (speed_tmp > max_speed) && (speed_tmp > stop) )
		speed = max_speed;
		
	return speed_tmp;
	
}


void PidController::control(const radl_in_t * in, radl_out_t * out){
    Msg msg;
    
    pose p = poseUpdate(in);
    //cout << "Pose p is: " << p.x << " , " << p.y << endl;
    myrobot.x = p.x;
    myrobot.y = p.y;
    myrobot.yaw = p.yaw;

    if(myrobot.x != 0.0 || myrobot.y != 0.0){
    // cout << "myrobot is: " << myrobot.x << " , " << myrobot.y << endl;
	    if ( 1 == flag ){
	      double goal_x = path_x.back();
	      double goal_y = path_y.back();
	      //double goal_x = path_x[0];
        //double goal_y = path_y[0];
	      if ( goalCheck(goal_x, goal_y, myrobot.x, myrobot.y) ){
		flag = 0;
		msg.angle = 0.0;
		msg.velocity = 0.0;
		cout << "GOAL REACHED" << endl;
		out->drive_parameters->velocity = msg.velocity;
		out->drive_parameters->angle = msg.angle;
		//exit(0);
			return;
	      }
	      
	      if ( (STOP_Dist != -1) && (STOP_Dist < 150) ){
      		flag = 0;
      		msg.angle = 0.0;
      		msg.velocity = 0.0;
      		cout << "STOP Detected" << endl;
      		out->drive_parameters->velocity = msg.velocity;
      		out->drive_parameters->angle = msg.angle;
      		exit(0);
	      }  
	      
	      // Set look ahead distance
	      look_ahead_dist = look_ahead(myrobot.speed);
	      //cout << "Look Ahead = " << look_ahead_dist << "myrobot.speed = " << myrobot.speed << endl;
	      
	      // find starting point along the path --> closest point on the path
	      
	      vector<double> startingPoints;

	      for( int i = 0; i < planner_coord; i ++ ){
      		startingPoints.push_back(0.0);
      		double d = dist(path_x[i], myrobot.x, path_y[i], myrobot.y);
      		startingPoints[i] = d;
	      }
	      
	      vector<double>::iterator it = startingPoints.begin();
	      //cout << " startingPoints first is : " << *it << endl;
	      //cout << "startingPoints size is: " << startingPoints.size() << endl;
	      
	      int startIndex = (int)(min_element(it, startingPoints.end()) - it);
	      //int startIndex = minIndex(it, startingPoints.end());
	      cout << "startIndex = " << startIndex << endl;
	      
	      // Find carrot point on path
	      vector<double> possible_l;
	      for( int i=0; i < planner_coord; i ++ ){
      		possible_l.push_back(0.0);
	      }
	      
	      for ( int i = startIndex; i < planner_coord; i ++ ){
		double d = dist(path_x[i], myrobot.x, path_y[i], myrobot.y);
		double difference = abs(d - look_ahead_dist);
		possible_l[i] = difference;
	      }
	      
	      double l_carrot = *min_element((possible_l.begin()+startIndex), possible_l.end());
	      //cout << "l_carrot = " << l_carrot << endl;
	      
	      int carrotIndex;
	      
	      for ( int i = 0; i < planner_coord; i++ ){
		if ( i < startIndex ){
		  continue;
		}
		else if ( i >= startIndex ){
		  if ( possible_l[i] == l_carrot ){
		    carrotIndex = i;
		  }
		}
	      }
	      
	      cout << "carrotIndex = " << carrotIndex << endl;
	      
	      double carrot_x = path_x[carrotIndex];
	      double carrot_y = path_y[carrotIndex];
	      cout << "carrot_x = " << carrot_x << "carrot_y = " << carrot_y << endl;
	      
	      // Find desired steer value
	      double desired_steer = calculate_desired_steer(carrot_x, 
		    carrot_y, myrobot.x, myrobot.y, 
		    myrobot.yaw); // TEST IF NEGATION IS NEEDED IN GAZEBO
	      cout << "desired_steer = " << desired_steer << endl;
	      
	      // Calculate steering output to publish using pd controller
	      double diff_error = desired_steer - prev_steer;
	      
	      double steerOutput = PD_controller(desired_steer, diff_error);
	      prev_steer = steerOutput;
	      
	      // Speed control
	      speed = speedControl(speeds[carrotIndex], prev_speed);
	      prev_speed = speed;
	      myrobot.speed = speed;
	      
	      // Publish message
	      msg.angle = steerOutput;
	      msg.velocity = 7;
	      cout << "flag = 1, vel = " << msg.velocity << ", angle = " << msg.angle << endl;
	      out->drive_parameters->velocity = msg.velocity;
	      out->drive_parameters->angle = msg.angle;
	    }
    }
    // if flag == 0;
    msg.angle = 0.0;
    msg.velocity = 0.0;
    
}

//void camera(const radl_in_t * in){
//  STOP_Dist = in->stop_sign_distance->data;
//  STOP_Dist = 200;
//  cout << "stop dist is : " << STOP_Dist << endl;
//}

/*void PidController::desired_track(const radl_in_t * in){
    path_y.clear(); path_x.clear(); speeds.clear();

    vector<double> coords (10);

  copy ( in->path_planner->data.begin(), in->path_planner->data.end(), coords.begin() );
  
    vector<double>::iterator it = coords.begin();
    
    planner_coord = 0;
    
    while( it != coords.end() ){
      path_x.push_back(*it);
      path_y.push_back(*(++it));
      speeds.push_back(*(++it));
      planner_coord ++;
    }
    flag = 1;
}*/

PidController::PidController (){
  string line;
  const char *tmp;
  ifstream myfile ("/home/f1/catkin_ws/src/race/src/path.txt");
  cout<<"/home/f1/catkin_ws/src/race/src/path.txt" << endl;
  if (myfile.is_open())
  {
    while (getline (myfile,line)) {
      if(line == "")
        break;
      tmp = line.c_str();
      path_x.push_back((strtod(tmp,NULL)));
      planner_coord ++;
      getline (myfile,line);
      tmp = line.c_str();
      path_y.push_back(strtod(tmp,NULL));
      getline (myfile,line);
      tmp = line.c_str();
      speeds.push_back(strtod(tmp,NULL));
    }
    myfile.close();
    reverse(path_x.begin(), path_x.end());
    reverse(path_y.begin(), path_y.end());
    reverse(speeds.begin(), speeds.end());

    cout << "Path reading finished" << endl;
  } else cout << "Unable to open file\n"; 
  flag = 1;
	cout << "hello world from pidController" << endl;
}


void PidController::step(const radl_in_t * in, const radl_in_flags_t* inflags,
                       radl_out_t * out, radl_out_flags_t* outflags){
    //desired_track(in);
    //cout << "incoming pose msg x" << in->slam_out_pose->position_x << endl;

    control(in, out);
    //camera(in);
}
