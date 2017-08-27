#include "pid_controller.h"

// include cpp libraries
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <math.h>
#include <algorithm>
#include <stdlib.h>
#include <iterator>

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


using namespace std;

double look_ahead_dist, goalRadius, speed;
double prev_steer = 0;
double prev_speed = 0;
int STOP_Dist = -1;
int flag = 0;
int planner_coord;


vector<double> path_y, path_x, speeds;


pose poseUpdate(const radl_in_t * in)
{
    // orientation data
    quaternion q;
    pose e;
    q.x = in->slam_out_pose->orientation_x;
    q.y = in->slam_out_pose->orientation_y;
    q.z = in->slam_out_pose->orientation_z;
    q.w = in->slam_out_pose->orientation_w;
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
  cout << "goalRadius" << goalRadius << endl;
  if ( goalRadius < 2.0 )
    return TRUE;
  else
    return FALSE;
}



void Robot::robot () {
    this->x = this->y = this->speed = 0;
}

Robot myrobot;

void desired_track(const radl_in_t * in){
    path_y.clear(); path_x.clear(); speeds.clear();
    vector<double> *coords = in->path_planner->data;
    vector<double>::iterator it = coords->begin();
    
    planner_coord = 0;
    
    while( it != coords->end() ){
      path_x.push_back(*it);
      path_y.push_back(*(++it));
      speeds.push_back(*(++it));
      planner_coord ++;
    }
    flag = 1;
}


// Set Look ahead distance
double look_ahead(double curr_speed){
  if ( curr_speed < 2 )
    look_ahead_dist = 1;
  if ( curr_speed > 2 && curr_speed < 10 )
    look_ahead_dist = 1;
  if ( curr_speed > 10 )
    look_ahead_dist = 1.5;
  
  return look_ahead_dist;
}

// calculate desired steering angle
double calculate_desired_steer(double carrot_x, double carrot_y, double curr_x, double curr_y, double curr_yaw){
  double delta_x = carrot_x - curr_x;
  // l = look ahead distance from robot to carrot
  cout << curr_x << " " << curr_y << " " << carrot_x << " " << carrot_y << endl;
  double l = sqrt((carrot_x - curr_x) * (carrot_x - curr_x) + (carrot_y - curr_y) * (carrot_y - curr_y));
  cout << "l = " << l << endl;
  
  double theta = -(asin( delta_x / l )) + 1.56;
  cout << " theta = " << theta << endl;
  cout << "curr_yaw = " << curr_yaw << endl;
  
  return curr_yaw - theta;
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
  else if( steerOutput < -0.5 )
  	steerOutput < -0.5;
  else if( steerOutput > 0.5 )
  	steerOutput = 0.5;
  
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


void control(const radl_in_t * in, radl_out_t * out){
    Msg msg;
    
    pose p = poseUpdate(in);
    myrobot.setPose(p.x, p.y, p.yaw);
    
    if ( 1 == flag ){
      double goal_x = path_x.back();
      double goal_y = path_y.back();
      
      if ( goalCheck(goal_x, goal_y, myrobot.x, myrobot.y) ){
        flag = 0;
        msg.angle = 0.0;
        msg.velocity = -2.0;
        cout << "GOAL REACHED" << endl;
        out->drive_parameters->velocity = msg.velocity;
        out->drive_parameters->angle = msg.angle;
        exit(0);
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
      cout << "Look Ahead = " << look_ahead_dist << "myrobot.speed = " << myrobot.speed << endl;
      
      // find starting point along the path --> closest point on the path
      
      vector<double> startingPoints;
      vector<double>::iterator it = startingPoints.begin();
      
      for( int i = 0; i < planner_coord; i ++ ){
        startingPoints.push_back(0.0);
        double d = dist(path_x[i], myrobot.x, path_y[i], myrobot.y);
        startingPoints[i] = d;
      }
      
      
      int startIndex = (int)(min_element(it, startingPoints.end()) - it);
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
      cout << "l_carrot = " << l_carrot << endl;
      
      int carrotIndex;
      
      for ( int i = 0; i < planner_coord; i++ ){
        if ( i < startIndex ){
          continue;
        }
        else if ( i > startIndex ){
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
          -myrobot.yaw); // TEST IF NEGATION IS NEEDED IN GAZEBO
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
      msg.velocity = speed;
      cout << "flag = 1, vel = " << msg.velocity << ", angle = " << msg.angle << endl;
      out->drive_parameters->velocity = msg.velocity;
      out->drive_parameters->angle = msg.angle;
    }
    
    // if flag == 0;
    msg.angle = 0.0;
    msg.velocity = 0.0;
    
   
    
    
    
    
    
}

void camera(const radl_in_t * in){
  
}



void PidController::step(const radl_in_t * in, const radl_in_flags_t* inflags,
                       radl_out_t * out, radl_out_flags_t* outflags){
    desired_track(in);
    control(in, out);
    camera(in);
}



