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
#include <ncurses.h> 

using namespace std;

#define	TRUE	1
#define	FALSE	0

struct quaternion{
  double x, y, z, w;
};

struct pose{
  double x, y, roll, pitch, yaw;
};

double look_ahead_dist, goalRadius, speed;
double prev_steer = 0.0;
double prev_speed = 0.0;
int STOP_Dist = -1;
int planner_coord = 0;

int minIndex(vector<double>::iterator start, vector<double>::iterator end) 
{
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
  // cout << counter << endl;
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
	  
  if (*RADL_THIS->print_debug) 
  {
    cout << "-----------------------------------" << endl;
    cout << "q.x = " << q.x << ", q.y = " << q.y << ", q.z = " << q.z << ", q.w = " << q.w << endl;
    cout << "e.roll = " << e.roll << ", e.pitch = " << e.pitch << ", e.yaw = " << e.yaw << endl;
    cout << "e.x = " << e.x << ", e.y = " << e.y << endl;
  }

	return e;
}

double dist(double x1, double x2, double y1, double y2)
{
  double d = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
  return d;
}

bool goalCheck(double goal_x, double goal_y, double curr_x, double curr_y)
{
  double goalRadius = dist(goal_x, curr_x, goal_y, curr_y);

  if (*RADL_THIS->print_debug) 
    cout << "goalRadius " << goalRadius << endl;

  if ( goalRadius < 2.0 )
    return TRUE;
  else
    return FALSE;
}

void Robot::robot () 
{
  this->x = this->y = this->speed = 0;
}

Robot myrobot;

// Set Look ahead distance
double look_ahead(double curr_speed)
{
  if ( curr_speed < 2 )
    look_ahead_dist = 1;
  if ( curr_speed > 2 && curr_speed < 10 )
    look_ahead_dist = 1;
  if ( curr_speed > 10 )
    look_ahead_dist = 1.5;
  
  return look_ahead_dist;
}

// calculate desired steering angle
double calculate_desired_steer(double carrot_x, double carrot_y, double curr_x, double curr_y, double curr_yaw)
{
  double delta_x = carrot_x - curr_x;
  // l = look ahead distance from robot to carrot
  double l = sqrt((carrot_x - curr_x) * (carrot_x - curr_x) + (carrot_y - curr_y) * (carrot_y - curr_y));
  
  if (*RADL_THIS->print_debug) 
    cout << "look ahead dist from robot to carrot = " << l << endl;

  if (l == 0) 
    return curr_yaw;
  
  double theta = -(asin( delta_x / l )) + 1.56;
  if (*RADL_THIS->print_debug) 
  {
    cout << "theta = " << theta << endl;
    cout << "curr_yaw = " << curr_yaw << endl;
  }

  return curr_yaw - theta;
}

// PD controller for steering
double PD_controller(double desired_steer, double diff_error)
{
  double kp = 1.0;
  double kd = 0.09;
  
  double steerOutput = -(kp * desired_steer) - (kd * diff_error);
  if( (steerOutput < 0.1) && (steerOutput > 0.0) )
    steerOutput = 0.1;
  else if( (steerOutput > -0.1) && (steerOutput < 0.0) )
    steerOutput = -0.1;
  else if( steerOutput < -0.5 )
    steerOutput = -0.5;
  else if( steerOutput > 0.5 )
    steerOutput = 0.5;
  
  if (*RADL_THIS->print_debug) 
    cout << "steerOutput = " << steerOutput << ", desired_steer = " << desired_steer << ", diff_error = " << diff_error << endl;

  return steerOutput;
}

// open loop controller for speed
double speedControl(double speed_percentage, double prev_speed)
{
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

void PidController::control(const radl_in_t * in, const radl_in_flags_t* iflags, radl_out_t * out, radl_out_flags_t* oflags)
{
	if ( !radl_is_stale(iflags->key_input) && !radl_is_timeout(iflags->key_input) ) {  
    switch (in->key_input->key) { 
      case KEY_UP:
        out->drive_parameters->velocity = prev_speed + 0.1;
        out->drive_parameters->angle = prev_steer;
        cout << "Turn UP        ";
        break;
      case KEY_DOWN:
        out->drive_parameters->velocity = prev_speed - 0.1;
        out->drive_parameters->angle = prev_steer;
        cout << "Turn DOWN      ";
        break;
      case KEY_RIGHT:
        out->drive_parameters->velocity = prev_speed;
        out->drive_parameters->angle = prev_steer + 0.1;
        cout << "Turn RIGHT     ";
        break;
      case KEY_LEFT:
        out->drive_parameters->velocity = prev_speed;
        out->drive_parameters->angle = prev_steer - 0.1;
        cout << "Turn LEFT      ";
        break;
      case KEY_HOME: 
        out->drive_parameters->velocity = 0.0;
        out->drive_parameters->angle = 0.0;
        cout << "RESTART        ";
        flag = 1;
        break;
      default: 
        out->drive_parameters->velocity = 0.0;
        out->drive_parameters->angle = 0.0;
        cout << "Emergency STOP ";
        flag = 0;
        break;
    }

    if (out->drive_parameters->velocity > *RADL_THIS->max_speed)
      out->drive_parameters->velocity = *RADL_THIS->max_speed; 

    cout << "velocity = " << out->drive_parameters->velocity << " angle = " << out->drive_parameters->angle  << endl;
    prev_speed = out->drive_parameters->velocity;
    prev_steer = out->drive_parameters->angle;

    return;
  }
  else {
    pose p = poseUpdate(in);
    myrobot.setPose(p.x, p.y, p.yaw);
    
    if ( 1 == flag ){
      double goal_x = path_x.back();
      double goal_y = path_y.back();
      
      if ( goalCheck(goal_x, goal_y, myrobot.x, myrobot.y) ){
        flag = 0;
        cout << "GOAL REACHED" << endl;
        out->drive_parameters->velocity = 0.0; 
        out->drive_parameters->angle = 0.0; 
	      return;
      } 
      
      if ( (STOP_Dist != -1) && (STOP_Dist < 150) ){
        cout << "STOP Detected " << STOP_Dist << endl;
        out->drive_parameters->velocity = 0.0;
        out->drive_parameters->angle = 0.0; 
	      return;
      }  
      
      // Set look ahead distance
      look_ahead_dist = look_ahead(myrobot.speed);
      if (*RADL_THIS->print_debug) 
        cout << "Look Ahead = " << look_ahead_dist << " myrobot.speed = " << myrobot.speed << endl;
      
      // find starting point along the path --> closest point on the path
      
      vector<double> startingPoints;

      for ( int i = 0; i < planner_coord; i++ ){
        startingPoints.push_back(0.0);
        double d = dist(path_x[i], myrobot.x, path_y[i], myrobot.y);
        startingPoints[i] = d;
      }
      //for ( int i = 0; i < planner_coord; i++ ) 
      //  cout << "startingPoints[" << i << "] " << startingPoints[i] << endl;
      
      vector<double>::iterator it = startingPoints.begin();
      if (*RADL_THIS->print_debug) 
      {
        cout << "startingPoints first = " << *it << endl;
        cout << "startingPoints size = " << startingPoints.size() << endl;
      }

      int startIndex = (int)(min_element(it, startingPoints.end()) - it);
      //int startIndex = minIndex(it, startingPoints.end());
      if (*RADL_THIS->print_debug) 
        cout << "startIndex = " << startIndex << endl;
      
      // Find carrot point on path
      vector<double> possible_l;
      for ( int i=0; i < planner_coord; i ++ ){
        possible_l.push_back(0.0);
      }
      
      for ( int i = startIndex; i < planner_coord; i++ ){
        double d = dist(path_x[i], myrobot.x, path_y[i], myrobot.y);
        double difference = abs(d - look_ahead_dist);
        possible_l[i] = difference;
      }
      //for ( int i = 0; i < planner_coord; i++ ) 
      //  cout << "possible_l[" << i << "] " << possible_l[i] << endl;
      
      double l_carrot = *min_element((possible_l.begin()+startIndex), possible_l.end());
      if (*RADL_THIS->print_debug) 
        cout << "l_carrot = " << l_carrot << endl;
   
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
  
      if (*RADL_THIS->print_debug) 
        cout << "carrotIndex = " << carrotIndex << endl;
  
      double carrot_x = path_x[carrotIndex];
      double carrot_y = path_y[carrotIndex];
      if (*RADL_THIS->print_debug) 
        cout << "carrot_x = " << carrot_x << " carrot_y = " << carrot_y << endl;
  
      // Find desired steer value
      double desired_steer = calculate_desired_steer(carrot_x, carrot_y, myrobot.x, myrobot.y, -myrobot.yaw); // TEST IF NEGATION IS NEEDED IN GAZEBO
      if (*RADL_THIS->print_debug) 
        cout << "desired_steer = " << desired_steer << endl;
  
      // Calculate steering output to publish using pd controller
      double diff_error = desired_steer - prev_steer;
  
      double steerOutput = PD_controller(desired_steer, diff_error);
      prev_steer = steerOutput;
  
      // Speed control
      speed = speedControl(speeds[carrotIndex], prev_speed);
      if (speed > *RADL_THIS->max_speed)
        speed = *RADL_THIS->max_speed; 

      prev_speed = speed;
      myrobot.speed = speed;
  
      // Publish message
      cout << "velocity = " << speed << " angle = " << steerOutput << endl;
      out->drive_parameters->velocity = speed; 
      out->drive_parameters->angle = steerOutput; 
    }
  } 
}

void camera(const radl_in_t * in)
{
  STOP_Dist = in->stop_sign_distance->data;
}

PidController::PidController ()
{
  string line;
  const char *tmp;

  ifstream myfile (*RADL_THIS->path_filename);
  if (myfile.is_open())
  {
    while (getline (myfile,line)) {
      if(line == "")
         break;
      tmp = line.c_str();
      path_x.push_back(strtod(tmp,NULL));
      planner_coord ++;
      getline (myfile,line);
      tmp = line.c_str();
      path_y.push_back(strtod(tmp,NULL));
      getline (myfile,line);
      tmp = line.c_str();
      speeds.push_back(strtod(tmp,NULL));
    }
    myfile.close();
    cout << "Path reading finished " << *RADL_THIS->path_filename << " " << planner_coord << endl;
    for (int i = 0; i < planner_coord; i++) 
      cout << i << " path_x = " << path_x[i] << " path_y = " << path_y[i] << " speeds = " << speeds[i] << endl;
  } 
  else    
    cout << "Unable to open " << *RADL_THIS->path_filename << endl; 

  flag = 1;
}

void PidController::step(const radl_in_t * in, const radl_in_flags_t* inflags, radl_out_t * out, radl_out_flags_t* outflags)
{
  control(in, inflags, out, outflags);
  camera(in);
}
