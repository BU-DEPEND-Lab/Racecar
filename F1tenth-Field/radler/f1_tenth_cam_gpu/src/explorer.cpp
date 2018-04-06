#include "explorer.h"

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

#define ESTOP   0
#define AUTO    1
#define TELEOP  2

int STOP_Dist = -1;

double START_SPEED = 8.0;
double START_SPEED_B = -9.0;

double prev_speed = START_SPEED;
double prev_steer = 0.0;
double prev_x = 0.0; 
double prev_y = 0.0; 

int mode = ESTOP;

double distance(double x1, double x2, double y1, double y2)
{
  double d = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
  return d;
}


void Explorer::step(const radl_in_t * in, const radl_in_flags_t* inflags, radl_out_t * out, radl_out_flags_t* outflags)
{
  double curr_speed = prev_speed;
  double curr_steer = prev_steer;
  int stop_sign_distance = in->stop_sign_distance->data;
  if (stop_sign_distance < 150 && stop_sign_distance != -1) {
    curr_speed = 0.0;
    curr_steer = 0.0;
    cout << "STOPSIGN at " << stop_sign_distance << endl;
    mode = ESTOP;
  }

	if ( !radl_is_stale(inflags->key_input) && !radl_is_timeout(inflags->key_input) ) {  
    switch (in->key_input->key) { 
      case KEY_UP:
        if (curr_speed < START_SPEED && curr_speed >= 0)  
          curr_speed = START_SPEED;
        else 
          curr_speed += 0.1;
        cout << "Turn UP        ";
        mode = TELEOP;
        break;
      case KEY_DOWN:
        if (curr_speed > START_SPEED_B && curr_speed <= 0)
          curr_speed = START_SPEED_B;
        else 
          curr_speed -= 0.1;
        cout << "Turn DOWN      ";
        mode = TELEOP;
        break;
      case KEY_RIGHT:
        curr_steer += 0.5;
        cout << "Turn RIGHT     ";
        mode = TELEOP;
        break;
      case KEY_LEFT:
        curr_steer -= 0.5;
        cout << "Turn LEFT      ";
        mode = TELEOP;
        break;
      case KEY_HOME: 
        curr_speed = START_SPEED;
        curr_steer = 0.0;
        cout << "RESTART        ";
        mode = AUTO;
        break;
      default: 
        curr_speed = 0.0;
        curr_steer = 0.0;
        cout << "Emergency STOP ";
        mode = ESTOP;
        break;
    }
  }

  if (mode == AUTO) {
    double current_x = in->slam_out_pose->position_x;
    double current_y = in->slam_out_pose->position_y;
    double l = distance(prev_x, current_x, prev_y, current_y);

    cout << "Travel distance = " << l << endl;
    if (l > 0.01) {  // already moving && change direction
      if (in->nav_cmd_vel->linear_x > 0 && prev_speed < START_SPEED_B)  
        curr_speed = START_SPEED; 
      else if (in->nav_cmd_vel->linear_x < 0 && prev_speed > START_SPEED)
        curr_speed = START_SPEED_B;
    }
    else {
      if (prev_speed > START_SPEED) {  // forward 
        if (in->nav_cmd_vel->linear_x > 0) 
          curr_speed += 0.1; 
        else 
          curr_speed = START_SPEED_B; 
      }
      else if (prev_speed < START_SPEED_B) { 
        if (in->nav_cmd_vel->linear_x < 0) 
          curr_speed -= 0.1; 
        else 
          curr_speed = START_SPEED;
      }
    }
    curr_steer = in->nav_cmd_vel->angular_z;
  }

  if (curr_speed > *RADL_THIS->max_speed)
    curr_speed = *RADL_THIS->max_speed; 
  if (curr_speed < *RADL_THIS->max_speed_b)
    curr_speed = *RADL_THIS->max_speed_b; 

  if (curr_speed != prev_speed) {
    switch (mode) {
      case ESTOP:
        cout << "ESTOP  ";
        break;
      case AUTO:
        cout << "AUTO   ";
        break;
      case TELEOP:
        cout << "TELEOP ";
        break;
      default:
        cout << "ERROR  ";
        break;
    }
    cout << " velocity = " << curr_speed << " angle = " << curr_steer  << endl;
  }
  out->drive_parameters->velocity = curr_speed;
  out->drive_parameters->angle = curr_steer;
  prev_speed = curr_speed;
  prev_steer = curr_steer;
  prev_x = in->slam_out_pose->position_x;
  prev_y = in->slam_out_pose->position_y;

  return;
}

Explorer::Explorer ()
{
}
