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

double prev_steer = 0.0;
double prev_speed = 0.0;
double error, angle;
double prev_error = 0.0;
double kp = 14.0, kd = 0.6;
double vel_input = 8.7;
double corrective_val = 11.41;

PidController::PidController() {}

void PidController::step(const radl_in_t * in, const radl_in_flags_t* iflags, radl_out_t * out, radl_out_flags_t* oflags)
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
    error = in->distance->data;
    angle = kp * error + kd * (error - prev_error);
    if (angle > 100) {
      angle = 100;
    }
    if (angle < -100) {
      angle = -100;
    }
    prev_error = error;
    
    out->drive_parameters->velocity = vel_input;
    out->drive_parameters->angle = angle + corrective_val;
  } 
}

