#include "ros/ros.h"
#include "controller_gateway.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "race/drive_param.h"

using namespace std;

geometry_msgs::PoseStamped::ConstPtr out_posestamped;
double lx=0.0, ly=0.0, lz=0.0, ax=0.0, ay=0.0, az=0.0;

void pose_subhandler(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	out_posestamped = msg;
}

void vel_subhandler(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
  lx = cmd_vel->linear.x;
  ly = cmd_vel->linear.y;
  lz = cmd_vel->linear.z;
  ax = cmd_vel->angular.x;
  ay = cmd_vel->angular.y;
  az = cmd_vel->angular.z;
}

ControllerGateway::ControllerGateway() 
{
	pose_sub = h.subscribe("/slam_out_pose", 1, pose_subhandler);
	vel_sub = h.subscribe("/cmd_vel", 1, vel_subhandler);
	// pub = h.advertise<geometry_msgs::Point>("drive_parameters", 1);
	pub = h.advertise<race::drive_param>("/drive_parameters", 1);
}

void ControllerGateway::step(const radl_in_t* i, const radl_in_flags_t* i_f, radl_out_t* o, radl_out_flags_t* o_f) 
{
  race::drive_param out_msgs;

  if (out_posestamped) {
  	// Forward ROS msg to Radler
    o->slam_out_pose->position_x = out_posestamped->pose.position.x;
  	o->slam_out_pose->position_y = out_posestamped->pose.position.y;
  	o->slam_out_pose->position_z = out_posestamped->pose.position.z;
  	o->slam_out_pose->orientation_x = out_posestamped->pose.orientation.x;
  	o->slam_out_pose->orientation_y = out_posestamped->pose.orientation.y;
  	o->slam_out_pose->orientation_z = out_posestamped->pose.orientation.z;
  	o->slam_out_pose->orientation_w = out_posestamped->pose.orientation.w;

    o->nav_cmd_vel->linear_x = lx;
    o->nav_cmd_vel->linear_y = ly;
    o->nav_cmd_vel->linear_z = lz;
    o->nav_cmd_vel->angular_x = ax;
    o->nav_cmd_vel->angular_y = ay;
    o->nav_cmd_vel->angular_z = az;
  			
  	// Forward Radler msg to ROS
  	// geometry_msgs::Point out_msgs;
  	// out_msgs.x = i->drive_parameters->velocity;
  	// out_msgs.y = i->drive_parameters->angle;
//    if (!radl_is_timeout(i_f->drive_parameters))
//    {
  	  out_msgs.velocity = i->drive_parameters->velocity;
  	  out_msgs.angle = i->drive_parameters->angle;
  	  pub.publish(out_msgs);
/*    }
    else 
    {
      cout << "STOP due to drive_parameters TIMEOUT" << endl; 
  	  out_msgs.velocity = 0.0; 
  	  out_msgs.angle = 0.0;
  	  pub.publish(out_msgs);
    } */
    if (*RADL_THIS->print_debug)
    {
      cout << "velocity = " << out_msgs.velocity << " angle = " << out_msgs.angle << endl;
    }
 	} else {
    radl_turn_on(radl_STALE, &o_f->slam_out_pose);
 	}
}
