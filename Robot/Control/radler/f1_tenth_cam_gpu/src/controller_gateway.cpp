#include "ros/ros.h"
#include "controller_gateway.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "race/drive_param.h"

using namespace std;

geometry_msgs::PoseStamped::ConstPtr out_posestamped;

void subhandler(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{
	out_posestamped = msg;
}

ControllerGateway::ControllerGateway() 
{
	sub = h.subscribe("/slam_out_pose", 1, subhandler);
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
  			
  	// Forward Radler msg to ROS
  	// geometry_msgs::Point out_msgs;
  	// out_msgs.x = i->drive_parameters->velocity;
  	// out_msgs.y = i->drive_parameters->angle;
    if (!radl_is_timeout(i_f->drive_parameters)) 
    {
  	  out_msgs.velocity = i->drive_parameters->velocity;
  	  out_msgs.angle = i->drive_parameters->angle;
  	  pub.publish(out_msgs);
    }
    else 
    {
      cout << "STOP due to drive_parameters TIMEOUT" << endl; 
  	  out_msgs.velocity = 0.0; 
  	  out_msgs.angle = 0.0;
  	  pub.publish(out_msgs);
    }
 	} else {
    radl_turn_on(radl_STALE, &o_f->slam_out_pose);
 	}
}
