#include "controller_gateway.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point.h"


using namespace std;

sensor_msgs::LaserScan::ConstPtr out_scan;
int theta = 50;

void scan_subhandler(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	out_scan = msg;
}


ControllerGateway::ControllerGateway() 
{
	scan_sub = h.subscribe("/scan", 1, scan_subhandler);
	talker_pub = h.advertise<geometry_msgs::Point>("/drive_parameters", 1);
}

void ControllerGateway::step(const radl_in_t* i, const radl_in_flags_t* i_f, radl_out_t* o, radl_out_flags_t* o_f) 
{
  geometry_msgs::Point out_msgs;

  if (out_scan) {
    // Forward ROS msg to Radler
    o->scan_data->data[0] = out_scan->ranges[(int) 4*(theta+45)];	
    o->scan_data->data[1] = out_scan->ranges[180];

    // Forward Radler msg to ROS
    out_msgs.x = i->drive_parameters->velocity;
    out_msgs.y = i->drive_parameters->angle;
    talker_pub.publish(out_msgs);
   
    if (*RADL_THIS->print_debug) {
      cout << "velocity = " << out_msgs.x << " angle = " << out_msgs.y << endl;
    }
  } else {
    radl_turn_on(radl_STALE, &o_f->scan_data);
  }
}
