#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block ros_image_filters_0706/Subscribe
extern SimulinkSubscriber<sensor_msgs::Image, SL_Bus_ros_image_filters_0706_sensor_msgs_Image> Sub_ros_image_filters_0706_335;

// For Block ros_image_filters_0706/Monitor/Publish
extern SimulinkPublisher<sensor_msgs::Image, SL_Bus_ros_image_filters_0706_sensor_msgs_Image> Pub_ros_image_filters_0706_490;

// For Block ros_image_filters_0706/Parameter Publisher/Publish2
extern SimulinkPublisher<geometry_msgs::Point, SL_Bus_ros_image_filters_0706_geometry_msgs_Point> Pub_ros_image_filters_0706_588;

void slros_node_init(int argc, char** argv);

#endif
