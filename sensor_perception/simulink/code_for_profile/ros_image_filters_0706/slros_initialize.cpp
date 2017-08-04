#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "ros_image_filters_0706";

// For Block ros_image_filters_0706/Subscribe
SimulinkSubscriber<sensor_msgs::Image, SL_Bus_ros_image_filters_0706_sensor_msgs_Image> Sub_ros_image_filters_0706_335;

// For Block ros_image_filters_0706/Monitor/Publish
SimulinkPublisher<sensor_msgs::Image, SL_Bus_ros_image_filters_0706_sensor_msgs_Image> Pub_ros_image_filters_0706_490;

// For Block ros_image_filters_0706/Parameter Publisher/Publish2
SimulinkPublisher<geometry_msgs::Point, SL_Bus_ros_image_filters_0706_geometry_msgs_Point> Pub_ros_image_filters_0706_588;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

