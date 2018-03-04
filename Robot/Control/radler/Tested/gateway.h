#include RADL_HEADER
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
using namespace std;

class Gateway {
public:
	Gateway() {
		sub = h.subscribe("slam_out_pose", 1, &Gateway::subhandler, this);
		pub = h.advertise<geometry_msgs::Point>("drive_parameters", 1);
	}

	void subhandler(const geometry_msgs::PoseStamped::ConstPtr& msg) {
		this->out_posestamped = msg;
	}

  	void step(const radl_in_t* i, const radl_in_flags_t* i_f,
		radl_out_t* o, radl_out_flags_t* o_f) {
  		if (this->out_posestamped) {
  			// Forward ROS msg to Radler
  			o->slam_out_pose->position_x = this->out_posestamped->pose.position.x;
  			o->slam_out_pose->position_y = this->out_posestamped->pose.position.y;
  			o->slam_out_pose->position_z = this->out_posestamped->pose.position.z;
  			o->slam_out_pose->orientation_x = this->out_posestamped->pose.orientation.x;
  			o->slam_out_pose->orientation_y = this->out_posestamped->pose.orientation.y;
  			o->slam_out_pose->orientation_z = this->out_posestamped->pose.orientation.z;
  			o->slam_out_pose->orientation_w = this->out_posestamped->pose.orientation.w;

  			
  			// Forward Radler msg to ROS
  			geometry_msgs::Point out_msgs;
  			out_msgs.x = i->drive_parameters->velocity;
  			cout << out_msgs.x << endl;
  			out_msgs.y = i->drive_parameters->angle;
        //out_msgs.x = 0;
        //out_msgs.y = 0;

  			this->pub.publish(out_msgs);


  		} else {
      radl_turn_on(radl_STALE, &o_f->slam_out_pose);
    	}
  	}
private:
	ros::NodeHandle h;
	ros::Subscriber sub;
	ros::Publisher pub;
	geometry_msgs::PoseStamped::ConstPtr out_posestamped;

};
