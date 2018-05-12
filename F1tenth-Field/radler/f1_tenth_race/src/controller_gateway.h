#include RADL_HEADER
#include "ros/ros.h"

using namespace std;

class ControllerGateway {
  private:
	  ros::NodeHandle h;
	  ros::Subscriber scan_sub;
	  ros::Publisher talker_pub;
  public:
	  ControllerGateway();
  	void step(const radl_in_t*, const radl_in_flags_t*, radl_out_t*, radl_out_flags_t*);
};
