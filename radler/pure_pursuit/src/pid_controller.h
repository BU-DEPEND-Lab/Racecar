#include "radl__pid_controller.h"
#include "stdio.h"
#include <vector>
using namespace std;

class PidController {
  public:
    PidController ();
    void step(const radl_in_t*, const radl_in_flags_t*, radl_out_t*, radl_out_flags_t*);

  private:
  	void control(const radl_in_t * in, radl_out_t * out);
  	void desired_track(const radl_in_t * in);

  	vector<double> path_x, path_y, speeds;
  	int flag, planner_coord;
};


class Robot {
  public:
	void robot ();
    void setPose (double new_x, double new_y, double new_yaw) {x = new_x; y = new_y; yaw = new_yaw;}
    void setSpeed(double new_speed) {speed = new_speed;}

	double x, y, yaw;
    double speed;
};
