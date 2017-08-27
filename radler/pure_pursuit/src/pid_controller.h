#include "radl__pid_controller.h"

class PidController {
  public:
    void step(const radl_in_t*, const radl_in_flags_t*, radl_out_t*, radl_out_flags_t*);
};

class Robot {
  public:
	void robot ();
    void setPose (double new_x, double new_y, double new_yaw);
    void setSpeed(double new_speed) {speed = new_speed;}

	double x, y, yaw;
    double speed;
};
