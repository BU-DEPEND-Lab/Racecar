#include "radl__purePursuit.h"

class purePursuit {
  public:
    void step(const radl_in_t*, const radl_in_flags_t*, radl_out_t*, radl_out_flags_t*);
};

class robot {
    double x, y, yaw;
    double speed;
  public:
    void setPose (double new_x, double new_y, double new_yaw);
    void setSpeed(double new_speed) {speed = new_speed;}
};
