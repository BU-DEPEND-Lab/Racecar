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



