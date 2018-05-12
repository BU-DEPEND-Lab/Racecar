#include "dist_finder.h" 
#include "math.h"

using namespace std;

double desired_trajectory = 1.0;
int theta = 50;
double a, b, AB, AC, CD;
double error;
double tangle, alpha;

DistFinder::DistFinder () {}

void DistFinder::step(const radl_in_t * in, const radl_in_flags_t* inflags, radl_out_t * out, radl_out_flags_t* outflags)
{
//a = in->scan->data[(int) (4*(theta+45))];
//b = in->scan->data[180];
  a = in->scan_data->data[0];
  b = in->scan_data->data[1];
  tangle = (a*cos(theta) - b) / a*sin(theta);
  alpha = atan(tangle);
  AB = b*cos(alpha);
  AC = 0.05;
  CD = AB + AC*sin(alpha);
  error = CD - desired_trajectory;
  out->distance->data = error;
}

