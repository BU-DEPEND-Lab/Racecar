#include "radl__pid_controller.h"
#include "stdio.h"
using namespace std;

class PidController {
  public:
    PidController();
    void step(const radl_in_t*, const radl_in_flags_t*, radl_out_t*, radl_out_flags_t*);

  private:
    int flag;
}; 
 
