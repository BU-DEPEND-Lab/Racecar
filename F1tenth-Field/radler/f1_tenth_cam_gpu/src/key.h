#include RADL_HEADER

#define KEY_UP        24  
#define KEY_DOWN      25 
#define KEY_LEFT      27
#define KEY_RIGHT     26

class Key {
 	public:
		Key();
  		void step(const radl_in_t*, const radl_in_flags_t*, radl_out_t*, radl_out_flags_t*);
};
