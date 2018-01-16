#include "ros/ros.h"
#include "key.h"
#include <iostream>
#include <ncurses.h>
#include <cctype>

using namespace std; 

Key::Key() 
{
  WINDOW* mainWin(initscr());
  cbreak();
  noecho();
  keypad(mainWin, true);

  printw("- Controls -                     \n");
  printw("   upwards/downwards arrow     -- Up/Down            \n");
  printw("   leftwards/rightwards arrow  -- Turn Left/Right    \n");
  printw("   HOME                        -- Resart             \n");
  printw("   Any other keys              -- Emergency Stop     \n");
}

void Key::step(const radl_in_t * in, const radl_in_flags_t* iflags,
               radl_out_t * out, radl_out_flags_t* oflags) 
{
	int key; 
	key = getch(); 

  erase();
  move(0,0);

	if ( *RADL_THIS->print_debug ) {
    switch(key) {
      case KEY_UP:
        printw("UP \n");
        break;
      case KEY_DOWN:
        printw("DOWN \n");
        break;
      case KEY_LEFT:
        printw("LEFT \n");
        break;
      case KEY_RIGHT:
        printw("RIGHT \n");
        break;
      case KEY_HOME:
        printw("RESTART \n");
        break;
      default:
        printw("STOP \n");
        break;
    }
	}

	out->key_event->key = key;
}
