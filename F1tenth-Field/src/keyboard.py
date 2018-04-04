#!/usr/bin/env python

import rospy
from race.msg import drive_param
import curses
from geometry_msgs.msg import Point
#import signal
#TIMEOUT = 0.1 # number of seconds your want for timeout
forward = 0;
left = 0;



stdscr = curses.initscr()
curses.cbreak()
stdscr.keypad(1)
rospy.init_node('keyboard_talker', anonymous=True)
#pub = rospy.Publisher('drive_parameters', drive_param, queue_size=10)
pub = rospy.Publisher('drive_parameters', Point, queue_size=10)


stdscr.refresh()

key = ''
while key != ord('q'):
#	signal.setitimer(signal.ITIMER_REAL,0.05)
#	key = input()
	key = stdscr.getch()
	stdscr.refresh()
#	signal.alarm(0)
	if key == curses.KEY_UP: 
		forward = forward + 0.1;
		stdscr.addstr(2, 20, "Up  ")
		stdscr.addstr(2, 25, '%.2f' % forward)
		stdscr.addstr(5, 20, "    ")
	elif key == curses.KEY_DOWN:
		forward = forward - 0.1; 
		stdscr.addstr(2, 20, "Down")
		stdscr.addstr(2, 25, '%.2f' % forward)
		stdscr.addstr(5, 20, "    ")
	if key == curses.KEY_LEFT:
		left = left - 0.1; 
		stdscr.addstr(3, 20, "left")
		stdscr.addstr(3, 25, '%.2f' % left)
		stdscr.addstr(5, 20, "    ")
	elif key == curses.KEY_RIGHT:
		left = left + 0.1; 
		stdscr.addstr(3, 20, "rgt ")
		stdscr.addstr(3, 25, '%.2f' % left)
		stdscr.addstr(5, 20, "    ")
	if key == curses.KEY_DC:
		left = 0.1
		forward = 0.1
		stdscr.addstr(5, 20, "Stop")
	msg = Point()
	msg.x = forward  # velocity
	msg.y = left  # angle
	pub.publish(msg)
curses.endwin()
