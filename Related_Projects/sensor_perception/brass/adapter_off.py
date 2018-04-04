#!/usr/bin/env python

# rosrun monitor adapter.py [adapt_on]; 

import rospy
import sys
from rlskf import rlse_online
from navigation_phase1.msg import ekfparam
from sensor_msgs.msg import Imu
import numpy as np
import threading

# global variables
imu_adapted_pub = None
last_imu_x_acc, last_imu_y_acc, last_imu_wz = 0.0, 0.0, 0.0
x_err, y_err, wz_err = 0.0, 0.0, 0.0
meas_x_err, meas_y_err, meas_wz_err = 0.0, 0.0, 0.0
meas_x_err_old, meas_y_err_old, meas_wz_err_old = 0.0, 0.0, 0.0
N = 2		# number of weights to learn
weights_x = np.zeros( (N,1) ) 
weights_y = np.zeros( (N,1) )  
weights_wz = np.zeros((N,1) )
P_x = np.eye(N,N)*0.9
P_y = np.eye(N,N)*0.9
P_wz = np.eye(N,N)*0.9
adapt_on = 0  	# adaptation off by default
useZeroValue = True
converged = False
NUM_SAMPLES = 20 # to compute running average, approximately
WEIGHT = 1.0/NUM_SAMPLES  # x = WEIGHT*new_data + (1-WEIGHT)*x
n_cont_positives = 0        # number of continuous positives in err

# ----------------------------------------------------------------------
def regularize( weights ):
    weights[0,0] = 0.9*weights[0,0]
    weights[1,0] = 1.1*weights[0,0]
# ----------------------------------------------------------------------

# ----------------------------------------------------------------------
def callbackEKF( msg ):
    # Update the weights that are used to compute error
    # x_err' = a*x_err + b*est_error + c*1  -- we update the weights here, and compute new x_err
    # use rlse_online to get a,b,c
    global weights_x, weights_y, P_x, P_y, x_err, y_err, meas_x_err, meas_y_err
    global last_imu_x_acc, last_imu_y_acc, meas_x_err_old, meas_y_err_old
    global weights_wz, P_wz, wz_err, meas_wz_err, last_imu_wz, meas_wz_err_old
    global adapt_on, converged, useZeroValue
    lock2.acquire()
    if abs( msg.vb0x ) < 0.1:    # odometer/EKF says we are hardly moving; find IMU bias
        weights_x[1,0] = (1-WEIGHT) * weights_x[1,0] + WEIGHT * (0 - last_imu_x_acc)
        weights_y[1,0] = (1-WEIGHT) * weights_y[1,0] + WEIGHT * (0 - last_imu_y_acc)
        weights_wz[1,0] = (1-WEIGHT) * weights_wz[1,0] + WEIGHT * (0 - last_imu_wz)
        x_err_now, y_err_now, wz_err_now = 0, 0, 0
        x_err = np.dot( np.array( [ [x_err_now, 1] ]), weights_x )[0,0]
        y_err = np.dot( np.array( [ [y_err_now, 1] ]), weights_y )[0,0]
        wz_err = np.dot( np.array( [ [wz_err_now, 1] ]), weights_wz )[0,0]
        useZeroValue = True
        print 'C',      # calibrating - learning bias
    else:
        useZeroValue = False
        headingt = msg.hdgt
        cHDGt = np.cos( headingt )
        sHDGt = np.sin( headingt )
        rotateBack = np.array([[cHDGt, -sHDGt], [sHDGt,  cHDGt]])
        est_ax, est_ay = np.dot( rotateBack, np.array([ [msg.aN], [msg.aE] ]) )
        est_wz = msg.hdgdot
        est_ax = min( 3,  max( -3, est_ax ) )
        est_ay = min( 1,  max( -1, est_ay ) )
        est_wz = min( 1,  max( -1, est_wz ) )
        if abs(est_ax) < 0.01 and abs(est_ay) < 0.01:
            est_ax, est_ay = 0.0, 0.0
        #print 'est_ax = ', est_ax
        # x_data = np.array( [ [meas_x_err, meas_x_err_old, 1] ] )
        # y_data = np.array( [ [meas_y_err, meas_x_err_old, 1] ] )
        x_data = np.array( [ [meas_x_err, 1] ] )
        y_data = np.array( [ [meas_y_err, 1] ] )
        wz_data = np.array( [ [meas_wz_err, 1] ] )
        x_err_now = est_ax - last_imu_x_acc
        y_err_now = est_ay - last_imu_y_acc
        wz_err_now = est_wz - last_imu_wz
        # print 'estimation error at t-1 acc_x {0} and wz'.format(x_err - x_err_now)
        print 'L {0}, {1},'.format(x_err - x_err_now, wz_err - wz_err_now)
        # print 'est {0}, {1}; actual {2} {3}'.format(x_err, wz_err, x_err_now, wz_err_now)
        # print 'hdgt={0}, hdg0={1}'.format(msg.hdgt, msg.hdg0)
        old_weights_x = np.array( weights_x )
        old_weights_y = np.array( weights_y )
        old_weights_wz = np.array( weights_wz )
        rlse_online( x_data, np.array( [x_err_now] ), weights_x, P_x)
        rlse_online( y_data, np.array( [y_err_now] ), weights_y, P_y)
        rlse_online( wz_data, np.array( [wz_err_now] ), weights_wz, P_wz)
        meas_x_err_old, meas_y_err_old, meas_wz_err_old = meas_x_err, meas_y_err, meas_wz_err
        meas_x_err, meas_y_err, meas_wz_err = x_err_now, y_err_now, wz_err_now
        diff_weights = np.sum( abs(weights_x - old_weights_x) + abs(weights_wz - old_weights_wz) )/np.sum(abs(weights_x)+abs(weights_wz))
        converged = True if diff_weights <= 0.1 else False
        '''if not converged:   # regularize while not converged
            regularize( weights_x )
            regularize( weights_y )
            regularize( weights_wz ) '''
        x_err = np.dot( np.array( [ [x_err_now, 1] ]), weights_x )[0,0]
        y_err = np.dot( np.array( [ [y_err_now, 1] ]), weights_y )[0,0]
        wz_err = np.dot( np.array( [ [wz_err_now, 1] ]), weights_wz )[0,0]
    lock2.acquire()
    # return x_err, y_err, wz_err, converged?
    #print 'wx ', weights_x[:,0], 'wz ', weights_wz[:,0]
# ----------------------------------------------------------------------

# ----------------------------------------------------------------------
def IMUCallback( msg ):
    global x_err, y_err, wz_err, imu_adapted_pub
    global last_imu_x_acc, last_imu_y_acc, last_imu_wz
    #global g_get_state, old_speed		# ASHISH: REMOVE
    lock.acquire()
    last_imu_x_acc = (1-WEIGHT)*last_imu_x_acc + WEIGHT*msg.linear_acceleration.x
    last_imu_y_acc = (1-WEIGHT)*last_imu_y_acc + WEIGHT*msg.linear_acceleration.y
    last_imu_wz = (1-WEIGHT)*last_imu_wz + WEIGHT*msg.angular_velocity.z
    if adapt_on == 0:
        pass
    elif converged:
        msg.linear_acceleration.x = last_imu_x_acc + x_err
        msg.linear_acceleration.y = last_imu_y_acc + y_err
        msg.angular_velocity.z = last_imu_wz + wz_err
        #msg.angular_velocity.z = 0
    elif useZeroValue: 
        msg.linear_acceleration.x = 0
        msg.linear_acceleration.y = 0
        msg.angular_velocity.z =  0
    else:
        msg.linear_acceleration.x = last_imu_x_acc  # 0
        msg.linear_acceleration.y = last_imu_y_acc  # 0
        msg.angular_velocity.z =  last_imu_wz  # 0
    lock.release()
    # ASHISH: REMOVE NEXT 8 lines, except LAST ONE ---------
    '''
    if adapt_on == 0:
        pass
    else:
        state = g_get_state(model_name="rmp440le")
        print state.twist.linear.x, state.twist.linear.y
        print state.pose.position.x, state.pose.position.y
        print state.twist.angular.z
        yaw= tf.transformations.euler_from_quaternion([state.pose.orientation.x,state.pose.orientation.y,state.pose.orientation.z,state.pose.orientation.w])[2]
        new_speed = (state.twist.linear.x, state.twist.linear.y, yaw)
        # new_speed = math.sqrt(state.twist.linear.x**2 + state.twist.linear.y**2)
        msg.linear_acceleration.x = 20*math.sqrt((new_speed[0] - old_speed[0])**2 + (new_speed[1]-old_speed[1])**2) 	# 20 Hz freq of EKF 
        #msg.angular_velocity.z = state.twist.angular.z 
        msg.angular_velocity.z = (yaw - old_speed[2])*20
        old_speed = new_speed
        # END of trial ----------
    '''
    imu_adapted_pub.publish( msg )
# ----------------------------------------------------------------------

# ----------------------------------------------------------------------
def main():
    '''publish adapted IMU'''
    global imu_adapted_pub
    # global g_get_state		# ASHISH: Checking if giving TRUE state helps!
    rospy.init_node('imu_virtual_generator_off', anonymous=True)
    imu_adapted_pub = rospy.Publisher('/imu_virtual_off', Imu, queue_size = 1)
    rospy.Subscriber("/ekf_pub_off", ekfparam, callbackEKF)
    #rospy.Subscriber("/imu_data", Imu, IMUCallback)     # For gazebo
    rospy.Subscriber("/imu", Imu, IMUCallback)      # For segway rmp
    # ASHISH: Remove lines below
    # rospy.wait_for_service("/gazebo/get_model_state")
    # g_get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    rospy.spin()

if __name__ == '__main__':
    #global adapt_on
    #if len(sys.argv) > 1:
    #   adapt_on = int(sys.argv[1])
    #else:
    adapt_on = 0
    val = 'ON' if adapt_on > 0 else 'OFF'
    print  'Started IMU Adapter; adaptation is {0}'.format(val)
    lock = threading.Lock()
    lock2 = threading.Lock()
    main()
# ----------------------------------------------------------------------
