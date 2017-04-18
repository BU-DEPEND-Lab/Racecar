#!/usr/bin/env python
import rospy
import sys
import math
import utm
import tf
import numpy as np
#from gazebo_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist, Vector3Stamped
from sensor_msgs.msg import Imu, MagneticField
from navigation_phase1.msg import ekfparam
import threading
import argparse
import time
from geometry_msgs.msg import PoseStamped
from race.msg import odometry

old_gps_t, old_odo_t, old_imu_t, old_mag_t = 0, 0, 0, 0
# global variables
gps_lock = threading.Lock()
both_done = False
moved_at_all = False
hdgdotOld = 0	# old value of estimate of hdgdot (x[5])

#Global Constants
D2R = math.pi/180.0;
g = 9.81; # (m/s^2)

## Transform functions for Frame of refernce: EKF (bx,by,bz)  <--> Gazebo/ROS (bx,by, bz) for commanding the vehicle along a 
## pre-defined trajectory. Note that EKF uses a righ-handed coordinate system like Gazebo/ROS but "rotated" 180 degrees along the 
## z axis. So EKF uses "clock-wise" "screws into the plane z" while ROS uses "counter-clockwise" "twists up the plane z".
## we set only linear velocity (VE) along ROS for moving forward along bx. So no change in tranform here
## For angular velocity (hgdot) we need to multiply by -1.0 for the axis change
## The scaling factor of below is the "magic" additional/multiple twists/sec to make the vehicle respond quickly. 
## within the time frame for showing demo in real time. This may be removed if the vehicle is made to be more responsive or
## we can "live" with the reactiveness but the "simulation"time will be a lot longer but a more accurate picture.
## Needs investigation. Since Phase IA and Phase IB are both navigation case studies with open-loop control of segway
## no need to explore at this time. But critical for Phase II and Phase III.  

#Filter associated variables
FilterFreq = 20.0 # hz
FilterTimeSteps = float(1.0/FilterFreq)
FilterTimeLength = 0.0

#Global Variables 
GPSMeas0 = {'t':0,'X':0,'Y':0,'VX':0,'VY':0,'easting':-1,'northing':-1}
GPSMeas1 = {'t':0,'X':0,'Y':0,'VX':0,'VY':0,'easting':-1,'northing':-1}
ODOMeas0 = {'t':0,'VXb':0,'wz':0}
ODOMeas1 = {'t':0,'VXb':0,'wz':0}
IMUMeas0 = {'t':0,'AXb':0,'wz':0,'yaw':0}
IMUMeas1 = {'t':0,'AXb':0,'wz':0,'yaw':0}
Head0 = {'t':0, 'hdgt':0}
Head1 = {'t':0, 'hdgt':0}
INPUT0 = {'VXb':0, 'wz':0}
INPUT1 = {'VXb':0, 'wz':0}
gps_read, gps_write = GPSMeas0, GPSMeas1
odo_read, odo_write = ODOMeas0, ODOMeas1
imu_read, imu_write = IMUMeas0, IMUMeas1
mag_read, mag_write = Head0, Head1
cmdvel_read, cmdvel_write = INPUT0, INPUT1

gps_first_read = True

EKF_SVS = ekfparam()
# initial estimate of state
EKF_SVS.pN, EKF_SVS.pE = 0, 0
EKF_SVS.vb0x, EKF_SVS.vb0y = 0, 0
EKF_SVS.hdg0, EKF_SVS.hdgb = 0, 0
EKF_SVS.abx, EKF_SVS.aby = 0, 0
EKF_SVS.gb = 0
EKF_SVS.vN, EKF_SVS.vE = 0, 0
EKF_SVS.aN, EKF_SVS.aE = 0, 0
EKF_SVS.hdgt, EKF_SVS.hdgdot = 0, 0
EKF_SVS.time = 0.0

#IMU1 Characteristics that the EKF uses for it's state space creation, time update and measurement updates computations #HIGH GRADE
#IMU1 = {'Anullshiftx':0.02,'Anullshifty':0.02,'Gnullshift':0.5*D2R,'Atau':300.0,'Asigmab1x':0.01/100.0,'Asigmab1y':0.02/100.0,'Gtau':300.0,'Gsigmab1':(1.0/3600.0)*D2R,'Awideband':0.001,'Gwideband':0.001*D2R,'MeasFreq':20.0}# Make sure it matches with frequency at urdf
IMU1 = {'Anullshiftx':0.00,'Anullshifty':0.00,'Gnullshift':0,'Atau':01.0,'Asigmab1x':0.00/100.0,'Asigmab1y':0.00/100.0,'Gtau':01.0,'Gsigmab1':(0.0/3600.0)*D2R,'Awideband':0.000,'Gwideband':0.000*D2R,'MeasFreq':20.0}# Make sure it matches with frequency at urdf

#IMU1 Characteristics that the EKF uses for it's state space creation, time update and measurement updates computations #LOW GRADE
#IMU1 = {'Anullshiftx':0.02,'Anullshifty':0.02,'Gnullshift':0.5*D2R,'Atau':300.0,'Asigmab1x':0.01,'Asigmab1y':0.02,'Gtau':300.0,'Gsigmab1':(180.0/3600.0)*D2R,'Awideband':0.001,'Gwideband':0.001*D2R,'MeasFreq':20.0}

#GPS Characteristics that the EKF uses for it's state space creation, time update and measurement updates computations
GPS = {'Poswideband':0.1,'Velwideband':0.1,'MeasFreq':1.0}# Make sure it matches with frequency at urdf
#ODO Characteristics that the EKF uses for it's state space creation, time update and measurement updates computations
ODO = {'vbxwideband':0.5,'vbywideband':0.5,'MeasFreq':10.0}# Make sure it matches with frequency at urdf

N = 6
PE_IDX, PN_IDX, VX_IDX, VXOLD_IDX, HDG_IDX, HDGDOT_IDX = range(N)
# ----------------------------------------------------------------------

# ----------------------------------------------------------------------
def GPSPosreceived( msg ):
    global gps_read, gps_write, gps_first_read, both_done, moved_at_all
    easting,northing = msg.pose.pose.position.y, msg.pose.pose.position.x # convert WGS84 to UTM

    #print easting, northing
    gps_pos_ts = float(msg.header.stamp.secs) + (float(msg.header.stamp.nsecs)/1000000000.0)
    gps_write['t'] = gps_pos_ts
    gps_write['VX'] = EKF_SVS.vN
    gps_write['VY'] = EKF_SVS.vE 
    if gps_first_read :  # first time 
        gps_read['easting'],gps_read['northing'] = (easting,northing) #origin UTM coord
        gps_write['easting'],gps_write['northing'] = (easting,northing) #origin UTM coord
        gps_first_read = False
        gps_write['X'], gps_write['Y'] = 0, 0
    elif not moved_at_all and abs(EKF_SVS.vb0x) < 0.1 :  # first time 
        gps_read['easting'],gps_read['northing'] = (easting,northing) #origin UTM coord
        gps_write['easting'],gps_write['northing'] = (easting,northing) #origin UTM coord
        gps_write['X'], gps_write['Y'] = 0, 0
    elif not moved_at_all and abs(EKF_SVS.vb0x) >= 0.1 :  # first time 
        moved_at_all = True
        gps_easting, gps_northing = gps_read['easting'],gps_read['northing'] 
        gps_write['X'], gps_write['Y'] = ((northing-gps_northing),(easting-gps_easting))
    else:
        gps_easting, gps_northing = gps_read['easting'],gps_read['northing'] 
        gps_write['X'], gps_write['Y'] = ((northing-gps_northing),(easting-gps_easting))
    #print gps_write['X'], gps_write['Y'], odo_read['VXb']
    #gps_write['t'] = gps_pos_ts
    #gps_write['VX'] = (gps_write['X'] - gps_read['X']) / (gps_pos_ts - gps_read['t'])
    #gps_write['VY'] = (gps_write['Y'] - gps_read['Y']) / (gps_pos_ts - gps_read['t'])
    #gps_write['VX'] = EKF_SVS.vN
    #gps_write['VY'] = EKF_SVS.vE 
    gps_read, gps_write  = gps_write, gps_read
    # gps_lock.acquire()
    # if both_done:
        # gps_read, gps_write  = gps_write, gps_read
        # both_done = False
    # else:
        # both_done = True
    # gps_lock.release()
    # print 'GPS mesg received', gps_write['easting'], gps_write['northing']
    # print 'gps_read=', gps_read, 'gps_write=', gps_write
# ----------------------------------------------------------------------

# ----------------------------------------------------------------------
def GPSVelreceived( msg ):
    global gps_read, gps_write, both_done
    ts = float(msg.header.stamp.secs) + (float(msg.header.stamp.nsecs)/1000000000.0)
    gps_write['t'] = ts
    gps_write['VX'] = msg.vector.x
    gps_write['VY'] = msg.vector.y
    gps_lock.acquire()
    if both_done:
        gps_read, gps_write  = gps_write, gps_read
        both_done = False
    else:
        both_done = True
    gps_lock.release()
    #print 'GPSv mesg received', msg.vector.x, msg.vector.y
# ----------------------------------------------------------------------

# ----------------------------------------------------------------------
def OdometryMeasurementAvailable( msg ):
    global odo_read, odo_write, R

    #Vmy = float(msg.twist.twist.linear.y)
    VX_body = float(msg.twist.twist.linear.x)
    odo_write['t'] = float(msg.header.stamp.secs) + (float(msg.header.stamp.nsecs)/1000000000.0)
    odo_write['VXb'] = VX_body
    odo_write['wz'] = float(msg.twist.twist.angular.z)
    odo_read, odo_write = odo_write, odo_read

    # Segway does not publish /gps_velocity_fix; so, GPSVelreceived  never called; hence we do it here
    #print 'o', msg.twist.twist.angular.z, msg.twist.twist.linear.x
# ----------------------------------------------------------------------

# ----------------------------------------------------------------------
def IMU1MeasurementAvailable(msg ):
    global imu_read, imu_write
    global mag_read, mag_write

    imu_write['t'] = float(msg.header.stamp.secs) + (float(msg.header.stamp.nsecs)/1000000000.0)
    imu_write['wz'] = float(msg.angular_velocity.z) # * HDGDOT_ROS2EKF_TRANSFORM # transform function
    imu_write['AXb'] = float(msg.linear_acceleration.x)
    if abs(msg.orientation.w) > 0.1:
        roll,pitch,yaw = tf.transformations.euler_from_quaternion([msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w])
        mag_write['t'] = imu_write['t']
        mag_write['hdgt'] = yaw
        mag_read, mag_write  = mag_write, mag_read
    imu_write['yaw'] = 0 #yaw
    imu_read, imu_write  = imu_write, imu_read

def CallbackCmdVel( msg ):
    global cmdvel_write
    v = abs(msg.linear.x)
    v1 = v-1 if v>=1 and v < 6 else (5 if v >= 6 else v)
    v2 = v1 if v >= 0 else -v1
    cmdvel_write['VXb'] = v2
    cmdvel_write['wz'] = v*msg.angular.z / 1000.0
# ----------------------------------------------------------------------
def get_sensor_data( y, dt, hdg, R):    
# y = C x   y = [gps_pe, gps_pn, gps_ve, gps_vn, odo_vx, odo_hdgdot, imu_ax, imu_hdgdot]
    # y = np.zeros( (8,1) )
    global gps_read, gps_write 
    global odo_read, odo_write 
    global imu_read, imu_write 
    global gps_first_read
    global old_gps_t, old_odo_t, old_imu_t, old_mag_t
    #print 'pN=', gps_read['X'], 'pE=', gps_read['Y'], 'gvX=', gps_read['VX'], 'gvY=', gps_read['VY']
    #print 'oVx=', odo_read['VXb'], 'oWz=', odo_read['wz'], 'iAx=', imu_read['AXb'], 'yaw=', imu_read['yaw']
    # If all sensors have stale values then return FALSE
    gps_stale = ( gps_read['t'] <= old_gps_t )
    odo_stale = ( odo_read['t'] <= old_odo_t )
    imu_stale = ( imu_read['t'] <= old_imu_t )
    mag_stale = ( mag_read['t'] <= old_mag_t )
    #print gps_stale, odo_stale, imu_stale
    if gps_stale and odo_stale and imu_stale:
        return False
    old_gps_t, old_odo_t, old_imu_t, old_mag_t = gps_read['t'], odo_read['t'], imu_read['t'], mag_read['t']
    if gps_first_read == False:     # if I have seen one GPS reading
        curr_time = max( imu_read['t'], odo_read['t'], gps_read['t'] )
        delta_t = curr_time - gps_read['t']
        y[0,0] = gps_read['X'] + delta_t * gps_read['VX']
        y[1,0] = gps_read['Y'] + delta_t * gps_read['VY']
        y[2,0] = gps_read['VX']+ delta_t  * imu_read['AXb']*math.cos( hdg )
        y[3,0] = gps_read['VY']+ delta_t  * imu_read['AXb']*math.sin( hdg )
    else:
        y[0,0] += odo_read['VXb']* math.cos(hdg)* dt
        y[1,0] += odo_read['VXb']* math.sin(-hdg)* dt    # CHECK: ASHISH
        y[2,0] = odo_read['VXb']* math.cos(hdg)
        y[3,0] = odo_read['VXb']* math.sin(-hdg)
    y[4,0] = odo_read['VXb']
    y[5,0] = odo_read['wz']
    y[6,0] = imu_read['AXb']
    y[7,0] = imu_read['wz']
    y[8,0] = mag_read['hdgt'] if mag_read['hdgt'] != 0 else hdg

    # Now set the variance of the sensor. Set GPS variance based on Odom velocity
    VX_body = odo_read['VXb']
    variance = 10/VX_body if VX_body != 0 else 500
    variance = max( variance, 25)   # variance = 5m, squared is 25.
    if gps_stale :
        R[0,0], R[1,1], R[2,2], R[3,3]  = 5000, 5000, 10, 10
    else:
        R[0,0], R[1,1] = variance, variance
        R[2,2], R[3,3] = 0.5, 0.5 # 10*variance, 10*variance
        # R[0,0], R[1,1], R[2,2], R[3,3]  = 4, 4, 20, 20
    if odo_stale :
        R[4,4], R[5,5]  = 10, 1
    else:
        R[4,4], R[5,5]  = 0.2, 0.01   # 0.01, 0.001
    if imu_stale :
        R[6,6], R[7,7]  = 20, 1
    else:
        R[6,6], R[7,7]  = 1, 0.001 # 0.5, 0.001  # 1, 0.001
    if mag_stale :
        R[8,8]  = 1
    else:
        R[8,8]  = 0.01
    return True
# ----------------------------------------------------------------------

# ----------------------------------------------------------------------
def get_actuator_data( u ):
    global cmdvel_read, cmdvel_write
    u[0,0] = cmdvel_read['VXb']
    u[1,0] = cmdvel_read['wz']
    cmdvel_read, cmdvel_write = cmdvel_write, cmdvel_read
# ----------------------------------------------------------------------

# ----------------------------------------------------------------------
def kf(x, P, A, B, C, Q, R, u, y):
  '''x(t+1) = Ax(t)+Bu(t)+N(0,Q);  y(t) = C x(t) + N(0,R)'''  
  x = np.dot(A, x) + np.dot(B, u)           # x = Ax + Bu
  P = np.dot(A, np.dot(P, A.T)) + Q         # P = APA' + Q
  K1 = np.dot(P, C.T) 
  K2 = np.dot(C, np.dot(P, C.T))
  K3 = np.linalg.inv(R + K2)
  K =  np.dot( K1, K3)                      # K = PC'(R+CPC')^-
  e = y - np.dot(C, x)
  xplus = np.dot(K, e)
  '''if abs(xplus[0,0]) > 100 or abs(xplus[1,0]) > 100:
      print 'orig_x = ', x '''
  x += np.dot(K, e)
  P -= np.dot(K, np.dot(C, P))
  '''if abs(x[0,0]) > 1000 or abs(x[1,0]) > 1000:
      print 'x = ', x
      print 'P = ', P
      print 'e = ', e
      print 'K = ', K
      print 'K1 = ', K1
      print 'K2 = ', K2
      print 'K3 = ', K3
      print 'A = ', A
      print 'B = ', B
      print 'C = ', C
      print 'Q = ', Q
      print 'R = ', R
      print 'u = ', u
      print 'y = ', y'''
  return x,P
# ----------------------------------------------------------------------

# ----------------------------------------------------------------------
def set_AC( A, C, hdg, dt):
    A[0,2] =  1.5*np.cos(hdg) * dt      # u*t + 1/2*a*t*t = 1/2*(v-v0)/dt*dt*dt = 1/2*v*dt - 1/2*v0*dt
    A[1,2] =  -1.5*np.sin(hdg) * dt
    A[0,3] = -0.5*np.cos(hdg)*dt
    A[1,3] = 0.5*np.sin(hdg)*dt
    C[2,2] =  math.cos( hdg ) # ve = vx * cos(hdg) 
    C[3,2] =  -math.sin( hdg ) # vn = vx * sin(hdg)
# ----------------------------------------------------------------------

# ----------------------------------------------------------------------
if __name__=="__main__":
    N = 6
    A = np.identity( N )
    hdg = 0.0
    dt = FilterTimeSteps
    A[0,2] =  1.5*np.cos(hdg) * dt      # u*t + 1/2*a*t*t = 1/2*(v-v0)/dt*dt*dt = 1/2*v*dt - 1/2*v0*dt
    A[1,2] =  -1.5*np.sin(hdg) * dt
    A[0,3] = -0.5*np.cos(hdg)*dt
    A[1,3] = 0.5*np.sin(hdg)*dt
    #A[2,2] -= dt    	# vb' = vb + (u-vb)*dt ... NOW, vb' = vb + (vb-vbold) = 2*vb - vbold
    #A[2,2], A[2,3] = 1, 0	# vb-vbold will be the "actuator input" now.
    A[3,2], A[3,3] = 1, 0
    A[4,5] = dt	  	# hdg' = hdg + hdgdot * dt
    #A[5,5] -= dt    	# hdgdot' = hdgdot + (hdgdot - hdgdotOld)
    #A[5,5] = 1    	# hdgdot' = hdgdot + (hdgdot - hdgdotOld)

    B = np.zeros( (N,2) )		# Inputs: Getting rid of v and w; instead 0 and hdgdotOld
    # B[2,0], B[5,1] = dt, dt
    B[2,0], B[5,1] = 1, 1		# Inputs are (v-vold, hdgdot - hdgdotOld)

    # y = C x   y = [gps_pN, gps_pE, gps_vN, gps_vE, odo_vx, odo_hdgdot, imu_ax, imu_hdgdot]
    C = np.zeros( (9,N) )
    C[0,0] = 1
    C[1,1] = 1
    C[2,2] =  np.cos( hdg ) # vN = vx * cos(hdg) 
    C[3,2] =  -np.sin( hdg ) # vE = vx * sin(hdg)
    C[4,2] = 1      # odo_vxb = vxb state variable
    # C[5,5] = -1000  # odo_hdgdot = hdgdot  ASHISH: I think odo omega is reported with a negative sign....also appears to report on rad/ms!!!
    C[5,5] = 1  # ASHISH: Segway seems to give me the correct omega... unlike GAZEBO (commented above)
    C[6,2] = 1.0/dt  # iAx = vxb - vxbold
    C[6,3] = -1.0/dt   # imu_ax = vx - vxold
    # C[7,5] = 1  # GAZEBO: imu_hdgdot = hdgdot
    C[7,5] = -1  # SEGWAY gives negative sign for OMEGA: imu_hdgdot = hdgdot
    C[8,4] = 1

    #Q,RGPS,RODO,SVS = StateSpaceCreate(FilterTimeLength, SVS, IMU1, GPS, ODO)

    y = np.zeros( (9,1) )

    u = np.zeros( (2,1) )

    # process noise; x,y,vxb,vxold,hdg,hdgdot
    Q = np.identity ( N )
    Q[0,0], Q[1,1], Q[2,2], Q[3,3] = 0.01, 0.01, 0.01, 0.000001  # 0.01, 0.01, 0.01,  0.01
    Q[4,4], Q[5,5] = 0.0001, 0.0001  # 0.0001, 0.001

    # sensor noise; [gps_pe, gps_pn, gps_ve, gps_vn, odo_vx, odo_hdgdot, imu_ax, imu_hdgdot]
    R = np.identity( 9 )
    R[0,0], R[1,1] = 4, 4 	# 4, 4
    R[2,2], R[3,3] = 20, 20	# 5, 5		# ASHISH: changed from 2 to 5 here, 1,0.4 below
    R[4,4], R[5,5] = 0.1, 0.001   # 10.0, 4	# ASHISH: CHECK -- random changes here too
    R[6,6], R[7,7] = 0.2, 0.02		# ASHISH: CHECK -- changed for experiments
    R[8,8] = 0.01

    print ("EKF SCRIPT --------------------------------\n")

    #ROS Node Initialization and Topics to publish to and Subscribe from
    rospy.init_node('ukf_ekf')
    #Publish to ekf statistics
    ekf_pub = rospy.Publisher('/ekf_pub', ekfparam, queue_size = 1)
    #rospy.Subscriber("/odom", Odometry, OdometryMeasurementAvailable)   # /rmp/odom
    rospy.Subscriber("/zed/odom", Odometry, OdometryMeasurementAvailable)   # /rmp/odom
    #rospy.Subscriber("/rmp/odom", Odometry, OdometryMeasurementAvailable)   # /rmp/odom
    rospy.Subscriber("/zed/odom", Odometry, GPSPosreceived)             # /gps/fix
    rospy.Subscriber("/imu_virtual", Imu, IMU1MeasurementAvailable)     # /imu/imu -> virtual
    #rospy.Subscriber("/imu", Imu, IMU1MeasurementAvailable)     # /imu/imu -> virtual
    #rospy.Subscriber("/cmd_vel", Twist, CallbackCmdVel)                 # /rmp/base/vel_cmd, but not used so comment out.

    rate = rospy.Rate(FilterFreq)

    ekf_pub.publish(EKF_SVS)

    x = np.zeros ( (N,1) )      # CURRENT ESTIMATE
    P = np.identity ( N )*100      # CURRENT ESTIMATE
    old_vxb = x[2,0]
    print "Starting EKF--------------------------------\n"
    while not rospy.is_shutdown():
        set_AC(A, C, x[HDG_IDX], dt)      # A,C are time variant, so we have to update them
        if get_sensor_data( y, dt, EKF_SVS.hdgt, R ):
          #get_actuator_data( u )
          u[0,0], u[1,0] = x[VX_IDX]-x[VXOLD_IDX], x[HDGDOT_IDX]-hdgdotOld
          hdgdotOld = x[HDGDOT_IDX]
          x,P = kf(x, P, A, B, C, Q, R, u, y)

          headingt = x[4,0]
          cHDGt = np.cos( headingt )
          sHDGt = np.sin( headingt )
          rotate = np.array([[cHDGt, -sHDGt], [sHDGt,  cHDGt]])
          EKF_SVS.pN = x[0,0]
          EKF_SVS.pE = x[1,0]
          EKF_SVS.vN, EKF_SVS.vE = np.dot( rotate, np.array( [ [x[2,0]], [0] ]) )
          EKF_SVS.aN, EKF_SVS.aE = np.dot( rotate, np.array( [ [(x[2,0]-old_vxb)/dt], [0] ]) )
          EKF_SVS.vb0x = x[2,0]
          old_vxb = x[2,0]
          EKF_SVS.hdgb = x[4,0]  
          EKF_SVS.hdgt = x[4,0]  
          EKF_SVS.hdgdot = x[5,0]  
          #print 'estimated pN=', x[0,0], 'pE=', x[1,0], 'vB=', x[2,0], 'vBo=', x[3,0], 'hdg=', x[4,0], 'hdgdot=', x[5,0]
          #print pN, pE, vB, hdg, GPS_N, GPS_E, ODO_V, IMU_ACC 
          print x[0,0], x[1,0], x[2,0], x[4,0], y[0,0], y[1,0], y[4,0], y[6,0]
          sys.stdout.flush()
          if abs(x[0,0]) > 100 or abs(x[1,0]) > 100:
              print 'detected error'
              print 'gps_read = ', gps_read
              print 'gps_write = ', gps_write
              print 'odo_read = ', odo_read
              print 'odo_write = ', odo_write
              print 'imu_read = ', imu_read
              print 'imu_write = ', imu_write
              print 'estimated pN=', x[0,0], 'pE=', x[1,0], 'vB=', x[2,0], 'vBo=', x[3,0], 'hdg=', x[4,0], 'hdgdot=', x[5,0]
          else:
              pass
        else:
            pass
        ekf_pub.publish(EKF_SVS)

        # Sleep until next iteration
        rate.sleep()

    print "EKF COMPLETE-----------------------------\n"
