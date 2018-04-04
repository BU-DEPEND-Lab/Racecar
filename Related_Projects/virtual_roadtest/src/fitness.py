import numpy as np

import logging
from os.path import expanduser

home = expanduser("~")


"""normalize array to 0-1 range"""
def normalize(a):
    z = np.copy(a)
    max_ = np.nanmax(a)
    min_ = np.nanmin(a)

    for i in range(0,a.size):
        z[i] = (a[i]-min_)/(max_-min_)
    return z
    
"""steering metric - avg speed"""    
def steering(a): #drive_parameters[:, 2]
    avg = np.mean(a, axis=0, dtype=np.float64)   
    return avg
      
"""throttle metric - avg angle output to motors"""      
def throttle(a): #drive_parameters[:, 1]
    avg = np.mean(a, axis=0, dtype=np.float64)   
    return avg    

"""top speed metric - TO BE UPDATED BASED ON FINAL RANGE OF VALUES"""
def topSpeed(speeds):
    top_speed = np.nanmax(speeds)
    top_speed = top_speed/100
    return top_speed   
        
def CalculateFitness():
    home = expanduser("~")
    logging.basicConfig(filename=home+'/catkin_ws/src/virtual_racetrack/virtual_roadtest/log/test_logs.log', 
                     level=logging.DEBUG,format='%(asctime)s:%(levelname)s:%(message)s')
                     
    logging.info('-----------Begin CalculateFitness-------------')
    
    drive_parameters = np.genfromtxt(home+'/catkin_ws/src/virtual_racetrack/virtual_roadtest/src/_slash_drive_parameters.csv', delimiter=',')
    drive_parameters = np.delete(drive_parameters, 0,axis=0) # delete the titles       
    

    speed = drive_parameters[:, 1]
    angle = drive_parameters[:, 2]
    norm_speed = normalize(speed)
    norm_angle = normalize(angle)
    
    steering_m =  steering(norm_speed)    
    throttle_m =  throttle(norm_angle)      
    top_speed = topSpeed(speed)
    print "steering  = " , steering_m
    print "throttle  = " , throttle_m  
    print "top speed = " , top_speed
    
    fitness = float(np.mean([steering_m ,throttle_m, top_speed]))
    print "Fitness   = " , fitness
    
    
    logging.info('-----------End of CalculateFitness-------------')


    return fitness
        