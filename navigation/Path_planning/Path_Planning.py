#!/usr/bin/env python

import random
import matplotlib.pyplot as plt 
import numpy as np
import copy
import collections
import math
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import OccupancyGrid
import rospy





global start
global stop
global X
global Y
global a
X = 2048
Y = 2048
scale = (1495-1024)*(1/23.6212158203)
start = [1024,1024]
stop = [1040,1024]
x = [start[0],stop[0]]
y = [start[1],stop[1]]

###############################################################################
#                          Function Definition
#   Name  - callback2
#   Function  - Used to update present location. Use case -> Present location
#               is START location
#   Arguments  - pgmf -> open file object with .pgm extension
###############################################################################

def callback2(data):
    global start
    start = [int(data.pose.position.x*scale)+1024,int(data.pose.position.y*scale)+1024]
    print start
    rate = rospy.Rate(1) # 10hz

###############################################################################
#                          Function Definition
#   Name  - Neighbors
#   Function  - Finds the neighboring points around a given point in a grid.
#               Depending on the requirement, it is used for finding all the 
#               points between the high and low x, y distance. And it is also 
#               used to find a points for the search step. For 3, returns
#               every 10th neighbor to prevent over-crowding of branches.
#   Arguments  - x -> 1(low 3 and high 4) - Check if Valid move,
#                     2(low 1 and high 2) - Check for noise
#                     3(low 20 and high 21) - Only consider perimeter for jump
#                pt -> point of reference
###############################################################################

def Neighbors(x , pt):
    global X
    global Y
    low = 3 
    high = 4 
    neighbors = lambda x, y : [(x2, y2) for x2 in range(x-low, x+high)
                                   for y2 in range(y-low, y+high)
                                   if ((-1 < x <= X) and
                                       (-1 < y <= Y) and
                                       (x != x2 or y != y2) and
                                       (0 <= x2 <= X) and
                                       (0 <= y2 <= Y))]
    if x == 1:
        pass 
    if x == 2:
        low = 1
        high = 2
    if x == 3:
        neighbors = lambda x, y : [(x2, y2) for x2 in range(x-10, x+11)
                               for y2 in range(y-10, y+11)
                               if ((-1 < x <= X) and
                                   (-1 < y <= Y) and
                                   (x != x2 or y != y2) and
                                   (0 <= x2 <= X) and
                                   (0 <= y2 <= Y) and
                                   ((abs(x2 - x) >9) or
                                   (abs(y2 - y) >9)))]
        cnt = 0
        n = []
        for alt in neighbors(pt[0],pt[1]):
            if cnt == 10:
                n.append(alt)
                cnt = 0
            cnt +=1
        return n
    return neighbors(pt[0],pt[1])
    
###############################################################################
#                          Function Definition
#   Name  - CheckNoise
#   Function  - Used for RemoveNoise. Returns 1 is the given point is noise
#               Else Returns 0. Condition for noise is that there should be 
#               2 or more non movalble pixels around any non movable pixels
#   Arguments  - pt -> point of reference
###############################################################################

def CheckNoise(pt):
    global a
    cntnei = 0
    for nei in Neighbors(2,(pt)):
        if(a[nei[0]][nei[1]] != 0):
            cntnei += 1
    if cntnei >=3:
        return 0
    return 1
    
###############################################################################
#                          Function Definition
#   Name  - RemoveNoise
#   Function  - Removes noise by Checking each non movable pixel via
#               CheckNoise.Convert noise pixel to movable point.     
#   Arguments  - N/A
###############################################################################

def RemoveNoise():
    global a
    SetWall = []
    print "Removing Noise ...  "
    for line in range(X):
        for lin in range(Y):
            if a[line][lin] != 0:
                if(CheckNoise((line,lin)) == 1):
                    a[line][lin] = 0
            if a[line][lin]!=0:
                for n in Neighbors(1,(line,lin)):
                    if n not in SetWall and a[n[0]][n[1]] == 0:
                        a[n[0]][n[1]] = 150
                        SetWall.append(n)

    
###############################################################################
#                          Function Definition
#   Name  - ExtendWall
#   Function  - Extends the wall positions to act as a buffer for the car.
#               This is done preemtively to prevent the car from crashing into 
#               wall. Increases the wall buffer w.r.t. Neighbors function with
#               argument x = 1.    
#   Arguments  - N/A
###############################################################################

def ExtendWall():
    global a
    SetWall = []
    for line in range(X):
        for lin in range(Y):
            if a[line][lin]!=0:
                for n in Neighbors(1,(line,lin)):
                    if n not in SetWall and a[n[0]][n[1]] == 0:
                        a[n[0]][n[1]] = 150
                        SetWall.append(n)
                        
###############################################################################
#                          Function Definition
#   Name  - dot
#   Function  - Used to find dot product between two vectors
#   Arguments  - vA -> Vector A, vB -> Vector B
###############################################################################

def dot(vA, vB):
    return vA[0]*vB[0]+vA[1]*vB[1]
    
###############################################################################
#                          Function Definition
#   Name  - ang
#   Function  - Used to find angle between two lines
#   Arguments  - lineA, lineB
###############################################################################   

def ang(lineA, lineB):
    # Get nicer vector form
    vA = [(lineA[0][0]-lineA[1][0]), (lineA[0][1]-lineA[1][1])]
    vB = [(lineB[0][0]-lineB[1][0]), (lineB[0][1]-lineB[1][1])]
    # Get dot prod
    dot_prod = dot(vA, vB)
    # Get magnitudes
    magA = dot(vA, vA)**0.5
    magB = dot(vB, vB)**0.5
    try:
        # Get angle in radians and then convert to degrees
        angle = math.acos(dot_prod/magB/magA)
    except:
        angle = 0
    # Basically doing angle <- angle mod 360
    ang_deg = math.degrees(angle)%360

    if ang_deg-180>=0:
        # As in if statement
        return 360 - ang_deg
    else: 

        return ang_deg

###############################################################################
#                          Function Definition
#   Name  - AngleDef
#   Function  - Used to create a list of all the angular changes in the entire
#               path. It does this by finding the angular difference at each 
#               position in the map w.r.t the next position
#   Arguments  - Line 
###############################################################################   

def AngleDef(Line):
    
    anglist = {}
    anglist[str(0)] = 0
    AF = LineComp(Line)
    for pt in range(1,len(Line)-1):
        anglist[str(AF.index(Line[pt]))] = ang((Line[pt-1],Line[pt]),(Line[pt],Line[pt+1]))
    anglist[str(len(Line)-1)] = 0
    return anglist
    
###############################################################################
#                          Function Definition
#   Name  - SpeedList
#   Function  - Calculates the speeds at which the car has to travel
#   Arguments  - Line , angleList
###############################################################################   

def SpeedList(Line, angleList):
    speedlist = collections.OrderedDict()
    for pt in range(len(Line)-20):
        flag = 0
        for key in angleList.keys():

            """ Change ranges depending on use case """
            # TODO: relate speed change to angle difference and momentum
            if (int(key) - pt) == 0:
                speed = 30
                speedlist[str(pt)] = speed
                flag = 1                
                break
            elif ((int(key) - pt) < 20) and ((int(key) - pt) > 0):
                speed = 100 - (70.0*(1 - ((int(key) - pt))/20.0))
                speedlist[str(pt)] = int(speed)
                flag = 1
                break
            elif ((int(key) - pt)> -10) and ((int(key) - pt) < 0):
                speed = 100 - (70.0*(1 + ((int(key) - pt))/10.0))
                speedlist[str(pt)] = int(speed)
                flag = 1
                break
            else:
                flag = 1
                speedlist[str(pt)] = 100
        if flag == 0:
            speedlist[str(pt)] = 100

    for pt2 in range(20):
        speedlist[str(pt2+pt)] = int(100 - (100.0*(pt2/19.0)))
    return speedlist

                                   
###############################################################################
#                          Function Definition
#   Name  - get_line
#   Function  - Returns all the points in a grid that fall between two given
#               referrence points. 
#   Arguments  - x1, y1 -> (pt1), x2, y2 -> (pt2) 
###############################################################################

def get_line(x1, y1, x2, y2):
    points = []
    issteep = abs(y2-y1) > abs(x2-x1)
    if issteep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2
    rev = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        rev = True
    deltax = x2 - x1
    deltay = abs(y2-y1)
    error = int(deltax / 2)
    y = y1
    ystep = None
    if y1 < y2:
        ystep = 1
    else:
        ystep = -1
    for x in range(x1, x2 + 1):
        if issteep:
            points.append((y, x))
        else:
            points.append((x, y))
        error -= deltay
        if error < 0:
            y += ystep
            error += deltax
    # Reverse the list if the coordinates were reversed
    if rev:
        points.reverse()
    return points
    
###############################################################################
#                          Function Definition
#   Name  - CheckValid
#   Function  - Checks if a move is valid between two given referrence points.
#               The condition is that the two points should not be non movable 
#               points and every point between the two should have a buffer 
#               from the wall. The buffer if found using Neighbors with x = 1.
#               Returns 1 for valid move, and 0 for invalid move.
#   Arguments  - pt1, pt2
###############################################################################

def CheckValid(pt1, pt2):
    global a
#    print " check valid ", a[pt1[0]][pt1[1]]
#    print " check valid ", a[pt2[0]][pt2[1]]
    for pt in get_line(pt1[0],pt1[1],pt2[0],pt2[1]):
        if a[pt[0]][pt[1]] != 0:
#            print "invalid line"
            return 0
        nvalid = Neighbors(2,(pt[0],pt[1]))
        for nei in nvalid:
            if a[nei[0]][nei[1]] != 0:
                
#                print "invalid neighbors ,",nei,a[nei[0]][nei[1]]               
                return 0
#    print "Returning 1"
    return 1
    
###############################################################################
#                          Function Definition
#   Name  - CheckDirect
#   Function  - This function check if a point from one tree has a direct line 
#               of sight move to any branch of the other tree. This is done to 
#               curb run time, and prevent wastage of search if a direct path 
#               is already visible.   
#   Arguments  - pt(point of referrence), Dict(Dictionary of the branches of
#                the other tree.)
###############################################################################
    
def CheckDirect(pt, Dict):
    for key in Dict.keys():
        if(CheckValid(pt,eval(key))) == 1:
            return(eval(key))
    return(0)
    
###############################################################################
#                          Function Definition
#   Name  - LineOptimization
#   Function  - This function optimizes the line by removing redundant points. 
#               It does this by checking every point on the line with all the
#               points after it to check for line of sight. If found, it 
#               removes all the intermediate points.   
#   Arguments  - Line1 -> Final Path
###############################################################################

def LineOptimization(Line1):
    
    """ Variable declaration for line optimization """
    Line2 = []
    nextpt = 1
    pt1 = 0
    pt2 = 1
    
    """ Line Optimization """
    while pt1<len(Line1)-1:
        if CheckValid(Line1[pt1],Line1[pt2]) == 1:
            nextpt = pt2
        pt2 +=1
        if pt2 >= len(Line1):
            Line2.append(Line1[pt1])
            pt1 = nextpt
            pt2 = pt1 + 1
    Line2.append(Line1[pt1])
    return Line2

###############################################################################
#                          Function Definition
#   Name  - LineComp
#   Function  - This Function adds all intermediate points to the Line for 
#               easier movement control by the car
#   Arguments  - Line -> Line with scattered points instead of being continuous
###############################################################################

def LineComp(Line):
    NewLine = []
    for pt in range(len(Line)-1):
        NewLine+=get_line(Line[pt][0],Line[pt][1],Line[pt+1][0],Line[pt+1][1])
    return NewLine
    
pub = rospy.Publisher('path_planner',Float64MultiArray , queue_size=10)  


def callback(data):
    global X
    global Y
    global start
    global stop
    global a
    a = np.reshape(data.data,(2048,2048))
    X = len(a)-1
    Y = len(a[0])-1
    a = np.rot90(a,3)
#    a = np.fliplr(a)
#    print np.where(a == 100)
    """ Variable declerations for the Search tree that will begin from START """
    SetTrees = collections.defaultdict(list)
    SetTrees[str(start)].append(start)
    pos = str(start)
    SetDismissed = []
    n =  Neighbors(3,(start[0],start[1]))
#    plt.imshow(a)
#    plt.show()
    
    """ Variable declerations for the Search tree that will begin from STOP """    
    SetTreesopp = collections.defaultdict(list)
    SetTreesopp[str(stop)].append(stop)
    SetLines = []
    posopp = str(stop)
    nopp = Neighbors(3,(stop[0],stop[1]))
    
    """ Variable Declaration for The final path """
    LineFinal = []
    LineFinal2 = []
    GoalFlag = 0
    
    """ Check if there is a direct path from START to STOP """
    if CheckValid(start,stop) == 1:
        GoalFlag = 1
        LineFinal.append(start)
        LineFinal2.append(stop) 
    
    """ Main logic """
    while GoalFlag != 1:
        
        """ Logic for tree search from START """
        minn = 100000000000 # High number as default distance
        for k in n:
            """ Find the optimal point, i.e. the point closest to the goal """
            print a[k[0]][k[1]]," ", k             
            if((k[0] - stop[0])**2 + (k[1] - stop[1])**2) < minn:
                if a[k[0]][k[1]] != 0 or CheckValid(k,eval(pos)) == 0:
                    SetDismissed.append(k)
                if (k not in SetDismissed) and (str(k) not in SetTrees.keys()):
                    p = k
                    minn = (k[0] - stop[0])**2 + (k[1] - stop[1])**2
        if  p == 0: # If the point selected is movable
            ret = CheckDirect(p , SetTreesopp) # Check for direct line of sight
            if (ret != 0): # There is direct line of sight
                print "Reached Goal start"
                GoalFlag = 1
                
                """ Add point and line of sight branch to tree """
                SetLines.append([ret,p])
                SetTrees[str(p)] = copy.deepcopy(SetTrees[pos])
                SetTrees[str(p)].append(p)
                
                """ Show final tree from START """
#                plt.imshow(a, extent=[0, np.shape(a)[1], np.shape(a)[0],0])
#                prev = copy.deepcopy(start)
#                for line in SetTrees[str(p)]:
#                    LineFinal.append(line)
#                    plt.plot([line[1],prev[1]],[line[0],prev[0]],color = 'blue')
#                    prev = copy.deepcopy(line)
#                prev = copy.deepcopy(stop)
                
                """ Show final tree from STOP """            
#                for line in SetTreesopp[str(ret)]:
#                    LineFinal2.append(line)
#                    plt.plot([line[1],prev[1]],[line[0],prev[0]],color = 'blue')
#                    prev = copy.deepcopy(line)  
#                plt.plot([ret[1],p[1]],[ret[0],p[0]],color = 'blue')
#                plt.show()
                break
            else: # There is no direct line of sight
                
                """ Add point to as a branch to START tree """
                SetTrees[str(p)] = copy.deepcopy(SetTrees[pos])
                SetTrees[str(p)].append(p)
                SetLines.append([eval(pos),p])
                
                """ Choose next branch at random to search from """
                pos = random.choice(SetTrees.keys())
                n = Neighbors(3,(eval(pos)[0],eval(pos)[1]))
    
        else: # The point is not movable
            SetDismissed.append(p) # Add point to not movable list
            pos = random.choice(SetTrees.keys()) # Choose next branch at random
            n = Neighbors(3,(eval(pos)[0],eval(pos)[1]))
    
        """ Logic for tree search from STOP """        
        minn = 100000000000 # High number as default distance
        for k in nopp:
            popp = 0
            """ Find the optimal point, i.e. the point closest to the goal """
            if((k[0] - start[0])**2 + (k[1] - start[1])**2) < minn:
                print a[k[0]][k[1]], k 
                if a[k[0]][k[1]] != 0 or CheckValid(k,eval(posopp)) == 0:
                    SetDismissed.append(k)
                if (k not in SetDismissed) and (str(k) not in SetTreesopp.keys()):
                    popp = k
                    minn = (start[0] - k[0])**2 + (start[1] - k[1])**2
        if popp == 0: # If the point selected is movable
            ret = CheckDirect(popp , SetTrees) # Check for direct line of sight
            if (ret != 0): # There is direct line of sight
                GoalFlag = 1
                print "Reached Goal stop"
                
                """ Add point and line of sight branch to tree """
                SetLines.append([ret,popp])
                SetTreesopp[str(popp)] = copy.deepcopy(SetTreesopp[posopp])
                SetTreesopp[str(popp)].append(popp)
                
                """ Show final tree from START """
#                plt.imshow(a, extent=[0, np.shape(a)[1], np.shape(a)[0],0])
#                prev = copy.deepcopy(start)
#                for line in SetTrees[str(ret)]:
#                    plt.plot([line[1],prev[1]],[line[0],prev[0]],color = 'blue')
#                    LineFinal.append(line)
#                    prev = copy.deepcopy(line)
#                prev = copy.deepcopy(stop)
#    
#                """ Show final tree from STOP """
#                for line in SetTreesopp[str(popp)]:
#                    plt.plot([line[1],prev[1]],[line[0],prev[0]],color = 'blue')
#                    prev = copy.deepcopy(line)
#                    LineFinal2.append(line)
#                plt.plot([ret[1],popp[1]],[ret[0],popp[0]],color = 'blue')
#                plt.show()
                break
            else: # There is no direct line of sight
    
                """ Add point to as a branch to STOP tree """            
                SetTreesopp[str(popp)] = copy.deepcopy(SetTreesopp[posopp])
                SetTreesopp[str(popp)].append(popp)
                SetLines.append([eval(posopp),popp])
                
                """ Choose next branch at random to search from """
                posopp = random.choice(SetTreesopp.keys())
                nopp = Neighbors(3,(eval(posopp)[0],eval(posopp)[1]))
    
        else: # The point is not movable
            SetDismissed.append(p) # Add point to not movable list
            posopp = random.choice(SetTreesopp.keys()) # Choose next branch at random
            nopp = Neighbors(3,(eval(posopp)[0],eval(posopp)[1]))
            
        """ Show all branches from both trees """
#        plt.imshow(a, extent=[0, np.shape(a)[1], np.shape(a)[0],0])
#        for line in SetLines:
#            plt.plot([line[0][1],line[1][1]],[line[0][0],line[1][0]],color = 'green')
#        plt.show()
        print "Searching ... " # To enable auto scroll in iPython
            
    """ Convert the two branches from START and STOP into one """
    LineFinal2.reverse() # Reverse the final path from STOP tree
    LineFinal+=LineFinal2 # Combine into one path
    
    """ Try processing multiple times to see different results """
    LF1 = LineComp(LineFinal)   
    LF = LineOptimization(LF1)
    
    """ Show final Line """
#    plt.imshow(a, extent=[0, np.shape(a)[1], np.shape(a)[0],0])
#    prev = copy.deepcopy(LF[0])
#    for line in LF:
#        plt.plot([line[1],prev[1]],[line[0],prev[0]],color = 'blue')
#        prev = copy.deepcopy(line)
    rate = rospy.Rate(1) # 10hz
    angs = AngleDef(LF)
    SF = np.array(LineComp(LF))
    SF = SF.astype(float)
    print "SSSSSSFFFFFF = ", SF
    speeds = SpeedList(SF,angs)
    print "SPEEEEEEDDDDDSSSS = ",speeds
    arr = Float64MultiArray()
    setsend = [[0,0,100]for ii in range(len(SF))]
    for i in range(len(SF)-1):
        for k in range(2):
            SF[i][k] -= 1024.0
            SF[i][k] /= scale
    for i in range(len(SF)-1):
        setsend[i] = [SF[i][0],SF[i][1],speeds[str(i)]]
    arr.data = np.ndarray.flatten(np.array(setsend)).tolist() 

    print "ARRRRRRRRRRRRRRRRR = ", arr
    pub.publish(arr)
   


def bag_read():

    rospy.init_node('bag_read', anonymous=True)
    rospy.Subscriber("slam_out_pose",PoseStamped , callback2)
    rospy.Subscriber("map", OccupancyGrid, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
	bag_read()
 
 
 

    
    
