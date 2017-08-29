#!/usr/bin/env python

import random
from numpy import dot
import numpy as np
import copy
import collections
import math
import time
from std_msgs.msg import Float64MultiArray
import rospy
from PIL import Image
from os.path import expanduser

home = expanduser("~")
coordpath = home+"/catkin_ws/src/gzbo2_generator/output/coord.txt"
cc = open(coordpath)
ccstr = cc.read()
ccstr = ccstr[ccstr.index("2048"):-1]
ccstr = ccstr[7:]
stop = (eval(ccstr)[0]+1024,eval(ccstr)[1]+1024)
start = (1024,1024)
scale = 19.93970181650108
mvp = 255
nmvp = 0
obs = 0
wal = 150
walhigh = 3
wallow = 2
path = home+"/catkin_ws/src/gzbo2_generator/output/map.png"



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
    low = 4
    high = 5
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
        low = 10
        high = 10
    if x == 4:
        low = wallow
        high = walhigh
    if x == 3:
        neighbors = lambda x, y : [(x2, y2) for x2 in range(x-15, x+16)
                               for y2 in range(y-15, y+16)
                               if ((-1 < x <= X) and
                                   (-1 < y <= Y) and
                                   (x != x2 or y != y2) and
                                   (0 <= x2 <= X) and
                                   (0 <= y2 <= Y) and
                                   ((abs(x2 - x) >13) or
                                   (abs(y2 - y) >13)))]
        cnt = 0
        n = []
        for alt in neighbors(pt[0],pt[1]):
            if cnt == 2:
                n.append(alt)
                cnt = 0
            cnt +=1
        return n
    return neighbors(pt[0],pt[1])
    
###############################################################################
#                          Function Definition
#   Name  - ExtendStarStopt
#   Function  - Extend the walls in the map to give enough moving room for the 
#               Search algorithm. Done because Gazebo gives coordinates of
#               Start and Stop right next to the wall
#   Arguments  - N/A
###############################################################################

def ExtendStartStop():
    print "Extending start ..."
    n = Neighbors(2,start)
    for i in n:
        a[i[0]][i[1]] = mvp
    n = Neighbors(2,stop)
    for i in n:
        a[i[0]][i[1]] = mvp
        
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
    
    if math.isnan(ang_deg):
        return 180
    return ang_deg
    

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
    for pt in get_line(pt1[0],pt1[1],pt2[0],pt2[1]):
        if a[pt[0]][pt[1]] != mvp:
            return 0
        nvalid = Neighbors(1,(pt[0],pt[1]))
        for nei in nvalid:
            if a[nei[0]][nei[1]] != mvp:
                return 0
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
#   Name  - CutNeighbors
#   Function  - Returns Valid movable points from neighbors   
#   Arguments  - pt(point of referrence), Dict(Dictionary of the branches of
#                the other tree.)
###############################################################################
    
def CutNeighbors(Nei, pos, sD, ap):
    nv = []
#    print Nei, pos, ap
    for nc in Nei:
        if (((CheckValid(eval(pos),nc) == 1) and 
        (ang((ap,eval(pos)),(eval(pos),nc)) <=90 )and
        (nc not in sD))):
           nv.append(nc) 
    return nv

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
#    nextpt = 1
    pt1 = 0
    pt2 = 1
    print "Length of path Before = ",len(Line1)

    """ Line Optimization """
    while (pt1<(len(Line1)-1) and pt2 <(len(Line1)-1)):
#        print pt2
        while ((CheckValid(Line1[pt1],Line1[pt2]) == 1) and 
        (pt2 <(len(Line1)-1))):
#            print pt1, pt2, len(Line1)
#            print pt2, pt1
            pt2 +=1
        Line2.append(Line1[pt1])
        pt1 = pt2
        pt2 = pt1+1
#        if pt2 <= pt1:
#            Line2.append(Line1[pt1])
#            pt1 = nextpt
#            pt2 = len(Line1)-1
    Line2.append(Line1[len(Line1)-1])
    print "Length of path After = ",len(Line2)
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
    
######################## Velocity Vector Calculations #########################

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
#   Name  - AngleDef
#   Function  - Used to create a list of all the angular changes in the entire
#               path. It does this by finding the angular difference at each 
#               position in the map w.r.t the next position
#   Arguments  - Line 
###############################################################################   
def AngleDef(Line):
    anglist = {}
    AF = LineComp(Line)
    for pt in range(1,len(Line)-1):
        anglist[str(AF.index(Line[pt]))] = ang((Line[pt-1],Line[pt]),(Line[pt],Line[pt+1]))
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
            
            minspeed = 100.0 - int((angleList[key]/45.0) * 70.0)
            """ Change ranges depending on use case """
            if (int(key) - pt) == 0:
                speed = minspeed
                speedlist[str(pt)] = int(speed)
                flag = 1                
                break
            elif ((int(key) - pt) < 20) and ((int(key) - pt) > 0):
                speed = 100 - ((100.0 - minspeed)*(1 - ((int(key) - pt))/20.0))
                speedlist[str(pt)] = int(speed)
                flag = 1                
                break
            elif ((int(key) - pt)> -10) and ((int(key) - pt) < 0):
                speed = 100 - ((100.0 - minspeed)*(1 + ((int(key) - pt))/10.0))
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
    speedlist[len(speedlist)] = 0.0
    return speedlist

rospy.init_node('Path_planner', anonymous=True)
pub = rospy.Publisher('path_planner',Float64MultiArray , queue_size=10)  
a = np.array(Image.open(path).convert('L'))
X = len(a)-1
Y = len(a[0])-1
#RemoveNoise()
#ExtendWall()
#for rep in range(2):
#    a = np.rot90(a) # First remove excess columns and then Excess Rows
#    b = len(a)
#    for line in range(b):
#        if(np.count_nonzero(a[b-line-1]) < 1):
#            a = np.delete(a,b-line-1,0)
a = np.rot90(a,3)
a = np.fliplr(a)
ExtendStartStop()
ErrorFree = 0
while ErrorFree == 0:
    try:
        """ Variable declerations for the Search tree that will begin from START """
        SetTrees = collections.defaultdict(list)
        SetTrees[str(start)].append(start)
        pos = str(start)
        SetDismissed = []
        n =  Neighbors(3,(start[0],start[1]))
        AnglePrevRef = (start[0]-1, start[1]-1)

        """ Variable declerations for the Search tree that will begin from STOP """    
        SetTreesopp = collections.defaultdict(list)
        SetTreesopp[str(stop)].append(stop)
        posopp = str(stop)
        nopp = Neighbors(3,(stop[0],stop[1]))
        AnglePrevRefopp = start
        prepre = start

        """ Variable Declaration for The final path """
        LineFinal = []
        LineFinal2 = []
        GoalFlag = 0
        LineFinal.append(start)
        LineFinal2.append(stop)  

        """ Check if there is a direct path from START to STOP """
        st_line = 0
        if CheckValid(start,stop) == 1:
            GoalFlag = 1
            st_line = 1
        search_start = time.time()
        while GoalFlag != 1:
            finStep = 0
            while finStep == 0:
                nv = CutNeighbors(n,pos,SetDismissed, AnglePrevRef)
                if len(nv) == 0:
                    nf = 1
                else:
                    nf = 0
                    nc = random.choice(nv)
        #            print ang((AnglePrevRef,eval(pos)),(eval(pos),nc))
                if nf == 1:
                    #print nv
        #            print "nf reached at ", pos
                    #print "nf ",pos, AnglePrevRef
                    pos = str(SetTrees[str((AnglePrevRef[0],AnglePrevRef[1]))][-2])
                    AnglePrevRef = SetTrees[str((AnglePrevRef[0],AnglePrevRef[1]))][-3]
                    #print "rebound ",pos, AnglePrevRef


                else:
                    print "[",time.time() - search_start,"] : ","Searching ... " # To enable auto scroll in iPython
                    prepre = AnglePrevRef
                    SetDismissed.append(nc)
                    ret = CheckDirect(nc , SetTreesopp) # Check for direct line of sight
        #            print "pos = ", pos, " nc = ", nc
                    AnglePrevRef = eval(pos)        
                    if (ret != 0): # There is direct line of sight
                        print "Found Goal"
                        GoalFlag = 1
                        finStep = 1
                        """ Add point and line of sight branch to tree """
            #                print p
                        SetTrees[str(nc)] = copy.deepcopy(SetTrees[pos])
                        SetTrees[str(nc)].append(nc)
                        
                        """ Show final tree from START """
                        for line in SetTrees[str(nc)]:
                            LineFinal.append(line)

                    else: # There is no direct line of sight
                        
                        """ Add point to as a branch to START tree """
                        SetTrees[str(nc)] = copy.deepcopy(SetTrees[pos])
            #                print p
                        SetTrees[str(nc)].append(nc)
                        """ Choose next branch at random to search from """
                        pos = str(nc)
                    n = Neighbors(3,(eval(pos)[0],eval(pos)[1]))    
        ErrorFree = 1
    except IndexError:
        print "Caught Index Error"
        ErrorFree = 0           
""" Function calls for Final-Processing of Final Path """

""" Convert the two branches from START and STOP into one """
LineFinal2.reverse() # Reverse the final path from STOP tree
LineFinal+=LineFinal2 # Combine into one path

""" Try processing multiple times to see different results """
start_time5 = time.time()
LF1 = LineComp(LineFinal)
if not st_line: 
    print " Starting Optimization"    
    print "Map - without optimization"
    LF1 = LineComp(LF1)
    LF2 = LineOptimization(LF1)
    print "Map - first optimization"
    LF2 = LineComp(LF2)
    LF = LineOptimization(LF2)

else:
    LF = LF1
stop_time5 = time.time()
time5 = stop_time5 - start_time5

angs = AngleDef(LF)
SF = np.array(LineComp(LF))
SF = SF.astype(float)
#print "SSSSSSFFFFFF = ", SF
speeds = SpeedList(SF,angs)
#print "SPEEEEEEDDDDDSSSS = ",speeds
arr = Float64MultiArray()
setsend = [[0,0,100]for ii in range(len(SF))]
for i in range(len(SF)-1):
    for k in range(2):
        SF[i][k] -= 1024.0
        SF[i][k] /= scale
for i in range(len(SF)-1):
    setsend[i] = [SF[i][0],SF[i][1],speeds[str(i)]]
arr.data = np.ndarray.flatten(np.array(setsend)).tolist() 
print "Path Sent"

#print "ARRRRRRRRRRRRRRRRR = ", arr
pub.publish(arr)




