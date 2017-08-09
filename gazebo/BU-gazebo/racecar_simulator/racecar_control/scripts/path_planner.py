#!/usr/bin/env python

import rospy
import random
import numpy as np
from nav_msgs.msg import OccupancyGrid
import scipy.misc
from math import sqrt
import scipy.ndimage
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray

global start
global stop
scale = (1495-1024)*(1/23.6212158203)
start = [1030,1450]
stop = [1024,1024]
x = [start[0],stop[0]]
y = [start[1],stop[1]]


def callback2(data):
    global start
    start = [int(data.pose.position.x*scale)+1024,int(data.pose.position.x*scale)+1024]
#    print start

def countup(x,y,a):
    px = x
    py = y-1
    count = 0
    while(a[px][py]==0):
        count+=1
        py-=1
    return(count)
def countdown(x,y,a):
    px = x
    py = y+1
    count = 0
    while(a[px][py]==0):
        count+=1
        py+=1
    return(count)
def countleft(x,y,a):
    px = x-1
    py = y
    count = 0
    while(a[px][py]==0):
        count+=1
        px-=1
    return(count)
def countright(x,y,a):
    px = x+1
    py = y
    count = 0
    while(a[px][py]==0):
        count+=1
        px+=1
    return(count)
def lookup(x,y,a):
    px = x
    py = y-5
    block = 0
    if a[px][py] ==0:
        block = 1
    return(block)
def lookdown(x,y,a):
    px = x
    py = y+5
    block = 0
    if a[px][py] ==0:
        block = 1
    return(block)
def lookleft(x,y,a):
    px = x-5
    py = y
    block = 0
    if a[px][py] ==0:
        block = 1
    return(block)
def lookright(x,y,a):
    px = x+5
    py = y
    block = 0
    if (px<len(a) & py<len(a[0])):
        if a[px][py] ==0:
            block = 1
    return(block)
def direc(x,y,a):
    if(countup(x,y)+countdown(x,y))>=(countleft(x,y)+countright(x,y)):
        return(1)
    return(0)
def direcp(x,y,xx,yy,a):
    if(direc(x,y) == 1):
        if (abs(yy-stop[1])>abs(y-stop[1])):
            return(1)
        return(0)
    if(direc(x,y) == 0):
        if (abs(xx-stop[0])>abs(x-stop[0])):
            return(1)
        return(0)
pub = rospy.Publisher('path_planner',Float64MultiArray , queue_size=10)  
def callback(data):
    a = np.reshape(data.data,(2048,2048))
    setx = [[] for ii in range(10)]
    sety = [[] for ii in range(10)]
    setall = [[] for ii in range(10)]
    X = len(a)-1
    Y = len(a[0])-1
    neighbors = lambda x, y : [(x2, y2) for x2 in range(x-5, x+6)
                               for y2 in range(y-5, y+6)
                               if (-1 < x <= X and
                                   -1 < y <= Y and
                                   (x != x2 or y != y2) and
                                   (0 <= x2 <= X) and
                                   (0 <= y2 <= Y))]
    for rep in range(5):
        pointx = start[0]
        pointy = start[1]
        X = len(a)-1
        Y = len(a[0])-1
        while (pointx != stop[0])|(pointy!=stop[1]):
            n = neighbors(pointx,pointy)
            p = random.choice(n)
            wcount = 0
           
            while((((abs(stop[0]-p[0]))>(abs(stop[0]-pointx)))&
                   ((abs(stop[1]-p[1]))>(abs(stop[1]-pointy))))|
                    (a[p[0]][p[1]]!=0)|
                     ((lookup(p[0],p[1],a) == 0)&
                     (lookdown(p[0],p[1],a) == 0)&
                     (lookleft(p[0],p[1],a) == 0)&
                     (lookright(p[0],p[1],a) == 0))|
                     ([p[0],p[1]] in setall)):
                p = random.choice(n)
                wcount+=1
                if wcount == 300:
                    break
            if (stop[0],stop[1]) in n:
                 p = (stop[0],stop[1])
            setx[rep].append(p[0])
            sety[rep].append(p[1])
            setall[rep].append([p[0],p[1]])
            pointx = p[0]
            pointy = p[1]
            if ((len(setall[rep])>2000)|(wcount==300)):
                setx[rep] = []
                sety[rep] = []
                setall[rep] = []
                pointx = start[0]
                pointy = start[1]
                X = len(a)-1
                Y = len(a[0])-1
        ptr = 1
        while (ptr<len(setall[rep])-1):
            kk = neighbors(setx[rep][ptr-1],sety[rep][ptr-1])
            if (((sqrt((abs(setx[rep][ptr+1] - setx[rep][ptr-1])^2) + 
                ((abs(sety[rep][ptr+1] - sety[rep][ptr-1])^2)))<
               (sqrt((abs(setx[rep][ptr] - setx[rep][ptr-1])^2) + 
                ((abs(sety[rep][ptr] - sety[rep][ptr-1])^2)))))|
                ((setx[rep][ptr+1],sety[rep][ptr+1]) in kk))|
                (a[((setx[rep][ptr+1]-setx[rep][ptr-1])/2)+setx[rep][ptr-1]]
                [((sety[rep][ptr+1]-sety[rep][ptr-1])/2)+sety[rep][ptr-1]] == 0)):
                    del setx[rep][ptr]
                    del sety[rep][ptr]
                    del setall[rep][ptr]
                    ptr+=1
            else:
                ptr+=1
    pos = 0           
    for rep in range(5):
        if(len(setx[rep])<len(setx[pos])):
            pos = rep
    arr = Float64MultiArray()
    rate = rospy.Rate(100) # 10hz
    sety[pos]= scipy.ndimage.gaussian_filter(sety[pos],6)
    setx[pos] = scipy.ndimage.gaussian_filter(setx[pos],10)
#    fig, ax = plt.subplots()
#    ax.plot(sety[pos], setx[pos])
#    ax.imshow(a)
#    plt.show()

    setsend = [[0,0]for ii in range(len(setx[pos]))]
    for i in range(len(setx[pos])):
        setsend[i] = [setx[pos][i],sety[pos][i]]
    arr.data = np.ndarray.flatten(np.array(setsend)).tolist() 
    for i in range(len(arr.data)):
        arr.data[i] -= 1024
        arr.data[i]/=scale

    pub.publish(arr)
    print arr
    exit(0)


def bag_read():

    rospy.init_node('bag_read', anonymous=True)
    rospy.Subscriber("slam_out_pose",PoseStamped , callback2)
    rospy.Subscriber("map", OccupancyGrid, callback)
 

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    bag_read()
