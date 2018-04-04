import subprocess
import time
import numpy as np
import os

#st_timeCustom = []
#end_timeCustom = []
#pathSizeCustom = []
#count_Custom = 0;
#i = 0
#while i< 1000:
#    process = 0
#    cmd1 = 0
#    output = 0
#    err = 0
#    print "Custom: - Cycle ",i
#    st_timeCustom.append(time.time())
#    cmd1 = '/home/f1/Desktop/Akash/Path_planning/Generator/LooseFiles/AHS 0' 
#    process = subprocess.Popen(cmd1, stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
#    (output,err)=process.communicate()
#    exit_code = process.wait()    
#    end_timeCustom.append(time.time())
#    if(output):        
#        pathSizeCustom.append(int(output))
#        i+=1      
#        count_Custom+=1
#    else:
#        i+=1        
#        print "Seg Fault"
#        st_timeCustom.pop()
#        end_timeCustom.pop()
#        st_timeCustom.append('SEG FAULT')
#        end_timeCustom.append('SEG FAULT')
#        pathSizeCustom.append('SEG FAULT')
#
#f = open("/home/f1/Desktop/Akash/Path_planning/Generator/LooseFiles/resultAHS_0.20_single.txt", 'w')
#
#f.write("AHS : - "+'\n')
#f.write("Average Time = " + str(np.sum(np.array([X for X in end_timeCustom
#if X !='SEG FAULT']) 
#- np.array([X for X in st_timeCustom if X !='SEG FAULT']))/count_Custom)+'\n')
#f.write("Largest Time : - " +  str(max(np.array([X for X in end_timeCustom 
#if X !='SEG FAULT'])
# - np.array([X for X in st_timeCustom if X !='SEG FAULT'])))+'\n')
#f.write("Smallest Time : - " + str(min(np.array([X for X in end_timeCustom 
#if X !='SEG FAULT']) 
#- np.array([X for X in st_timeCustom if X !='SEG FAULT'])))+'\n')
#f.write("Average Path Length = " + str(np.sum([X for X in pathSizeCustom 
#if X !='SEG FAULT'])/count_Custom)+'\n')
#f.write("Largest Path Length = " + str(max([X for X in pathSizeCustom 
#if X !='SEG FAULT']))+'\n')
#f.write("Smallest Path Length = " + str(min([X for X in pathSizeCustom if X !='SEG FAULT']))+'\n')
#f.write("Number of valid searches = " + str(count_Custom) + '\n')
#f.write("Start_times = " + str(st_timeCustom) + '\n')
#f.write("End_times = " + str(end_timeCustom) + '\n')
#f.write("Lengths = " + str(pathSizeCustom) + '\n')
#f.close()
i = 0
st_timeRRT = []
end_timeRRT = []
pathSizeRRT = []
count_RRT = 0;

while(i<1000):
    print "RRT: - Cycle ", i
    st_timeRRT.append(time.time())
    cmd1 = '/home/f1/Desktop/Akash/Path_planning/Generator/LooseFiles/RRT 0'
    process = subprocess.Popen(cmd1, stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
    (output,err)=process.communicate()
    exit_code = process.wait()
    end_timeRRT.append(time.time())
    if(output) and (output != '200') :        
        pathSizeRRT.append(int(output))
        i+=1 
        count_RRT += 1;

    else:
        i+=1        
        print "Seg Fault"
        st_timeRRT.pop()
        end_timeRRT.pop()
        st_timeRRT.append('SEG FAULT')
        end_timeRRT.append('SEG FAULT')
        pathSizeRRT.append('SEG FAULT')

        
        
f = open("/home/f1/Desktop/Akash/Path_planning/Generator/LooseFiles/resultRRT_0.15_single.txt", 'w')

f.write("RRT : - "+'\n')
f.write("Average Time = " + str(np.sum(np.array([X for X in end_timeRRT 
if X !='SEG FAULT']) 
- np.array([X for X in st_timeRRT if X !='SEG FAULT']))/count_RRT)+'\n')
f.write("Largest Time : - " +  str(max(np.array([X for X in end_timeRRT 
if X !='SEG FAULT'])
 - np.array([X for X in st_timeRRT if X !='SEG FAULT'])))+'\n')
f.write("Smallest Time : - " + str(min(np.array([X for X in end_timeRRT 
if X !='SEG FAULT']) 
- np.array([X for X in st_timeRRT if X !='SEG FAULT'])))+'\n')
f.write("Average Path Length = " + str(np.sum([X for X in pathSizeRRT 
if X !='SEG FAULT'])/count_RRT)+'\n')
f.write("Largest Path Length = " + str(max([X for X in pathSizeRRT 
if X !='SEG FAULT']))+'\n')
f.write("Smallest Path Length = " + str(min([X for X in pathSizeRRT if X !='SEG FAULT']))+'\n')
f.write("Number of valid searches = " + str(count_RRT) + '\n')
f.write("Start_times = " + str(st_timeRRT) + '\n')
f.write("End_times = " + str(end_timeRRT) + '\n')
f.write("Lengths = " + str(pathSizeRRT) + '\n')
f.close()

#i = 0
#st_timeAstar = []
#end_timeAstar = []
#pathSizeAstar = []
#count_Astar = 0;
#
#while(i<1000):
#    print "Astar: - Cycle ", i
#    st_timeAstar.append(time.time())
#    cmd1 = '/home/f1/Desktop/Akash/Path_planning/Generator/LooseFiles/Astar ' + str(i)
#    process = subprocess.Popen(cmd1, stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
#    (output,err)=process.communicate()
#    exit_code = process.wait()
#    end_timeAstar.append(time.time())
#    if(output):        
#        pathSizeAstar.append(int(output))
#        i+=1 
#        count_Astar += 1;
#
#    else:
#        i+=1        
#        print "Seg Fault"
#        st_timeAstar.pop()
#        end_timeAstar.pop()
#        st_timeAstar.append('SEG FAULT')
#        end_timeAstar.append('SEG FAULT')
#        pathSizeAstar.append('SEG FAULT')
#
#        
#        
#f = open("/home/f1/Desktop/Akash/Path_planning/Generator/LooseFiles/resultAstar.txt", 'w')
#
#f.write("Astar : - "+'\n')
#f.write("Average Time = " + str(np.sum(np.array([X for X in end_timeAstar 
#if X !='SEG FAULT']) 
#- np.array([X for X in st_timeAstar if X !='SEG FAULT']))/count_Astar)+'\n')
#f.write("Largest Time : - " +  str(max(np.array([X for X in end_timeAstar 
#if X !='SEG FAULT'])
# - np.array([X for X in st_timeAstar if X !='SEG FAULT'])))+'\n')
#f.write("Smallest Time : - " + str(min(np.array([X for X in end_timeAstar 
#if X !='SEG FAULT']) 
#- np.array([X for X in st_timeAstar if X !='SEG FAULT'])))+'\n')
#f.write("Average Path Length = " + str(np.sum([X for X in pathSizeAstar
#if X !='SEG FAULT'])/count_Astar)+'\n')
#f.write("Largest Path Length = " + str(max([X for X in pathSizeAstar 
#if X !='SEG FAULT']))+'\n')
#f.write("Smallest Path Length = " + str(min([X for X in pathSizeAstar if X !='SEG FAULT']))+'\n')
#f.write("Number of valid searches = " + str(count_Astar) + '\n')
#f.write("Start_times = " + str(st_timeAstar) + '\n')
#f.write("End_times = " + str(end_timeAstar) + '\n')
#f.write("Lengths = " + str(pathSizeAstar) + '\n')
#f.close()
#from threading import Timer
#
#
#
#def kill():
#    try:
#        print "Killing"
#        cmd1 = 'killall PRMDijkstra '
#        subprocess.Popen(cmd1, stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
#    except OSError:
#        pass # ignore
#
#
#
#i = 0
#st_timePRM = []
#end_timePRM = []
#pathSizePRM = []
#count_PRM = 0;
#
#while(i<1000):
#    print "PRM: - Cycle ", i
#    st_timePRM.append(time.time())
#    cmd1 = '/home/f1/Desktop/Akash/Path_planning/Generator/LooseFiles/PRMDijkstra ' + str(i)
#    process = subprocess.Popen(cmd1, stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
#    t = Timer(300, kill)
#    t.start()
#    (output,err)=process.communicate()
#    exit_code = process.wait()
#    t.cancel()
#    end_timePRM.append(time.time())
#    if(output):        
#        pathSizePRM.append(int(output))
#        i+=1 
#        count_PRM += 1;
#    else:
#        i+=1        
#        print "Seg Fault"
#        st_timePRM.pop()
#        end_timePRM.pop()
#        st_timePRM.append('SEG FAULT')
#        end_timePRM.append('SEG FAULT')
#        pathSizePRM.append('SEG FAULT')
#
#        
#        
#f = open("/home/f1/Desktop/Akash/Path_planning/Generator/LooseFiles/resultPRM_500.txt", 'w')
#
#f.write("PRM : - "+'\n')
#f.write("Average Time = " + str(np.sum(np.array([X for X in end_timePRM 
#if X !='SEG FAULT']) 
#- np.array([X for X in st_timePRM if X !='SEG FAULT']))/count_PRM)+'\n')
#f.write("Largest Time : - " +  str(max(np.array([X for X in end_timePRM
#if X !='SEG FAULT'])
# - np.array([X for X in st_timePRM if X !='SEG FAULT'])))+'\n')
#f.write("Smallest Time : - " + str(min(np.array([X for X in end_timePRM
#if X !='SEG FAULT']) 
#- np.array([X for X in st_timePRM if X !='SEG FAULT'])))+'\n')
#f.write("Average Path Length = " + str(np.sum([X for X in pathSizePRM
#if X !='SEG FAULT'])/count_PRM)+'\n')
#f.write("Largest Path Length = " + str(max([X for X in pathSizePRM 
#if X !='SEG FAULT']))+'\n')
#f.write("Smallest Path Length = " + str(min([X for X in pathSizePRM if X !='SEG FAULT']))+'\n')
#f.write("Number of valid searches = " + str(count_PRM) + '\n')
#f.write("Start_times = " + str(st_timePRM) + '\n')
#f.write("End_times = " + str(end_timePRM) + '\n')
#f.write("Lengths = " + str(pathSizePRM) + '\n')
#f.close()
