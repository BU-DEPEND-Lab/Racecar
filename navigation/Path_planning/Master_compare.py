from subprocess import call
import subprocess
import time
import numpy as np
import os

st_timeCustom = []
end_timeCustom = []
pathSizeCustom = []
i = 0
count_Custom = 0;
while i< 1000:
    print "Custom: - Cycle ",i
    st_timeCustom.append(time.time())
    cmd1 = '/home/a/Desktop/Path_planning/Generator/LooseFiles/Planning_Gazebo_Customized ' + str(i)
    process = subprocess.Popen(cmd1, stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
    (output,err)=process.communicate()
    exit_code = process.wait()    
    end_timeCustom.append(time.time())
    if(output):        
        pathSizeCustom.append(int(output))
        i+=1      
        count_Custom+=1
    else:
        i+=1        
        print "Seg Fault"
        st_timeCustom.pop()
        end_timeCustom.pop()
i = 0
st_timeRRT = []
end_timeRRT = []
pathSizeRRT = []
count_RRT = 0;

while(i<1000):
    print "RRT: - Cycle ", i
    st_timeRRT.append(time.time())
    cmd1 = '/home/a/Desktop/Path_planning/Generator/LooseFiles/RRT ' + str(i)
    process = subprocess.Popen(cmd1, stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
    (output,err)=process.communicate()
    exit_code = process.wait()
    end_timeRRT.append(time.time())
    if(output):        
        pathSizeRRT.append(int(output))
        i+=1 
        count_RRT += 1;

    else:
        i+=1        
        print "Seg Fault"
        st_timeRRT.pop()
        end_timeRRT.pop()

i = 0
st_timeAstar = []
end_timeAstar = []
pathSizeAstar = []
count_Astar = 0;

while(i<1000):
    print "Astar: - Cycle ", i
    st_timeAstar.append(time.time())
    cmd1 = '/home/a/Desktop/Path_planning/Generator/LooseFiles//Astar ' + str(i)
    process = subprocess.Popen(cmd1, stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
    (output,err)=process.communicate()
    exit_code = process.wait()
    end_timeAstar.append(time.time())
    if(output):        
        pathSizeAstar.append(int(output))
        i+=1   
        count_Astar += 1;

    else:
        i+=1        
        print "Seg Fault"
        st_timeAstar.pop()
        end_timeAstar.pop()



'''st_timeCustom_opt = []
end_timeCustom_opt = []
pathSizeCustom_opt = []
i = 0
while i<100:
    print "Custom_opt: - Cycle ",i
    st_timeCustom_opt.append(time.time())
    cmd1 = '/home/f1/Desktop/LooseFiles/Planning_Gazebo_Customized_opt'
    process = subprocess.Popen(cmd1, stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
    (output,err)=process.communicate()
    exit_code = process.wait()    
    end_timeCustom_opt.append(time.time())
    if(output):        
        pathSizeCustom_opt.append(int(output))
        i+=1        
    else:
        print "Seg Fault"
        st_timeCustom_opt.pop()
        end_timeCustom_opt.pop()
        '''

f = open("/home/a/Desktop/Path_planning/Generator/LooseFiles/result.txt", 'w')

f.write("RRT : - ")
f.write("Average Time = " , np.sum(np.array(end_timeRRT) - np.array(st_timeRRT))/count_RRT)
f.write("Largest Time : - ", max(np.array(end_timeRRT) - np.array(st_timeRRT)))
f.write("Smallest Time : - ", min(np.array(end_timeRRT) - np.array(st_timeRRT)))
f.write("Average Path Length = " , np.sum(pathSizeRRT)/count_RRT)
f.write("Largest Path Length = " , max(pathSizeRRT))
f.write("Smallest Path Length = " , min(pathSizeRRT))

f.write("Astar: - ")
f.write("Average Time = " , np.sum(np.array(end_timeAstar) - np.array(st_timeAstar))/count_Astar)
f.write("Largest Time : - ", max(np.array(end_timeAstar) - np.array(st_timeAstar)))
f.write("Smallest Time : - ", min(np.array(end_timeAstar) - np.array(st_timeAstar)))
f.write("Average Path Length = " , np.sum(pathSizeAstar)/count_Astar)
f.write("Largest Path Length = " , max(pathSizeAstar))
f.write("Smallest Path Length = " , min(pathSizeAstar))

f.write("Custom( Dubbed Ant Hill Search) : - ")
f.write("Average Time = " ,  np.sum(np.array(end_timeCustom) - np.array(st_timeCustom))/count_Custom)
f.write("Largest Time = " ,  max(np.array(end_timeCustom) - np.array(st_timeCustom)))
f.write("Smallest Time = " ,  min(np.array(end_timeCustom) - np.array(st_timeCustom)))
f.write("Average Path Length = " , np.sum(pathSizeCustom)/count_Custom)
f.write("Largest Path Length = " , max(pathSizeCustom))
f.write("Smallest Path Length = " , min(pathSizeCustom))
'''
f.write("Custom_opt( Dubbed Ant Hill Search) : - ")
f.write("Average Time = " ,  np.sum(np.array(end_timeCustom_opt) - np.array(st_timeCustom_opt))/100)
f.write("Largest Time = " ,  max(np.array(end_timeCustom_opt) - np.array(st_timeCustom_opt)))
f.write( "Smallest Time = " ,  min(np.array(end_timeCustom_opt) - np.array(st_timeCustom_opt)))
f.write( "Average Path Length = " , np.sum(pathSizeCustom_opt)/100)
f.write( "Largest Path Length = " , max(pathSizeCustom_opt))
f.write( "Smallest Path Length = " , min(pathSizeCustom_opt))
'''
f.close()