import threading
import subprocess
import time
import numpy as np

class MyClass(threading.Thread):
    def __init__(self, i):
        self.stdout = None
        self.stderr = None
        threading.Thread.__init__(self)

    def run(self):
        cmd1 = '/home/f1/Desktop/Akash/Path_planning/Generator/LooseFiles/AHS ' + str(i)
        p = subprocess.Popen(cmd1.split(),
                             shell=False,
                             stdout=subprocess.PIPE,
                             stderr=subprocess.PIPE)

        self.stdout, self.stderr = p.communicate()

st_timeCustom = []
end_timeCustom = []
pathSizeCustom = []
count_Custom = 0;
i = 0
while i< 1000:
    out = []
    process = 0
    cmd1 = 0
    print "Custom: - Cycle ",i
    st_timeCustom.append(time.time())
    myclass = MyClass(i)
    myclass.start()
    myclass.join()
    myclass1 = MyClass(i)
    myclass1.start()
    myclass1.join()
    myclass2 = MyClass(i)
    myclass2.start()
    myclass2.join()
    myclass3 = MyClass(i)
    myclass3.start()
    myclass3.join()    
    myclass4 = MyClass(i)
    myclass4.start()
    myclass4.join()
    myclass5 = MyClass(i)
    myclass5.start()
    myclass5.join()
    myclass6 = MyClass(i)
    myclass6.start()
    myclass6.join()
    myclass7 = MyClass(i)
    myclass7.start()
    myclass7.join()    
    myclass8 = MyClass(i)
    myclass8.start()
    myclass8.join()
    myclass9 = MyClass(i)
    myclass9.start()
    myclass9.join()
    out.append(myclass.stdout)
    out.append(myclass1.stdout)
    out.append(myclass2.stdout)
    out.append(myclass3.stdout)
    out.append(myclass4.stdout)
    out.append(myclass5.stdout)
    out.append(myclass6.stdout)
    out.append(myclass7.stdout)
    out.append(myclass8.stdout)
    out.append(myclass9.stdout)
    end_timeCustom.append(time.time())
    flag = 0
    nf = 0
    
    while not flag and len(out)>0:
        if not min(out):
            out.pop(out.index(min(out)))
        else:
            flag = 1
    if(len(out) != 0):
        if(min(list(map(int, out)))):        
            pathSizeCustom.append(min(list(map(int, out))))
            i+=1      
            count_Custom+=1
    else:
        i+=1        
        print "Seg Fault"
        st_timeCustom.pop()
        end_timeCustom.pop()
        st_timeCustom.append('SEG FAULT')
        end_timeCustom.append('SEG FAULT')
        pathSizeCustom.append('SEG FAULT')

f = open("/home/f1/Desktop/Akash/Path_planning/Generator/LooseFiles/resultAHS_multi_10.txt", 'w')

f.write("AHS : - "+'\n')
f.write("Average Time = " + str(np.sum(np.array([X for X in end_timeCustom
if X !='SEG FAULT']) 
- np.array([X for X in st_timeCustom if X !='SEG FAULT']))/count_Custom)+'\n')
f.write("Largest Time : - " +  str(max(np.array([X for X in end_timeCustom 
if X !='SEG FAULT'])
 - np.array([X for X in st_timeCustom if X !='SEG FAULT'])))+'\n')
f.write("Smallest Time : - " + str(min(np.array([X for X in end_timeCustom 
if X !='SEG FAULT']) 
- np.array([X for X in st_timeCustom if X !='SEG FAULT'])))+'\n')
f.write("Average Path Length = " + str(np.sum([X for X in pathSizeCustom 
if X !='SEG FAULT'])/count_Custom)+'\n')
f.write("Largest Path Length = " + str(max([X for X in pathSizeCustom 
if X !='SEG FAULT']))+'\n')
f.write("Smallest Path Length = " + str(min([X for X in pathSizeCustom if X !='SEG FAULT']))+'\n')
f.write("Number of valid searches = " + str(count_Custom) + '\n')
f.write("Start_times = " + str(st_timeCustom) + '\n')
f.write("End_times = " + str(end_timeCustom) + '\n')
f.write("Lengths = " + str(pathSizeCustom) + '\n')
f.close()