import matplotlib.pyplot as plt
from collections import defaultdict
import numpy as np
#files = ["AHS", "AHS_multi", "AHS_multi_10", "RRT", "Astar"] 
#folders = ["0.05","0.06","0.08","0.10","0.15","0.20"]
#variance = defaultdict(list)
#standard_div = defaultdict(list)
#for fol in folders:
#    for fil in files:
#        f = open("/home/f1/Desktop/Akash/Path_planning/Generator/LooseFiles/Results/Density"+fol+"/result"+fil+".txt", 'r')
#        f.readline();
#        f.readline();
#        f.readline();
#        f.readline();
#        f.readline();
#        f.readline();
#        f.readline();
#        f.readline();
#        st_line = f.readline();
#        st_line = st_line[14:]
#        st_line = eval(st_line)
#        e_line = f.readline();
#        e_line = e_line[12:]
#        e_line = eval(e_line)
#        time_diff = []
#        for i in range(len(e_line)):
#            if(e_line[i]!="SEG FAULT"):
#                time_diff.append(e_line[i] - st_line[i]) 
##        plt.hist(time_diff, 100, alpha=0.75)
#        standard_div[fil].append(np.std(time_diff))
#        variance[fil].append(np.var(time_diff, dtype=np.float64))
#        plt.savefig('Histograms/Density_'+fol+'/Hist_Time_'+fil+'.png')
#        plt.show()
#        lengths = f.readline();
#        lengths = eval(lengths[10:])
#        lengths[:] = [x for x in lengths if (x !="SEG FAULT" and x>200)] 
#        plt.hist(lengths, 100, alpha=0.75)
#        plt.savefig('Histograms/Density_'+fol+'/Hist_Lengths_'+fil+'.png')
#        plt.show()
#    plt.legend(["AHS", "AHS_multi", "AHS_multi_10"], loc = 'upper right')
#    plt.savefig('Histograms/Density_'+fol+'/Hist_Time__all_AHS.png')
#    plt.show()
        

f = open("/home/f1/Desktop/Akash/Path_planning/Generator/LooseFiles/resultRRT_0.20_single.txt", 'r')
f.readline();
f.readline();
f.readline();
f.readline();
f.readline();
f.readline();
f.readline();
f.readline();
st_line = f.readline();
st_line = st_line[14:]
st_line = eval(st_line)
e_line = f.readline();
e_line = e_line[12:]
e_line = eval(e_line)
time_diff = []
for i in range(len(e_line)):
    if(e_line[i]!="SEG FAULT"):
        time_diff.append(e_line[i] - st_line[i]) 
plt.hist(time_diff, 100, alpha=0.75)
plt.xlabel("Seconds")
plt.ylabel("Population")
#standard_div[fil].append(np.std(time_diff))
#variance[fil].append(np.var(time_diff, dtype=np.float64))
#plt.savefig('')
plt.savefig('Histograms/Density_0.05/Hist_Time_RRT_0.20_Single.png')
plt.show()
lengths = f.readline();
lengths = eval(lengths[10:])
lengths[:] = [x for x in lengths if (x !="SEG FAULT" and x>200)] 
plt.hist(lengths, 100, alpha=0.75)
plt.xlabel("Steps")
plt.ylabel("Population")
plt.savefig('Histograms/Density_0.05/Hist_Lengths_RRT_0.20_Single.png')
plt.show()
#plt.legend(["AHS", "AHS_multi", "AHS_multi_10"], loc = 'upper right')
