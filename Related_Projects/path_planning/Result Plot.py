import matplotlib.pyplot as plt

Density = [5, 6, 8, 10, 15, 20]
AHS = [0.712271142907, 0.803805411469, 1.16156155589, 1.09272956656, 2.30317799283, 3.42959228273]
RRT = [4.64698552389, 6.57131851728, 12.6499944138, 13.8534940099, 41.0938716172, 62.8179020533]
Astar = [3.40665845814, 5.03072585266,9.55196212015, 9.69112773322, 60.181295669 , 126.766188779]
#PRM = [2.3999672867, 2.26138239966, 2.23662336988, 4.13226826657, 5.79062772893,  6.98721167106]
PRM_500 = [36.0844333369, 38.5184159098]
AHS_6 = [4.04776828647, 4.80927861101, 7.10710605369, 6.93795888369, 15.6749364933, 21.5217252647]
AHS_10 = [6.96739321466,7.95961140372, 12.2960997123, 11.8866959437,  26.1011856757, 35.9294425877]

AHS_pl = [250,258,276,280,316,335]
RRT_pl = [381, 369, 351, 346, 271, 255]
Astar_pl = [209, 212, 218, 219, 241, 242]
#PRM_pl = [233, 239, 259, 247, 295, 328]
PRM_pl_500 = [260, 292]
AHS_pl_6 = [225,229,239,239,267,285]
AHS_pl_10 = [220, 225, 232,232, 258, 274]

plt.xlim(0, 25)
plt.ylim(0, 150)

plt.plot(Density, AHS, 'bo', markersize=8)
plt.plot(Density, RRT, 'rx', markersize=8, mew=5)
plt.plot(Density, Astar, 'k+', markersize=10, mew=5)
#plt.plot(Density, PRM, 'mp', markersize=8)
plt.plot([15,20], PRM_500, 'mp', markersize=10)
plt.plot(Density, AHS_6, 'go', markersize=8)
plt.plot(Density, AHS_10, 'co', markersize=8)


plt.plot(Density, AHS, 'b', linewidth=2 )
plt.plot(Density, RRT, 'r' ,linewidth=2)
plt.plot(Density, Astar,'k' ,linewidth=2)
#plt.plot(Density, PRM, 'm', linewidth=2)
plt.plot([15,20], PRM_500, 'm', markersize=10)
plt.plot(Density, AHS_6, 'g', linewidth=2)
plt.plot(Density, AHS_10, 'c', linewidth=2)

plt.ylabel('Average Time taken (Seconds)')
plt.xlabel('Density Percentage (%)')
plt.legend(['AHS Time','RRT Time','A* Time','PRM Time', 'AHS_6_Agents', 'AHS_10_Agents'], loc = 'upper left')
plt.plot()
plt.savefig('Result_Time.png')
plt.show()

plt.xlim(0, 25)
plt.ylim(200, 600)
plt.plot(Density, AHS_pl, 'b*', markersize=8)
plt.plot(Density, RRT_pl, 'r|', markersize=8, mew=5)
plt.plot(Density, Astar_pl, 'k_', markersize=8, mew=5)
#plt.plot(Density, PRM_pl, 'mp', markersize=8)
plt.plot([15,20], PRM_pl_500, 'mp', markersize=10)
plt.plot(Density, AHS_pl_6, 'g*', markersize=8)
plt.plot(Density, AHS_pl_10, 'c*', markersize=8)

plt.plot(Density, AHS_pl, 'b', linewidth=2 )
plt.plot(Density, RRT_pl, 'r' ,linewidth=2)
plt.plot(Density, Astar_pl,'k' ,linewidth=2)
#plt.plot(Density, PRM_pl, 'm' ,linewidth=2)
plt.plot([15,20], PRM_pl_500, 'm', markersize=10)
plt.plot(Density, AHS_pl_6, 'g', linewidth=2 )
plt.plot(Density, AHS_pl_10, 'c', linewidth=2 )

plt.xlabel('Density Percentage (%)')
plt.ylabel('Average Path Length')
plt.legend(['AHS Length','RRT Length','A* Length','PRM Length', 'AHS_6_Agents', 'AHS_10_Agents'], loc = 'upper right')

plt.plot()
plt.savefig('Result_Length.png')
plt.show()

import matplotlib.pyplot as plt