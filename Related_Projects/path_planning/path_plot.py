import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import cv2
def rgb2gray(rgb):
    return np.dot(rgb[...,:3], [0.299, 0.587, 0.114])
    
fname = '/home/f1/Desktop/Akash/Path_planning/Generator/LooseFiles/Maps/map4.png'
image = mpimg.imread(fname)     
gray = rgb2gray(image)
kernel = np.ones((5,5), np.uint8)

gray = cv2.erode(gray, kernel, iterations=3)

height = gray[950:, 950:].shape[0]
width = gray[950:, 950:].shape[1]
for i in range(height - 1):
    for j in range(int(width * float(height - i) / height)):
        temp = gray[i][j]
        gray[i][j] = gray[j][i]
        gray[j][i] = temp
plt.imshow(gray[950:, 950:], cmap = plt.get_cmap('gray'))

f = open('/home/f1/Desktop/Akash/Path_planning/Generator/LooseFiles/Paths_AHS/path4.txt','r')
message = f.read()
num = map(float,message.splitlines())
del num[2::3]
b = num[::2]
c = num[1::2]
b = [(x*20)+75 for x in b ]
c = [(x*20)+75 for x in c ]

plt.scatter(b,c, s=2, c='b', marker = 'x')
#plt.show()
f = open('/home/f1/Desktop/Akash/Path_planning/Generator/LooseFiles/Paths_RRT/path4.txt','r')
message = f.read()
num = map(float,message.splitlines())
del num[2::3]
b = num[::2]
c = num[1::2]
b = [(x*20)+73 for x in b ]
c = [(x*20)+73 for x in c ]
plt.scatter(b,c, s=2, c='r', marker = 'x')
#plt.show()
f = open('/home/f1/Desktop/Akash/Path_planning/Generator/LooseFiles/Paths_Astar/path4.txt','r')
message = f.read()
num = map(float,message.splitlines())
del num[2::3]
b = num[::2]
c = num[1::2]
b = [(x*20)+73 for x in b ]
c = [(x*20)+73 for x in c ]
plt.scatter(b,c, s=2, c='g', marker = 'x')

f = open('/home/f1/Desktop/Akash/Path_planning/Generator/LooseFiles/Paths_PRM/path4.txt','r')
message = f.read()
num = map(float,message.splitlines())
del num[2::3]
b = num[::2]
c = num[1::2]
b = [(x*20)+73 for x in b ]
c = [(x*20)+73 for x in c ]
plt.scatter(b,c, s=2, c='c', marker = 'x')
plt.legend(["AHS", "RRT", "A*", "PRM"])
plt.savefig("/home/f1/Desktop/Akash/Path_planning/Generator/LooseFiles/Plot_10_2.png")
plt.show()
