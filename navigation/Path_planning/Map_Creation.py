from subprocess import call
import time
import numpy as np

for i in range(1000):
    call("java Generator_obs "+str(i), shell = True)
