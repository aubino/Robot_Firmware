import os 
import numpy as np 
from matplotlib import pyplot as plt
saving_dir = "/home/aubin/pid_report"
selected_curves = {} 
for dir in os.listdir(saving_dir) : 
    speeds : np.ndarray = np.load(os.path.join(saving_dir,dir,"speeds.npy"))
    times : np.ndarray  = np.load(os.path.join(saving_dir,dir,"times.npy"))
    if (np.std(speeds)<0.2*4*3.1415) : 
        speed_further = speeds[int(3*speeds.size/4): speeds.size]
        if (np.std(speed_further)<0.05*4*3.1415) : 
            name = dir.split("/")[-1]
            selected_curves[name] = (speeds,times)
            plt.plot(speeds)
            plt.show()
