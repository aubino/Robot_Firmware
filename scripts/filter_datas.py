import os 
import numpy as np 
import matplotlib
from matplotlib import pyplot as plt
plt.rcParams["figure.autolayout"] = True
# matplotlib.use("gtk")
saving_dir = "/home/aubin/pid_report"
selected_curves = {} 
selected_list = []
fig, ax = plt.subplots()
for dir in os.listdir(saving_dir) : 
    speeds : np.ndarray = np.load(os.path.join(saving_dir,dir,"speeds.npy"))
    times : np.ndarray  = np.load(os.path.join(saving_dir,dir,"times.npy"))
    times = times #- times[0] 
    if (np.std(speeds)<0.5125*4*3.1415) : 
        # speed_further = speeds[int(3*speeds.size/4): speeds.size]
        # if (np.std(speed_further)<0.25*4*3.1415) : 
        name = dir.split("/")[-1]
        selected_curves[name] = (speeds,times)
        selected_list.append(speeds)
        ax.plot(speeds,'tab:orange')
        ax.set_title(name)
        ax.plot(speeds)
        plt.savefig(name +".png" )
        ax.clear()
        # plt.show()
        # selected_list.append(name)
        # plt.show(block=False)
