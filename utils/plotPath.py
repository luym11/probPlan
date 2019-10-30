import matplotlib.pyplot as plt
import numpy as np
import pickle

def plotPathPic(path, monteCarloAverageFireMap):
    fig = plt.figure()
    ax = fig.gca()
    min_val, max_val = 0, 20
    intersection_matrix = np.transpose(monteCarloAverageFireMap[25])
    ax.matshow(intersection_matrix, cmap=plt.cm.Reds, origin='lower')
    ax.set_xticks(np.arange(0,20,1))
    ax.set_yticks(np.arange(0,20,1))
    plt.xlim(0,20)
    plt.ylim(0,20)
    plt.plot(8, 0, 'bD')
    Fire = [[3,12], [18,18],[13,12]]
    Fire_array = np.asarray(Fire)
    plt.plot(Fire_array[:,0], Fire_array[:,1], 'rx')
    plt.grid(linestyle = '--')
    plt.gca().set_aspect('equal', adjustable='box')
    pathArray = np.array(path)
    x = pathArray[:,0]
    y = pathArray[:,1]
    plt.plot(x,y,c='c')
    plt.show()