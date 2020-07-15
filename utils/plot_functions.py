import matplotlib.pyplot as plt
import numpy as np
import pickle
import itertools

def plot_path(pathList1, pathList2, intersection_matrix):
    fig = plt.figure()
    ax = fig.gca()
    ax.matshow(intersection_matrix, cmap=plt.cm.Reds, origin='lower')
    ax.set_xticks(np.arange(0,20,1))
    ax.set_yticks(np.arange(0,20,1))
    plt.xlim(0,20)
    plt.ylim(0,20)
    plt.plot(8, 0, 'bD')
    plt.plot(0, 16, 'bD')
    plt.plot(19, 12, 'gD')
    Fire = [[3,12], [9,12]]
    Fire_array = np.asarray(Fire)
    plt.plot(Fire_array[:,0], Fire_array[:,1], 'rx')
    Wall = [[6,0],
        [6,1],
        [6,2],
        [6,3],
        [6,4],
        [6,5],
        [10,0],
        [10,1],
        [10,2],
        [10,3],
        [10,4],
        [10,5],
        [6,14],
        [7,14],
        [8,14],
        [9,14],
        [10,14],
        [11,14],
        [12,14],
        [13,14],
        [15,14],
        [16,14],
        [17,14],
        [18,14],
        [19,14],
        [17,9],
        [17,10],
        [17,11],
        [17,12],
        [17,13]]
    Wall_array = np.asarray(Wall)
    plt.plot(Wall_array[:,0], Wall_array[:,1], 'kp')
    Target = [[2,5],[9,16],[13,16]]
    Target_array = np.asarray(Target)
    plt.plot(Target_array[:,0], Target_array[:,1], 'yD')
    plt.grid(linestyle = '--')
    plt.gca().set_aspect('equal', adjustable='box')
    path1 = list(itertools.chain.from_iterable(pathList1))
    pathArray = np.array(path1)
    x = pathArray[:,0]
    y = pathArray[:,1]
    plt.plot(x,y,c='c')
    path2 = list(itertools.chain.from_iterable(pathList2))
    pathArray2 = np.array(path2)
    x2 = pathArray2[:,0]
    y2 = pathArray2[:,1]
    plt.plot(x2,y2,c='g')
    plt.show()