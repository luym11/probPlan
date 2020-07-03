import matplotlib.pyplot as plt
import numpy as np
import pickle

def plotPathPic(path, path2,path3, monteCarloAverageFireMap,k):
    fig = plt.figure()
    ax = fig.gca()
    min_val, max_val = 0, 20
    intersection_matrix = np.transpose(monteCarloAverageFireMap[k])
    ax.matshow(intersection_matrix, cmap=plt.cm.Reds, origin='lower')
    ax.set_xticks(np.arange(0,20,1))
    ax.set_yticks(np.arange(0,20,1))
    plt.xlim(0,20)
    plt.ylim(0,20)
    plt.plot(8, 0, 'bD')
    # plt.plot(0, 16, 'bD')
    plt.plot(11, 19, 'gD')
    Fire = [[9,12], [3,6],(7,9)]
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
    Target = [[8,16]]
    Target_array = np.asarray(Target)
    plt.plot(Target_array[:,0], Target_array[:,1], 'yD')
    plt.grid(linestyle = '--')
    plt.gca().set_aspect('equal', adjustable='box')
    pathArray = np.array(path)
    x = pathArray[:,0]
    y = pathArray[:,1]
    plt.plot(x,y,c='m')
    pathArray2 = np.array(path2)
    x2 = pathArray2[:,0]
    y2 = pathArray2[:,1]
    plt.plot(x2,y2,c='g')
    # pathArray3 = np.array(path3)
    # x3 = pathArray3[:,0]
    # y3 = pathArray3[:,1]
    # plt.plot(x3,y3,c='c')
    plt.show()

if __name__ == '__main__':

    monteCarloAverageFireMap = pickle.load(open('MCAFMs1000', "rb"))
    aDP1 = pickle.load(open('aDP1mkxx',"rb"))

    oldPathFull = [(8, 0, -1),
    (8, 1, -1),
    (8, 2, -1),
    (8, 3, -1),
    (8, 4, -1),
    (8, 5, -1),
    (8, 6, -1),
    (8, 7, -1),
    (8, 8, -1),
    (7, 8, -1),
    (7, 9, -1),
    (6, 9, -1),
    (6, 10, -1),
    (5, 10, -1),
    (5, 11, -1),
    (5, 12, -1),
    (5, 13, -1),
    (5, 14, -1),
    (5, 15, -1),
    (5, 16, -1),
    (6, 16, -1),
    (7, 16, -1),
    (8, 16, -2),
    (8, 17, -2),
    (8, 18, -2),
    (8, 19, -2),
    (9, 19, -2),
    (10, 19, -2),
    (11, 19, -3)]

    oldPath = [(7, 8, -1),
    (7, 9, -1),
    (6, 9, -1),
    (6, 10, -1),
    (5, 10, -1),
    (5, 11, -1),
    (5, 12, -1),
    (5, 13, -1),
    (5, 14, -1),
    (5, 15, -1),
    (5, 16, -1),
    (6, 16, -1),
    (7, 16, -1),
    (8, 16, -2),
    (8, 17, -2),
    (8, 18, -2),
    (8, 19, -2),
    (9, 19, -2),
    (10, 19, -2),
    (11, 19, -3)]

    dStarPath = [(8, 7, -1),
    (7, 7, -1),
    (6, 7, -1),
    (5, 7, -1),
    (5,8,-1),
    (5,9,-1),
    (5, 10, -1),
    (5, 11, -1),
    (5, 12, -1),
    (5, 13, -1),
    (5, 14, -1),
    (5, 15, -1),
    (5, 16, -1),
    (6, 16, -1),
    (7, 16, -1),
    (8, 16, -2),
    (8, 17, -2),
    (8, 18, -2),
    (8, 19, -2),
    (9, 19, -2),
    (10, 19, -2),
    (11, 19, -3)]

    newPath5 = [(9, 8, -1),
    (10, 8, -1),
    (11, 8, -1),
    (12, 8, -1),
    (13, 8, -1),
    (14, 8, -1),
    (14, 9, -1),
    (14, 10, -1),
    (14, 11, -1),
    (14, 12, -1),
    (14, 13, -1),
    (14, 14, -1),
    (14, 15, -1),
    (14, 16, -1),
    (13, 16, -1),
    (12, 16, -1),
    (11, 16, -1),
    (10, 16, -1),
    (9, 16, -1),
    (8, 16, -2),
    (8, 17, -2),
    (8, 18, -2),
    (8, 19, -2),
    (9, 19, -2),
    (10, 19, -2),
    (11, 19, -3)]

    newPath55 =[(9, 8, -1),
    (10, 8, -1),
    (11, 8, -1),
    (12, 8, -1),
    (12, 9, -1),
    (13, 9, -1),
    (14, 9, -1),
    (14, 10, -1),
    (14, 11, -1),
    (14, 12, -1),
    (14, 13, -1),
    (14, 14, -1),
    (14, 15, -1),
    (14, 16, -1),
    (13, 16, -1),
    (12, 16, -1),
    (11, 16, -1),
    (10, 16, -1),
    (9, 16, -1),
    (8, 16, -2),
    (8, 17, -2),
    (8, 18, -2),
    (8, 19, -2),
    (9, 19, -2),
    (10, 19, -2),
    (11, 19, -3)]

    newPath3 = [(7, 8, -1),
    (6,8, -1),
    (5, 8, -1),
    (5, 9, -1),
    (5, 10, -1),
    (5, 11, -1),
    (5, 12, -1),
    (5, 13, -1),
    (5, 14, -1),
    (5, 15, -1),
    (5, 16, -1),
    (6, 16, -1),
    (7, 16, -1),
    (8, 16, -2),
    (8, 17, -2),
    (8, 18, -2),
    (8, 19, -2),
    (9, 19, -2),
    (10, 19, -2),
    (11, 19, -3)]

    connectPath = [(8, 8, -1),
    (8, 9, -1),
    (7, 9, -1)]


    plotPathPic(path, connectPath, connectPath, aDP1.updatedAverageFireMap,9)


