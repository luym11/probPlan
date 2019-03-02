import numpy as np
from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
from matplotlib import cm

def pre_process(fireMapArray):
    n = len(fireMapArray[0])
    m = len(fireMapArray[0][0])
    _fireMapArray = np.asarray(fireMapArray)
    _fireMapArray[_fireMapArray> 0] = 1
    for t in range(len(_fireMapArray)):
        for i in range(n):
            for j in range(m):
                _fireMapArray[t][i][j] = _fireMapArray[t][i][j] * (t+1)
    return _fireMapArray

def press_mapArray(fireMapArray):
    # press to a map, with different firemap value indicating time
    n = len(fireMapArray[0])
    m = len(fireMapArray[0][0])
    pressedFireMap = np.copy(fireMapArray[0])
    for t in range(len(fireMapArray)-1):
        for i in range(n):
            for j in range(m):
                if fireMapArray[t+1][i][j] > 0 and not pressedFireMap[i][j] > 0:
                    pressedFireMap[i][j] = fireMapArray[t+1][i][j]
    return pressedFireMap

def static_representation(_map, fireMapArray):
    # generate 3d representation of static things: startPoint, target, (endPoint), wall
    T = len(fireMapArray)
    startPoint3d = strech_generator(_map, -1, T, fireMapArray)
    targetPoint3d = strech_generator(_map, -2, T, fireMapArray)
    wall3d = strech_generator(_map, 1, T, fireMapArray)
    return startPoint3d, targetPoint3d, wall3d

def strech_generator(_map, value, horizon, fireMapArray):
    # called in static_representaiton
    n = len(fireMapArray[0])
    m = len(fireMapArray[0][0])
    T = len(fireMapArray)
    _strechedMap = np.zeros((T, n, m), dtype=int)
    # another stupid implementation
    for t in range(T):
        for x in range(n):
            for y in range(m):
                if _map[x][y] == value:
                    _strechedMap[t][x][y] = t
    return _strechedMap

def plot_process(fireMapArray, path, _map, startPoint, target, wall):
    processedFireMapArray = pre_process(fireMapArray)
    pressedFireMap = press_mapArray(processedFireMapArray)
    pressedMax = np.max(pressedFireMap)
    pressedFireMap[pressedFireMap <= 0.1] = pressedMax

    n = len(processedFireMapArray[0])
    m = len(processedFireMapArray[0][0])
    T = len(fireMapArray)
    x = np.arange(0,n,1)
    y = np.arange(0,m,1)
    xs, ys = np.meshgrid(x, y)
    zs = np.asarray(pressedFireMap)
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot_surface(ys, xs, zs, rstride=1, cstride=1, cmap='hot')
    # statics
    wallArray = np.asarray(wall)
    startPointArray = np.asarray(startPoint)
    targetArray = np.asarray(target)
    ax.scatter(wallArray[:,0],wallArray[:,1], T+3, color='k', marker='s', linewidth=5)
    ax.scatter(startPointArray[:,0], startPointArray[:,1], T+3, color='r', linewidth=5)
    ax.scatter(targetArray[:,0], targetArray[:,1], T+3, color='b', linewidth=5)

    ax.set_ylim(ax.get_ylim()[::-1])
    ax.set_zlim(ax.get_zlim()[::-1])
    ax.view_init(elev=-158, azim=-48)

    # plot path
    try: 
        ax.plot([x for (x, y, c) in path], [y for (x, y, c) in path], [c for (x, y, c) in path], color='b', label='parametric curve', linewidth=4)#, fmt='ro-', linewidth=2)
    except:
        print("DP didn't find a proper path")


    plt.show()
    # fig.savefig('combined_planning_3d.jpeg')