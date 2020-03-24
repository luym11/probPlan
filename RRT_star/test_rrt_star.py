# This part of code is the main function. It does the following things: 
# 1. Rescue problem setup
# 2. Ground truth (DP) solution
# 3. Other solutions (time varying RRT-star)
# 4. Plots



import numpy as np
import math
import csv
import time
import matplotlib.pyplot as plt
import problem_setting
import automaton_dp
import rrt_star
import plot3d_fire
import time


# problem setup
p1 = problem_setting.ProblemSetting(_stochastic_environment_flag=1, _setting_num=1)
##########################################################################
# Original DP approach

# monteCarloFireMap read in
_t = time.time()
monteCarloAverageFireMap = [[] for i in range(len(p1.FireMap))]
monteCarloFireMap = [[[] for k in range(len(p1.FireMap))] for h in range(p1.monteCarloHorizon)]
for k in range(len(p1.FireMap)):
    monteCarloAverageFireMap[k] = np.loadtxt('./monteCarloAverage/monteCarloAverageFireMap'+str(k)+'.txt')
for h in range(p1.monteCarloHorizon):
    for k in range(len(p1.FireMap)):
        monteCarloFireMap[h][k] = np.loadtxt('./monteCarlo/monteCarloFireMapTrial'+str(h)+'/monteCarloFireMapAt'+str(k)+'.txt')
elapsed_ = time.time() - _t
print('Loading took: %d seconds' %(elapsed_))

# aDP1 = automaton_dp.AutomatonDP(p1.M,p1.N,p1.T,p1.Map,monteCarloAverageFireMap, monteCarloFireMap, p1.StartPoint,p1.EndPoint,p1.Target,p1.Wall,p1.Fire,Q=4)
# aDP1.solve()


########################################################################
# RRT star method
# settings for RRT-star
mIter = 1000 # max sample number
show_animation = False

fireList_array_array = [ [np.nonzero(firemap)[0][:], np.nonzero(firemap)[1][:]] for firemap in p1.FireMap]

fireList_tuple_array = [ [tuple([arr[0][i], arr[1][i], math.sqrt(2)]) for i in range(len(arr[0]))] for arr in fireList_array_array ]

print("start RRT star path planning")
Fire_w_size = [ [p1.Fire[i][0], p1.Fire[i][1], 0.6] for i in range(len(p1.Fire)) ]
Wall_w_size = [ [p1.Wall[i][0], p1.Wall[i][1], 0.6] for i in range(len(p1.Wall)) ]
fireList = [tuple(Fire_w_size[i]) for i in range(len(p1.Fire))]
wallList = [tuple(Wall_w_size[i]) for i in range(len(p1.Wall))]

rrt = rrt_star.RRT(start=p1.StartPoint[0], goal=p1.Target[0],
              wallList=wallList, fireList=fireList, fireMap=monteCarloFireMap, randArea=[0, p1.M-1], maxIter=mIter)

path = rrt.Planning(animation=show_animation)

# 3d plot function for visualization
#plot3d_fire.plot_process_2(p1.FireMap, path, aDP1.path, p1.Map, p1.StartPoint, p1.Target, p1.Wall)

# Draw final path
rrt.DrawGraph()
try:  
    plt.plot([x for (x, y, c, p) in path], [y for (x, y, c, p) in path], '-r')
except:
    print("RRT didn't find a proper path in 2D")
try:  
    plt.plot([x for (x, y, c) in aDP1.path], [y for (x, y, c) in aDP1.path], '-b')
except:
    print("DP didn't find a proper path in 2D")
plt.axis([0, p1.M-1, 0, p1.M-1])
plt.grid(True)    
# plt.show()
plt.savefig('combined_planning_2d.jpeg')