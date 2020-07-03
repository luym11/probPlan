from astar import AStar
import sys
import math
import matplotlib.pyplot as plt
import problem_setting
from path_evaluation import PathEvaluation
import numpy as np
import time
import pickle
import seaborn as sns
import os
from utils.plotPath import plotPathPic

class MaxProbAstar(AStar):
    def __init__(self, M, N, Wall, monteCarloFireMap, startTime=0, xT=0,yT=0,observedFireLocationArray = None,searchSize = 2):
        self.M = M
        self.N = N
        self.Wall = Wall
        self.startTime = startTime
        self.xT = xT
        self.yT = yT
        self.observedFireLocationArray = observedFireLocationArray
        self.searchSize = searchSize
        self.pe = PathEvaluation(monteCarloFireMap,xT=xT,yT=yT)
        self.savedSafeProb = np.ones([20,20,20,20,20]) * -1

    def heuristic_cost_estimate(self, n1, n2):
        return 0

    def distance_between(self, n1, n2):
        # -log(Pr(n2 is safe | n1 is safe & obsv))
        # Note n1 and n2 are always neighbors
        pathToN1 = list(self.reconstruct_path(self.searchNodes[n1], False))
        t = len(pathToN1) - 1 + self.startTime # not the best way, can store this t in the node
        try:
            if self.savedSafeProb[t,n1[0],n1[1],n2[0],n2[1]] != -1:
                safeProb = self.savedSafeProb[t,n1[0],n1[1],n2[0],n2[1]] 
            else:
                safeProb = self.pe.evaluate_segment_obsv(t,n1,t+1,n2,self.startTime,self.observedFireLocationArray)
                self.savedSafeProb[t,n1[0],n1[1],n2[0],n2[1]]  = safeProb
        except:
            print('n1 is {0}'.format(n1))
            print('n2 is {0}'.format(n2))
            print('observedArray is {0}'.format(self.observedFireLocationArray))
        # assert safeProb != 0, "safeProb is 0!"
        if safeProb == 0:
            return float('inf')
        return -math.log(safeProb)

    def neighbors(self, node):
        x, y = node
        return[(nx, ny) for nx, ny in[(x, y - 1), (x, y + 1), (x - 1, y), (x + 1, y)]if 0 <= nx < self.N and 0 <= ny < self.M and not [nx,ny] in self.Wall and self.xT-self.searchSize <= nx <= self.xT+self.searchSize and self.yT-self.searchSize <= ny <= self.yT+self.searchSize]
    
    def is_goal_reached(self, current, goal):
        return current == goal

if __name__ == '__main__':
    # aim to find safest path between two points
    # thus, only need a start point and a end point here,
    # what is in p1 does not matter (see _init_ of this class)
    p1 = problem_setting.ProblemSetting(target = [[18,16]],_stochastic_environment_flag=1, _setting_num=1)
    # monteCarloFireMap read in
    monteCarloAverageFireMap = pickle.load(open('MCAFMs300', "rb"))
    monteCarloFireMap = pickle.load(open('MCFMs300', "rb"))

    # test
    _t = time.time()
    startTime = 3
    _path, searchNodes, lastNode = MaxProbAstar(p1, monteCarloFireMap, startTime).astar((8,0),(10,18))
    path = list(_path)
    elapsed_ = time.time() - _t
    pe=PathEvaluation(monteCarloFireMap)
    pr=pe.evaluate_path_t(path, startTime)
    print(path)
    print('Prob is %f, Computation took: %d seconds' %(pr, elapsed_)) 

    # plot a path
    plotPathPic(path, monteCarloAverageFireMap)