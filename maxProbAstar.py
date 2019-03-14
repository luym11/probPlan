from astar import AStar
import sys
import math
import matplotlib.pyplot as plt
import problem_setting
from path_evaluation import PathEvaluation
import numpy as np
import time


class MaxProbAstar(AStar):
    def __init__(self, p):
        # p is the problem
        self.pe = PathEvaluation(monteCarloFireMap) # pe is PathEvaluation instance
        self.M = p.M
        self.N = p.N
        self.Wall = p.Wall

    def heuristic_cost_estimate(self, n1, n2):
        return 0

    def distance_between(self, n1, n2):
        # -log(Pr(till n2 is safe | till n1 is safe))
        pathToN1 = list(self.reconstruct_path(self.searchNodes[n1], False))
        pathToN2 = pathToN1.copy()
        pathToN2.append(n2)
        p1 = self.pe.evaluate_path(pathToN1)
        p2 = self.pe.evaluate_path(pathToN2)
        if p2 == 0 or p1 == 0:
            return float('inf')
        # print('try moving from (%d,%d) to (%d,%d)' %(self.searchNodes[n1].data[0],self.searchNodes[n1].data[1],self.searchNodes[n2].data[0],self.searchNodes[n2].data[1]))
        # print('path to be: ')
        # print(pathToN1)
        # print(pathToN2)
        # print(p2/p1)
        # print()
        return -math.log(p2/p1)

    def neighbors(self, node):
        x, y = node
        return[(nx, ny) for nx, ny in[(x, y - 1), (x, y + 1), (x - 1, y), (x + 1, y)]if 0 <= nx < self.N and 0 <= ny < self.M and not [nx,ny] in self.Wall]
    
    def is_goal_reached(self, current, goal):
        return current == goal

if __name__ == '__main__':
    p1 = problem_setting.ProblemSetting(_stochastic_environment_flag=1, _setting_num=1)
    # monteCarloFireMap read in
    monteCarloAverageFireMap = [[] for i in range(len(p1.FireMap))]
    monteCarloFireMap = [[[] for k in range(len(p1.FireMap))] for h in range(p1.monteCarloHorizon)]
    for k in range(len(p1.FireMap)):
        monteCarloAverageFireMap[k] = np.loadtxt('./monteCarloAverage/monteCarloAverageFireMap'+str(k)+'.txt')
    for h in range(p1.monteCarloHorizon):
        for k in range(len(p1.FireMap)):
            monteCarloFireMap[h][k] = np.loadtxt('./monteCarlo/monteCarloFireMapTrial'+str(h)+'/monteCarloFireMapAt'+str(k)+'.txt')
    _t = time.time()
    _path, searchNodes, lastNode = MaxProbAstar(p1).astar((8,0),(12,16))
    path = list(_path)
    elapsed_ = time.time() - _t
    pe=PathEvaluation(monteCarloFireMap)
    pr=pe.evaluate_path(path)
    print('Prob is %f, Computation took: %d seconds' %(pr, elapsed_)) 