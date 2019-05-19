from astar import AStar
import sys
import math
import matplotlib.pyplot as plt
import problem_setting
from path_evaluation import PathEvaluation
import numpy as np
import time


class MaxProbAstar(AStar):
    def __init__(self, p, monteCarloFireMap, _target=[-1,-1]):
        # p is the problem
        self.pe = PathEvaluation(monteCarloFireMap, target=_target) # pe is PathEvaluation instance
        self.M = p.M
        self.N = p.N
        self.Wall = p.Wall

    def heuristic_cost_estimate(self, n1, n2):
        return 0

    def distance_between(self, n1, n2):
        # -log(Pr(till n2 is safe | till n1 is safe))
        pathToN1 = list(self.reconstruct_path(self.searchNodes[n1], False))
        t = len(pathToN1) - 1 # not the best way, can store this t in the node
        safeProb = self.pe.evaluate_segment(t,n1,n2)
        if safeProb == 0:
            return float('inf')
        return -math.log(safeProb)

    def neighbors(self, node):
        x, y = node
        return[(nx, ny) for nx, ny in[(x, y - 1), (x, y + 1), (x - 1, y), (x + 1, y)]if 0 <= nx < self.N and 0 <= ny < self.M and not [nx,ny] in self.Wall]
    
    def is_goal_reached(self, current, goal):
        return current == goal

if __name__ == '__main__':
    p1 = problem_setting.ProblemSetting(target = [[10, 18]],_stochastic_environment_flag=1, _setting_num=1)
    # monteCarloFireMap read in
    monteCarloAverageFireMap = [[] for i in range(len(p1.FireMap))]
    monteCarloFireMap = [[[] for k in range(len(p1.FireMap))] for h in range(p1.monteCarloHorizon)]
    for k in range(len(p1.FireMap)):
        monteCarloAverageFireMap[k] = np.loadtxt('./monteCarloAverage/monteCarloAverageFireMap'+str(k)+'.txt')
    for h in range(p1.monteCarloHorizon):
        for k in range(len(p1.FireMap)):
            monteCarloFireMap[h][k] = np.loadtxt('./monteCarlo/monteCarloFireMapTrial'+str(h)+'/monteCarloFireMapAt'+str(k)+'.txt')
    _t = time.time()
    _path, searchNodes, lastNode = MaxProbAstar(p1, monteCarloFireMap, _target=[10,18]).astar((8,0),(10,18))
    path = list(_path)
    elapsed_ = time.time() - _t
    pe=PathEvaluation(monteCarloFireMap)
    pr=pe.evaluate_path(path)
    print(path)
    print('Prob is %f, Computation took: %d seconds' %(pr, elapsed_)) 