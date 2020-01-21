import numpy as np 
from maxProbAstar import MaxProbAstar
from problem_setting import ProblemSetting
from path_evaluation import PathEvaluation
import copy
import math
import matplotlib.pyplot as plt
import time
import automaton_dp
import plot3d_fire
import pickle
import seaborn as sns
import imageio
import os

_SIZE = 20

class UpdateEnv():

    def __init__(self, fireMap, aDP1):
        self.fireMap = fireMap
        self.aDP1 = aDP1 

    def observe(self, s, T, xT, yT):
        # s: observe the s-th episode
        # should return a list of elements that are enough to construct a new problem
        # the problem should have only the fire locations that we observed, and the rest of the 
        # elements remain the same
        # Use the observed current run map, redo the MC generating for this new problem (actually the new problem is solely for MC process)
        # assume observe size 2
        self.T = T
        self.xT=xT
        self.yT=yT
        currentRunMap = self.fireMap[s][T]
        fireLocationArray = np.array([[xT+i,yT+j] for i in range(-2,3) for j in range(-2,3) if currentRunMap[xT+i,yT+j] == 1]) # can be empty, so need to test
        if(fireLocationArray.size == 0):
            fireLocationArray = None
        else:
            pass
            # currentRunMapArray = np.array(currentRunMap)
            # currentFireState = currentRunMapArray[rows,cols]
        self.currentLocalProblem = ProblemSetting(fire=list(fireLocationArray))
        currentMonteCarloFireMap = self.currentLocalProblem.compute_monteCarlo_3(_GenerateHorizon = 500, startTime = T)
        # results saving
        self.currentMonteCarloFireMap = currentMonteCarloFireMap
        self.pe = PathEvaluation(currentMonteCarloFireMap)

    # def observe_full(self, s, T):
    #     # observe everything from s episode at T
    #     currentRunMap = self.fireMap[s][T]
    #     fireLocationArrayFull_ = np.array(np.nonzero(currentRunMap))
    #     fireLocationArrayFull = fireLocationArrayFull_.reshape(fireLocationArrayFull_.shape[1],fireLocationArrayFull_.shape[0])
    #     self.currentLocalProblemFull = ProblemSetting(fire=list(fireLocationArrayFull))
    #     currentMonteCarloFireMapFull = self.currentLocalProblemFull.compute_monteCarlo_3(_GenerateHorizon = 500, startTime = T)
    #     # results saving
    #     self.currentMonteCarloFireMapFull = currentMonteCarloFireMapFull
    #     self.peFull = PathEvaluation(currentMonteCarloFireMapFull)


    def dijk_to_edge(self, edgeSize):
        # Use MC generated fireMap to do Dijk to every edge cell
        # combine the safe Prob of forward Dijk with backward DP to select the best

        # safety till this current location is not required for the path replanning. So we don't compute it now

        # Dijkstra to every edge, note down safety, also note down time (length)
        possibleEdge = [] # the edge cells that can be reached
        possibleLength = [] # corresponding path lengths to reach these possible cells
        possibleProbs = [] # corresponding safe probabilities from current cell to edge cells
        possiblePathsBefore = [] # corresponding safe paths from current cell to edge cells
        possiblePathsAfter = [] # corresponding safe paths from edge cells to the rest of the mission
        self.allEdge = np.array([[self.xT+i,self.yT+j] for i in range(-edgeSize,edgeSize+1) for j in range(-edgeSize,edgeSize+1) if (i==-2 or j==-2 or i==2 or j==2)]) #TODO: would be great if also check non-empty here and do sth accordingly. Actually, many edge cases not checked here. But we just ignore them now. 
        for loc2 in self.allEdge:
            try:
                _pathSegment, searchNodes, lastNode = MaxProbAstar(self.currentLocalProblem, self.currentMonteCarloFireMap, 0).astar(tuple([self.xT,self.yT]),tuple(loc2))
                pathSegment = list(_pathSegment)
                safeProbSegment = self.pe.evaluate_path_t(pathSegment, 0)
                # assert safeProbSegment != 0, "safeProbSegment is 0! "
                if safeProbSegment == 0:
                    minusLogSafeProbSegment = float('inf')
                else:
                    minusLogSafeProbSegment = -math.log(safeProbSegment)
                    possibleEdge.append(loc2)
                    possibleLength.append(len(pathSegment))
                    possibleProbs.append(minusLogSafeProbSegment)
                    possiblePathsBefore.append(pathSegment)
                # minusLogSafeProbSegment = -math.log(safeProbSegment)
            except:
                # if an exception raised here, that means no path is found, no MC instance is safe
                minusLogSafeProbSegment = float('inf')
                pathSegment = [[-1,-1]]  # this indicates the path is not valid
        # combine with safety from DP for afterwards, select the safest overall one
        self.possibleEdge = possibleEdge
        self.possibleLength = possibleLength
        self.possibleProbs = possibleProbs
        self.fullPossibleProbs = copy.deepcopy(possibleProbs)
        self.possiblePathsBefore = possiblePathsBefore
        for i in range(len(possibleEdge)):
            costsFromNow, routesFromNow, controlsFromNow = self.aDP1.query_optimal_solution(possibleLength[i] - 1+ self.T,[possibleEdge[i][0],possibleEdge[i][1], -1]) # also a hardcode here
            logCostsFromNow = -math.log(costsFromNow[possibleLength[i] - 1 + self.T]/100)
            self.fullPossibleProbs[i] += logCostsFromNow
            possiblePathsAfter.append(routesFromNow)
        self.possiblePathsAfter = possiblePathsAfter

if __name__ == '__main__':
    # load stuff with pickle
    monteCarloFireMap = pickle.load(open('MCFMs1000', "rb"))
    monteCarloAverageFireMap = pickle.load(open('MCAFMs1000', "rb"))

    # # problem setup. Now just pickle load
    # xTar = 8
    # yTar = 16
    # xEnd = 11
    # yEnd = 19
    # p1 = ProblemSetting(target = [[xTar,yTar]], endPoint = [[xEnd,yEnd]],_stochastic_environment_flag=1, _setting_num=1)
    # aDP1 = automaton_dp.AutomatonDP(p1.M,p1.N,p1.T,p1.Map,monteCarloAverageFireMap, monteCarloFireMap, p1.StartPoint,p1.EndPoint,p1.Target,p1.Wall,p1.Fire,Q=4)
    # aDP1.solve()
    aDP1 = pickle.load(open('aDP1',"rb"))

    # # replan process
    # upe = UpdateEnv(monteCarloFireMap, aDP1)
    # # 1. select proper s,T,xT,yT pair, do observe()
    # # s = 20, T=7, xT=8, yT=7
    upe = pickle.load(open('upe', "rb"))
