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
import os
# from newProblemSetting import NewProblemSetting

_SIZE = 20

class UpdateEnv():

    def __init__(self, fireMap, averageFireMap, aDP1, STPFlag=0, obsvEffectRange=-1):
        self.fireMap = fireMap
        self.aDP1 = aDP1 
        self.STPFlag = STPFlag # if this flag is 1, meaning we are updating STP
        self.obsvEffectRange = obsvEffectRange
        self.averageFireMap = averageFireMap

    def observe_and_construct(self, s, T, xT, yT):
        # s: observe the s-th episode
        # observe at T in (xT,yT), and construct a new problem based solely on the observed fire
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

    def observe(self, s, T, xT, yT):
        # s: observe the s-th episode
        # observe at T in (xT,yT), and return the observed fire
        # assume observe size 1
        self.T = T
        self.xT=xT
        self.yT=yT
        currentRunMap = self.fireMap[s][T]
        fireLocationArray = np.array([[xT+i,yT+j] for i in range(-1,2) for j in range(-1,2) if currentRunMap[xT+i,yT+j] == 1]) # can be empty, so need to test
        if(fireLocationArray.size == 0):
            fireLocationArray = None
        else:
            pass
        return fireLocationArray

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
        # Dijk to every edge cell: 2 flags involved, self.STPFlag and monteCarloFlag; we currently want to use Bayes rather than MC, and MK rather than STP (for simplicity and proof of concept)
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
                # _pathSegment, searchNodes, lastNode = MaxProbAstar(self.currentLocalProblem, self.currentMonteCarloFireMap, startTime = 0, monteCarloFlag = 0, STPFlag = self.STPFlag, monteCarloAverageFireMap = aDP1.updatedAverageFireMap).astar(tuple([self.xT,self.yT]),tuple(loc2))
                _pathSegment, searchNodes, lastNode = MaxProbAstar(None, None, aDP1=aDP1,startTime = 0, monteCarloFlag = 0, STPFlag = self.STPFlag, monteCarloAverageFireMap = aDP1.updatedAverageFireMap).astar(tuple([self.xT,self.yT]),tuple(loc2))
                # _pathSegment, searchNodes, lastNode = MaxProbAstar(None, None, aDP1=aDP1,startTime = 0, monteCarloFlag = 0, STPFlag = upe.STPFlag, monteCarloAverageFireMap = aDP1.updatedAverageFireMap).astar(tuple([upe.xT,upe.yT]),tuple(loc2))
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

    def safe_prob_from_model(self, T, k, x, y):
        # returns safe probability of grid cell (x,y) at time T+k
        return 1-self.averageFireMap[T+k][x][y]

    def observe_prob_from_model(self, T, fireLocationArray):
        # returns the probability of observing fireLocationArray at T, based on the model (MC simulations)
        counter = 0
        totalEpisodeNum = len(self.aDP1.FireMap)
        for s in range(totalEpisodeNum):
            if self.is_obsv_in_episode_s_T(fireLocationArray, s, T):
                counter += 1
        # print('Observed fire appears in {0} episodes'.format(counter))
        return counter/totalEpisodeNum

    def is_obsv_in_episode_s_T(self, fireLocationArray, s, T):
        # returns if fireLocationArray is a subset of the fire in episode s at time T
        # also, should satisfy that within range of view, two fires are exactly the same
        currentRunMap = self.fireMap[s][T]
        # fireLocationArrayFullsT_ = np.array(np.nonzero(currentRunMap))
        fireLocationArrayFullsT_ = np.array([[self.xT+i,self.yT+j] for i in range(-1,2) for j in range(-1,2) if currentRunMap[self.xT+i,self.yT+j] == 1])
        if (fireLocationArrayFullsT_.size == 0):
            fireLocationArrayFullsT_ = None
            return fireLocationArray is None
        else:
            # fireLocationArrayFullsT = fireLocationArrayFullsT_.reshape(fireLocationArrayFullsT_.shape[1],fireLocationArrayFullsT_.shape[0])
            fireLocationArrayFullsTList = fireLocationArrayFullsT_.tolist()
            fireLocationArrayList = fireLocationArray.tolist()
            # only or r
            return all(elem in fireLocationArrayFullsTList for elem in fireLocationArrayList) #and all(elem in fireLocationArrayList for elem in fireLocationArrayFullsTList)

    def observe_prob_given_safe_cell(self, T, k, fireLocationArray, x, y):
        # returns the probability of observing fireLocationArray given (x,y) is safe at T+k
        observedEpisodeNum = 0
        safeEpisodeNum = 0
        totalEpisodeNum = len(self.aDP1.FireMap)
        for s in range(totalEpisodeNum):
            if self.fireMap[s][T+k][x][y] == 0:
                safeEpisodeNum += 1
                if self.is_obsv_in_episode_s_T(fireLocationArray, s, T):
                    observedEpisodeNum += 1
        # print('safeEpisodeNum is {0}, observedEpisodeNum is {1}'.format(safeEpisodeNum, observedEpisodeNum))
        if safeEpisodeNum == 0:
            return 0
        else:
            return observedEpisodeNum/safeEpisodeNum

aDP1 = pickle.load(open('aDP1mkxx',"rb"))