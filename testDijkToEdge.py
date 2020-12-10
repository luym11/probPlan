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
import functools
# from newProblemSetting import NewProblemSetting

_SIZE = 20

class UpdateEnv():

    def __init__(self, M, N, Fire, Wall, fireMap, averageFireMap, obsvEffectRange=-1, replanRange=-1,tT = 8, aDP1=None):
        self.fireMap = fireMap
        self.obsvEffectRange = obsvEffectRange
        self.averageFireMap = averageFireMap
        self.replanRange = replanRange
        self.tT = tT
        self.observedFireLocationArray = None
        self.Fire = Fire
        self.Wall = Wall
        self.M = M
        self.N = N
        self.aDP1 = aDP1
    # def observe_and_construct(self, s, T, xT, yT):
    #     # s: observe the s-th episode
    #     # observe at T in (xT,yT), and construct a new problem based on updated belief of fire
    #     # the problem should have only the fire locations updated from observation, and the rest of the 
    #     # elements remain the same
    #     # Use the observed current run map, redo the MC generating for this new problem (actually the new problem is solely for MC process)
    #     # assume observe size 2
    #     # NOT FULLY READY. STH HAS TO BE DONE FOR RECONSTRUCTING THE PROBLEM
    #     self.T = T
    #     self.xT=xT
    #     self.yT=yT
    #     currentRunMap = self.fireMap[s][T]
    #     fireLocationArray = np.array([[xT+i,yT+j] for i in range(-self.obsvEffectRange,self.obsvEffectRange+1) for j in range(-self.obsvEffectRange,self.obsvEffectRange+1) if (self.xT+i < 20) and (self.yT+j < 20 ) and (self.xT+i >= 0) and (self.yT+j >= 0 ) and currentRunMap[xT+i,yT+j] == 1]) # can be empty, so need to test
    #     if(fireLocationArray.size == 0):
    #         fireLocationArray = None
    #     else:
    #         pass
        
    #     # current local problem needs more fire: should run a full MC Bayes update to conditional safe probs, and then run DP again
    #     self.currentLocalProblem = ProblemSetting(fire=list(fireLocationArray))
    #     currentMonteCarloFireMap = self.currentLocalProblem.compute_monteCarlo_3(_GenerateHorizon = 500, startTime = T)
    #     # results saving
    #     self.currentMonteCarloFireMap = currentMonteCarloFireMap
    #     self.pe = PathEvaluation(currentMonteCarloFireMap)

    def observe(self, s, T, xT, yT):
        # s: observe the s-th episode
        # observe at T in (xT,yT), and return the observed fire
        self.T = T
        self.xT=xT
        self.yT=yT
        currentRunMap = self.fireMap[s][T]
        fireLocationArray = np.array([[xT+i,yT+j] for i in range(-self.obsvEffectRange,self.obsvEffectRange+1) for j in range(-self.obsvEffectRange,self.obsvEffectRange+1) if (self.xT+i < 20) and (self.yT+j < 20 ) and (self.xT+i >= 0) and (self.yT+j >= 0 ) and currentRunMap[xT+i,yT+j] == 1]) # can be empty, so need to test
        if(fireLocationArray.size == 0):
            fireLocationArray = None
        else:
            pass
        self.observedFireLocationArray = fireLocationArray
        self.pe = PathEvaluation(self.fireMap,xT=xT,yT=yT)
        return fireLocationArray

    def observe_from_fire(self, f, T, xT, yT):
        # observed f, at xT,yT at T
        self.T = T
        self.xT=xT
        self.yT=yT
        fireLocationArray = np.array(f) 
        self.observedFireLocationArray = fireLocationArray
        self.pe = PathEvaluation(self.fireMap,xT=xT,yT=yT)


    def dijk_to_edge(self, edgeSize):
        # Main idea: combine the safe Prob of forward Dijk with backward DP to select the best
        # safety till this current location is not required for the path replanning. So we don't compute it now

        # Dijkstra to every edge, note down safety, also note down time (length)
        possibleEdge = [] # the edge cells that can be reached
        possibleLength = [] # corresponding path lengths to reach these possible cells
        possibleProbs = [] # corresponding safe probabilities from current cell to edge cells
        possiblePathsBefore = [] # corresponding safe paths from current cell to edge cells
        possiblePathsAfter = [] # corresponding safe paths from edge cells to the rest of the mission
        self.allEdge = np.array([[self.xT+i,self.yT+j] for i in range(-edgeSize,edgeSize+1) for j in range(-edgeSize,edgeSize+1) if ((abs(i) + abs(j) == edgeSize) and ([self.xT+i,self.yT+j] not in self.Wall) and ([self.xT+i,self.yT+j] not in self.Fire) and (self.xT+i < 20) and (self.yT+j < 20 ) and (self.xT+i >= 0) and (self.yT+j >= 0 ) )]) 
        assert len(self.allEdge) != 0, "All grids at Edge are non-reachable!"
        # find part 1 of the replanned path & evaluate it
        self.dijkPathFinder = MaxProbAstar(self.M,self.N,self.Wall,self.fireMap,startTime = self.T,xT=self.xT,yT=self.yT, observedFireLocationArray = self.observedFireLocationArray, searchSize = edgeSize)
        for loc2 in self.allEdge:
            try:
                _t = time.time()
                _pathSegment, searchNodes, lastNode = self.dijkPathFinder.astar(tuple([self.xT,self.yT]),tuple(loc2))
                elapsed_ = time.time() - _t
                print('Dijk took: %f seconds' %(elapsed_))
                pathSegment = list(_pathSegment)
                # safeProbSegmentList = [1-aDP1.updatedAverageFireMap[self.T+1+ind][x][y] for ind,(x,y) in enumerate(pathSegment[1:])]
                safeProbSegmentList = [self.pe.evaluate_segment_obsv(self.T+ind,[pathSegment[ind][0],pathSegment[ind][1]],self.T+ind+1,[pathSegment[ind+1][0],pathSegment[ind+1][1]],self.T,self.observedFireLocationArray) for ind,(x,y) in enumerate(pathSegment[:-1])]
                safeProbSegment = functools.reduce(lambda a,b:a*b, safeProbSegmentList)
                elapsed_2 = time.time() - _t
                print('Eval took: %f seconds' %(elapsed_2))
                assert safeProbSegment > 0, "safeProbSegment is LE to 0! "
                if safeProbSegment <= 0:
                    minusLogSafeProbSegment = float('inf')
                else:
                    minusLogSafeProbSegment = -math.log(safeProbSegment)
                    possibleEdge.append(loc2)
                    possibleLength.append(len(pathSegment))
                    possibleProbs.append(minusLogSafeProbSegment)
                    possiblePathsBefore.append(pathSegment)
            except:
                # if an exception raised here, that means no path is found, no MC instance is safe
                minusLogSafeProbSegment = float('inf')
                pathSegment = [[-1,-1]]  # this indicates the path is not valid
        self.possibleEdge = possibleEdge
        self.possibleLength = possibleLength
        self.possibleProbs = possibleProbs
        self.fullPossibleProbs = copy.deepcopy(possibleProbs)
        self.possiblePathsBefore = possiblePathsBefore

        # evaluate old offline path (part 2)
        for i in range(len(possibleEdge)):
            costsFromNow, routesFromNow, controlsFromNow = self.aDP1.query_optimal_solution(possibleLength[i] - 1+ self.T, [possibleEdge[i][0],possibleEdge[i][1], -1]) # also a hardcode here
            routesFromNowCopied = copy.deepcopy(routesFromNow)
            self.remove_duplicated_path_segs(routesFromNowCopied)
            try:
                _t = time.time()
                # updated safe probability for part 2
                safeProbSegmentList2 = [self.pe.evaluate_segment_obsv(self.T+possibleLength[i]-1+ind,[routesFromNow[ind][0],routesFromNow[ind][1]],self.T+possibleLength[i]-1+ind+1,[routesFromNow[ind+1][0],routesFromNow[ind+1][1]],self.T,self.observedFireLocationArray) for ind,(x,y,q) in enumerate(routesFromNowCopied[:-1])]
                safeProbSegment2 = functools.reduce(lambda a,b:a*b, safeProbSegmentList2)
                elapsed_3 = time.time() - _t
                print('EvalOld took: %f seconds' %(elapsed_3))
                assert safeProbSegment2 > 0, "safeProbSegment is LE to 0! "
                if safeProbSegment2 <= 0:
                    minusLogSafeProbSegment2 = float('inf')
                logCostsFromNow = -math.log(safeProbSegment2)
            except:
                print('math domain error')
                logCostsFromNow = float('Inf')
            self.fullPossibleProbs[i] += logCostsFromNow
            possiblePathsAfter.append(routesFromNowCopied)
        self.possiblePathsAfter = possiblePathsAfter

    def remove_duplicated_path_segs(self, path):
        endPath = tuple([self.aDP1.EndPoint[0][0],self.aDP1.EndPoint[0][1],-3])
        while(path[-2] == endPath):
            del path[-1]
        return
    def safe_prob_from_model(self, T, k, x, y):
        # returns safe probability of grid cell (x,y) at time T+k
        return 1-self.averageFireMap[T+k][x][y]

    def observe_prob_from_model(self, T, fireLocationArray):
        # returns the probability of observing fireLocationArray at T, based on the model (MC simulations)
        counter = 0
        totalEpisodeNum = len(self.fireMap)
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
        fireLocationArrayFullsT_ = np.array([[self.xT+i,self.yT+j] for i in range(-self.obsvEffectRange,self.obsvEffectRange+1) for j in range(-self.obsvEffectRange,self.obsvEffectRange+1) if currentRunMap[self.xT+i,self.yT+j] == 1])
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
        totalEpisodeNum = len(self.fireMap)
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

if __name__ == '__main__':

    for T in range(tT,aDP1.T):
        _t = time.time()
        # diffProbMap = np.zeros([aDP1.T-T,20,20])
        if T == tT:
            xT = 8; yT = 8
        else:
            xT = newPath[0][0];yT = newPath[0][1]
        if upe.fireMap[s][T][xT][yT] > 0.99:
            print('In episode {0} at time {1}, ({2},{3}) is burnt'.format(s,T,xT,yT))
            break
        if [[xT,yT]] == upe.aDP1.Target:
            print('Succeeded')
            break
        # observed fire: f
        f = upe.observe(s,T,xT,yT)
        upe.observedFireLocationArray = copy.deepcopy(f)
        print('Observed fires at time {0}, ({1},{2}):'.format(T,xT,yT))
        print(f) # f=np.array([[7,8]])
        py = upe.observe_prob_from_model(T, f)
        for k in range(1, min(aDP1.T-T, upe.replanRange + 1)): # we care from T+1
            for x in range( max(0,xT - upe.replanRange), min(19,xT + upe.replanRange + 1) ):
                for y in range( max(0,xT - upe.replanRange), min(19,xT + upe.replanRange + 1) ):
                    px = upe.safe_prob_from_model(T,k,x,y)
                    pyx = upe.observe_prob_given_safe_cell(T,k,f,x,y)
                    if py != 0:
                        pxNew = pyx*px/py
                    else:
                        pxNew = px
                        print("this can't be observed")
                    aDP1.updatedAverageFireMap[T+k][x][y] = 1 - pxNew
                    # pxDiff = pxNew - px
                    # # threshold 
                    # if abs(pxDiff) < 0.1:
                    #     pxDiff = 0
                    # diffProbMap[k,x,y] = pxDiff
        elapsed_ = time.time() - _t
        print('Partially updating MK prob took: %f seconds' %(elapsed_))            
      
        _t = time.time()
        upe.dijk_to_edge(upe.replanRange)
        elapsed_ = time.time() - _t
        print('Replanning took: %f seconds' %(elapsed_))

        maxNeighborIndex = np.argmax(np.exp(np.multiply(-1,upe.fullPossibleProbs)))
        newPath = copy.deepcopy(upe.possiblePathsAfter[maxNeighborIndex])
        for p in range(len(upe.possiblePathsBefore[maxNeighborIndex])-2,0,-1):
            newPath.insert(0,(upe.possiblePathsBefore[maxNeighborIndex][p][0],upe.possiblePathsBefore[maxNeighborIndex][p][1],-1)) # NOTICE: only -1 for now
