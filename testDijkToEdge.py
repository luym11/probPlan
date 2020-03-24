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

    def __init__(self, fireMap, averageFireMap, aDP1, STPFlag=0, obsvEffectRange=-1):
        self.fireMap = fireMap
        self.aDP1 = aDP1 
        self.STPFlag = STPFlag # if this flag is 1, meaning we are updating STP
        self.obsvEffectRange = obsvEffectRange
        self.averageFireMap = averageFireMap
        self.pe = PathEvaluation(fireMap)

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
        fireLocationArray = np.array([[xT+i,yT+j] for i in range(-self.obsvEffectRange*2,self.obsvEffectRange*2+1) for j in range(-self.obsvEffectRange*2,self.obsvEffectRange*2+1) if currentRunMap[xT+i,yT+j] == 1]) # can be empty, so need to test
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
        fireLocationArray = np.array([[xT+i,yT+j] for i in range(-self.obsvEffectRange,self.obsvEffectRange+1) for j in range(-self.obsvEffectRange,self.obsvEffectRange+1) if currentRunMap[xT+i,yT+j] == 1]) # can be empty, so need to test
        if(fireLocationArray.size == 0):
            fireLocationArray = None
        else:
            pass
        return fireLocationArray


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
        self.allEdge = np.array([[self.xT+i,self.yT+j] for i in range(-edgeSize,edgeSize+1) for j in range(-edgeSize,edgeSize+1) if ((abs(i) + abs(j) == edgeSize) and ([self.xT+i,self.yT+j] not in aDP1.Wall) and ([self.xT+i,self.yT+j] not in aDP1.Fire) and (self.xT+i < 20) and (self.yT+j < 20 ) and (self.xT+i >= 0) and (self.yT+j >= 0 ) )]) #TODO: would be great if also check non-empty here and do sth accordingly. Actually, many edge cases not checked here. But we just ignore them now. 
        # Update: now checked Fire, edge of map
        for loc2 in self.allEdge:
            print('loc2 is {0}'.format(loc2))
            # try:
            _pathSegment, searchNodes, lastNode = MaxProbAstar(aDP1=self.aDP1,startTime = 0, monteCarloFlag = 0, STPFlag = self.STPFlag, monteCarloAverageFireMap = aDP1.updatedAverageFireMap).astar(tuple([self.xT,self.yT]),tuple(loc2)) 
            print('Passed MaxProbAstar')
            pathSegment = list(_pathSegment)
            safeProbSegmentList = [1-aDP1.updatedAverageFireMap[self.T+1+ind][x][y] for ind,(x,y) in enumerate(pathSegment[1:])]
            safeProbSegment = functools.reduce(lambda a,b:a*b, safeProbSegmentList)
            # assert safeProbSegment != 0, "safeProbSegment is 0! "
            if safeProbSegment <= 0:
                print('{0} is removed'.format(loc2))
                minusLogSafeProbSegment = float('inf')
            else:
                minusLogSafeProbSegment = -math.log(safeProbSegment)
                possibleEdge.append(loc2)
                possibleLength.append(len(pathSegment))
                possibleProbs.append(minusLogSafeProbSegment)
                possiblePathsBefore.append(pathSegment)
            # minusLogSafeProbSegment = -math.log(safeProbSegment)
            # except:
            #     # if an exception raised here, that means no path is found, no MC instance is safe
            #     minusLogSafeProbSegment = float('inf')
            #     pathSegment = [[-1,-1]]  # this indicates the path is not valid
        # combine with safety from DP for afterwards, select the safest overall one
        self.possibleEdge = possibleEdge
        self.possibleLength = possibleLength
        self.possibleProbs = possibleProbs
        self.fullPossibleProbs = copy.deepcopy(possibleProbs)
        self.possiblePathsBefore = possiblePathsBefore
        for i in range(len(possibleEdge)):
            costsFromNow, routesFromNow, controlsFromNow = self.aDP1.query_optimal_solution(possibleLength[i] - 1+ self.T,[possibleEdge[i][0],possibleEdge[i][1], -1]) # also a hardcode here
            # A discount factor is introduced here
            logCostsFromNow = -math.log(costsFromNow[possibleLength[i] - 1 + self.T]/100 / (1-self.averageFireMap[possibleLength[i] - 1 + self.T][possibleEdge[i][0]][possibleEdge[i][1]]) * (1-self.aDP1.updatedAverageFireMap[possibleLength[i] - 1 + self.T][possibleEdge[i][0]][possibleEdge[i][1]]) )
            self.fullPossibleProbs[i] += logCostsFromNow
            possiblePathsAfter.append(routesFromNow)
        self.possiblePathsAfter = possiblePathsAfter
        # np.exp(np.multiply(-1,upe.fullPossibleProbs))

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

if __name__ == '__main__':
    # flags
    _STPFlag=0 # use MK prob now, as a proof of concept
    _obsvEffectRange=2

    # aDP1 = pickle.load(open('aDP1mk',"rb"))

    # loads
    _t = time.time()
    monteCarloAverageFireMap = pickle.load(open('MCAFMs1000', "rb"))
    elapsed_ = time.time() - _t
    print('Load MCFireMap took: %f seconds' %(elapsed_))

    _t = time.time()
    aDP1 = pickle.load(open('aDP1mkxx_79',"rb"))
    elapsed_ = time.time() - _t
    print('Load aDP1 took: %f seconds' %(elapsed_))

    T = 8; xT = 8; yT = 8; s = 138 # s=138 for (7,9), 7 for (7,7) # 653 for lots
    upe = UpdateEnv(aDP1.FireMap, monteCarloAverageFireMap, aDP1, STPFlag=_STPFlag, obsvEffectRange=_obsvEffectRange)

    # if _STPFlag == 0:
    #     _t = time.time()
    #     diffProbMap = np.zeros([aDP1.T-T,20,20])
    #     f = upe.observe(s,T,xT,yT)
    #     print('Observed fires')
    #     print(f) # f=np.array([[7,8]])
    #     py = upe.observe_prob_from_model(T, f)
    #     for k in range(1, aDP1.T-T): # we care from T+1
    #         for x in range(20):
    #             for y in range(20):
    #                 px = upe.safe_prob_from_model(T,k,x,y)
    #                 pyx = upe.observe_prob_given_safe_cell(T,k,f,x,y)
    #                 if py != 0:
    #                     pxNew = pyx*px/py
    #                 else:
    #                     pxNew = px
    #                     print("this can't be observed")
    #                 aDP1.updatedAverageFireMap[T+k][x][y] = 1 - pxNew
    #                 pxDiff = pxNew - px
    #                 # threshold 
    #                 if abs(pxDiff) < 0.1:
    #                     pxDiff = 0
    #                 diffProbMap[k,x,y] = pxDiff
    #     elapsed_ = time.time() - _t
    #     print('Updating MK prob took: %f seconds' %(elapsed_))            
    #     for k in range(aDP1.T-T):
    #         fig=plt.figure()
    #         # threshold
    #         ax=sns.heatmap(np.transpose(diffProbMap[k,:,:]),vmin=-1,vmax=0)
    #         plt.ylim(reversed(plt.ylim()))
    #         fig.savefig('fig11/plot_worse'+str(k)+'.png')
    #         plt.close()
    # else: # update STP
    #     pass

    # with open('aDP1mkxx','wb') as fp:
    #     pickle.dump(aDP1, fp)

    f = upe.observe(s,T,xT,yT)
    print("Observed Fires:")
    print(f)
    _t = time.time()
    upe.dijk_to_edge(5)
    elapsed_ = time.time() - _t
    print('Replanning took: %f seconds' %(elapsed_))

    # # upe.dijk_to_edge(2)
    # # np.exp(np.multiply(-1,upe.fullPossibleProbs))
    # upe.observe_and_construct(s,T,xT,yT)
    # # upe.pe.evaluate_path_t(newPath5, 0)
    # # upe.possiblePathsBefore
    # aDP2 = automaton_dp.AutomatonDP(upe.currentLocalProblem.M,upe.currentLocalProblem.N,40,upe.currentLocalProblem.Map,upe.averageFireMap, upe.currentMonteCarloFireMap, [[8,8]],[[11,19]],[[8,16]],upe.currentLocalProblem.Wall,upe.currentLocalProblem.Fire,Q=4)