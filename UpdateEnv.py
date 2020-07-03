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
    # load stuff with pickle
    # monteCarloFireMap = pickle.load(open('MCFMs1000', "rb"))
    # monteCarloAverageFireMap = pickle.load(open('MCAFMs1000', "rb"))

    # flags
    _STPFlag=0 # use MK prob now, as a proof of concept
    _obsvEffectRange=2
    # We use Bayes rather than MC for MaxProbAStar always



    # # problem setup. Now just pickle load
    # xTar = 8
    # yTar = 16
    # xEnd = 11
    # yEnd = 19
    # p1 = ProblemSetting(target = [[xTar,yTar]], endPoint = [[xEnd,yEnd]],_stochastic_environment_flag=1, _setting_num=1)
    # aDP1 = automaton_dp.AutomatonDP(p1.M,p1.N,p1.T,p1.Map,monteCarloAverageFireMap, monteCarloFireMap, p1.StartPoint,p1.EndPoint,p1.Target,p1.Wall,p1.Fire,Q=4, STPFlag=_STPFlag)
    # aDP1.solve()
    # with open('aDP1xx','wb') as fp:
    #     pickle.dump(aDP1, fp)
    aDP1 = pickle.load(open('aDP1mk',"rb"))

    # # replan process
    # # 1. select proper s,T,xT,yT pair, do observe_and_construct()

    # Two important params

    upe = UpdateEnv(aDP1.FireMap, aDP1.averageFireMap, aDP1, STPFlag=_STPFlag, obsvEffectRange=_obsvEffectRange)
    # # upe = pickle.load(open('upe', "rb"))
    # TODO: Define xT,yT,T,s
    T = 8; xT = 8; yT = 8; s = 138

    if _STPFlag == 0:
        _t = time.time()
        diffProbMap = np.zeros([aDP1.T-T,20,20])
        f = upe.observe(s,T,xT,yT)
        print('Observed fires')
        print(f) # f=np.array([[7,8]])
        py = upe.observe_prob_from_model(T, f)
        for k in range(1, aDP1.T-T): # we care from T+1
            for x in range(20):
                for y in range(20):
                    px = upe.safe_prob_from_model(T,k,x,y)
                    pyx = upe.observe_prob_given_safe_cell(T,k,f,x,y)
                    if py != 0:
                        pxNew = pyx*px/py
                    else:
                        pxNew = px
                        print("this can't be observed")
                    aDP1.updatedAverageFireMap[T+k][x][y] = pxNew
                    pxDiff = pxNew - px
                    # threshold 
                    if abs(pxDiff) < 0.1:
                        pxDiff = 0
                    diffProbMap[k,x,y] = pxDiff
        elapsed_ = time.time() - _t
        print('Updating MK prob took: %f seconds' %(elapsed_))            
        for k in range(aDP1.T-T):
            fig=plt.figure()
            # threshold
            ax=sns.heatmap(np.transpose(diffProbMap[k,:,:]),vmin=-1,vmax=0)
            plt.ylim(reversed(plt.ylim()))
            fig.savefig('fig11/plot_worse'+str(k)+'.png')
            plt.close()
        # # Worse plots
        #     fig=plt.figure()
        #     # threshold
        #     ax=sns.heatmap(np.transpose(diffProbMap[k,:,:]),vmin=0,vmax=1)
        #     plt.ylim(reversed(plt.ylim()))
        #     fig.savefig('fig_only78_s20_long/plot_better'+str(k)+'.png')
        #     plt.close()
    else: # update STP
        pass
        # f = upe.observe(s,T,xT,yT)
        # print('Observed fires')
        # print(f)
        # py = upe.observe_prob_given_safe_cell()

    with open('aDP1mkxx_79','wb') as fp:
        pickle.dump(aDP1, fp)

    # ONLY THIS IS NEEDED NOW
    # aDP1 = pickle.load(open('aDP1mkxx',"rb"))