import math
import numpy as np
from multiprocessing import Process
import time
_PROB_INF = 10000
_COST_INF = 49

class PathEvaluation(object):

    def __init__(self, monteCarloFireMap,xT=0,yT=0):
        self.monteCarloHorizon = len(monteCarloFireMap)
        self.monteCarloFireMap = monteCarloFireMap
        self.casesThatworked = []
        self.obsvEffectRange = 2
        self.xT=xT
        self.yT=yT

    def evaluate_path(self, path):
        # note this path must be from initial point. For Monte Carlo methods, the best way is
        # to compute directly from the initial point, because if we're to compute the conditional probability, we still
        # need the probability from initial point as numerator. 
        # imagine we have a path: {Root->A->B}, we will want to compute the probability of this path being safe, we do
        # NumberOfExamplesWith{Root->A->B}Succeed/TotalNumberOfExamples
        #              ┌────┐
        #              │ B  │
        #              └────┘
        #              *
        #              *
        #              **
        #               *
        #               **
        #                **      *
        #                  *   ***
        #        **         ****
        #       **         ***┌────┐
        #      **       ****  │ A  │
        #      *     ****     └────┘
        #     ********
        #     *
        #    *
        #   **
        # ***
        # *┌────┐
        #  │Root│
        #  └────┘
        #############################
        # No matter how path is structured, any of its element should has it's 0th and 1th entry as 
        # x, y coordinates
        
        # monteCarloFireMapArray[h][t] is h episode at time=t
        positiveCounter = 0
        for h in range(self.monteCarloHorizon): # every episode
            if(self.is_path_safe_in_episode(path, self.monteCarloFireMap[h])):
                positiveCounter += 1
        if self.monteCarloHorizon == 0:
            safeProb = 0
        else:
            safeProb = positiveCounter/self.monteCarloHorizon
        return safeProb

    def is_path_safe_in_episode(self, path, episodeFireMap):
        T = len(path)
        for t in range(1,T):
            if(not self.is_path_safe_at_t(path[t], episodeFireMap[t])):
                return False
        return True

    def is_path_safe_at_t(self, tPath, tFireMap):
        if(tFireMap[tPath[0]][tPath[1]] > 0.9):
            return False
        return True

    def evaluate_segment(self, t, p1, p2):
        # evaluate a length-1 segment from p1->p2 at t
        positiveCounter = 0
        totalCounter = 0
        for h in range(self.monteCarloHorizon): # every episode
            if (self.is_path_safe_at_t(p1, self.monteCarloFireMap[h][t])):
                totalCounter += 1
                if(self.is_segment_safe_in_episode(p1, p2, t, self.monteCarloFireMap[h])):
                    positiveCounter += 1
        if totalCounter == 0:
            safeProb = 0
        else:
            safeProb = positiveCounter/totalCounter
        return safeProb


    def evaluate_segment_obsv(self, t1, p1, t2, p2, t_obsv, fireLocationArray):
        # given p1 at t1 safe, how safe is p2 at t2. All MC samples used to evaluate this safe probability has to correspond to observation at t_obsv with fireLocationArray
        self.t_obsv = t_obsv
        self.fireLocationArray = fireLocationArray 
        self.p1 = p1
        self.p2 = p2
        self.t1 = t1
        self.t2 = t2
        return self._map_mc()

    def _map_mc(self):
        jobs = []
        outlist = list()
        for _ in range(4):
            process = Process(target=self._once_mc, args=(self.monteCarloFireMap, outlist))
            jobs.append(process)
        for j in jobs:
            j.start()
        for j in jobs:
            j.join()
        # result_objects = map(self._once_mc, self.monteCarloFireMap)
        result_array = np.array(outlist)
        return sum(result_array[:,0])/sum(result_array[:,1])

    def _once_mc(self, oneFireMapEpisode, outlist):
        positiveCounter = 0
        totalCounter = 0
        if (self.CurrentMCSampleCorrespondsObsv(self.t_obsv, self.fireLocationArray, oneFireMapEpisode)): 
            if (self.is_path_safe_at_t(self.p1, oneFireMapEpisode[self.t1])):
                totalCounter = 1
                if(self.is_path_safe_at_t(self.p2, oneFireMapEpisode[self.t2])):
                    positiveCounter = 1
        outlist.append(np.array([positiveCounter, totalCounter]))
        # return np.array([positiveCounter, totalCounter])


    def CurrentMCSampleCorrespondsObsv(self, t_obsv, fireLocationArray, currentEpisode):
        # returns a boolean value indicating if firelocationArray corresponds to currentEpisode[t_obsv], meaning them having same fire status at same locations
        fireMapAtObsv = currentEpisode[t_obsv]
        fireLocationArrayFullAtCurrentEpisode = np.array([[self.xT+i,self.yT+j] for i in range(-self.obsvEffectRange,self.obsvEffectRange+1) for j in range(-self.obsvEffectRange,self.obsvEffectRange+1) if fireMapAtObsv[self.xT+i,self.yT+j] == 1])
        if (fireLocationArrayFullAtCurrentEpisode.size == 0):
            fireLocationArrayFullAtCurrentEpisode = None
            return fireLocationArray is None
        else:
            fireLocationArrayFullAtCurrentEpisodeList = fireLocationArrayFullAtCurrentEpisode.tolist()
            fireLocationArrayList = fireLocationArray.tolist()
            return all(elem in fireLocationArrayFullAtCurrentEpisodeList for elem in fireLocationArrayList) #and all(elem in fireLocationArrayList for elem in fireLocationArrayFullAtCurrentEpisodeList)

    def is_segment_safe_in_episode(self, p1, p2, t, episodeFireMap):
        if episodeFireMap[t+1][p2[0]][p2[1]] > 0.9:
            return False
        return True

    def evaluate_path_t(self, path, _T):
        # note this path is from time _T. 
        
        # monteCarloFireMapArray[h][t] is h episode at time=t
        positiveCounter = 0
        for h in range(self.monteCarloHorizon): # every episode
            if(self.is_path_safe_in_episode_t(path, self.monteCarloFireMap[h], _T)):
                positiveCounter += 1
                self.casesThatworked.append(h)
        if self.monteCarloHorizon == 0:
            safeProb = 0
        else:
            safeProb = positiveCounter/self.monteCarloHorizon
        return safeProb

    def is_path_safe_in_episode_t(self, path, episodeFireMap, _T):
        T = len(path)
        for t in range(1,T):
            if(not self.is_path_safe_at_t(path[t], episodeFireMap[_T+t])):
                return False
        return True