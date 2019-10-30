import math
import numpy as np
_PROB_INF = 10000
_COST_INF = 49

class PathEvaluation(object):

    def __init__(self, monteCarloFireMap, target=[[-1, -1]], observeTime=0, observeFireState=None,x=0,y=0):
        self.monteCarloHorizon = len(monteCarloFireMap)
        self.monteCarloFireMap = monteCarloFireMap
        self.target = target[0]

        # observation will disable many samples
        if x==-1 and (not observeFireState is None): # the case where we observe all the map
            deleteList = []
            # remove some monteCarloFireMaps
            for i in range(self.monteCarloHorizon):
                currentEpisodeMapArray = np.array(self.monteCarloFireMap[i][observeTime])
                currentEpisodeFireState = currentEpisodeMapArray
                if not np.array_equal(observeFireState, currentEpisodeFireState):
                    deleteList.append(i)
            for index in sorted(deleteList, reverse=True):
                del self.monteCarloFireMap[index]
            self.monteCarloHorizon = len(self.monteCarloFireMap)
            print('new monteCarloHorizon is {}'.format(self.monteCarloHorizon))
        elif not observeFireState is None: 
            locationArray = np.array([[x+i,y+j] for i in range(-1,2) for j in range(-1,2)])
            rows=locationArray[:,0]
            cols=locationArray[:,1]
            deleteList = []
            # remove some monteCarloFireMaps
            for i in range(self.monteCarloHorizon):
                currentEpisodeMapArray = np.array(self.monteCarloFireMap[i][observeTime])
                currentEpisodeFireState = currentEpisodeMapArray[rows,cols]
                if not np.array_equal(observeFireState, currentEpisodeFireState):
                    deleteList.append(i)
            for index in sorted(deleteList, reverse=True):
                del self.monteCarloFireMap[index]
            self.monteCarloHorizon = len(self.monteCarloFireMap)
            print('new monteCarloHorizon is {}'.format(self.monteCarloHorizon))
        #self.monteCarloFireMapArray = np.array(monteCarloFireMap)

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