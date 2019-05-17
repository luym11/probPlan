import math
import numpy as np
_PROB_INF = 10000
_COST_INF = 49

class PathEvaluation(object):

    def __init__(self, monteCarloFireMap):
        self.monteCarloHorizon = len(monteCarloFireMap)
        self.monteCarloFireMap = monteCarloFireMap
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
        for h in range(self.monteCarloHorizon): # every episode
            if(self.is_segment_safe_in_episode(p1, p2, t, self.monteCarloFireMap[h])):
                positiveCounter += 1
        safeProb = positiveCounter/self.monteCarloHorizon
        return safeProb

    def is_segment_safe_in_episode(self, p1, p2, t, episodeFireMap):
        if t >= _COST_INF:
            if episodeFireMap[t][p1[0]][p1[1]] > 0.9:
                return False
        elif episodeFireMap[t][p1[0]][p1[1]] > 0.9 or episodeFireMap[t+1][p2[0]][p2[1]] > 0.9:
            return False
        return True
