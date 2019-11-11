import math
import matplotlib.pyplot as plt
import numpy as np
import pickle
import copy
from maxProbAstar import MaxProbAstar
from problem_setting import ProblemSetting
from path_evaluation import PathEvaluation
from utils.plot_functions import plot_path

class Agent():
    def __init__(self, robotLoc, exitLoc):
        self.p1 = ProblemSetting(target = [[18,16]],_stochastic_environment_flag=1, _setting_num=1)
        # monteCarloFireMap read in
        self.monteCarloAverageFireMap = pickle.load(open('MCAFMs3000', "rb"))
        self.monteCarloFireMap = pickle.load(open('MCFMs3000', "rb"))
        self.pe = PathEvaluation(self.monteCarloFireMap)
        self.robotLoc = robotLoc
        self.exitLoc = exitLoc
        self.path = [] # is a list of list, each element of the big list is a path segment
        self.targetList = []
        self.minusLogSafeProb = 0 # float; -log(safeProb of self.path)
        self.newPath = []
        self.newTargetList = []
        self.newMinusLogSafeProb = 0

    def compute_RPC(self, loc1, loc2, startTime):
        try:
            _pathSegment, searchNodes, lastNode = MaxProbAstar(self.p1, self.monteCarloFireMap, startTime).astar(tuple(loc1),tuple(loc2))
            pathSegment = list(_pathSegment)
            safeProbSegment = self.pe.evaluate_path_t(pathSegment, startTime)
            assert safeProbSegment != 0, "safeProbSegment is 0! "
            if safeProbSegment == 0:
                minusLogSafeProbSegment = float('inf')
            else:
                minusLogSafeProbSegment = -math.log(safeProbSegment)
            # minusLogSafeProbSegment = -math.log(safeProbSegment)
        except:
            # if an exception raised here, that means no path is found, no MC instance is safe
            minusLogSafeProbSegment = float('inf')
            pathSegment = [[-1,-1]]  # this indicates the path is not valid
        return minusLogSafeProbSegment, pathSegment

    def compute_RPC_for_targetList(self, targetList):
        # sequential as in targetList
        # self.robotLoc -> targetList -> exitLoc
        # miniMax
        locList = copy.deepcopy(targetList)
        locList.insert(0, self.robotLoc)
        locList.insert(len(locList), self.exitLoc)
        currentPathLength = 0
        path = []
        minusLogSafeProb = 0
        for i in range(len(locList)-1):
            minusLogSafeProbSegment, pathSegment = self.compute_RPC(locList[i], locList[i+1], currentPathLength)
            path.append(copy.deepcopy(pathSegment))
            minusLogSafeProb += minusLogSafeProbSegment
            currentPathLength += (len(pathSegment)-1)
        return minusLogSafeProb, path
    
    def insert_newTarget(self, newTarget):
        ###############################
        # FUNCTION: insertion heuristic
        # evaluate the optimal safe probability when inserting newTarget to self.targetList,
        # all related variables (new) will be changed 
        ###############################
        # INPUT
        # newTarget: [int, int]; the target we want to insert to self.targetList
        ###############################
        currentMinusLogSafeProbList = []
        currentPathList = []
        if len(self.targetList) == 0:
            currentTargetList = [newTarget]
            currentMinusLogSafeProb, currentPath = self.compute_RPC_for_targetList(currentTargetList)
            currentMinusLogSafeProbList.append(currentMinusLogSafeProb)
            currentPathList.append(copy.deepcopy(currentPath))
        else:
            for i in range(len(self.targetList) + 1):  
                currentTargetList = copy.deepcopy(self.targetList)
                # insert the newTarget at i
                currentTargetList.insert(i, newTarget)
                currentMinusLogSafeProb, currentPath = self.compute_RPC_for_targetList(currentTargetList)
                currentMinusLogSafeProbList.append(currentMinusLogSafeProb)
                currentPathList.append(copy.deepcopy(currentPath))
        self.newMinusLogSafeProb = min(currentMinusLogSafeProbList)
        _index = currentMinusLogSafeProbList.index(self.newMinusLogSafeProb)
        currentTargetList = copy.deepcopy(self.targetList)
        currentTargetList.insert(_index, newTarget)
        self.newTargetList = copy.deepcopy(currentTargetList)
        self.newPath = copy.deepcopy(currentPathList[_index])
            
        
    def auction_result(self, result: bool):
        ##############################
        # FUNCTION
        # receive result from the auction, if True, 
        # update the related variables
        ##############################
        if result:
            self.path = self.newPath
            self.targetList = self.newTargetList
            self.minusLogSafeProb = self.newMinusLogSafeProb
        

if __name__ == '__main__':
    a1 = Agent([8,0],[19,12])
    a2 = Agent([0,16],[19,12])

    a2.insert_newTarget([9,16])
    a2.auction_result(True)
    print(a2.path)
    print(a2.minusLogSafeProb)
    print(math.exp(-a2.minusLogSafeProb))
    print(a2.targetList)
    print()

    a2.insert_newTarget([13,16])
    a2.auction_result(True)
    print(a2.path)
    print(a2.minusLogSafeProb)
    print(math.exp(-a2.minusLogSafeProb))
    print(a2.targetList)
    print()
    

    a1.insert_newTarget([2,5])
    a1.auction_result(True)
    print(a1.path)
    print(a1.minusLogSafeProb)
    print(math.exp(-a1.minusLogSafeProb))
    print(a1.targetList)
    print()

    # path plot
    plot_path(a1.path, a2.path, np.transpose(a1.monteCarloAverageFireMap[25]))