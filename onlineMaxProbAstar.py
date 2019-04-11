import numpy as np
import matplotlib.pyplot as plt
import time
import os
import math
import sys

import problem_setting
from maxProbAstar import MaxProbAstar
from path_evaluation import PathEvaluation

_INF = 1000

class OnlineMaxProbAstar():
    def __init__(self, step_size=1000, start = (8,0), target = (10,18)):
        # the frequency to do recompute, [1,_INF]
        # 1 means recompute every step, _INF means don't recompute
        self.step_size = step_size
        # initial problem
        self.p = problem_setting.ProblemSetting(_stochastic_environment_flag=1, _setting_num=1)
        self.start = start
        self.target = target

        self.probs = []
        self.paths = []

    def generateMonteCarloMap(self, currentFireMap):
        # maybe don't need to save files, just generate
        self.p.Fire = np.transpose(np.nonzero(currentFireMap)).tolist()
        return self.p.compute_monteCarlo_2(self.p.monteCarloHorizon)

    def execution(self):
        i = 0
        start = self.start
        target = self.target
        executed_path = [start]
        while start != target and i < _INF * 2: # until find the target
            # the one and only one evolving fire map
            currentFireMap = self.p.FireMap[i]
            monteCarloFireMap = self.generateMonteCarloMap(currentFireMap)
            try:
                _path, searchNodes, lastNode = MaxProbAstar(self.p, monteCarloFireMap).astar(start, target)
            except:
                print('Problem arisesssssssssssssssssssssssssssssssss')
                print(start)
                print(target)
            path = list(_path)
            pe=PathEvaluation(monteCarloFireMap)
            pr=pe.evaluate_path(path)
            # appending stuff
            self.paths.append(path)
            self.probs.append(pr)
            if len(path) > self.step_size+1 and (target not in path[1:1+self.step_size]):
                executed_path.extend(path[1:1+self.step_size]) # add these new steps to actually executed path
                print('Prob is %f from step %d' %(pr, i))
                # update
                start = path[self.step_size]
                i += self.step_size
            elif len(path) != 1:
                executed_path.extend(path[1:])
                print('Prob is %f from step %d to the end' %(pr, i))
                break
            else:
                print('Prob is %f from step %d to the end' %(pr, i))
                break
        return executed_path
    
if __name__ == '__main__':
    s = sys.argv[1]
    onlineMPA = OnlineMaxProbAstar(step_size=int(float(s)), start = (8,0), target = (10,18))
    _t = time.time()
    executed_path = onlineMPA.execution()
    elapsed_ = time.time() - _t
    print('step size')
    print(onlineMPA.step_size)
    print('time execution')
    print(elapsed_)
    print('original path')
    print(onlineMPA.paths[0])
    print('executed path')
    print(executed_path)
    print('probs')
    print(onlineMPA.probs)
    print()
