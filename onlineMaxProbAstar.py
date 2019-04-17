import numpy as np
import matplotlib.pyplot as plt
import time
import os
import math
import sys
import multiprocessing as mp

import problem_setting
from maxProbAstar import MaxProbAstar
from path_evaluation import PathEvaluation

_INF = 1000

class OnlineMaxProbAstar():
    def __init__(self, step_size=1000, _start = (8,0), _target = (10,18)):
        # the frequency to do recompute, [1,_INF]
        # 1 means recompute every step, _INF means don't recompute
        self.step_size = step_size
        # initial problem
        self.p = problem_setting.ProblemSetting(target=[list(_target)], _stochastic_environment_flag=1, _setting_num=1)
        self.start = _start
        self.target = _target

        self.paths = []

        self.badFlag = False

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
                # print(start)
                # print(self.p.FireMap[i][start[0]][start[1]])
                # print(np.array(self.p.FireMap[i]))
                print('No feasible path found!')
                self.badFlag = True
                break
            path = list(_path)
            # appending stuff
            self.paths.append(path)
            assert (len(path) > 1),path
            if len(path) > self.step_size+1:
                executed_path.extend(path[1:1+self.step_size]) # add these new steps to actually executed path
                # update
                start = path[self.step_size]
                i += self.step_size
            else:
                executed_path.extend(path[1:])
                break
        return executed_path

def multi_process_task(s):
    onlineMPA = OnlineMaxProbAstar(step_size=s, _start = (8,0), _target = (10,18))
    executed_path = onlineMPA.execution()
    flag = onlineMPA.badFlag
    pe=PathEvaluation([onlineMPA.p.FireMap])
    pr1=pe.evaluate_path(onlineMPA.paths[0])
    pr2=pe.evaluate_path(executed_path)
    print()
    print('original path')
    print(onlineMPA.paths[0])
    print('executed path')
    print(executed_path)
    print(flag,pr1,pr2)
    print()
    return flag,pr1,pr2
if __name__ == '__main__':
    # s = sys.argv[1]
    # onlineMPA = OnlineMaxProbAstar(step_size=int(float(s)), start = (8,0), target = (10,16))
    # _t = time.time()
    # executed_path = onlineMPA.execution()
    # elapsed_ = time.time() - _t
    # # if(not onlineMPA.badFlag):
    # print('step size')
    # print(onlineMPA.step_size)
    # print('time execution')
    # print(elapsed_)
    # print('original path')
    # print(onlineMPA.paths[0])
    # print('executed path')
    # print(executed_path)
    # pe=PathEvaluation([onlineMPA.p.FireMap])
    # pr1=pe.evaluate_path(onlineMPA.paths[0])
    # pr2=pe.evaluate_path(executed_path)
    # print(pr1)
    # print(pr2)
    # print()
    step_size = 10
    
    _t = time.time()
    pool = mp.Pool(processes=10)
    results = [pool.apply(multi_process_task,args=(step_size,)) for x in range(100)]
    elapsed_ = time.time() - _t
    print('step size')
    print(step_size)
    print('time execution')
    print(elapsed_)
    print('Print combined results for successing')
    print(results)
