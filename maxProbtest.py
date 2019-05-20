import numpy as np
import matplotlib.pyplot as plt
import time
import problem_setting
import automaton_dp
import plot3d_fire
from maxProbAstar import MaxProbAstar
from path_evaluation import PathEvaluation

for x in range(10,17):
        # problem setup
        p1 = problem_setting.ProblemSetting(target = [[x,18]],_stochastic_environment_flag=1, _setting_num=1)
        ##########################################################################
        # Original DP approach

        # monteCarloFireMap read in
        if(x==10):
                monteCarloAverageFireMap = [[] for i in range(len(p1.FireMap))]
                monteCarloFireMap = [[[] for k in range(len(p1.FireMap))] for h in range(p1.monteCarloHorizon)]
                for k in range(len(p1.FireMap)):
                        monteCarloAverageFireMap[k] = np.loadtxt('./monteCarloAverage/monteCarloAverageFireMap'+str(k)+'.txt')
                for h in range(p1.monteCarloHorizon):
                        for k in range(len(p1.FireMap)):
                                monteCarloFireMap[h][k] = np.loadtxt('./monteCarlo/monteCarloFireMapTrial'+str(h)+'/monteCarloFireMapAt'+str(k)+'.txt')

        # aDP1 = automaton_dp.AutomatonDP(p1.M,p1.N,p1.T,p1.Map,monteCarloAverageFireMap, monteCarloFireMap, p1.StartPoint,p1.EndPoint,p1.Target,p1.Wall,p1.Fire,Q=4)
        # # try:
        # aDP1.solve()
        # # except:
        # #         print("DP doesn't give a solution")

        # # plot3d_fire.plot_process(p1.FireMap, aDP1.path, p1.Map, p1.StartPoint, p1.Target, p1.Wall)

        # maxProbAstar (Dijk) approach
        _t = time.time()
        _path, searchNodes, lastNode = MaxProbAstar(p1, monteCarloFireMap, _target=[x,18]).astar((8,0),(x,18))
        path = list(_path)
        elapsed_ = time.time() - _t
        pe=PathEvaluation(monteCarloFireMap)
        pr=pe.evaluate_path(path)
        print('Prob is %f, Computation took: %d seconds' %(pr, elapsed_)) 
        print(path)
        print()
        print()