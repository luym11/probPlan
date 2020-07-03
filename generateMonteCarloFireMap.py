import problem_setting
import numpy as np
import os
p1 = problem_setting.ProblemSetting(_stochastic_environment_flag=1, _setting_num=1)
monteCarloAverageFireMap, monteCarloFireMap = p1.compute_monteCarlo_2(10000)