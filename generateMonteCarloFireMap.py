import problem_setting
import numpy as np
import os
p1 = problem_setting.ProblemSetting(_stochastic_environment_flag=1, _setting_num=1)
monteCarloAverageFireMap = p1.compute_monteCarlo(3000)