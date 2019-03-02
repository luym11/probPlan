import problem_setting
import numpy as np
import math

class AnalyticFire(problem_setting.ProblemSetting):
    def setFireMap(self, t, prev_firemap = None):
        m = len(self.Map[0])
        n = len(self.Map)
        firemap_ = [[0 for col in range(m)] for row in range(n)]
        if t == 0:
            for i in range(len(self.Fire)):
                firemap_[self.Fire[i][0]][self.Fire[i][1]] = 1 # probability is 1
        else:
            for x in range(m):
                for y in range(n):
                    neighbors = self.generate_neighbors(x, y, prev_firemap)
                    firemap_[x][y] = self.compute_fire_prob(neighbors, x, y, prev_firemap[x][y])
        return firemap_
    def generate_neighbors(self, x, y, prev_firemap):
        neighbors = []
        for i in range(-1,2):
            for j in range(-1,2):
                if self.isInsideMap(self.Map, [x+i, y+j]):
                    v = prev_firemap[x+i][y+j] # firemap value
                    if v != 0: 
                        if not(i == 0 and j == 0):
                            if i == 0 or j == 0:
                                neighbors.append([x+i, y+j, v, 1])
                            else:
                                neighbors.append([x+i, y+j, v, math.sqrt(2)])
        return neighbors

    def compute_fire_prob(self, neighbors, x, y, prev_value):
        pxt = 1-prev_value
        for n in neighbors:
            if self.Map[x][y] == 1:
                pf = self.pf1
            else:
                pf = self.pf0
            pxt *= ( (1-n[2]) + n[2] * (1-pf/n[3]) )
        return 1-pxt


    def analytic_compute_fire(self):
        firemaps = [[] for row in range(self.T)]
        for t in range(self.T):
            if t == 0:
                firemaps[t] = self.setFireMap(0)
            else:
                firemaps[t] = self.setFireMap(t, firemaps[t-1])
        return firemaps

if __name__ == '__main__':
    af = AnalyticFire(_stochastic_environment_flag=1, _setting_num=1)
    # get the empirical probability from MC simulations' average
    monteCarloAverageFireMap = [[] for i in range(len(af.FireMap))]
    for k in range(len(af.FireMap)):
        monteCarloAverageFireMap[k] = np.loadtxt('./monteCarloAverage/monteCarloAverageFireMap'+str(k)+'.txt')
    # analytical one
    analyticAverageFireMap = af.analytic_compute_fire()