import math
import numpy as np
import csv
import time
import matplotlib.pyplot as plt
import os

class ProblemSetting():

    def setMap(self, m,n,s,e,t,w):
        ################
        # construct a map using given configuration specifications
        # map values are used for different map contents
        # called inside mapCreator()
        # NOTICE
        # fire will NOT be presented here, it's in the FireMap
        ################
        # Args:
        # M: length, to be called second (map[n][m])
        # N: width, to be called first
        # s: start point, with map value -1
        # e: end point, -3
        # t: target/rescueee point, -2
        # w: walls, 1
        # others: free spaces, 0
        ################
        # Returns: 
        # map_: static map for everything else except the fire
        ################
        map_ = [[0 for col in range(m)] for row in range(n)]
        for i in range(len(s)):
            map_[s[i][0]][s[i][1]] = -1
        for i in range(len(e)):
            map_[e[i][0]][e[i][1]] = -3
        for i in range(len(t)):
            map_[t[i][0]][t[i][1]] = -2
        for i in range(len(w)):
            map_[w[i][0]][w[i][1]] = 1
        return map_

    def setFireMap(self, t, prev_firemap = None):
        ################
        # called in mapGenerator()
        # compute predictions for distribution of the fire at time t
        # will use global variables Fire
        ################
        # t: time step we want to compute for the prediction of the fire
        # prev_firemap: fire map at t-1. If t!=0, this value should be provided
        # firemap_: return a computed map at time t
        ################
        m = len(self.Map[0])
        n = len(self.Map)
        firemap_ = [[0 for col in range(m)] for row in range(n)]
        if t == 0:
            for i in range(len(self.Fire)):
                firemap_[self.Fire[i][0]][self.Fire[i][1]] = 1 # probability is 1
        else:
            for x in range(m):
                for y in range(n):
                    Df, Nf = self.generate_neighbors_fire(x, y, prev_firemap)
                    firemap_[x][y] = self.compute_fire_prob(x, y, Df, Nf, prev_firemap)
        return firemap_

    def generate_neighbors_fire(self, x, y, prev_firemap):
        # count direct and diagonal neighbors of a point (x,y) on prev_firemap, 
        # to compute probability of (x,y) getting comtaminated in the next time step
        Df = 0
        Nf = 0
        for i in range(-1,2):
            for j in range(-1,2):
                if self.isInsideMap(self.Map, [x+i, y+j]):
                    v = prev_firemap[x+i][y+j] # firemap value
                    # by now, the value of v will only be 0 or 1
                    if v != 0: 
                        if not(i == 0 and j == 0):
                            if i == 0 or j == 0:
                                Nf = Nf + 1
                            else:
                                Df = Df + 1
        return Df, Nf

    def compute_fire_prob(self, x, y, Df, Nf, prev_firemap):
        # Once contaminated, remains contaminated
        if prev_firemap[x][y] == 1:
            return 1
        else:
            # check if it's a wall
            if self.Map[x][y] == 1:
                pf = self.pf1
            else:
                pf = self.pf0
            pr = 1 - ((1-pf) ** Nf) * ((1-pf/math.sqrt(2)) ** Df)
            # a simulation process: return either 0 or 1
            if np.random.random_sample() <= pr:
                return 1
            else:
                return 0

    def mapCreator(self):
        # map: static map for everything else except the fire
        map = self.setMap(self.M,self.N,self.StartPoint,self.EndPoint,\
            self.Target,self.Wall) # This is a static map, presenting everything but the fire
        return map

    def mapGenerator(self, T):
        ################
        # use the cover function to compute a realization of the map from 0~(T-1) for FireMap
        # generated fire maps will only have entries 0 or 1
        ################
        # T: time horizon
        # firemaps: firemaps[i] is the map at time i, firemaps is a list of 2d matrices with 0/1 as entries
        ################
        firemaps = [[] for row in range(T)]
        for t in range(T):
            if self.stochastic_environment_flag != 1 or t == 0:
                firemaps[t] = self.setFireMap(0) # keep the original distribution of fire, static
            else:
                firemaps[t] = self.setFireMap(t, firemaps[t-1])
        return firemaps

    def isInsideMap(self, map, res):
        ################
        # examine if a rescuer is inside the map's boundary
        ################
        # m, n: the size of the arena
        # res: a rescuer, mostly consists of 2 coordinates
        # flag: boolean, 1 for inside, 0 for outside
        ################
        x = res[0]
        y = res[1]
        m = len(map[0])
        n = len(map)
        if x < m and x >= 0 and y < n and y >= 0:
            flag = True
        else:
            flag = False
        return flag

    def plot_fire(self, firemap):
        intersection_matrix = np.transpose(firemap)
        plt.matshow(intersection_matrix, cmap=plt.cm.Reds, origin='lower')
        plt.show()

    def compute_monteCarlo(self, _GenerateHorizon):
        # return something looks like FireMap
        # self.FireMap is a series of M*N matrices with elements 0 or 1
        # We will generate it multiple times and do an average to compute
        # the imperical probability of fire for each grid on the map
        # also we will save all the instances generated, to compute conditional empirical prabability
        self.monteCarloHorizon = _GenerateHorizon
        firemap_sum = np.asarray(np.copy(self.FireMap), dtype=np.float32)
        try:
            os.mkdir('./monteCarlo')
            os.mkdir('./monteCarloAverage')
        except:
            print('Folder existed!')
        for h in range(_GenerateHorizon):
            print('started {} MC simulation'.format(h+1))
            new_firemap = np.asarray(self.mapGenerator(self.T), dtype=np.float32)
            try:
                os.mkdir('./monteCarlo/monteCarloFireMapTrial'+str(h))
            except:
                print('Folder existed!')
            for k in range(len(firemap_sum)):
                firemap_sum[k] = np.add(firemap_sum[k], new_firemap[k])
                # save all instances
                np.savetxt('./monteCarlo/monteCarloFireMapTrial'+str(h)+'/monteCarloFireMapAt'+str(k)+'.txt', new_firemap[k])
        for k in range(len(firemap_sum)):
            print('started {} averaging'.format(k+1))
            firemap_sum[k] = np.divide(firemap_sum[k], _GenerateHorizon)
            np.savetxt('./monteCarloAverage/monteCarloAverageFireMap'+str(k)+'.txt', firemap_sum[k])
        return firemap_sum

    def compute_monteCarlo_2(self, _GenerateHorizon):
        # Do as compute_monteCarlo but not saving
        # Time horizon fixed to be len(self.FireMap)
        monteCarloFireMap = [[[] for k in range(len(self.FireMap))] for h in range(_GenerateHorizon)]
        for h in range(_GenerateHorizon):
            # monteCarloFireMap[h] = np.asarray(self.mapGenerator(self.T), dtype=np.float32)
            new_firemap = np.asarray(self.mapGenerator(self.T), dtype=np.float32)
            for k in range(len(self.FireMap)):
                monteCarloFireMap[h][k] =  new_firemap[k]
        return monteCarloFireMap

    def __init__(self, target = [[18,16]], _stochastic_environment_flag=1, _setting_num=1):
        self.stochastic_environment_flag = _stochastic_environment_flag

        # problem settings: 
        # if the fire is changing overtime: 1
        # if the fire remains its initial locations: 0
        self.stochastic_environment_flag = 1
        # initial settings for the problem
        self.M = 20 # map[n][m]
        self.N = 20
        self.T = 50 # time horizon for DP computation: usually less than the time for the fire to spread the whole map
        # read in the (initial)map for map value checking
        # do notice the difference between this and the Matlab code (1 smaller in indices as py starts from 0)
        self.StartPoint = [[8,0]]
        self.EndPoint = [[19,12]]
        self.Target = target
        # Target = [[8, 12]]
        self.Wall = [[6,0],
            [6,1],
            [6,2],
            [6,3],
            [6,4],
            [6,5],
            [10,0],
            [10,1],
            [10,2],
            [10,3],
            [10,4],
            [10,5],
            [6,14],
            [7,14],
            [8,14],
            [9,14],
            [10,14],
            [11,14],
            [12,14],
            [13,14],
            [15,14],
            [16,14],
            [17,14],
            [18,14],
            [19,14],
            [17,9],
            [17,10],
            [17,11],
            [17,12],
            [17,13]]
        # This only reflects Fire at t=0, and will NOT be updated
        # Fire = [[1, 18], [11, 16]] 
        self.Fire = [[1, 18], [7,11],[2,4]]
        self.pf0 = 0.087 # 0.087
        self.pf1 = 0.01

        self.Map = []
        self.FireMap = []

        self.monteCarloHorizon = 3000

        if(_setting_num == 0):
            self.M = 4
            self.N = 4
            self.T = 8
            # read in the (initial)map for map value checking
            # do notice the difference between this and the Matlab code (1 smaller in indices as py starts from 0)
            self.StartPoint = [[0,0]]
            self.EndPoint = [[2,3]]
            self.Target = [[2,3]]
            self.Wall = [[1,3]]
            self.Fire = [[0,2]]
        # Generate static Map and fire map list FireMap
        self.Map = self.mapCreator()
        self.FireMap = self.mapGenerator(self.T)
        # Test if fire is spreaded expectedly
        # self.plot_fire(self.FireMap[49])

    # def __del__(self):
    #     class_name = self.__class__.__name__
    #     print(class_name, 'problem_settings_destroyed')
