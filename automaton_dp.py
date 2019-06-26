import math
import numpy as np
import csv
import time
import matplotlib.pyplot as plt
import path_evaluation

class AutomatonDP:

    def __init__(self, M, N, T, Map, averageFireMap, FireMap, StartPoint, EndPoint, Target, Wall, Fire, Q=4, N_control=5,_observeTime=0, _observeFireState=None, _observeX=0,_observeY=0):
        self.M = M
        self.N = N
        self.T = T
        self.StartPoint = StartPoint
        self.EndPoint = EndPoint
        self.Target = Target
        self.Wall = Wall
        self.Fire = Fire
        self.Map = Map
        self.averageFireMap = averageFireMap
        self.FireMap = FireMap# all instances of MC simulation
        self.Q = Q
        self.N_control = N_control

        self.pr = 0 # uncertainty for robot dynamics # used to be 0.05 for single target & fair comparison

        self.P = []
        self.P_u = []
        self.P_k = []
        self.state_mapper = {}
        self.visited = set()

        self.path = []
        
        self.g = []
        self.gT = []

        self.controls = []
        self.routes = []
        self.Js = []

        self.N_state = self.M * self.N * self.Q
        self.P = [[[]for time_range in range(self.T)]for control_range in range(self.N_control)]
        self.P_u = [[]for time_range in range(self.T)]
        self.P_k = [[0 for col in range(self.N_state)] for row in range(self.N_state)]
        self.state_mapper = {}

        self.pe = path_evaluation.PathEvaluation(self.FireMap, target = self.Target, observeTime=_observeTime, observeFireState=_observeFireState, x=_observeX, y=_observeY)
    def dfs(self, state, k):
        ################
        # do dfs from the initial state to generate all possible states at time k
        # dfs start from **state** at time **k**, generating all the transitable neighbors at time k
        # actually here we still have some states that don't exist in reality, but already smaller than N_state
        # called in main()
        ################
        self.visited.add(state) # very important line
        # print("start dfs %s at time %d" %(state, k,))
        index = self.state_mapper.get(state)
        if index == 2:
            pass
            # print("successfully END rescue at time %d" %(k,))
            return
        elif index == 3:
            pass
            # print("reach the fire and FAIL at time %d" %(k,))
            return
        else:
            # not a terminal state, expand it
            neighbors, Dr, Nr, flag = self.generate_neighbors(state, k) # returns a set of tuples and 2 numbers
            if flag != 1:
                #P_k[index][index] = 1-pr*(Nr+Dr/np.sqrt(2))
                for nei in neighbors:
                    if state[0] == nei[0] and state[1] == nei[1]:
                        # nothing
                        self.P_k[index][self.state_mapper.get(nei)] = 1-self.pr*(Nr+Dr/np.sqrt(2))
                    elif state[0] == nei[0] or state[1] == nei[1]:
                        self.P_k[index][self.state_mapper.get(nei)] = self.pr
                    else:
                        self.P_k[index][self.state_mapper.get(nei)] = self.pr/np.sqrt(2)
                    if not nei in self.visited:
                        self.dfs(nei, k)
            else:
                #P_k[index][index] = 1-pr*(Nr+Dr/np.sqrt(2))
                for nei in neighbors:
                    if(k >= self.T-1):
                        probToFireAtNeighborState = 1
                    else:
                        # probToFireAtNeighborState = self.averageFireMap[k+1][nei[0]][nei[1]]
                        probToFireAtNeighborState = 1-self.pe.evaluate_segment(k,[state[0],state[1]],[nei[0],nei[1]])
                    if state[0] == nei[0] and state[1] == nei[1]:
                        self.P_k[index][self.state_mapper.get(nei)] = (1-self.pr*(Nr+Dr/np.sqrt(2))) * (1-probToFireAtNeighborState)
                        self.P_k[index][3] = self.P_k[index][3] + (1-self.pr*(Nr+Dr/np.sqrt(2))) * probToFireAtNeighborState
                    elif state[0] == nei[0] or state[1] == nei[1]:
                        self.P_k[index][self.state_mapper.get(nei)] = self.pr * (1-probToFireAtNeighborState)
                        self.P_k[index][3] = self.P_k[index][3] + self.pr * probToFireAtNeighborState
                    else:
                        self.P_k[index][self.state_mapper.get(nei)] = self.pr/np.sqrt(2) * (1-probToFireAtNeighborState)
                        self.P_k[index][3] = self.P_k[index][3] + self.pr/np.sqrt(2) * probToFireAtNeighborState
                    if not nei in self.visited:
                        self.dfs(nei, k)
        # print("End dfs %s of time %d" %(state, k))
        return

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

    def generate_neighbors(self, state, k):
        ################
        # generate all neighbors at time k, Map and averageFireMap[k] will be used here
        ################
        x = state[0]
        y = state[1]
        # q = state[2]
        neighbors = set()
        map = self.Map # map used in this generating process
        firemap = self.averageFireMap[k]
        Dr = 0
        Nr = 0
        probablilistic_failure_flag = 0
        # position 1,2,3,4,5,6,7,8,9
        for i in range(-1,2):
            for j in range(-1,2):
                if self.isInsideMap(map, [x+i,y+j]):
                    try:
                        v = map[x+i][y+j] # map value of the neighbor
                        vf = firemap[x+i][y+j]
                        if v != 1: # NOT a wall
                            state_ = self.confirm_neighbor_state(state, x+i, y+j, v)
                            if not(i == 0 and j == 0):
                                if i == 0 or j == 0:
                                    Nr = Nr + 1
                                else:
                                    Dr = Dr + 1
                            self.add_mapper(state_, k)
                            neighbors.add(state_)
                        if vf >= 0.0001:
                            # and vf <= 0.9999: # there's a non zero but also not 1 possibility to go to fire and fail
                            # The reason I wrote <=0.9999 was, if it's with probability 1 to be fire, it will be 
                            # already considered in confirm_neighbor_state(), but as add_mapper and set.add() can prevent 
                            # adding same things twice, vf<=0.9999 can be ignored here. 
                            # So this if sentence is only for those non-fire state but has a non-zero probability to be 
                            # contaminated
                            self.add_mapper(tuple([-1, -1, 2]), k)
                            neighbors.add(tuple([-1, -1, 2]))
                            probablilistic_failure_flag = 1 # flag basically means there's possibility to go to Fail state
                    except:
                        print(x+i, y+j)
        ## if computing probabilities, then 0,0 case needs to be considered HERE
        return neighbors, Dr, Nr, probablilistic_failure_flag

    def confirm_neighbor_state(self, state, x1, y1, v):
        # as long as there's no wall/edge, then the neighbors(including diagonals) on the map are reachable
        # state: orignal state
        # ([x1, y1, q1]) is the new candidate state, v is Map[k][x1][y1] value
        # x = state[0]
        # y = state[1]
        q = state[2]
        
        if q == -3 or q == -2:
            state_ = state
        elif v == 2 or q == 2: # fire, rescue terminates
            state_ = tuple([-1, -1, 2]) # goes to loose state directly
        elif v == -3: # reach the exit, then q needs to be examined to see if the state transits
            if q == -2:
                state_ = tuple([x1, y1, -3]) # goes to win state directly
            else:
                state_ = tuple([x1, y1, q])
        elif v == 0 or v == -1: # q remains
            state_ = tuple([x1, y1, q])
        elif v == -2: # reach the target, check q for state transition
            if q == -1:
                state_ = tuple([x1, y1, -2]) # transit from m_{-1} to m_{-2}
            else:
                state_ = tuple([x1, y1, q])
        else: 
            print("Unexpectd map value read, please check the program")
            state_ = tuple([-1, -1, 2])
        return state_

    def add_mapper(self, state, k):
        if not state in self.state_mapper:
            self.state_mapper.setdefault(state, len(self.state_mapper.keys()))


    def generate_p_for_P_k(self, k, u):
        #print(len(list(state_mapper[k])))
        for state in list(self.state_mapper):
            if not state in self.visited:
                self.dfs2(state, k, u)
        
    def dfs2(self, state, k, u):
        self.visited.add(state)
        index = self.state_mapper.get(state)
        if index == 2:
            pass
            # print("successfully END rescue at time %d" %(k,))
            return
        elif index == 3:
            pass
            # print("reach the fire and FAIL at time %d" %(k,))
            return
        else:
            # not a terminal state, compute for its neighbors
            state_ = self.next_state(state, k, u)
            neighbors, Dr, Nr, flag = self.generate_neighbors(state_, k) # returns a set of tuples and 2 numbers
            if flag != 1:
                for nei in neighbors:
                    if state_[0] == nei[0] and state_[1] == nei[1]:
                        # nothing
                        self.P_k[index][self.state_mapper.get(nei)] = 1-self.pr*(Nr+Dr/np.sqrt(2))
                    elif state_[0] == nei[0] or state_[1] == nei[1]:
                        self.P_k[index][self.state_mapper.get(nei)] = self.pr
                    else:
                        self.P_k[index][self.state_mapper.get(nei)] = self.pr/np.sqrt(2)
                    if not nei in self.visited:
                        self.dfs2(nei, k, u)
            else:
                for nei in neighbors:
                    if(k >= self.T-1):
                        probToFireAtNeighborState = 1
                    else:
                        # probToFireAtNeighborState = self.averageFireMap[k+1][nei[0]][nei[1]]
                        probToFireAtNeighborState = 1-self.pe.evaluate_segment(k,[state[0],state[1]],[nei[0],nei[1]])
                    if state_[0] == nei[0] and state_[1] == nei[1]:
                        self.P_k[index][self.state_mapper.get(nei)] = (1-self.pr*(Nr+Dr/np.sqrt(2))) * (1-probToFireAtNeighborState)
                        self.P_k[index][3] = self.P_k[index][3] + (1-self.pr*(Nr+Dr/np.sqrt(2))) * probToFireAtNeighborState
                    elif state_[0] == nei[0] or state_[1] == nei[1]:
                        self.P_k[index][self.state_mapper.get(nei)] = self.pr * (1-probToFireAtNeighborState)
                        self.P_k[index][3] = self.P_k[index][3] + self.pr * probToFireAtNeighborState
                    else:
                        self.P_k[index][self.state_mapper.get(nei)] = self.pr/np.sqrt(2) * (1-probToFireAtNeighborState)
                        self.P_k[index][3] = self.P_k[index][3] + self.pr/np.sqrt(2) * probToFireAtNeighborState
                    if not nei in self.visited:
                        self.dfs2(nei, k, u)            
        return

    def next_state(self, state, k, u):
        x = state[0]
        y = state[1]
        q = state[2]
        state_ = state
        if q == -3 or q == 2 or q== -2:
            state_ = state
        elif u == 1:
            if self.isInsideMap(self.Map, tuple([x,y+1,q])):
                if self.Map[x][y+1] != 1:
                    #state_ = tuple([x,y+1,q])
                    state_ = self.confirm_neighbor_state(state, x, y+1, self.Map[x][y+1])
        elif u == 2: 
            if self.isInsideMap(self.Map, tuple([x+1,y,q])):
                if self.Map[x+1][y] != 1:
                    #state_ = tuple([x+1,y,q])
                    state_ = self.confirm_neighbor_state(state, x+1, y, self.Map[x+1][y])
        elif u == 3: 
            if self.isInsideMap(self.Map, tuple([x,y-1,q])):
                if self.Map[x][y-1] != 1:
                    #state_ = tuple([x,y-1,q])
                    state_ = self.confirm_neighbor_state(state, x, y-1, self.Map[x][y-1])
        elif u == 4: 
            if self.isInsideMap(self.Map, tuple([x-1,y,q])):
                if self.Map[x-1][y] != 1:
                    #state_ = tuple([x-1,y,q])
                    state_ = self.confirm_neighbor_state(state, x-1, y, self.Map[x-1][y])
        else:
            state_ = state
        return state_

    def markovDP( self, P, g, gT, T ):

        _t = time.time()
        # DP solver for finite state MDP
        # P being a 4-dim matrix. namely P[u][k][i][j] is the transition probability from state
        # i to j under control action u at time k
        # g[u][k][i][j] is stage cost from i to j under u at k
        # gT is terminal cost array
        # T is terminal time
        # Output results: 
        # J0 is vector of optimal cost to go for each state at time 0
        # mu0 is vectorr of optimal control action for each state at time 0
        N = len(gT) # number of states
        M = len(P) # number of control inputs
        npP = np.array(P)
        J = [[0 for time in range(T+1)] for state in range(N)]
        #print(npJ.shape)
        J0 = [0 for state in range(N)]
        mu0 = [0 for state in range(N)]
        choice = [[0 for time in range(T)] for state in range(N)]
        for i in range(N):
            J[i][T] = gT[i]
        npJ = np.array(J)
        All_J_i_k = [0 for control in range(M)] 
        for k in reversed(range(T)): # backward recuision
            for i in range(N): # for each state
                All_J_i_k = [0 for control in range(M)]
                for u in range(M):
                    All_J_i_k[u] =  np.dot(npJ[:,k+1] + np.array(g[u][k][i][:]), np.array(npP[u,k])[i,:])
                npJ[i, k] = np.amax(All_J_i_k)
                choice[i][k] = np.argmax(All_J_i_k)
        

        elapsed_ = time.time() - _t
        print('computing solution took: %d seconds' %(elapsed_)) 

        for p in range(N):
            J0[p] = npJ[p, 0]
            mu0[p] = choice[p][0]

        # controls for initial start point at different time
        mu = [0 for time in range(T)]
        for t in range(T):
            mu[t] = choice[0][t]
            
        
        # save the results as some visible form, namely extract the best route for robot at "start"
        control = [0 for time in range(T)]
        route = [0 for time in range(T+1)]
        route[0] = tuple([self.StartPoint[0][0],self.StartPoint[0][1],-1])
        prev_state = tuple([self.StartPoint[0][0],self.StartPoint[0][1],-1])
        for t in range(T):
            control[t] = choice[self.state_mapper[prev_state]][t]
            current_state = self.next_state(prev_state, t, control[t])
            prev_state = current_state
            route[t+1] = current_state
        # save almost all the history
        # for all the points at every time step save: (1)full optimal path; (2) associated controls
        # controls[t][s][\tau]: from (t,s), controls[t][s][\tau] should apply till the end, as it's optimal
        # routes[t][s][\tau]: resulting path from controls[t][s][\tau]
        # Js[t[s] maximum cost-to-go can get at (t,s), by following the optimal policy
        controls =[[[0 for time in range(T-timeStep)] for n in range(N)] for timeStep in range(T)]
        routes = [[[0 for time in range(T-timeStep+1)] for n in range(N)] for timeStep in range(T)]
        Js = [[0 for n in range(N)] for timeStep in range(T)]
        for timeStep in range(T):
            for initialState in list(self.state_mapper.keys()):
                initialStateNumber = self.state_mapper.get(initialState)
                routes[timeStep][initialStateNumber][0] = initialState
                prevState = initialState
                Js[timeStep][initialStateNumber] = npJ[initialStateNumber, timeStep]
                for t in range(T-timeStep):
                    prevStateNumber = self.state_mapper.get(prevState)
                    controls[timeStep][initialStateNumber][t] = choice[prevStateNumber][timeStep+t]
                    currentState = self.next_state(prevState, t, controls[timeStep][initialStateNumber][t])
                    prevState = currentState
                    routes[timeStep][initialStateNumber][t+1] = currentState
        return J0, control, route, controls, routes, Js

    def solve(self):
        _t = time.time()
        for u in range(self.N_control):
            self.P_u = [[]for time_range in range(self.T)]
            if u == 0:
                self.state_mapper = {}
                for k in range(self.T):
                    # (tuple([pos_x, pos_y, automaton_state]), increasing int for numbering, )
                    self.state_mapper.setdefault(tuple([self.StartPoint[0][0],self.StartPoint[0][1],-1]), 0) # initial state
                    self.state_mapper.setdefault(tuple([self.EndPoint[0][0],self.EndPoint[0][1],-3]), 1) # "successfully end rescue" state
                    self.state_mapper.setdefault(tuple([self.Target[0][0],self.Target[0][1],-2]), 2) # "reach the target" state
                    self.state_mapper.setdefault(tuple([-1,-1,2]), 3) # "reach the fire" state
                    self.visited = set()
                    self.P_k = [[0 for col in range(self.N_state)] for row in range(self.N_state)]
                    self.dfs(tuple([self.StartPoint[0][0],self.StartPoint[0][1],-1]),k)
                    # manually set absorbing states
                    self.P_k[1][1] = 1
                    # FOR SINGLE TARGET
                    for i in range(len(self.P_k[2])):
                        self.P_k[2][i] = 0
                    self.P_k[2][2] = 1
                    self.P_k[3][3] = 1 
                    N_state_r = len(self.state_mapper)
                    self.P_u[k] = [[0 for col in range(N_state_r)] for row in range(N_state_r)]
                    for i in range(N_state_r):
                        for j in range(N_state_r):
                            self.P_u[k][i][j] = self.P_k[i][j] # should have faster way to do this
                    self.P[u][k] = [[0 for col in range(N_state_r)] for row in range(N_state_r)]
                    for i in range(N_state_r):
                        for j in range(N_state_r):
                            try:
                                self.P[u][k][i][j] = self.P_u[k][i][j] # should have faster way to do this
                            except:
                                print(u,k,i,j)
            else:
                for k in range(self.T):
                    self.visited = set()
                    self.P_k = [[0 for col in range(self.N_state)] for row in range(self.N_state)]
                    self.generate_p_for_P_k(k, u)# traversely fill the P_k for other controls other than u=0
                    # manually set absorbing states
                    self.P_k[1][1] = 1
                    # FOR SINGLE TARGET
                    for i in range(len(self.P_k[2])):
                        self.P_k[2][i] = 0
                    self.P_k[2][2] = 1
                    self.P_k[3][3] = 1 
                    N_state_r = len(self.state_mapper)
                    self.P_u[k] = [[0 for col in range(N_state_r)] for row in range(N_state_r)]
                    for i in range(N_state_r):
                        for j in range(N_state_r):
                            self.P_u[k][i][j] = self.P_k[i][j] # should have faster way to do this
                    self.P[u][k] = [[0 for col in range(N_state_r)] for row in range(N_state_r)]
                    for i in range(N_state_r):
                        for j in range(N_state_r):
                            try:
                                self.P[u][k][i][j] = self.P_u[k][i][j] # should have faster way to do this
                            except:
                                print(u,k,i,j)

        elapsed_ = time.time() - _t
        print('Constructing P took: %d seconds' %(elapsed_)) 

        # PRAMS
        N_state_r_last = len(self.state_mapper)

        # constructing cost matrix g, terminal cost vector gT
        # gT
        self.gT = [0.0 for state in range(N_state_r_last)]
        # FOR SINGLE TARGET
        # self.gT[1] = 100 # gain 100 for winning the game
        self.gT[2] = 100.0 # gain 100 for winning the game
        self.gT[3] = 0 # 0 for reaching the fire
        self.g = [[[[0 for j in range(N_state_r_last)] for i in range(N_state_r_last)]for time_range in range(self.T)]for control_range in range(self.N_control)]
        for t in range(self.T):
            # FOR SINGLE TARGET
            self.g[0][t][2][2] = 0 # encourage to finish as soon as possible
                    # self.g[u][t][1][x2] = 0
        
        # DP process and time counting
        J0, control, route, self.controls, self.routes, self.Js = self.markovDP( self.P, self.g, self.gT, self.T )

        # print optimal controls for the controls from starting point
        print('optimal controls are: ')
        print(control)

        print('printing payoffs for each state at t=0')
        print(J0)

        self.remove_duplicated_path_segs(route) # only do once
        # Visualize the optimal controls
        route_array = np.asarray(route)
        x = route_array[:,0]
        y = route_array[:,1]
        q = route_array[:,2]
        c = np.arange(0,len(x))
        self.path = np.vstack((x,y,c)).transpose()

        StartPoint_array = np.asarray(self.StartPoint)
        EndPoint_array = np.asarray(self.EndPoint)
        Target_array = np.asarray(self.Target)
        Wall_array = np.asarray(self.Wall)
        Fire_array = np.asarray(self.Fire)

        fig = plt.figure()
        ax = fig.gca()
        # fire plotting starts
        min_val, max_val = 0, 20
        intersection_matrix = np.transpose(self.averageFireMap[25])
        ax.matshow(intersection_matrix, cmap=plt.cm.Reds, origin='lower')
        # fire ends
        ax.set_xticks(np.arange(0,20,1))
        ax.set_yticks(np.arange(0,20,1))
        plt.xlim(0,20)
        plt.ylim(0,20)
        plt.plot(StartPoint_array[:,0], StartPoint_array[:,1], 'bD')
        plt.plot(EndPoint_array[:,0], EndPoint_array[:,1], 'gD')
        plt.plot(Target_array[:,0], Target_array[:,1], 'yD')
        plt.plot(Wall_array[:,0], Wall_array[:,1], 'kp')
        plt.plot(Fire_array[:,0], Fire_array[:,1], 'rx')
        plt.plot(x,y,c='m')
        plt.grid(linestyle = '--')
        # plt.show()
        fig.savefig('dp_planning_2d.jpeg')

        pe1 = path_evaluation.PathEvaluation(self.FireMap)
        DPscore = pe1.evaluate_path(route)
        print('printing optimal DP path together')
        print(route)
        print('DP solution length: %d' %(len(route)))
        print('DP solution probability: %f' %(DPscore) )

    def query_optimal_solution(self, queryStartTime, queryStartState):
        # return: 
        # cost-to-go, optimal route and optimal controls from (queryStartTime, queryStartState)
        queryStartStateNumber = self.state_mapper.get(tuple(queryStartState))
        costsFromNow = [0 for t in range(self.T)] # WILL BE filled with 0s in the front
        routesFromNow = self.routes[queryStartTime][queryStartStateNumber]
        controlsFromNow = self.controls[queryStartTime][queryStartStateNumber]
        for t in range(self.T - queryStartTime):
            stateNum = self.state_mapper.get(routesFromNow[t])
            costsFromNow[queryStartTime + t] = self.Js[queryStartTime + t][stateNum]
        return costsFromNow, routesFromNow, controlsFromNow

    def remove_duplicated_path_segs(self, path):
        endPath = tuple([self.Target[0][0],self.Target[0][1],-2])
        while(path[-2] == endPath):
            del path[-1]
        return