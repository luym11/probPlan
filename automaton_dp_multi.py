import math
import numpy as np
import csv
import time
import matplotlib.pyplot as plt
import itertools
import copy
import path_evaluation

class AutomatonDPMulti:

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

        # current code is based on this assumption. If this is changed, major modification needed
        self.pr = 0 # uncertainty for robot dynamics # used to be 0.05 for single target & fair comparison 

        self.state_mapper = {}
        self.visited = set()
        
        self.g = []
        self.gT = []

        self.controls = []
        self.routes = []
        self.Js = []

        self.N_state = self.M * self.N * self.Q * self.M * self.N
        self.P = [[[[]for time_range in range(self.T)]for control_range in range(self.N_control)]for control_range in range(self.N_control)]
        self.P_u = [[]for time_range in range(self.T)] # for a certain u1, u2 pair
        self.P_k = [[0 for col in range(self.N_state)] for row in range(self.N_state)]

        self.pe = path_evaluation.PathEvaluation(self.FireMap, observeTime=_observeTime, observeFireState=_observeFireState, x=_observeX, y=_observeY)
    def dfs(self, state, k):
        ################
        # do dfs from the initial state to generate all possible states at time k
        # dfs start from **state** at time **k**, generating all the transitable neighbors at time k
        # actually here we still have some states that don't exist in reality, but already smaller than N_state
        # called in solve()
        ################
        self.visited.add(state) # very important line
        print("start dfs %s at time %d" %(state, k,))
        index = self.state_mapper.get(state)
        if index == 1:
            pass
            # print("successfully END rescue at time %d" %(k,))
            return
        elif index == 2:
            pass
            # print("reach the fire and FAIL at time %d" %(k,))
            return
        else:
            # not a terminal state, expand it
            allNeighbors, neighbors1, Dr1, Nr1, flag1, neighbors2, Dr2, Nr2, flag2 = self.generate_neighbors(state, k)
            if (flag1 != 1 and flag2 != 1): # no fire in both neighborhoods
                for nei in allNeighbors:
                    if state[0] == nei[0] and state[1] == nei[1] and state[2] == nei[2] and state[3] == nei[3]:
                        self.P_k[index][self.state_mapper.get(nei)] = 1
                    else:
                        self.P_k[index][self.state_mapper.get(nei)] = 0
                    if not nei in self.visited:
                        self.dfs(nei, k)
            elif flag1 != 1: # no fire in neighborhood of robot 1
                for nei in allNeighbors:
                    if(k >= self.T-1):
                        probToFireAtNeighborState = 1
                    else:
                        probToFireAtNeighborState = 1-self.pe.evaluate_segment(k,[state[2],state[3]],[nei[2],nei[3]]) # Here it's an intersection
                    if state[0] == nei[0] and state[1] == nei[1] and state[2] == nei[2] and state[3] == nei[3]:
                        self.P_k[index][self.state_mapper.get(nei)] = 1-probToFireAtNeighborState
                        self.P_k[index][3] = self.P_k[index][3] + probToFireAtNeighborState
                    else:
                        self.P_k[index][self.state_mapper.get(nei)] = 0
                        self.P_k[index][3] = self.P_k[index][3] + 0
                    if not nei in self.visited:
                        self.dfs(nei, k)
            elif flag2 != 1: # no fire in neighborhood of robot 2
                for nei in allNeighbors:
                    if(k >= self.T-1):
                        probToFireAtNeighborState = 1
                    else:
                        probToFireAtNeighborState = 1-self.pe.evaluate_segment(k,[state[0],state[1]],[nei[0],nei[1]]) # Here it's an intersection
                    if state[0] == nei[0] and state[1] == nei[1] and state[2] == nei[2] and state[3] == nei[3]:
                        self.P_k[index][self.state_mapper.get(nei)] = 1-probToFireAtNeighborState
                        self.P_k[index][3] = self.P_k[index][3] + probToFireAtNeighborState
                    else:
                        self.P_k[index][self.state_mapper.get(nei)] = 0
                        self.P_k[index][3] = self.P_k[index][3] + 0
                    if not nei in self.visited:
                        self.dfs(nei, k)
            else: # fire in neighborhood of both robot 1 and 2
                for nei in allNeighbors:
                    if(k >= self.T-1):
                        probToFireAtNeighborState = 1
                    else:
                        probToFireAtNeighborState = 1-self.pe.evaluate_segment(k,[state[0],state[1]],[nei[0],nei[1]]) * self.pe.evaluate_segment(k,[state[2],state[3]],[nei[2],nei[3]]) # Here it's an intersection
                    if state[0] == nei[0] and state[1] == nei[1] and state[2] == nei[2] and state[3] == nei[3]:
                        self.P_k[index][self.state_mapper.get(nei)] = 1-probToFireAtNeighborState
                        self.P_k[index][3] = self.P_k[index][3] + probToFireAtNeighborState
                    else:
                        self.P_k[index][self.state_mapper.get(nei)] = 0
                        self.P_k[index][3] = self.P_k[index][3] + 0
                    if not nei in self.visited:
                        self.dfs(nei, k)
        print("End dfs %s of time %d" %(state, k))
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
        x1 = state[0]
        y1 = state[1]
        x2 = state[2]
        y2 = state[3]
        # q = state[4]
        neighbors1 = set()
        neighbors2 = set()
        _map = self.Map # map used in this generating process
        firemap = self.averageFireMap[k]
        Dr1 = 0
        Nr1 = 0
        Dr2 = 0
        Nr2 = 0
        probablilistic_failure_flag_1 = 0
        probablilistic_failure_flag_2 = 0
        # position 1,2,3,4,5,6,7,8,9 for robot 1 and robot 2
        for i in range(-1,2):
            for j in range(-1,2):
                if self.isInsideMap(_map, [x1+i,y1+j]):
                    try:
                        v = _map[x1+i][y1+j] # map value of the neighbor
                        vf = firemap[x1+i][y1+j]
                        if v != 1: # NOT a wall
                            state_ = self.confirm_neighbor_state(state, x1+i, y1+j, v)
                            if not(i == 0 and j == 0):
                                if i == 0 or j == 0:
                                    Nr1 = Nr1 + 1
                                else:
                                    Dr1 = Dr1 + 1
                            ## self.add_mapper(state_)
                            neighbors1.add(state_)
                        if vf >= 0.0001:
                            # and vf <= 0.9999: # there's a non zero but also not 1 possibility to go to fire and fail
                            # The reason I wrote <=0.9999 was, if it's with probability 1 to be fire, it will be 
                            # already considered in confirm_neighbor_state(), but as add_mapper and set.add() can prevent 
                            # adding same things twice, vf<=0.9999 can be ignored here. 
                            # So this if sentence is only for those non-fire state but has a non-zero probability to be 
                            # contaminated
                            ## self.add_mapper(tuple([-1, -1, 2]))
                            neighbors1.add(tuple([-1, -1, 2]))
                            probablilistic_failure_flag_1 = 1 # flag basically means there's possibility to go to Fail state
                    except:
                        print(x1+i, y1+j)
        for i in range(-1,2):
            for j in range(-1,2):
                if self.isInsideMap(_map, [x2+i,y2+j]):
                    try:
                        v = _map[x2+i][y2+j] # map value of the neighbor
                        vf = firemap[x2+i][y2+j]
                        if v != 1: # NOT a wall
                            state_ = self.confirm_neighbor_state(state, x2+i, y2+j, v)
                            if not(i == 0 and j == 0):
                                if i == 0 or j == 0:
                                    Nr2 = Nr2 + 1
                                else:
                                    Dr2 = Dr2 + 1
                            ## self.add_mapper(state_)
                            neighbors2.add(state_)
                        if vf >= 0.0001:
                            # and vf <= 0.9999: # there's a non zero but also not 1 possibility to go to fire and fail
                            # The reason I wrote <=0.9999 was, if it's with probability 1 to be fire, it will be 
                            # already considered in confirm_neighbor_state(), but as add_mapper and set.add() can prevent 
                            # adding same things twice, vf<=0.9999 can be ignored here. 
                            # So this if sentence is only for those non-fire state but has a non-zero probability to be 
                            # contaminated
                            ## self.add_mapper(tuple([-1, -1, 2]))
                            neighbors2.add(tuple([-1, -1, 2]))
                            probablilistic_failure_flag_2 = 1 # flag basically means there's possibility to go to Fail state
                    except:
                        print(x2+i, y2+j)
        allNeighbors = [self.add_mapper(tuple([x1,y1,x2,y2,self.union_q(q1,q2)])) for (x1,y1,q1) in neighbors1 for (x2,y2,q2) in neighbors2]
        # Remove None in allNeighbors
        allNeighborsClean = [i for i in allNeighbors if i]
        ## if computing probabilities, then 0,0 case needs to be considered HERE
        return allNeighborsClean, neighbors1, Dr1, Nr1, probablilistic_failure_flag_1, neighbors2, Dr2, Nr2, probablilistic_failure_flag_2

    def union_q(self, q1, q2):
        if (q1 == 2 or q2 == 2):
            return 2
        elif (q1 == 1 and q2 == 1):
            return 1
        elif (q1 == 3 or q2 == 3):
            return 3
        return 0

    def confirm_neighbor_state(self, state, x1, y1, v):
        # as long as there's no wall/edge, then the neighbors(including diagonals) on the map are reachable
        # state: orignal state
        # NOTE: this function deals with robots SEPARATELY
        # ([x1, y1, q1]) is the new candidate state, v is Map[k][x1][y1] value
        q = state[4]
        
        # if q == 1 or q == 2:
        if q == 1: # success state
            state_ = state
        elif v == 2 or q == 2: # fire, rescue terminates
            state_ = tuple([-1, -1, 2]) # goes to loose state directly
        elif v == -3: # reach the exit, then q needs to be examined to see if the state transits
            if q == 3:
                state_ = tuple([x1, y1, 1]) # goes to win state directly
            else:
                state_ = tuple([x1, y1, q])
        elif v == 0 or v == -1: # q remains
            state_ = tuple([x1, y1, q])
        elif v == -2: # reach the target, check q for state transition
            if q == 0:
                state_ = tuple([x1, y1, 3]) # transit from m_{-1} to m_{-2}
            else:
                state_ = tuple([x1, y1, q])
        else: 
            print("Unexpectd map value read, please check the program")
            state_ = tuple([-1, -1, 2])
        return state_

    def add_mapper(self, state):
        if not state in self.state_mapper:
            self.state_mapper.setdefault(state, len(self.state_mapper.keys()))
            return state
        return None

    def generate_p_for_P_k(self, k, u1, u2):
        #print(len(list(state_mapper[k])))
        for state in list(self.state_mapper):
            if not state in self.visited:
                self.dfs2(state, k, u1, u2)
        
    def dfs2(self, state, k, u1, u2):
        self.visited.add(state)
        print("start dfs2 %s at time %d with control %d, %d" %(state, k, u1, u2,))
        index = self.state_mapper.get(state)
        if index == 1:
            pass
            # print("successfully END rescue at time %d" %(k,))
            return
        elif index == 2:
            pass
            # print("reach the fire and FAIL at time %d" %(k,))
            return
        else:
            # not a terminal state, compute for its neighbors
            state_ = self.next_state(state, k, u1, u2)
            allNeighbors, neighbors1, Dr1, Nr1, flag1, neighbors2, Dr2, Nr2, flag2 = self.generate_neighbors(state_, k)
            if flag1 != 1 and flag2 != 1: # no fire in both neighborhoods
                for nei in allNeighbors:
                    if state_[0] == nei[0] and state_[1] == nei[1] and state_[2] == nei[2] and state_[3] == nei[3]:
                        self.P_k[index][self.state_mapper.get(nei)] = 1
                    else:
                        self.P_k[index][self.state_mapper.get(nei)] = 0
                    if not nei in self.visited:
                        for nextU1 in range(5):
                            for nextU2 in range(5):
                                # TODO: Do I exclude (0,0) case here? Not so clear
                                self.dfs2(nei, k, nextU1, nextU2)
            elif flag1 != 1: # no fire in the neighborhood of robot 1
                for nei in allNeighbors:
                    if(k >= self.T-1):
                        probToFireAtNeighborState = 1
                    else:
                        probToFireAtNeighborState = 1-self.pe.evaluate_segment(k,[state_[2],state_[3]],[nei[2],nei[3]]) # Here it's an intersection
                    if state[0] == nei[0] and state[1] == nei[1] and state[2] == nei[2] and state[3] == nei[3]:
                        self.P_k[index][self.state_mapper.get(nei)] = 1-probToFireAtNeighborState
                        self.P_k[index][3] = self.P_k[index][3] + probToFireAtNeighborState
                    else:
                        self.P_k[index][self.state_mapper.get(nei)] = 0
                        self.P_k[index][3] = self.P_k[index][3] + 0
                    if not nei in self.visited:
                        for nextU1 in range(5):
                            for nextU2 in range(5):
                                # TODO: Do I exclude (0,0) case here? Not so clear
                                self.dfs2(nei, k, nextU1, nextU2)
            elif flag1 != 2: # no fire in the neighborhood of robot 2
                for nei in allNeighbors:
                    if(k >= self.T-1):
                        probToFireAtNeighborState = 1
                    else:
                        probToFireAtNeighborState = 1-self.pe.evaluate_segment(k,[state_[0],state_[1]],[nei[0],nei[1]]) # Here it's an intersection
                    if state[0] == nei[0] and state[1] == nei[1] and state[2] == nei[2] and state[3] == nei[3]:
                        self.P_k[index][self.state_mapper.get(nei)] = 1-probToFireAtNeighborState
                        self.P_k[index][3] = self.P_k[index][3] + probToFireAtNeighborState
                    else:
                        self.P_k[index][self.state_mapper.get(nei)] = 0
                        self.P_k[index][3] = self.P_k[index][3] + 0
                    if not nei in self.visited:
                        for nextU1 in range(5):
                            for nextU2 in range(5):
                                # TODO: Do I exclude (0,0) case here? Not so clear
                                self.dfs2(nei, k, nextU1, nextU2) 
            else: # fire in neighborhood of both robot 1 and 2
                for nei in allNeighbors:
                    if(k >= self.T-1):
                        probToFireAtNeighborState = 1
                    else:
                        probToFireAtNeighborState = 1-self.pe.evaluate_segment(k,[state_[0],state_[1]],[nei[0],nei[1]]) * self.pe.evaluate_segment(k,[state_[2],state_[3]],[nei[2],nei[3]])
                    if state_[0] == nei[0] and state_[1] == nei[1] and state[2] == nei[2] and state[3] == nei[3]:
                        self.P_k[index][self.state_mapper.get(nei)] = 1-probToFireAtNeighborState
                        self.P_k[index][3] = self.P_k[index][3] + probToFireAtNeighborState
                    else:
                        self.P_k[index][self.state_mapper.get(nei)] = 0
                        self.P_k[index][3] = self.P_k[index][3] + 0
                    if not nei in self.visited:
                        for nextU1 in range(5):
                            for nextU2 in range(5):
                                # TODO: Do I exclude (0,0) case here? Not so clear
                                self.dfs2(nei, k, nextU1, nextU2)    
        print("End dfs2 %s at time %d with control %d, %d" %(state, k, u1, u2,))
        return

    def next_state(self, state, k, u1, u2):
        # Implementation notes: remember to reconstruct the full state with 5 entries
        # But the state in self.confirm_neighbor_state() is still the full state
        x1 = state[0]
        y1 = state[1]
        x2 = state[2]
        y2 = state[3]
        q1 = state[4]
        q2 = state[4]
        q = state[4]
        if q == 1 or q == 2:
            return state
        # If not at those absorbing states
        # u1 drives robot 1 (x1,y1,q1)
        if u1 == 1:
            if self.isInsideMap(self.Map, tuple([x1,y1+1,q1])):
                if self.Map[x1][y1+1] != 1:
                    state1_ = self.confirm_neighbor_state(state, x1, y1+1, self.Map[x1][y1+1])
        elif u1 == 2: 
            if self.isInsideMap(self.Map, tuple([x1+1,y1,q1])):
                if self.Map[x1+1][y1] != 1:
                    state1_ = self.confirm_neighbor_state(state, x1+1, y1, self.Map[x1+1][y1])
        elif u1 == 3: 
            if self.isInsideMap(self.Map, tuple([x1,y1-1,q1])):
                if self.Map[x1][y1-1] != 1:
                    state1_ = self.confirm_neighbor_state(state, x1, y1-1, self.Map[x1][y1-1])
        elif u1 == 4: 
            if self.isInsideMap(self.Map, tuple([x1-1,y1,q1])):
                if self.Map[x1-1][y1] != 1:
                    state1_ = self.confirm_neighbor_state(state, x1-1, y1, self.Map[x1-1][y1])
        else: # u1 == 0 or r1 goes out of the map
            state1_ = tuple([x1,y1,q1])
        # u2 drives robot 2 (x2,y2,q2)
        if u2 == 1:
            if self.isInsideMap(self.Map, tuple([x2,y2+1,q2])):
                if self.Map[x2][y2+1] != 1:
                    state2_ = self.confirm_neighbor_state(state, x2, y2+1, self.Map[x2][y2+1])
        elif u2 == 2: 
            if self.isInsideMap(self.Map, tuple([x2+1,y2,q2])):
                if self.Map[x2+1][y2] != 1:
                    state2_ = self.confirm_neighbor_state(state, x2+1, y2, self.Map[x2+1][y2])
        elif u2 == 3: 
            if self.isInsideMap(self.Map, tuple([x2,y2-1,q2])):
                if self.Map[x2][y2-1] != 1:
                    state2_ = self.confirm_neighbor_state(state, x2, y2-1, self.Map[x2][y2-1])
        elif u2 == 4: 
            if self.isInsideMap(self.Map, tuple([x2-1,y2,q2])):
                if self.Map[x2-1][y2] != 1:
                    state2_ = self.confirm_neighbor_state(state, x2-1, y2, self.Map[x2-1][y2])
        else: # u2 == 0 or r2 goes out of the map
            state2_ = tuple([x2,y2,q2])
        # combine to form the new state
        x1 = state1_[0]
        y1 = state1_[1]
        x2 = state2_[0]
        y2 = state2_[1]
        q1 = state1_[2]
        q2 = state2_[2]
        q = self.union_q(q1,q2)
        return tuple([x1,y1,x2,y2,q])

    def markovDP( self, P, g, gT, T ):
        _t = time.time()
        # DP solver for finite state MDP
        # P being a 5-dim matrix. namely P[u1][u2][k][i][j] is the transition probability from state
        # i to j under control action u1 and u2 at time k
        # g[u1][u2][k][i][j] is stage cost from i to j under u at k
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
        choice = [[[-1,-1] for time in range(T)] for state in range(N)]
        for i in range(N):
            J[i][T] = gT[i] # find faster way to do this
        npJ = np.array(J)
        All_J_i_k = [[0 for control1 in range(M)] for control2 in range(M)]
        for k in reversed(range(T)): # backward recuision
            for i in range(N): # for each state
                All_J_i_k = [[0 for control1 in range(M)] for control2 in range(M)]
                for u1 in range(M):
                    for u2 in range(M):
                        All_J_i_k[u1][u2] =  np.dot(npJ[:,k+1] + np.array(g[u1][u2][k][i][:]), np.array(npP[u1,u2,k])[i,:])
                npAll_J_i_k = np.array(All_J_i_k)
                ind = np.unravel_index(np.argmax(npAll_J_i_k, axis=None), npAll_J_i_k.shape) # tuple
                choice[i][k] = ind
                npJ[i, k] = npAll_J_i_k[ind]
        

        elapsed_ = time.time() - _t
        print('computing solution took: %d seconds' %(elapsed_)) 

        for p in range(N):
            J0[p] = npJ[p, 0]

        ## controls for initial start point at different time
        # mu = [None for time in range(T)]
        # for t in range(T):
        #     mu[t] = choice[0][t]
            
        
        # save the results as some visible form, namely extract the best route for robot at "start"
        control = [None for time in range(T)]
        route = [None for time in range(T+1)]
        route[0] = tuple([self.StartPoint[0][0],self.StartPoint[0][1],self.StartPoint[1][0],self.StartPoint[1][1],0])
        prev_state = copy.deepcopy(route[0])
        for t in range(T):
            control[t] = choice[self.state_mapper[prev_state]][t] # a 2-element tuple
            current_state = self.next_state(prev_state, t, control[t][0], control[t][1])
            prev_state = current_state
            route[t+1] = current_state
        # save almost all the history
        # for all the points at every time step save: (1)full optimal path; (2) associated controls
        # controls[t][s][\tau]: from (t,s), controls[t][s][\tau] should apply till the end, as it's optimal
        # routes[t][s][\tau]: resulting path from controls[t][s][\tau]
        # Js[t[s] maximum cost-to-go can get at (t,s), by following the optimal policy
        controls =[[[None for time in range(T-timeStep)] for n in range(N)] for timeStep in range(T)]
        routes = [[[None for time in range(T-timeStep+1)] for n in range(N)] for timeStep in range(T)]
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
                    currentState = self.next_state(prevState, t, controls[timeStep][initialStateNumber][t][0], controls[timeStep][initialStateNumber][t][1])
                    prevState = currentState
                    routes[timeStep][initialStateNumber][t+1] = currentState
        return J0, control, route, controls, routes, Js

    def solve(self):
        _t = time.time()
        # TODO: Do we really need this loop? 
        for u1 in range(self.N_control):
            for u2 in range(self.N_control):
                self.P_u = [[]for time_range in range(self.T)]
                if u1 == 0 and u2 == 0:
                    self.state_mapper = {}
                    for k in range(self.T):
                        # (tuple([pos_x, pos_y, automaton_state]), increasing int for numbering, )
                        self.state_mapper.setdefault(tuple([self.StartPoint[0][0],self.StartPoint[0][1],self.StartPoint[1][0],self.StartPoint[1][1],0]), 0) # initial state
                        self.state_mapper.setdefault(tuple([self.EndPoint[0][0],self.EndPoint[0][1],self.EndPoint[0][0],self.EndPoint[0][1],1]), 1) # "successfully end rescue" state
                        self.state_mapper.setdefault(tuple([-1,-1,-1,-1,2]), 2) # "reach the fire" state
                        self.state_mapper.setdefault(tuple([self.Target[0][0],self.Target[0][1],self.Target[0][0],self.Target[0][1],3]), 3) # "reach the target 1" state
                        self.visited = set()
                        self.P_k = [[0 for col in range(self.N_state)] for row in range(self.N_state)]
                        self.dfs(tuple([self.StartPoint[0][0],self.StartPoint[0][1],self.StartPoint[1][0],self.StartPoint[1][1],0]),k)
                        # manually set absorbing states
                        self.P_k[1][1] = 1
                        # FOR SINGLE TARGET
                        # for i in range(len(self.P_k[2])):
                        #     self.P_k[2][i] = 0
                        # self.P_k[2][2] = 1
                        self.P_k[3][3] = 1 
                        N_state_r = len(self.state_mapper)
                        self.P_u[k] = [[0 for col in range(N_state_r)] for row in range(N_state_r)]
                        for i in range(N_state_r):
                            for j in range(N_state_r):
                                self.P_u[k][i][j] = self.P_k[i][j] # should have faster way to do this
                        self.P[u1][u2][k] = [[0 for col in range(N_state_r)] for row in range(N_state_r)]
                        for i in range(N_state_r):
                            for j in range(N_state_r):
                                try:
                                    self.P[u1][u2][k][i][j] = self.P_u[k][i][j] # should have faster way to do this
                                except:
                                    print(u1,u2,k,i,j)
                else: # we have u1 and u2
                    for k in range(self.T):
                        self.visited = set()
                        self.P_k = [[0 for col in range(self.N_state)] for row in range(self.N_state)]
                        self.generate_p_for_P_k(k, u1, u2) # traversely fill the P_k for other controls other than u=0
                        # manually set absorbing states
                        self.P_k[1][1] = 1
                        # FOR SINGLE TARGET
                        # for i in range(len(self.P_k[2])):
                        #     self.P_k[2][i] = 0
                        # self.P_k[2][2] = 1
                        self.P_k[3][3] = 1 
                        N_state_r = len(self.state_mapper)
                        self.P_u[k] = [[0 for col in range(N_state_r)] for row in range(N_state_r)]
                        for i in range(N_state_r):
                            for j in range(N_state_r):
                                self.P_u[k][i][j] = self.P_k[i][j] # should have faster way to do this
                        self.P[u1][u2][k] = [[0 for col in range(N_state_r)] for row in range(N_state_r)]
                        for i in range(N_state_r):
                            for j in range(N_state_r):
                                try:
                                    self.P[u1][u2][k][i][j] = self.P_u[k][i][j] # should have faster way to do this
                                except:
                                    print(u1,u2,k,i,j)

        elapsed_ = time.time() - _t
        print('Constructing P took: %d seconds' %(elapsed_)) 

        # PRAMS
        N_state_r_last = len(self.state_mapper)

        # constructing cost matrix g, terminal cost vector gT
        # gT
        self.gT = [0.0 for state in range(N_state_r_last)]
        # FOR SINGLE TARGET
        self.gT[1] = 100 # gain 100 for winning the game
        # self.gT[2] = 100.0 # gain 100 for winning the game
        self.gT[2] = 0 # 0 for reaching the fire
        self.g = [[[[[0 for j in range(N_state_r_last)] for i in range(N_state_r_last)]for time_range in range(self.T)]for control_range in range(self.N_control)] for control_range in range(self.N_control)]
        # for t in range(self.T):
            # FOR SINGLE TARGET sth has to be done
        
        # DP process and time counting
        J0, control, route, self.controls, self.routes, self.Js = self.markovDP( self.P, self.g, self.gT, self.T )

        # print optimal controls for the controls from starting point
        print('optimal controls are: ')
        print(control)

        print('printing payoffs for each state at t=0')
        print(J0)

        self.remove_duplicated_path_segs(route) # only do once
        for timeStep in range(self.T):
            for n in range(N_state_r_last):
                self.remove_duplicated_path_segs(self.routes[timeStep][n])
        # Visualize the optimal controls
        route_array = np.asarray(route)
        x1 = route_array[:,0]
        y1 = route_array[:,1]
        x2 = route_array[:,2]
        y2 = route_array[:,3]
        q = route_array[:,4]
        c = np.arange(0,len(x1))
        # self.path1 = np.vstack((x1,y1,c)).transpose()

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
        plt.plot(x1,y1,c='m')
        plt.plot(x2,y2,c='b')
        plt.grid(linestyle = '--')
        # plt.show()
        fig.savefig('dp_planning_g.jpeg')

        print('printing optimal DP path together')
        print(route)
        print('DP solution length: %d' %(len(route)))
        # TODO: how do we evaluate the double path??
        pe1 = path_evaluation.PathEvaluation(self.FireMap)
        DPscore1 = pe1.evaluate_path(route)
        print('DP solution probability for robot 1: %f' %(DPscore1) )

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
        endPath = tuple([self.EndPoint[0][0],self.EndPoint[0][1],self.EndPoint[0][0],self.EndPoint[0][1],1])
        while(path[-2] == endPath):
            del path[-1]
        return