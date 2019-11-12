import numpy as np
import matplotlib.pyplot as plt
import time
import problem_setting
import automaton_dp
import plot3d_fire
from maxProbAstar import MaxProbAstar
from path_evaluation import PathEvaluation
import pickle
import seaborn as sns
import imageio
import os

_SIZE = 20

# use 300 samples for fast computation

# load stuff with pickle
monteCarloFireMap = pickle.load(open('MCFMs300', "rb"))
monteCarloAverageFireMap = pickle.load(open('MCAFMs300', "rb"))

# problem setup
targets = [[9,16],[13,16],[2,5]]
x = 8
y = 16
p1 = problem_setting.ProblemSetting(target = [[x,y]],_stochastic_environment_flag=1, _setting_num=1)


aDP1 = automaton_dp.AutomatonDP(p1.M,p1.N,p1.T,p1.Map,monteCarloAverageFireMap, monteCarloFireMap, p1.StartPoint,p1.EndPoint,p1.Target,p1.Wall,p1.Fire,Q=4)

aDP1.solve()

# we make an observation using s th MC sample at t=T, at location of the solution at t, its 3*3 surrounding neighbors
s = 8
T = 11 
# actually at (xT,yT) at T, based on aDP1 trajectory and T to obtain (xT,yT)
xT = 5
yT = 8

# query at this moment
costsFromNow1, routesFromNow1, controlsFromNow1 = aDP1.query_optimal_solution(T,[xT,yT,-1])
# value function at (q,x,y)
# currentValueFunc1 = costsFromNow1[T]
# should use route from aDP1 and examine in aDP2's examples
aDP2.pe.evaluate_path_t(routesFromNow1, T)


currentRunMap = monteCarloFireMap[s][T] # at s th trial, T
locationArray = np.array([[xT+i,yT+j] for i in range(-1,2) for j in range(-1,2)])
rows=locationArray[:,0]
cols=locationArray[:,1]
currentRunMapArray = np.array(currentRunMap)
currentFireState = currentRunMapArray[rows,cols]

# solve the problem using only the samples consistent with the observation
aDP2 = automaton_dp.AutomatonDP(p1.M,p1.N,p1.T,p1.Map,monteCarloAverageFireMap, monteCarloFireMap, p1.StartPoint,p1.EndPoint,p1.Target,p1.Wall,p1.Fire,Q=4, _observeTime=T, _observeFireState=currentFireState, _observeX=xT,_observeY=yT)
aDP2.solve()

# query at this moment
costsFromNow2, routesFromNow2, controlsFromNow2 = aDP2.query_optimal_solution(T,[xT,yT,-1])
# value function at (q,x,y)
currentValueFunc2 = costsFromNow2[T]

# compare value function of two process at different times
# check if duplicated items in state mapper
keysInStateMapper=[(x,y) for (x,y,q) in aDP1.state_mapper.keys()]
keysInstateMapperSet=set(keysInStateMapper)
# plot
# prepare data
valueFunction1 = np.array(aDP1.Js[T])
valueFunctionMap1 = np.zeros((_SIZE,_SIZE))
states1 = np.array(list(aDP1.state_mapper.keys()))
for [x,y,q] in states1:
    valueFunctionMap1[x,y] = valueFunction1[aDP1.state_mapper.get((x,y,q))]

valueFunction2 = np.array(aDP2.Js[T])
valueFunctionMap2 = np.zeros((_SIZE,_SIZE))
states2 = np.array(list(aDP2.state_mapper.keys()))
for [x,y,q] in states2:
    valueFunctionMap2[x,y] = valueFunction2[aDP1.state_mapper.get((x,y,q))]

sns.set()
plt.figure(1)
plt.subplot(311)
transposedValueFunctionMap1 = np.transpose(valueFunctionMap1)
ax = sns.heatmap(transposedValueFunctionMap1, annot=True, fmt='.2f', vmin=0, vmax=100)
plt.ylim(reversed(plt.ylim()))

plt.subplot(312)
transposedValueFunctionMap2 = np.transpose(valueFunctionMap2)
ax = sns.heatmap(transposedValueFunctionMap2, annot=True, fmt='.2f', vmin=0, vmax=100)
plt.ylim(reversed(plt.ylim()))

plt.subplot(313)
transposedSubtractedMap = np.subtract(transposedValueFunctionMap1, transposedValueFunctionMap2)
ax = sns.heatmap(transposedSubtractedMap, annot=True, fmt='.2f', vmin=-2, vmax=2)
plt.ylim(reversed(plt.ylim()))

plt.show()

plt.figure(2)
transposedSubtractedMap = np.subtract(transposedValueFunctionMap2, transposedValueFunctionMap1)
ax = sns.heatmap(transposedSubtractedMap, annot=True, fmt='.2f', vmin=-2, vmax=2)
plt.ylim(reversed(plt.ylim()))
plt.show()

# anime plots
oldRoute = np.array([(x,y) for (x,y,q) in routesFromNow1])
newRoute = np.array([(x,y) for (x,y,q) in routesFromNow2])
images = []
#os.mkdir("figs")
for tt in range(7,50):
    fig = plt.figure(tt)
    # prepare data
    valueFunction1 = np.array(aDP1.Js[tt])
    valueFunctionMap1 = np.zeros((_SIZE,_SIZE))
    states1 = np.array(list(aDP1.state_mapper.keys()))
    for [x,y,q] in states1:
        valueFunctionMap1[x,y] = valueFunction1[aDP1.state_mapper.get((x,y,q))]
    valueFunction2 = np.array(aDP2.Js[tt])
    valueFunctionMap2 = np.zeros((_SIZE,_SIZE))
    states2 = np.array(list(aDP2.state_mapper.keys()))
    for [x,y,q] in states2:
        valueFunctionMap2[x,y] = valueFunction2[aDP1.state_mapper.get((x,y,q))]
    transposedSubtractedMap = np.subtract(np.transpose(valueFunctionMap2), np.transpose(valueFunctionMap1))
    ax = sns.heatmap(transposedSubtractedMap, annot=False,vmin=-2, vmax=2)
    plt.plot(oldRoute[:tt-6,0] + 0.5,oldRoute[:tt-6,1] + 0.5,c='k')
    plt.plot(newRoute[:tt-6,0] + 0.5,newRoute[:tt-6,1] + 0.5,c='b')
    # plt.grid(linestyle = '--')
    plt.ylim(reversed(plt.ylim()))
    fig.savefig('figs/plot'+ str(tt) +'.jpeg')
    plt.close()
    images.append(imageio.imread('figs/plot'+ str(tt) +'.jpeg'))
imageio.mimsave('wave_of_difference.gif', images)

# draw image for P difference
images = []
#os.mkdir("figs")
for tt in range(11,50):
    fig = plt.figure(tt)
    # prepare data
    P1 = np.array(aDP1.P[1][11])
    valueFunctionMap1 = np.zeros((_SIZE,_SIZE))
    states1 = np.array(list(aDP1.state_mapper.keys()))
    for [x,y,q] in states1:
        valueFunctionMap1[x,y] = P1[aDP1.state_mapper.get((x,y,q))][aDP1.next_state([x,y,q],11)]
    P2 = np.array(aDP4.P[1][11])
    valueFunctionMap2 = np.zeros((_SIZE,_SIZE))
    states2 = np.array(list(aDP2.state_mapper.keys()))
    for [x,y,q] in states2:
        valueFunctionMap2[x,y] = P2[aDP1.state_mapper.get((x,y,q))]
    transposedSubtractedMap = np.subtract(np.transpose(valueFunctionMap2), np.transpose(valueFunctionMap1))
    ax = sns.heatmap(transposedSubtractedMap, annot=False,vmin=-1, vmax=1)
    plt.plot(oldRoute[:tt-6,0] + 0.5,oldRoute[:tt-6,1] + 0.5,c='k')
    plt.plot(newRoute[:tt-6,0] + 0.5,newRoute[:tt-6,1] + 0.5,c='b')
    # plt.grid(linestyle = '--')
    plt.ylim(reversed(plt.ylim()))
    fig.savefig('figs/plot'+ str(tt) +'.jpeg')
    plt.close()
    images.append(imageio.imread('figs/plot'+ str(tt) +'.jpeg'))
imageio.mimsave('wave_of_difference_P.gif', images)