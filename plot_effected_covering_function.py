import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import time
import problem_setting
import automaton_dp
import plot3d_fire
from maxProbAstar import MaxProbAstar
from path_evaluation import PathEvaluation
import pickle

# # save stuff as pickle
with open('MCFMs3000', 'wb') as fp:
    pickle.dump(monteCarloFireMap, fp)
with open('MCAFMs3000', 'wb') as fp:
    pickle.dump(monteCarloAverageFireMap, fp)

# load stuff with pickle
monteCarloFireMap = pickle.load(open('MCFMs', "rb"))
monteCarloAverageFireMap = pickle.load(open('MCAFMs', "rb"))

# firemap = monteCarloFireMap[s][10]
# # firemap=monteCarloAverageFireMap[10]
# sns.set()
# intersection_matrix = np.transpose(firemap)
# ax = sns.heatmap(intersection_matrix, annot=True, fmt='.2g')
# plt.show()

# at a certain time t, at locations xs and ys, we know part of the fire states directly. 
# use this information to get effected monteCarlAverageFireMap from t on

# let's do a sample
s = 8
T=10
x = 6
y = 14
currentRunMap = monteCarloFireMap[s][T] # at s th trial, T
locationArray = np.array([[x+i,y+j] for i in range(-1,2) for j in range(-1,2)])
rows=locationArray[:,0]
cols=locationArray[:,1]
currentRunMapArray = np.array(currentRunMap)
currentFireState = currentRunMapArray[rows,cols]

# compute covering function but only include samples consistent with our observation
# now we use 30000
_GenerateHorizon = 30000
firemap_sum = np.zeros([50,20,20], dtype=np.float32)
sampleCounter = 0
for h in range(_GenerateHorizon): # for all samples
    mcfm = np.array(monteCarloFireMap[h][T])
    if np.array_equal(currentFireState, mcfm[rows,cols]):
        sampleCounter += 1
        print('started {} MC simulation'.format(sampleCounter))
        new_firemap = monteCarloFireMap[h]
        for k in range(T, len(firemap_sum)):
            firemap_sum[k] = np.add(firemap_sum[k], new_firemap[k])
print('sampleConter is {}'.format(sampleCounter))
for k in range(T, len(firemap_sum)):
    firemap_sum[k] = np.divide(firemap_sum[k], sampleCounter)
# we obtained the effected covering function (only those from t are correct)

# plotting
sns.set()
t=10

plt.figure(1)
plt.subplot(311)
intersection_matrix0 = np.transpose(monteCarloFireMap[s][t])
ax = sns.heatmap(intersection_matrix0, annot=True, fmt='.2f', vmin=0, vmax=1)

plt.subplot(312)
intersection_matrix1 = np.transpose(monteCarloAverageFireMap[t])
ax = sns.heatmap(intersection_matrix1, annot=True, fmt='.2f', vmin=0, vmax=1)

plt.subplot(313)
intersection_matrix2 = np.transpose(firemap_sum[t])
ax = sns.heatmap(intersection_matrix2, annot=True, fmt='.2f', vmin=0, vmax=1)
plt.show()

plt.figure(2)
tt = 13
subtractedMap = np.subtract(monteCarloAverageFireMap[tt], firemap_sum[tt])
subtractedMatrix = np.transpose(subtractedMap)
ax = sns.heatmap(subtractedMatrix, annot=True, fmt='.2f', vmin=0, vmax=1)
plt.show()