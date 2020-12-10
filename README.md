# Safe path planning in dynamic uncertain environments (probPlan)

## Problem
We are to solve a path planning problem (where the robot needs to visit a set of targets) in an environment with dynamically evolving uncertain hazards. 
Our goal is to reach the targets as soon as possible with a safest path to prevent the target or the robot being affected by the hazards.  

## Solution
1. Offline solution: we designed an approximated dynamic programming algorithm that plans the safest path regarding to the probabilistically evolving hazards. 
2. Online solution: to address observations of the robot during execution of the offline plan, we also designed an online algorithm which reactes to the observations and refines the offline plan. 

We illustrate our solution in the following picture. 
**(a)** An offline safe plan is generated while taking the dynamics of the hazards into account. 
**(b)** While executing the offline plan, the robot observes its surrounding environment and **(c)** obtains the refined safe path for the mission. 
![probPlan](/probPlan.png)

## Related works
- [Safe mission planning under dynamical uncertainties (ICRA 2020)](https://arxiv.org/abs/2003.02913/) 

[ICRA 2020 talk for this paper](https://www.dropbox.com/s/4f1jqbmpypaj72z/icra202020.mp4?dl=0/)

- Real-time safe path planning via incorporating local observations of dynamic uncertainties (in prep.)

- Multi-robot task allocation for safe planning under dynamic uncertainties (co-advised master's thesis [in prep.])
