# Code for stochastic path planning problem

## Problem
We are to solve a path planning problem (currently from a point to another point) in an environment with stochastic hazards evloving. 
Our goal is to reach the target as soon as possible with a safest path to prevent the target or ourself being hurt by the hazards.  

## Approaches

### DP satisfaction
Set up an automaton specification satisfication problem, solve it with dynamic programming. The optimal path will be evaluated using Monte Carlo evaluation function at the last part of the solver. 

### Graph search using evaluation function
Simply apply any graph search algorithm but use self-developed Monte Carlo evaluation function for the cost of the path. Note that this evaluation function needs to be 
- addable
- transformed to be something for minimization with a lower bound

## Components

### Problem setting
Basic setting class of the problem. 
Use as `p1 = problem_setting.ProblemSetting(args)`

### Monte Carlo generator
Generate Monte Carlo simulated fire process using function in problem_setting, and save them into 2 folders. 

### Path evaluation
Evaluate the safety probability of a path from counting Monte Carlo simulation results. Called as
```python
pe = PathEvaluation()
safeProb = pe.evaluate_path(path)
```
where `path` is a list of [x,y] coordinates. 

### Plot in 3d
This script prints the trajectory in 3d using time as a dimension. Feed in path looks like [[x,y,cost],...] for 3d plot. 

### Analytic approximation of hazards

We approximate the hazards propagation by assuming a node being safe at $t-1$ and one of its neighbors contaminating it at $t$ to be independent, i.e., 

$$p_{x,t} = p_{x,t-1} \prod_{n \in N(x)}((p_{n,t-1})+ (1-p_{n,t-1})*(1-p_f))$$

This class is inherited from `ProblemSetting()`, results will be:

- `monteCarloAverageFireMap` as empirical Monte Carlo averaged fire map

- `analyticAverageFireMap` as approximated fire map



## Execution code

### dp_monte.py

Only solve the problem using DP method. 

### maxProbAstar.py

Solve the problem using A*(h=0) with path evaluation as "cost"

## Related papers
- [Safe mission planning under dynamical uncertainties (ICRA 2020)](https://arxiv.org/abs/2003.02913/)
[ICRA 2020 talk for this paper](https://www.dropbox.com/s/4f1jqbmpypaj72z/icra202020.mp4?dl=0/)

- Real-time safe path planning via incorporating local observations of dynamic uncertainties (in prep.)
