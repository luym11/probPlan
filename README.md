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

## Repository structure
The main functioning parts of the code is implemented in an OOP (Object-orienated programming) way. That means, for each functioning part there is implemented as a Python class, and usually associated with an example code in the class file to show how to use it. Here I will also list the important files and where they are used.
- RRT_star
  - rrt_star.py: a Python class where the RRT* algorithm is implemented.
  - test_rrt_star.py: an example code to show how to use this RRT* class.
- astar
  - __init__.py: an implementation of the standard A* algorithm. Note that this class has 4 abstract methods, where the user needs to implement for his/her needs. My implementation can be found in the file maxProbAstar.py, where I implemented those 4 abstract methods specially for the safe planning problem.
- maxProbAstar.py: my inheretance of the astar class for the planning problem under dynamic uncertainties.
- matlab_code: this folder contains some matlab codes when I first started to program the planning problem. Can be used for an initial understanding of the environment model.
- utils: this folder contains some modulized tools while developing. They are either used during plotting the result, or some small functions that can be isolated from the code logic. (eg., generate_discrete_path.py is for discretizing a continuous path to a grid map.)
- UpdateEnv.py: the functioning part for the online replanning. The two main methods in this class is observe_and_construct which takes the observation of the environment and update the environment model, and dijk_to_edge which uses Dijkstra algorithm to replan the robot to the replanning range.
- testDijkToEdge.py: an updated version for UpdateEnv.py, where the latest ideas of online replanning are implemented. 
- analytic_fire.py: this class implements an (pessimistic) apprximation for the hazard process which appeared in the Rakic thesis.
- anime_plots.py: generates a cartoon of the robot following a path while the environment is changing according to its model. 
- automaton_dp.py: this class implemented the solution of single agent planning problem using dynamic programming (the algorithm in ICRA 2020 paper). Main methods in this class are:
  - markovDP: build the problem automaton and also related transfer probabilities.
  - solve: using dynamic programming to solve for the optimal path and output.
- dp_monte.py: an example to use the above automaton_dp class
- generateMonteCarloFireMap.py: for generating and saving a large amount of Monte Carlo simulations of the environment. Save and load functions use the Python package "pickle", which utilizes binary flows to read and write data.
- path_evaluation.py: the main functioning part for using Monte Carlo samples to evaluate the safety of a path. Multiple methods are implemented there, including evaluating a whole path / path segments. 
- problem_setting.py: setting up the environment dynamics and parameters.


## Related works
- [Safe mission planning under dynamical uncertainties (ICRA 2020)](https://arxiv.org/abs/2003.02913/) 

[ICRA 2020 talk for this paper](https://www.dropbox.com/s/4f1jqbmpypaj72z/icra202020.mp4?dl=0/)

- Real-time safe path planning via incorporating local observations of dynamic uncertainties (in prep.)

- Multi-robot task allocation for safe planning under dynamic uncertainties (co-advised master's thesis submitted ([arXiv](https://arxiv.org/abs/2103.01840))
