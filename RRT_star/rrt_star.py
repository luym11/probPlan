import sys
sys.path.insert(1,'../')

import random
import math
import copy
import numpy as np
import matplotlib.pyplot as plt
import time
import path_evaluation
import utils

show_animation = True
_INF = 49

class RRT():
    """
    Class for RRT Planning
    """

    def __init__(self, start, goal, wallList, fireList, fireMap, randArea=[0,19],
                 expandDis=0.5, goalSampleRate=20, maxIter=1000):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        wall/fireList:obstacle INITIAL Positions [[x,y,size],...]
        fireMap: all trials of Monte Carlo simulations of fire
        randArea: Ramdom Samping Area [min,max]
        expandDis: eta in the paper, a parameter for steer() 
        half_size: a parameter used for segmentation check while adding vertice to the tree, 
                it should always be half of the size of obstacle/grid

        """
        self.start = Node(start[0], start[1])
        self.end = Node(goal[0], goal[1])
        self.minrand = randArea[0]
        self.maxrand = randArea[1]
        self.expandDis = expandDis
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.wallList = wallList
        self.fireList = fireList
        self.obstacleList = wallList + fireList
        self.fireMap = fireMap
        self.half_size = 0.5
        self.pe = path_evaluation.PathEvaluation(self.fireMap)


    def Planning(self, animation=True):
        """
        Pathplanning

        animation: flag for animation on or off
        """
        self.nodeList = [self.start] # a list of node
        _t = time.time()
        for i in range(self.maxIter):
            # An screen indicator
            if (i+1) % 100 == 0:
                elapsed_ = time.time() - _t
                print('computing first %d iterantions took: %d seconds' %(i, elapsed_))
                _t = time.time()
                print('started %d iteration' % (i, ))

            rndNode = self.get_random_point()
            nind = self.GetBestListIndex(self.nodeList, rndNode) # changed to max prob

            newNode = self.steer(rndNode, nind)

            if self.__CollisionCheck(newNode, self.obstacleList): # in X_{free}
                newNode = self.confirm_parent(newNode) # confirm parent connectable
                if newNode.parent != -1: # whether connect to nind or not
                    self.nodeList.append(newNode)
                    self.nodeList[newNode.parent].childList.append(len(self.nodeList)-1)
                    nearinds = self.find_near_nodes(newNode)
                    self.rewire(newNode, nearinds)
            if animation:
                self.DrawGraph([rndNode.x,rndNode.y])

        # generate path
        lastIndex, bestCost, bestProb = self.get_best_last_index()
        if lastIndex is None:
            return None
        path = self.gen_final_course(lastIndex, bestCost, bestProb)
        print("Optimal path waypoints")
        for path_segment in path:
            print(path_segment)
        return path

    def compute_probability_not_being_burnt(self, p1, p2):
        # p1 to be the old node on the tree, p2 to be the newly sampled node
        # we will compute for the safe probability of a tree path from
        # start point to p1, then adding an edge from p1->p2
        pathToP2 = self.gen_tree_path(p1,p2)
        l = len(pathToP2)-1
        # followings are path points only with two coordinates rather than Node
        discretePathSegList = [utils.generate_discrete_path_from_points(pathToP2[i],pathToP2[i+1]) for i in range(l)]
        discretePath = []
        for segPath in discretePathSegList:
            del segPath[-1]
            discretePath.extend(segPath)
        score = self.pe.evaluate_path(discretePath)
        return score

    def confirm_parent(self, newNode):
        # Spits out an updated newNode, with proper parent
        # binded
        if self.collision_check_full(newNode, self.nodeList[newNode.parent]):            
            pass
        else:
            newNode.parent = -1
        return newNode

    def collision_check_full(self, newNode, treeNode):
        dx = newNode.x - treeNode.x
        dy = newNode.y - treeNode.y
        d = math.sqrt(dx ** 2 + dy ** 2)
        theta = math.atan2(dy, dx)
        if self.check_collision_extend(treeNode, theta, d):
            return True
        return False

    def steer(self, rndNode, nind):
        bestNode = self.nodeList[nind]
        currentDistance = math.sqrt( (rndNode.y - bestNode.y) ** 2 +  (rndNode.x - bestNode.x) ** 2)
        currentProb = self.compute_probability_not_being_burnt(bestNode, rndNode)
        rndNode.cost = bestNode.cost + currentDistance
        rndNode.safeProb = currentProb
        # if currentDistance <= self.expandDis:
        #     rndNode.cost = nearestNode.cost + currentDistance
        # else:
        #     rndNode.x = nearestNode.x + self.expandDis * math.cos(theta)
        #     rndNode.y = nearestNode.y + self.expandDis * math.sin(theta)
        #     rndNode.cost = nearestNode.cost + self.expandDis
        rndNode.parent = nind # exact steering
        return rndNode

    def get_random_point(self):
        
        if random.randint(0, 100) > self.goalSampleRate:
            rnd = [random.uniform(self.minrand, self.maxrand),
                   random.uniform(self.minrand, self.maxrand)]
        else:  # goal point sampling, acutally a test to see if near enough to the point, and also specifying this point
            rnd = [self.end.x, self.end.y]

        return Node(rnd[0],rnd[1])

    def get_best_last_index(self):
        # compute distance to goal in all nodes from nodeList
        disglist = [self.calc_dist_to_goal(
            node.x, node.y) for node in self.nodeList]
        goalinds = [disglist.index(i) for i in disglist if i <= self.expandDis]
        problist = [node.safeProb for node in self.nodeList]
        #  print(goalinds)

        if len(goalinds) == 0:
            return None, None, None

        maxProbInd = goalinds[np.argmax([problist[i] for i in goalinds if self.nodeList[i].parent is not -1])]
        bestCost = disglist[maxProbInd] + self.nodeList[maxProbInd].cost
        bestProb = problist[maxProbInd]

        return maxProbInd, bestCost, bestProb

    def gen_final_course(self, goalind, bestCost, bestProb):
        path = [[self.end.x, self.end.y, bestCost, bestProb]]
        while(not self.nodeList[goalind].parent is None):
            node = self.nodeList[goalind]
            path.append([node.x, node.y, node.cost, node.safeProb])
            goalind = node.parent
        path.append([self.start.x, self.start.y, 0, 1])
        return path.reverse()

    def gen_tree_path(self, p, newP):
        # This will return a tree path from start point to point newP, via tree edges from start 
        # point to p, as they're already on the tree
        path = []
        path.append(p)
        if p.parent is None:
            return path
        ind = p.parent
        while(not self.nodeList[ind].parent is None):
            node = self.nodeList[ind]
            path.append(node)
            ind = node.parent
        path.append(self.start)
        path.reverse()
        path.append(newP)
        return path

    def calc_dist_to_goal(self, x, y):
        return np.linalg.norm([x - self.end.x, y - self.end.y])

    def find_near_nodes(self, newNode):
        nnode = len(self.nodeList)
        # r is set here!! Will need to look more carefully
        r = 50 * math.sqrt( math.log(nnode+1) / (nnode+1) ) # computed from proof, c.a. 8.5 for n=1 and 0.74 for n=3000
        # r = self.expandDis
        dlist = [(node.x - newNode.x) ** 2 +
                 (node.y - newNode.y) ** 2 for node in self.nodeList]
        nearinds = [dlist.index(dist) for dist in dlist if dist <= r ** 2]
        return nearinds

    def rewire(self, newNode, nearinds):
        nnode = len(self.nodeList) # WHAT A SIMPLE WAY... 
        for i in nearinds:
            nearNode = self.nodeList[i]

            dx = newNode.x - nearNode.x
            dy = newNode.y - nearNode.y
            d = math.sqrt(dx ** 2 + dy ** 2)
            currentProb = self.compute_probability_not_being_burnt(newNode, nearNode)

            scost = newNode.cost + d
            sprob = currentProb

            if nearNode.safeProb < sprob:
                theta = math.atan2(dy, dx)
                if self.check_collision_extend(nearNode, theta, d):
                    try:
                        self.nodeList[nearNode.parent].childList.remove(i)
                        nearNode.parent = nnode - 1
                        nearNode.cost = scost
                        nearNode.safeProb = sprob
                        self.nodeList[nnode - 1].childList.append(i)
                    except:
                        print('nearNode is isolate, has no parent')
                    

    def check_collision_extend(self, nearNode, theta, d):
        # actually this is a segment check
        tmpNode = copy.deepcopy(nearNode)

        for _ in range(int(d / self.half_size)):
            tmpNode.x += self.half_size * math.cos(theta)
            tmpNode.y += self.half_size * math.sin(theta)
            if not self.__CollisionCheck(tmpNode, self.obstacleList):
                return False

        return True

    def DrawGraph(self, rnd=None):
        """
        Draw Graph
        """
        plt.clf()
        plt.axis([0, 19, 0, 19])
        plt.axis("equal")
        ax = plt.gca()
        major_ticks=np.arange(0,22,5)
        minor_ticks=np.arange(0,22,1)
        ax.set_xticks(major_ticks)
        ax.set_xticks(minor_ticks, minor=True)
        ax.set_yticks(major_ticks)
        ax.set_yticks(minor_ticks, minor=True)
        ax.grid(which='both')
        if rnd is not None:
            plt.plot(rnd[0], rnd[1], "^k")
        for node in self.nodeList:
            if node.parent is not None and node.cost < _INF:
                plt.plot([node.x, self.nodeList[node.parent].x], [node.y, self.nodeList[node.parent].y], "-g")

        # Can't do more than drawing static initial fire. As sampling process has no relation with
        # evolving fire
        for (ox, oy, size) in self.fireList:
            plt.plot(ox, oy, "or", ms=15 * size)

        for (ox, oy, size) in self.wallList:
            plt.plot(ox, oy, "sk", ms=15*size)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.grid(True)
        plt.pause(0.02)

    def GetBestListIndex(self, nodeList, rndNode):
        # this returns the safest path from start point to rnd, within a radius to connect
        nearinds = self.find_near_nodes(rndNode)
        plist = [self.compute_probability_not_being_burnt(node, rndNode) for node in nodeList if nodeList.index(node) in nearinds]
        maxind = plist.index(max(plist))
        return maxind

    def __CollisionCheck(self, node, obstacleList):
        # not inside an obstacle
        for (ox, oy, size) in obstacleList:
            dx = ox - node.x
            dy = oy - node.y
            d = dx * dx + dy * dy           
            if d <= size ** 2:
                return False  # collision

        return True  # safe


    def deleteNode(self, nodeIndex):
        currentNode = self.nodeList[nodeIndex]
        try:
            parentNode = self.nodeList[currentNode.parent]
        except:
            print('currentNode.parent is ')
            print(currentNode.parent)
        # delete its existance at its parent's childList
        try:
            parentNode.childList.remove(nodeIndex)
        except:
            print('Some unexpected error')
            print(nodeIndex)
            print(self.nodeList[nodeIndex].x, self.nodeList[nodeIndex].y)
            print(self.nodeList[nodeIndex].cost)
            print(self.find_near_nodes(self.nodeList[nodeIndex]))
        # Do this recursively for all the elements in its childList
        for childIndex in currentNode.childList:
            self.deleteNode(childIndex)
            print("Actually deleted a child")
            # result: NO subtree is deleted, only A node at once
        # "delete" itself
        # self.nodeList[nodeIndex].childList.clear()
        self.nodeList[nodeIndex].parent = None
        self.nodeList[nodeIndex].cost = _INF


    def checkContaminationOfSegment(self, node, parentNode, currentCost):
        '''
        Will check the route from node to node.parent if it's within the range of fire
        at time when node is checked. FireMap at this time will be used for this check, 
        cost of the intermiediate waypoints will be the cost at the node
        Important Param: 30
        '''
        currentFireMap = self.fireMap[math.ceil(currentCost)]
        currentNode = node
        # obtained from line 184, r=50*sqrt(ln(2)/2)=30
        # divide into 30 points and check everyone
        parentNodePoint = np.asarray([parentNode.x, parentNode.y])
        currentNodePoint = np.asarray([currentNode.x, currentNode.y])
        dist_vec = currentNodePoint - parentNodePoint
        steps = np.linspace(0, 1, 30)
        xs = parentNodePoint[0] + steps*dist_vec[0]
        ys = parentNodePoint[1] + steps*dist_vec[1]
        for i in range(30):
            x_cor = int(round(xs[i]))
            y_cor = int(round(ys[i]))
            if currentFireMap[x_cor][y_cor] > 0:
                # print("line touched")
                return False # contaminated
        return True # safe

class Node():
    """
    RRT Node
    """
    def __init__(self, x, y, cost = 0.0, safeProb = 1):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent = None
        self.childList = []
        self.safeProb = safeProb 


def main():
    print("Start rrt planning")

    # ====Search Path with RRT====
    wallList = [
        (5, 5, 1),
        (3, 6, 2),
        (3, 8, 2),
        (3, 10, 2),
        (7, 5, 2),
        (9, 5, 2)
    ]  # [x,y,size(radius)]

    # Set Initial parameters
    rrt = RRT(start=[0, 0], goal=[5, 10],
              randArea=[-2, 15], wallList=obstacleList)
    path = rrt.Planning(animation=show_animation)

    # Draw final path
    if show_animation:
        rrt.DrawGraph()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.grid(True)
        plt.pause(0.01)  # Need for Mac
        plt.show()


if __name__ == '__main__':
    main()
