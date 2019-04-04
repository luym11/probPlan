import matplotlib.pyplot as plt
import problem_setting
import numpy as np
import imageio

def ani_plot(problem, path):
    images = []
    for t in range(len(path)):
        route_array = np.array(path[0:t+1])
        x = route_array[:,0]
        y = route_array[:,1]

        StartPoint_array = np.asarray(problem.StartPoint)
        EndPoint_array = np.asarray(problem.EndPoint)
        Target_array = np.asarray(problem.Target)
        Wall_array = np.asarray(problem.Wall)
        Fire_array = np.asarray(problem.Fire)

        fig = plt.figure()
        ax = fig.gca()
        # fire plotting starts
        min_val, max_val = 0, 20
        intersection_matrix = np.transpose(problem.FireMap[t])
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
        fig.savefig('plot'+ str(t) +'.jpeg')
        images.append(imageio.imread('plot'+ str(t) +'.jpeg'))
    imageio.mimsave('plot_movie.gif', images)