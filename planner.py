import numpy as np
from scipy.sparse.csgraph import shortest_path
from scipy.sparse.csgraph import breadth_first_tree
from scipy.sparse.csgraph import breadth_first_order
from numba import jit
import math

from kinematics import *

@jit(nopython=True)
def cell_collision(cell,robot_margin,cell_size,obstacles):
    i=cell[0]
    j=cell[1]
    a=i*cell_size
    b=j*cell_size
    o_size=len(obstacles)
    # print(f"i {i}")
    # print(f"j {j}")
    # print(f"cell_size {cell_size}")
    # print(f"a {a}")
    # print(f"b {b}")

    for i in range(o_size):
        o=obstacles[i]
        c1=min([o[0],o[2]])-robot_margin
        c2=max([o[0],o[2]])+robot_margin
        d1=min([o[1],o[3]])-robot_margin
        d2=max([o[1],o[3]])+robot_margin
        
        if(a>c2 or c1>(a+cell_size)):
            continue
        elif((b+cell_size<d1) or d2<b):
            continue
        else:
            return True
    return False


def build_matrix(m,obstacles,robot_margin,cell_size):
    grid_size=len(m)
    grid_size=int(math.sqrt(grid_size))
    print(f"grid size... {grid_size}")
    print(f"cell size... {cell_size}")
    env=np.ones([grid_size,grid_size],dtype=np.bool)
    for i in range(grid_size):
        for j in range(grid_size):
            cell=np.array([i,j],dtype=np.int)
            collision=cell_collision(cell,robot_margin,cell_size,obstacles)
            if(collision):
                env[i,j]=False
    for i in range(grid_size):
        for j in range(grid_size):
            if(not env[i,j]):
                continue
            center=i*grid_size+j
            if(j>0):
                left=i*grid_size+j-1
                if(env[i,j-1]):
                    m[center,left]=1
                    m[left,center]=1
            if(j<grid_size-1):
                right=i*grid_size+j+1
                if(env[i,j+1]):
                    m[center,right]=1
                    m[right,center]=1
            if(i>0):
                bottom=(i-1)*grid_size+j
                if(env[i-1,j]):
                    m[center,bottom]=1
                    m[bottom,center]=1
            if(i<grid_size-1):
                up=(i+1)*grid_size+j
                if(env[i+1,j]):
                    m[center,up]=1
                    m[up,center]=1
            if(i>0 and j>0):
                bottomleft=(i-1)*grid_size+j-1
                if(env[i-1,j-1]):
                    m[center,bottomleft]=1
                    m[bottomleft,center]=1
            if(i>0 and j+1<grid_size):
                upleft=(i-1)*grid_size+j+1
                if(env[i-1,j+1]):
                    m[center,upleft]=1
                    m[upleft,center]=1
            if(i+1<grid_size and j+1 <grid_size):
                upright=(i+1)*grid_size+j+1
                if(env[i+1,j+1]):
                    m[center,upright]=1
                    m[upright,center]=1
            if(i>0 and j+1<grid_size):
                bottomright=(i-1)*grid_size+j+1
                if(env[i-1,j+1]):
                    m[center,bottomright]=1
                    m[bottomright,center]=1
                
    print(m.shape)                
    return m


class Planner:
    def __init__(self,obstacles,robot_margin,grid_length=10,grid_size=100):
        self.grid_size=grid_size
        self.grid_length=grid_length
        self.m=np.zeros((grid_size*grid_size,grid_size*grid_size),dtype=int)
        self.cell_size=grid_length/grid_size
        self.obstacles=obstacles
        self.robot_margin=robot_margin
    def build_graph(self,):
        del self.m
        grid_size=self.grid_size
        m=np.zeros((grid_size**2, grid_size**2),dtype=np.int8)
        self.m=build_matrix(m,self.obstacles,self.robot_margin,self.cell_size)
        # print(self.m)
        # np.savetxt('matrix.txt', self.m)
    def path(self,start,end):
        sx,sy=start
        ex,ey=end
        scx=math.floor(sx/self.cell_size)
        scy=math.floor(sy/self.cell_size)
        s_encoding=scx*self.grid_size+scy
        ecx=math.floor(ex/self.cell_size)
        ecy=math.floor(ey/self.cell_size)
        e_encoding=ecx*self.grid_size+ecy
        D,Pr=breadth_first_order(self.m,s_encoding,directed=False)
        p=[]
        if(Pr[e_encoding]<0):
            print('no path exists')
            return -1
        p.append([ecx,ecy])
        encoding=e_encoding
        while(Pr[encoding]>=0):
            predecessor=Pr[encoding]
            x=predecessor//self.grid_size
            y=predecessor%self.grid_size
            p=[[x,y]]+p
            encoding=predecessor
        return p
   
        
    




#
#M = np.array([[0, 1, 1, 0, 0, 1],
#              [1, 0, 1, 1, 0, 0],
#              [1, 1, 0, 1, 0, 1],
#              [0, 1, 1, 0, 1, 0],
#              [0, 0, 0, 1, 0, 1],
#              [1, 0, 1, 0, 1, 0]],dtype=np.bool)
#
#M=np.ones((10000,10000),dtype=int)
#
#
#D, Pr = shortest_path(M, directed=False, method='D', return_predecessors=True)
#
#def get_path(Pr, i, j):
#    path = [j]
#    k = j
#    while Pr[i, k] != -9999:
#        path.append(Pr[i, k])
#        k = Pr[i, k]
#    return path[::-1]
#
#print(get_path(Pr,1,2))
#
#
#Tcsr = breadth_first_tree(M, 0, directed=False)
#D,Pr=breadth_first_order(m,0,directed=False)


if __name__ == "__main__":
    # check cell collision function
    
    # #test case 1
    # cell=np.array([1,1],dtype=int)
    # obstacles=np.array([[20,20,4.5,6.2],[1.8,2,4.5,6.2]],dtype=np.float)
    # print(cell_collision(cell,0,1,obstacles))
    # #test case 2
    # cell=np.array([1,1],dtype=int)
    # obstacles=np.array([[20,20,4.5,6.2],[1.1,1.1,1.5,1.5]],dtype=np.float)
    # print(cell_collision(cell,0,1,obstacles))
    
    # #check build_matrix function
    # grid_size=100
    # m=np.zeros((grid_size*grid_size,grid_size*grid_size),dtype=np.int8)
    # obstacles=np.array([[20,20,4.5,6.2],[1.1,1.1,1.5,1.5]],dtype=np.float)
    # m=build_matrix(m,obstacles,0,1)
    
    
    #test shortest path
    obstacles1=np.array([[20,20,4.5,6.2],[2.6,2.6,4,4]],dtype=np.float)
    #obstacles2=
    robot_margin=0.1
    grid_length=10
    grid_size=100
    # obs=np.array([[20,20,4.5,6.2],[2.6,2.6,4,4]],dtype=np.float)
    # obs = [[[.5, 0, 0], [1, .5, .5]], [[-1, 0, 0], [-.5, .5, .5]]]
    obs = np.array([[0.5, 1, 1.5, 2], [.5,8,1.5,9]], dtype=np.float)
    planner=Planner(obs,robot_margin,grid_length,grid_size)
    planner.build_graph()
    start = [0, 0]
    end = [5.5, 5.5]
    path=planner.path(start,end)
    if path != -1:
        with open("cell_path.txt", "w") as f:
            for cell in path:
                f.write(f"{cell}\n")
        k = Kinematics(0.1, 0.124, 0.028)
        x_thetas = k.grid_path_to_cspace(path, start, end)
        with open("x_theta_path.txt", "w") as f:
            for cell in x_thetas:
                f.write(f"{cell}\n")
        wheel_rotations = k.cspace_to_wheel_rotations()
        with open("path.txt", "w") as f:
            for rot in wheel_rotations:
                left, right = rot
                f.write(f"{right} {left}\n")
    
    
    
    


