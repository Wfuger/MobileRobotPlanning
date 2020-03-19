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
    l1_x=a
    l1_y=b+cell_size
    r1_x=a+cell_size
    r1_y=b
    for i in range(o_size):
        o=obstacles[i]
        c1=min([o[0],o[2]])-robot_margin
        c2=max([o[0],o[2]])+robot_margin
        d1=min([o[1],o[3]])-robot_margin
        d2=max([o[1],o[3]])+robot_margin
        l2_x=c1
        l2_y=d2
        r2_x=c2
        r2_y=d1
        
        if(l1_x>r2_x or l2_x>r1_x):
            continue
        elif(l1_y<r2_y or l2_y<r1_y):
            continue
        else:
            return True
    return False








def build_matrix(m,obstacles,robot_margin,cell_size,grid_size,width_size):
    print(f"grid size... {grid_size}")
    print(f"cell size... {cell_size}")
    env=np.ones([width_size,grid_size],dtype=np.bool)
    for i in range(width_size):  
        for j in range(grid_size):
            cell=np.array([i,j],dtype=np.int)
            collision=cell_collision(cell,robot_margin,cell_size,obstacles)
            if(collision):
                env[i,j]=False
    for i in range(width_size):
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
            if(i<width_size-1):
                up=(i+1)*grid_size+j
                if(env[i+1,j]):
                    m[center,up]=1
                    m[up,center]=1
            if(i>0 and j>0):
                bottomleft=(i-1)*grid_size+j-1
                if(env[i-1,j-1]):
                    m[center,bottomleft]=1.4
                    m[bottomleft,center]=1.4
            if(j>0 and i+1<width_size):
                upleft=(i+1)*grid_size+j-1
                if(env[i+1,j-1]):
                    m[center,upleft]=1.4
                    m[upleft,center]=1.4
            if(i+1<width_size and j+1 <grid_size):
                upright=(i+1)*grid_size+j+1
                if(env[i+1,j+1]):
                    m[center,upright]=1.4
                    m[upright,center]=1.4
            if(i>0 and j+1<grid_size):
                bottomright=(i-1)*grid_size+j+1
                if(env[i-1,j+1]):
                    m[center,bottomright]=1.4
                    m[bottomright,center]=1.4
                
    return m

def get_path(Pr, i, j):
    path = [j]
    k = j
    while Pr[i, k] != -9999:
        path.append(Pr[i, k])
        k = Pr[i, k]
    return path[::-1]
#
#print(get_path(Pr,0,5555))
#

def points_order(start,points):
    s=np.array(start)
    p=np.array(points)
    distance=np.linalg.norm(p-s,axis=1)
    order=np.argsort(distance)
    new=[]
    for i in order:
        new.append(points[i])
    return new

class Planner:
    def __init__(self,obstacles,robot_margin,grid_length=10,grid_width=10,grid_size=100):
        self.grid_width=grid_width
        
        self.grid_size=grid_size
        self.grid_length=grid_length
        self.cell_size=grid_length/grid_size
        self.width_size=math.floor(grid_width/self.cell_size)
        self.m=None
        self.obstacles=obstacles
        self.robot_margin=robot_margin
    def build_graph(self,):
        del self.m
        m=np.zeros((self.grid_size*self.width_size, self.grid_size*self.width_size),dtype=np.float16)
        self.m=build_matrix(m,self.obstacles,self.robot_margin,self.cell_size,self.grid_size,self.width_size)
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
    def multiple_path(self,start,points):
        points=points_order(start,points)
        paths=[]
        for i in range(len(points)):
            end=points[i]
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
                print('no path exists for ',i)
                paths.append(-1)
                return paths
            p.append([ecx,ecy])
            encoding=e_encoding
            while(Pr[encoding]>=0):
                predecessor=Pr[encoding]
                x=predecessor//self.grid_size
                y=predecessor%self.grid_size
                p=[[x,y]]+p
                encoding=predecessor
            paths.append(p)
            start=end
        return paths
    def multiple_dijk(self,start,points):
        points=points_order(start,points)
        paths=[]
        D,Pr=shortest_path(self.m,directed=False, method='D', return_predecessors=True)
        for i in range(len(points)):
            end=points[i]
            sx,sy=start
            ex,ey=end
            scx=math.floor(sx/self.cell_size)
            scy=math.floor(sy/self.cell_size)
            s_encoding=scx*self.grid_size+scy
            ecx=math.floor(ex/self.cell_size)
            ecy=math.floor(ey/self.cell_size)
            e_encoding=ecx*self.grid_size+ecy

            if(Pr[s_encoding,e_encoding]==-9999):
                print('no path exists for ',i)
                paths.append(-1)
                return paths
            encoding_path=get_path(Pr,s_encoding,e_encoding)
            p=[]
            for i in range(len(encoding_path)):
                encoding=encoding_path[i]
                x=encoding//self.grid_size
                y=encoding%self.grid_size
                p.append([x,y])
            paths.append(p)
            start=end
        return paths
        
   
        
    




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
#from time import time
#start=time()
#D, Pr = shortest_path(m, directed=False, method='D', return_predecessors=True)
#print(time()-start)
##
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
    # obstacles1=np.array([[20,20,4.5,6.2],[2.6,2.6,4,4]],dtype=np.float)
    #obstacles2=
    # robot_margin=0.1
    # grid_length=10
    # grid_size=100
    # obs=np.array([[20,20,4.5,6.2],[2.6,2.6,4,4]],dtype=np.float)
    # obs = [[[.5, 0, 0], [1, .5, .5]], [[-1, 0, 0], [-.5, .5, .5]]]
    # obs = np.array([[0.5, 1, 1.5, 2], [.5, 8, 1.5, 9]], dtype=np.float)
    # planner=Planner(obs,robot_margin,grid_length,grid_size)
    # planner.build_graph()
    # start = [0.05, 0.025]
    # end = [5.715, 5.735]
    L=10
    W=9
    robot_margin=0.1

    grid_size=100
    # obs=np.array([[20,20,4.5,6.2],[2.6,2.6,4,4]],dtype=np.float)
    # obs = [[[.5, 0, 0], [1, .5, .5]], [[-1, 0, 0], [-.5, .5, .5]]]
    # obs = [[],\

    obs = np.array([[1, 1, 2, 2],
        [5, 1,6, 3],
        [1, 6, 2, 8],
        [7, 7, 8, 10],
        [8, 2, 9, 3]], dtype=np.float)
    # obs = np.array([[0.5, 1, 1.5, 2], [3,3.5,3.5,2]], dtype=np.float)
    planner=Planner(obs,robot_margin,grid_length=L,grid_width=W,grid_size=grid_size)
    planner.build_graph()
    start = [5, 5]
    end = [1., 5.]
    points = [[1., 0.5], [3., 1.], [6., 9.],end]
    # [1., 0.5], [3., 1.], [6., 9.], [1., 5.]
    pts = [start, [1., 0.5], [3., 1.], [6., 9.], end]
    paths=planner.multiple_dijk(start,points)
    # paths = [[
    #     (50, 50), (49, 49), (48, 48), (47, 47), (46, 46), (45, 45), (44, 44), (43, 43), (42, 42), (41, 41), (40, 40), (39, 39), (38, 38), (37, 37), (36, 36), (35, 35), (34, 34), (34, 33), (34, 32), (34, 31), (34, 30), (34, 29), (34, 28), (33, 27), (33, 26), (32, 25), (31, 24), (30, 23), (30, 22), (29, 21), (28, 20), (27, 19), (26, 18), (26, 17), (26, 16), (25, 15), (24, 14), (24, 13), (23, 12), (23, 11), (23, 10), (23, 9), (22, 8), (21, 7), (20, 7), (19, 7), (18, 7), (17, 7), (16, 7), (15, 7), (14, 7), (13, 7), (12, 7), (11, 6), (10, 5)]
    # ]
    #     [ (30, 1), (30, 2), (30, 3), (30, 4), (30, 5), (30, 6), (30, 7), (30, 8), (30, 9), (30, 10), (29, 11), (28, 12), (27, 13), (26, 14), (25, 15), (24, 16), (23, 17), (22, 18), (21, 19), (20, 20) ],
    #     [ (20, 20), (20, 21), (20, 22), (20, 23), (20, 24), (20, 25), (20, 26), (20, 27), (20, 28), (20, 29), (20, 30), (20, 31), (20, 32), (20, 33), (20, 34), (20, 35), (20, 36), (20, 37), (20, 38), (20, 39), (20, 40) ],
    #     [ (20, 40), (21, 40), (22, 40), (23, 40), (24, 40), (25, 40), (26, 40), (27, 40), (28, 40), (29, 40), (30, 40), (31, 40), (32, 40), (33, 40), (34, 40), (35, 40), (36, 40), (37, 40), (38, 40), (39, 40), (40, 40), (41, 41), (42, 42) ],
    #     # [ (42, 42), (43, 43), (44, 44), (45, 45), (46, 46), (47, 47), (48, 48), (49, 49), (50, 50), (51, 51), (52, 52), (53, 53), (54, 54), (55, 55) ]
    # ]
    # path=planner.path(start,end)
    k = Kinematics(0.1, 0.12, 0.028)
    # with open('paths.txt', 'w') as f:
    #     for path in paths:
    #         f.write(", ".join([f"({left}, {right})" for left, right in path]))
    #         f.write("\n")
    rots = list()
    # if paths != -1:
    for i in range(len(paths)):
        print(f"start pts {pts[i]}")
        print(f"end pts {pts[i+1]}")
        print(f"start path {paths[i][0]}")
        print(f"end path {paths[i][-1]}")
        x_thetas = k.grid_path_to_cspace(paths[i], pts[i], pts[i+1])
        wheel_rotations = k.cspace_to_wheel_rotations()
        rots += wheel_rotations

    # print(f"rots {rots}")
            
    with open("path.txt", "w") as f:
        for rot in rots:
            left, right = rot
            f.write(f"{left} {right}\n")
        # with open("cell_path.txt", "w") as f:
        #     for cell in path:
        #         f.write(f"{cell}\n")
        
        # x_thetas = k.grid_path_to_cspace(path, start, end)
        # with open("x_theta_path.txt", "w") as f:
        #     for cell in x_thetas:
        #         f.write(f"{cell}\n")
        # wheel_rotations = k.cspace_to_wheel_rotations()
        # with open("path.txt", "w") as f:
        #     for rot in wheel_rotations:
        #         left, right = rot
        #         f.write(f"{left} {right}\n")
    
    
    
    


