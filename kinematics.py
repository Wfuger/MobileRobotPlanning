from math import sqrt, acos, pi

class Kinematics():
    def __init__(self, cell_size):
        self.cell_size = cell_size


    def grid_path_to_workspace(self, path):
        moves = list()
        current_theta = 0
        for i in range(1, len(path)):
            ax, ay = path[i - 1] 
            bx, by = path[i] 
            x, y, theta
            if ax == bx and ay == by:
                x, y = sqrt(2*(self.cell_size ** 2))
                # current_theta
        return moves


    def get_rotation(self, a, b, orientation):
        vec = self.get_vector(a, b)
        # print(f"vec {vec}")
        # print(f"vec mag {self.magnitude(vec)}")
        cosx = self.dot_product(vec, (1, 0))/(self.magnitude(vec))
        # print(f"cosx {cosx}")
        rad = acos(cosx)
        # print(f"orientation {orientation}")
        print(f"rad {rad}")
        # print(f"dot prod {self.dot_product(a, b)}")
        rotation = rad - orientation

        print(f"rotation {rotation}")
        if abs(rotation) > pi:
            rotation += 2*pi
        print(f"rotation {rotation}")
        return rotation


    def get_vector(self, a, b):
        return (b[0]-a[0], b[1]-a[1])

    def magnitude(self, a):
        return sqrt(a[0]**2 + a[1]**2)


    def dot_product(self, a, b):
        total = 0
        for i in range(len(a)):
            total += a[i] * b[i]
        return total
