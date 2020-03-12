from math import sqrt, acos, pi

# grid -> x, y, theta (c space)
# x, y, theta -> wheel rots (w space)
class Kinematics():
    def __init__(self, cell_size):
        self.cell_size = cell_size


    def grid_path_to_cspace(self, path):
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
        # dot_prod(a,b) = mag(a)*mag(b)*cos(theta)
        vec = self.get_vector(a, b)
        # cos(theta) = dot_prod(v, i^)/mag(v)
        cos_theta = self.dot_product(vec, (1, 0))/(self.magnitude(vec))
        rad = acos(cos_theta)

        # y direction of vector is negative so in quad 3/4
        if vec[1] < 0:
            rad *= -1

        rotation = rad - orientation

        if abs(rotation) > pi:
            rotation += 2*pi

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
