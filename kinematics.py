from math import sqrt, acos, pi

# x, theta -> wheel rots (w space)
class Kinematics():
    # all units in meters
    def __init__(self, cell_size, axle_length, wheel_radius):
        self.cell_size = cell_size
        self.axle_length = axle_length
        self.wheel_radius = wheel_radius


    # grid -> x, theta (c space)
    def grid_path_to_cspace(self, path):
        moves = list()
        current_theta = 0
        for i in range(1, len(path)):
            a = path[i - 1] 
            b = path[i] 
            x = 1
            theta = self.get_rotation(a, b, current_theta)
            if a[0] != b[0] and a[1] != b[1]:
                x = sqrt(2)
            translation_m = self.get_translation(x)
            moves.append((translation_m, theta))
            current_theta = self.new_orientation(a, b)

        self.moves = moves
        return moves

    def cspace_to_wheel_rotations():
        wheel_rotations = list()
        for x, theta in self.moves:
            if theta != 0:
                wheel_rotations.append(self.rotation_to_wheel_rotation(theta))
            wheel_rotations.append(translation_to_wheel_rotation(x))
        return wheel_rotations



    def rotation_to_wheel_rotation(self, theta):
        """
        Takes rotation desired of robot in radians
        returns (theta_left, theta_right)
        """
        if theta == 0:
            return (0, 0)
        elif theta > 0:
            theta_r = (self.axle_length/2)*(theta/self.wheel_radius)
            return (-theta_r, theta_r)
        else:
            theta_r = (self.axle_length/2)*(theta/self.wheel_radius)
            return (theta_r, -theta_r)


    def translation_to_wheel_rotation(self, x):
        theta_r = x/self.wheel_radius
        return (theta_r, theta_r)


    def get_translation(self, grid_units):
        return grid_units * self.cell_size


    def new_orientation(self, a, b):
        # dot_prod(a,b) = mag(a)*mag(b)*cos(theta)
        vec = self.get_vector(a, b)
        # cos(theta) = dot_prod(v, i^)/mag(v)
        cos_theta = self.dot_product(vec, (1, 0))/(self.magnitude(vec))
        rad = acos(cos_theta)
        return rad


    def get_rotation(self, a, b, orientation):
        rad = self.new_orientation(a, b)
        # y direction of vector is negative so in quad 3/4
        vec = self.get_vector(a, b)
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
