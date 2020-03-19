from math import sqrt, acos, pi

# x, theta -> wheel rots (w space)
class Kinematics():
    # all units in meters
    def __init__(self, cell_size, axle_length, wheel_radius, orientation = 0):
        self.cell_size = cell_size
        self.axle_length = axle_length
        self.wheel_radius = wheel_radius
        self.orientation = orientation


    # grid -> x, theta (c space)
    def grid_path_to_cspace(self, path, start, end):
        moves = list()
        current_theta = self.orientation
        # print(f"current theta {current_theta}")
        first_move = self.point_to_cell(start, path[1], self.orientation)
        # current_theta = self.new_orientation(start, (path[1][0]*self.cell_size, path[1][1]*self.cell_size))
        # print(f"first move {first_move}")
        # print(f"current theta {current_theta}")
        # print(f"start {start}")
        # print(f"current_theta {current_theta}")
        # moves.append(first_move)
        for i in range(1, len(path)):
            a = path[i - 1] 
            b = path[i] 
            x = 1
            theta = self.get_rotation(a, b, self.orientation)
            if a[0] != b[0] and a[1] != b[1]:
                x = sqrt(2)
            translation_m = self.get_translation(x)
            moves.append((translation_m, theta))
            self.orientation = self.new_orientation(a, b)

        last_move = self.cell_to_point(end, path[-2], current_theta)
        # print(f"last move {last_move}")
        # print(f"orientation {current_theta}")
        # self.orientation = t
        # self.orientation = self.new_orientation((path[-2][0]*self.cell_size, path[-2][1]*self.cell_size), end)
        # moves.append(last_move)
        self.moves = moves
        # print(f"orientation {self.orientation}")
        # print(f"path len {len(path)}")
        return moves

    def cspace_to_wheel_rotations(self):
        wheel_rotations = list()
        for x, theta in self.moves:
            if theta != 0:
                wheel_rotations.append(self.rotation_to_wheel_rotation(theta))
            wheel_rotations.append(self.translation_to_wheel_rotation(x))
        return wheel_rotations
        # simplified = list()
        # agg = 0
        # for left, right in wheel_rotations:
        #     if left != right:
        #         if agg != 0:
        #             simplified.append((agg, agg))
        #         simplified.append((left, right))
        #         agg = 0
        #     agg += right
        # if agg != 0:
        #     simplified.append((agg, agg))
            

        # return simplified


    def point_to_cell(self, point, cell, orientation):
        grid_x, grid_y = cell
        center_x = (grid_x * self.cell_size)
        center_y = (grid_y * self.cell_size)
        theta = self.get_rotation(point, (center_x, center_y), orientation)
        vec = self.get_vector(point, (center_x, center_y))
        mag = self.magnitude(vec)
        return mag, theta


    def cell_to_point(self, point, cell, orientation):
        grid_x, grid_y = cell
        center_x = (grid_x * self.cell_size) + (self.cell_size/2)
        center_y = (grid_y * self.cell_size) + (self.cell_size/2)
        # print(f"({center_x}, {center_y})")
        theta = self.get_rotation((center_x, center_y), point, orientation)
        vec = self.get_vector((center_x, center_y), point)
        mag = self.magnitude(vec)
        return mag, theta


    def rotation_to_wheel_rotation(self, theta):
        """
        Takes rotation desired of robot in radians
        returns (theta_left, theta_right)
        """
        theta_r = (self.axle_length/2)*(theta/self.wheel_radius)
        return (-theta_r, theta_r)


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
        # y direction of vector is negative so in quad 3/4
        if vec[1] < 0:
            rad *= -1
        return rad


    def get_rotation(self, a, b, orientation):
        rad = self.new_orientation(a, b)

        rotation = rad - orientation

        if abs(rotation) > 3.15:
            rotation += 2*pi
        if abs(rotation) > 3.15:
            rotation = -rad - orientation
            print(f"a{a} b{b}")
            print(f"rad {rad}")
            print(f"orientation {orientation}")
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
