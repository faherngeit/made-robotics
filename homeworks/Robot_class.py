import random
from math import sin, cos, sqrt, tan, acos, pi, exp

max_steering_angle = (
        pi / 4.0
)  # You do not need to use this value, but keep in mind the limitations of a real car.
bearing_noise = 0.1
steering_noise = 0.1
distance_noise = 5.0

tolerance_xy = 15.0  # Tolerance for localization in the x and y directions.
tolerance_orientation = 0.25  # Tolerance for orientation.


# --------
#
# the "world" has 4 landmarks.
# the robot's initial coordinates are somewhere in the square
# represented by the landmarks.

landmarks = [
    [100.0, 0.0],
    [0.0, 0.0],
    [0.0, 100.0],
    [100.0, 100.0],
]  # position of 4 landmarks
world_size = 100.0  # world is NOT cyclic. Robot is allowed to travel "out of bounds"


# Some utility functions

def get_position(p):
    x = 0.0
    y = 0.0
    orientation = 0.0
    for i in range(len(p)):
        x += p[i].x
        y += p[i].y
        # orientation is tricky because it is cyclic. By normalizing
        # around the first particle we are somewhat more robust to
        # the 0=2pi problem
        orientation += (((p[i].orientation - p[0].orientation + pi) % (2.0 * pi))
                        + p[0].orientation - pi)
    return [x / len(p), y / len(p), orientation / len(p)]


# The following code generates ground truth poses and measurements
def generate_ground_truth(motions):

    myrobot = ExtRobot()
    myrobot.set_noise(bearing_noise, steering_noise, distance_noise)

    Z = []
    T = len(motions)

    for t in range(T):
        myrobot = myrobot.move(motions[t])
        Z.append(myrobot.sense())
    #print 'Robot:    ', myrobot
    return [myrobot, Z]


# The following code prints the measurements associated
# with generate_ground_truth
def print_measurements(Z):

    T = len(Z)

    print( 'measurements = [[%.8s, %.8s, %.8s, %.8s],' % \
           (str(Z[0][0]), str(Z[0][1]), str(Z[0][2]), str(Z[0][3])))
    for t in range(1,T-1):
        print( '                [%.8s, %.8s, %.8s, %.8s],' % \
               (str(Z[t][0]), str(Z[t][1]), str(Z[t][2]), str(Z[t][3])))
    print( '                [%.8s, %.8s, %.8s, %.8s]]' % \
           (str(Z[T-1][0]), str(Z[T-1][1]), str(Z[T-1][2]), str(Z[T-1][3])))


# The following code checks to see if your particle filter
# localizes the robot to within the desired tolerances
# of the true position. The tolerances are defined at the top.
def check_output(final_robot, estimated_position):

    error_x = abs(final_robot.x - estimated_position[0])
    error_y = abs(final_robot.y - estimated_position[1])
    error_orientation = abs(final_robot.orientation - estimated_position[2])
    error_orientation = (error_orientation + pi) % (2.0 * pi) - pi
    correct = error_x < tolerance_xy and error_y < tolerance_xy \
              and error_orientation < tolerance_orientation
    return correct

class ExtRobot:

    # --------

    # init:
    #   creates robot and initializes location/orientation
    #

    def __init__(self, length=10.0):
        self.x = random.random() * world_size  # initial x position
        self.y = random.random() * world_size  # initial y position
        self.orientation = random.random() * 2.0 * pi  # initial orientation
        self.length = length  # length of robot
        self.bearing_noise = 0.0  # initialize bearing noise to zero
        self.distance_noise = 0.0  # initialize distance noise to zero
        self.steering_noise = 0.0  # initialize steering noise to zero

    def __repr__(self):
        return "[x=%.6s y=%.6s theta=%.6s]" % (
            str(self.x),
            str(self.y),
            str(self.orientation),
        )

    # --------
    # set:
    #   sets a robot coordinate
    #

    def sense(self, no_noise=True):
        dir_x = cos(self.orientation)
        dir_y = sin(self.orientation)
        bearings = []
        for x_l, y_l in landmarks:
            if no_noise:
                noise_x, noise_y = (0, 0)
            else:
                noise_x = random.normalvariate(0, self.bearing_noise)
                noise_y = random.normalvariate(0, self.bearing_noise)
            vec_x = x_l - self.x + noise_x
            vec_y = y_l - self.y + noise_y
            vec_len = sqrt(vec_x ** 2 + vec_y ** 2)
            angle = acos((dir_x * vec_x + dir_y * vec_y) / vec_len)
            vec_mult = dir_x * vec_y - dir_y * vec_x
            if vec_mult > 0:
                bearing = angle % (2*pi)
            else:
                bearing = -angle % (2*pi)
            bearings.append(bearing)
        return bearings

    def measurement_prob(self, measures):
        likelihood = self.get_normal_prob(measures)
        return likelihood

    def set(self, new_x, new_y, new_orientation):
        if new_orientation < 0 or new_orientation >= 2 * pi:
            raise ValueError("Orientation must be in [0..2pi]")
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)

    def get_normal_prob(self, x):
        prob = 1.
        if self.bearing_noise == 0:
            return 0.
        for (bearing, measure) in zip(self.sense(no_noise=True), x):
            prob *= 1 / sqrt(2 * pi * self.bearing_noise) * exp(- ((measure - bearing)/(pi * self.bearing_noise)) ** 2)
        return prob

    # --------
    # set_noise:
    # 	sets the noise parameters
    #

    def set_noise(self, new_b_noise, new_s_noise, new_d_noise):
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.bearing_noise = float(new_b_noise)
        self.steering_noise = float(new_s_noise)
        self.distance_noise = float(new_d_noise)

    ############# ONLY ADD/MODIFY CODE BELOW ###################

    # --------
    # move:
    #   move along a section of a circular path according to motion parameters
    #

    def move(self, motion):
        robot = ExtRobot(self.length)
        if motion[0] == 0:
            angle = self.orientation
            x_pos = self.x + motion[1] * cos(angle)
            y_pos = self.y + motion[1] * sin(angle)
        else:
            radius = self.length / tan(motion[0])
            rotation = motion[1] / radius

            x_c = self.x - radius * sin(self.orientation)
            y_c = self.y + radius * cos(self.orientation)

            angle = (self.orientation + rotation) % (2 * pi)
            x_pos = x_c + radius * sin(angle)
            y_pos = y_c - radius * cos(angle)
        robot.set(x_pos, y_pos, angle)
        robot.set_noise(self.bearing_noise, self.steering_noise, self.distance_noise)
        return robot
        # make sure your move function returns an instance of Robot class
        # with the correct coordinates.

    ############## ONLY ADD/MODIFY CODE ABOVE ####################


def particle_filter(motions, measurements, N=500):  # We will use 500 particles
    # --------
    #
    # Create particles (models of the Robot)
    #
    bearing_noise = 0.1
    steering_noise = 0.1
    distance_noise = 5.0

    particles = []
    for i in range(N):
        robot = ExtRobot()
        robot.set_noise(bearing_noise, steering_noise, distance_noise)
        particles.append(robot)

    # --------
    #
    # Update particles
    #

    for t in range(len(motions)):

        # motion update (prediction)
        particles_after_motion = []
        for i in range(N):
            particles_after_motion.append(particles[i].move(motions[t]))
        particles = particles_after_motion

        # measurement update (correction)
        weights = []
        for i in range(N):
            weights.append(particles[i].measurement_prob(measurements[t]))

        # resampling
        particles_resampled = []
        index = int(random.random() * N)
        beta = 0.0
        mw = max(weights)
        for i in range(N):
            beta += random.random() * 2.0 * mw
            while beta > weights[index]:
                beta -= weights[index]
                index = (index + 1) % N
            particles_resampled.append(particles[index])
        particles = particles_resampled

    return get_position(particles)