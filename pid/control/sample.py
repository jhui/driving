#
# PID controller
#

import random
import numpy as np
import matplotlib.pyplot as plt

class Robot(object):
    def __init__(self, scale=20.0):
        """
        Creates a robot with x, y location and orientation.
        """
        self.x = 0.0
        self.y = 0.0
        self.orientation = 0.0

        self.scale = scale

        # Noise in steering & speed
        self.steering_noise_sigma = 0.0
        self.speed_noise_sigma = 0.0

        # Apply a constant steering drift
        self.steering_drift = 0.0

    def set(self, x, y, orientation):
        """
        Sets a robot coordinate.
        """
        self.x = x
        self.y = y
        self.orientation = orientation % (2.0 * np.pi)

    def set_noise(self, steering_noise_sigma, distance_noise_sigma):
        """
        Sets the noise parameters.
        """
        self.steering_noise_sigma = steering_noise_sigma
        self.speed_noise_sigma = distance_noise_sigma

    def set_steering_drift(self, drift):
        """
        Sets the systematical steering drift parameter
        """
        self.steering_drift = drift

    def move(self, steering, speed, tolerance=0.001, max_steering_angle=np.pi / 4.0):
        """
        steering: front wheel steering angle, limited by max_steering_angle
        speed: speed of the car
        """
        steering = max(-max_steering_angle, min(max_steering_angle, steering))
        speed = max(speed, 0)

        # apply noise
        steering = random.gauss(steering, self.steering_noise_sigma)
        distance = random.gauss(speed, self.speed_noise_sigma)   # Distance traveled in 1 time interval

        # apply steering drift
        steering += self.steering_drift

        # Execute motion
        turn = np.tan(steering) * distance / self.scale

        if abs(turn) < tolerance:
            # approximate by straight line motion
            self.x += distance * np.cos(self.orientation)
            self.y += distance * np.sin(self.orientation)
            self.orientation = (self.orientation + turn) % (2.0 * np.pi)
        else:
            # approximate bicycle model for motion
            radius = distance / turn
            cx = self.x - (np.sin(self.orientation) * radius)
            cy = self.y + (np.cos(self.orientation) * radius)
            self.orientation = (self.orientation + turn) % (2.0 * np.pi)
            self.x = cx + (np.sin(self.orientation) * radius)
            self.y = cy - (np.cos(self.orientation) * radius)

    def __repr__(self):
        return '[x=%.5f y=%.5f orient=%.5f]' % (self.x, self.y, self.orientation)


############## ADD / MODIFY CODE BELOW ####################
# ------------------------------------------------------------------------
#
# run - does a single control run


def make_robot():
    """
    Resets the robot back to the initial position and drift.
    You'll want to call this after you call `run`.
    """
    robot = Robot()
    robot.set(0, 1, 0)
    robot.set_steering_drift(10 / 180 * np.pi)
    return robot


# NOTE: We use params instead of tau_p, tau_d, tau_i
def run(robot, params, n=100, speed=1.0):
    x_trajectory = []
    y_trajectory = []
    err = 0
    total_cte = 0
    prev_cte = robot.y
    for i in range(2 * n):
        cte = robot.y
        total_cte += cte

        cte = robot.y
        total_cte += cte
        steer = -params[0] * cte - params[1] * (cte - prev_cte) - params[2] * total_cte
        robot.move(steer, speed)
        x_trajectory.append(robot.x)
        y_trajectory.append(robot.y)
        if i >= n:
            err += cte ** 2
        prev_cte = cte
    return x_trajectory, y_trajectory, err / n


# Make this tolerance bigger if you are timing out!
def twiddle(tol=0.2):
    # Don't forget to call `make_robot` before you call `run`!
    p = [0, 0, 0]
    dp = [1, 1, 1]
    robot = make_robot()
    x_trajectory, y_trajectory, best_err = run(robot, p)
    for iter in range(300):
        for i in range(len(p)):
            p[i] += dp[i]
            robot = make_robot()
            x_trajectory, y_trajectory, err = run(robot, p)
            if (err < best_err):
                best_err = err
                dp[i] *= 1.1
            else:
                p[i] -= 2 * dp[i]
                robot = make_robot()
                x_trajectory, y_trajectory, err = run(robot, p)
                if (err < best_err):
                    best_err = err
                    dp[i] *= 1.1
                else:
                    p[i] += dp[i]
                    dp[i] *= 0.9

    return p, best_err


params, err = twiddle()
print("Final twiddle error = {}".format(err))
robot = make_robot()
x_trajectory, y_trajectory, err = run(robot, params)
n = len(x_trajectory)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 8))
ax1.plot(x_trajectory, y_trajectory, 'g', label='Twiddle PID controller')
ax1.plot(x_trajectory, np.zeros(n), 'r', label='reference')

plt.show()