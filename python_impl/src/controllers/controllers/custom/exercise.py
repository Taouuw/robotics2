import matplotlib.pyplot as plt
import numpy as np

from robot import Robot, Point


if __name__ == "__main__":

    robot = Robot.create_real()
    pad_height = 0.005


    xys = np.load("controllers/data/flame.npy")
    min_x = np.min(xys[:,0])
    min_y = np.min(xys[:,1])
    max_x = np.max(xys[:,0])
    max_y = np.max(xys[:,1])
    waypoints = [Point(0.1, (x-(min_x + max_x)*0.5) *.01 * 3.5, robot.ee_radius + pad_height + (max_y-y)*.01 * 3.5) for x,y in xys]

    ys = [waypoint.y for waypoint in waypoints]
    zs = [waypoint.z for waypoint in waypoints]

    plt.plot(zs, ys, marker='.')
    plt.xlabel(r'z [m]')
    plt.ylabel(r'y [m]')
    plt.gca().set_aspect('equal')
    plt.gca().invert_yaxis()
    plt.gca().invert_xaxis()
    plt.savefig("flame.png")
    plt.close()