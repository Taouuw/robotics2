import rclpy
import numpy as np
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from importlib.resources import files
# my custom subpackage
from .custom import Point, Domain, RobotAngles, RobotLengths, Robot
from .custom import RobotController, Schedule
from .custom import DemoController

def draw_traj(args=None):
    
    # parsing
    # xys = np.load(Path(__file__).parent / 'data' / 'flame.npy')

    robot = Robot.create_real()
    pad_height = 0.005

    xys = np.load(str(files("controllers.data").joinpath('flame.npy')))
    min_x = np.min(xys[:,0])
    min_y = np.min(xys[:,1])
    max_x = np.max(xys[:,0])
    max_y = np.max(xys[:,1])
    waypoints = [Point(0.1, (x-(min_x + max_x)*0.5) *.01 * 3.5, robot.ee_radius + pad_height + (max_y-y)*.01 * 3.5) for x,y in xys]
    
    schedule = Schedule.create(waypoints, ee=0.8, is_periodic=True)
    rclpy.init(args=args)

    robot_controller = RobotController(robot, schedule)
    
    try:
        rclpy.spin(robot_controller)
    except SystemExit as system_exit:
        print(system_exit)


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot_controller.destroy_node()
    rclpy.shutdown()

def jacobian_traj(args=None) :

    # xys = np.load(str(files("controllers.data").joinpath('flame.npy')))
    # min_x = np.min(xys[:,0])
    # min_y = np.min(xys[:,1])
    # max_x = np.max(xys[:,0])
    # max_y = np.max(xys[:,1])
    # waypoints = [Point(0.1, (x-(min_x + max_x)*0.5) *.01 * 3.5, (max_y-y)*.01 * 3.5) for x,y in xys]
    # waypoints = [Point(0.1, 0.2, 0.2), Point(0.1, -0.2, 0.2), Point(0.1, -0.2, 0.0), Point(0.1,0.2,0.0)]
    waypoints = [Point(0.1, 0.2, 0.1), Point(0.2, 0.0, 0.2), Point(0.1, -0.2, 0.15)]


    schedule = Schedule.create(waypoints, is_periodic=True)
    robot = Robot.create_real()

    rclpy.init(args=args)

    robot_controller = RobotController(robot, schedule, mode=2)
    
    try:
        rclpy.spin(robot_controller)
    except SystemExit as system_exit:
        print(system_exit)


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot_controller.destroy_node()
    rclpy.shutdown()

def grab_ab_traj(args=None):


    robot = Robot.create_real()
    pad_height = 0.005
    height = robot.ee_radius + pad_height
    schedule = Schedule.create_pick_and_place(Point(0.1,0.1, height), Point(0.1,-0.1, height))
    

    rclpy.init(args=args)

    robot_controller = RobotController(robot, schedule)
    
    try:
        rclpy.spin(robot_controller)
    except SystemExit as system_exit:
        print(system_exit)


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot_controller.destroy_node()
    rclpy.shutdown()

def grab_ba_traj(args=None):

    robot = Robot.create_real()
    pad_height = 0.005
    height = robot.ee_radius + pad_height
    # schedule1 = Schedule.create_pick_and_place(Point(0.1, 0.1, height), Point(0.1,-0.1, height))
    schedule2 = Schedule.create_pick_and_place(Point(0.1,-0.1, height), Point(0.1, 0.1, height))
    schedule = schedule2 # schedule1 + schedule2
    
    rclpy.init(args=args)

    robot_controller = RobotController(robot, schedule)
    
    try:
        rclpy.spin(robot_controller)
    except SystemExit as system_exit:
        print(system_exit)


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot_controller.destroy_node()
    rclpy.shutdown()

def berry_traj(args=None):

    robot = Robot.create_real()
    # schedule = Schedule.create_pick_and_place(Point(0.1,0.1, 0.02), Point(0.1,-0.1, 0.02), speed=0.1)
    pad_height = 0.005
    #height = robot.ee_radius + pad_height
    height = 0.03
    drop_height = robot.ee_radius + 0.15

    schedule = Schedule.create_pick(Point(0.1,0.1, height), speed=0.05).extended(Schedule.create_place(Point(0.1,-0.1, drop_height)))
    #schedule = Schedule.create_pick(Point(0.1,0.1, height)).extended(Schedule.create_place(Point(0.1,-0.1, drop_height)))


    rclpy.init(args=args)

    robot_controller = RobotController(robot, schedule)
    
    try:
        rclpy.spin(robot_controller)
    except SystemExit as system_exit:
        print(system_exit)


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot_controller.destroy_node()
    rclpy.shutdown()

def wipe_traj(args=None):

    robot = Robot.create_real()
    # pad_height = 0.005
    pick_height = 0.02
    height = robot.ee_radius
    sag = 0.06
    schedule = Schedule.create_pick(Point(0.1,0.1, pick_height), speed=0.1).extended(Schedule.create_wipe(Point(0.15, 0.1, height), Point(0.05, 0.1, height), Point(0.15, -0.1, height), Point(0.05, -0.1, height)))

    rclpy.init(args=args)

    robot_controller = RobotController(robot, schedule)
    
    try:
        rclpy.spin(robot_controller)
    except SystemExit as system_exit:
        print(system_exit)


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot_controller.destroy_node()
    rclpy.shutdown()

def demo_traj(args=None):

    robot = Robot.create_real()


    points = [Point(0.1, 0.1, 0.1), Point(0.2,0.1,0.3), Point(0.0,0.0,0.3), Point(0.0,0.0,0.7)]
    # waypoints = [points[0], points[0], points[2], points[2]]
    waypoints = points
    angless = sum([robot.inverses(point) for point in waypoints], start = [])

    rclpy.init(args=args)
    robot_controller = DemoController(robot, angless, timer_period=1)
    try:
        rclpy.spin(robot_controller)
    except SystemExit as system_exit:
        print(system_exit)


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot_controller.destroy_node()
    rclpy.shutdown()

def empty_traj(args=None):


    robot = Robot.create_real()
    # finding accessible waypoints
    trypoints = [Point(0.1, 0.1, 0.1), Point(0.2,0.1,0.3), Point(0.0,0.0,0.3), Point(0.0,0.0,0.7)]
    waypoints = []
    for point in trypoints:
        try:
            _ = robot.inverse(point)
            waypoints.append(point)
        except ValueError as error:
            pass

    schedule = Schedule.create(waypoints)

    rclpy.init(args=args)
    robot_controller = RobotController(robot, schedule)
    try:
        rclpy.spin(robot_controller)
    except SystemExit as system_exit:
        print(system_exit)


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    draw_traj()