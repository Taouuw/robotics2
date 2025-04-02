from __future__ import annotations

import rclpy
import numpy as np
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from importlib.resources import files
from typing import NamedTuple
# my custom subpackage
from .robot import Point, RobotAngles, Robot

class DemoController(Node):

    def __init__(self, robot : Robot, angless : list[RobotAngles], timer_period = 0.1, mode = 0):
        super().__init__('minimal_publisher')

        self.robot = robot
        self.mode = mode
        self.angless = angless
        self.i = 0
        self._angles = angless[0]
        self._ee = 0.0
        self._now = self.get_clock().now()
        self._beginning = self._now

        self._publisher = self.create_publisher(JointTrajectory, 'joint_cmds', 10)
        
        self._timer = self.create_timer(timer_period, self.timer_callback)
     
    def timer_callback(self):
        now = self.get_clock().now()
        self._now = now

        msg = JointTrajectory()
        msg.header.stamp = self._now.to_msg()

        self._angles = self.angless[self.i]
        self.i = (self.i + 1) %len(self.angless)

        point = JointTrajectoryPoint()
        point.positions = [angle for angle in self._angles.toROS()] + [self._ee]
        msg.points = [point]

        self._publisher.publish(msg)

        # raise SystemExit(f"{self} exited upon completion of schedule")

