from __future__ import annotations

import rclpy
import numpy as np
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from importlib.resources import files
from typing import NamedTuple
# my custom subpackage
from .robot import Point, RobotAngles, Robot

class BeyondScheduleError(ValueError):
    pass

class SchedulePoint(NamedTuple):
    position : Point
    ee : float
    time : float

class Schedule():
    """time paramatrized route of EE position targets.
    can be made periodic"""
    def __init__(self, waypoints : list[Point], ees: list [float], times : list[float], is_periodic=False):
        self.waypoints = waypoints
        self.is_periodic = is_periodic
        self.ees = ees
        self.times = times
        self.period = self.times[-1]

        if not all(a <= b for a,b in zip(times[:1], times[1:0])):
            raise ValueError(f"times are not sorted in ascending order!! {times}")

    @classmethod
    def _create(cls, waypoints : list[Point], ee: list [float] | float = 0, dt : list[float] | float = 5, is_periodic=False) -> Schedule:


        if type(ee) is float or type(ee) is int:
            ees = [ee for waypoint in waypoints]
        elif type(ee) is list:
            if len(ee) != len(waypoints):
                raise IndexError(f"number of time intervals dt {len(ee)} is not equal to number of waypoints {len(waypoints)}!")    
            ees = ee
        else:
            raise TypeError(f"invalid type for ees: {type(ee)}")

        if type(dt) is float or type(dt) is int:
            times = np.cumsum([dt/len(waypoints) for waypoint in waypoints]).tolist()
        elif type(dt) is list:
            if len(dt) != len(waypoints):
                raise IndexError(f"number of time intervals dt {len(dt)} is not equal to the number of waypoints {len(waypoints)}!")    
            times = np.cumsum(dt).tolist()
        else:
            raise TypeError(f"Invalid type for dt : {type(dt)}")

        return Schedule(waypoints, ees, times, is_periodic)

    @classmethod
    def create(cls, waypoints : list[Point], ee : list[float] | float = 0, is_periodic=False, speed=0.2, speed_ee=1) -> Schedule:
        dts_motion = [(waypoints[i]-waypoints[i-1]).distance()/speed for i in range(len(waypoints))]
        if type(ee) is list or type(ee) is np.ndarray:
            dts_ees = [abs(ee[i]-ee[i-1])/speed_ee for i in range(len(ee))]
            dts = [max(dt_motion, dt_ees) for dt_motion, dt_ees in zip(dts_motion, dts_ees)]
        else:
            dts = dts_motion

        if not is_periodic:
            dts[0] = 0

        return Schedule._create(waypoints, ee, dts, is_periodic)

    @classmethod
    def create_pick(cls, target : Point, ee=0.65, offset=Point(0,0,0.1), speed=0.2, speed_ee=1) -> Schedule:
        waypoints = [target+offset, target, target, target+offset]
        ees = [0, 0, ee, ee]

        return cls.create(waypoints, ees, False, speed, speed_ee)

    @classmethod
    def create_place(cls, target : Point, ee=0.65, offset=Point(0,0,0.1), speed=0.2, speed_ee=1) -> Schedule:
        waypoints = [target+offset, target, target, target+offset]
        ees = [ee, ee, 0, 0]

        return cls.create(waypoints, ees, False, speed, speed_ee)

    @classmethod
    def create_pick_and_place(cls, pick : Point, place : Point, ee=0.65, offset=Point(0,0,0.1), neutral=Point(0.1,0,0.2), is_periodic=False, speed=0.2, speed_ee=1) -> Schedule:
        waypoints = [neutral, pick+offset, pick, pick, pick+offset, place+offset, place, place, place+offset, neutral]
        ees = [0, 0, 0, ee, ee,ee,ee, 0,   0, 0]

        return cls.create(waypoints, ees, is_periodic, speed, speed_ee)

    @classmethod
    def create_wipe(cls, p00 : Point, p01 : Point, p10 : Point, p11 : Point, n=8, ee=0.8, is_periodic=False, speed=0.2, speed_ee=1) -> Schedule:
        dx0 = (p00 - p01)/n
        dx1 = (p10 - p11)/n

        # dy0 = (p00 - p10)/ny
        # dy1 = (p01 - p11)/ny

        waypoints = [p + d * i for i in range(n) for p,d in ((p00, dx0),(p10,dx1))]

        return cls.create(waypoints, ee, is_periodic, speed, speed_ee)

    @staticmethod
    def find_true(my_list : list[float], condition) :
        index = next((i for i,value in enumerate(my_list) if condition(value)), -1)
        return index

    def velocity(self, time : float) -> Point:

        if time >= self.period and not self.is_periodic:
            raise BeyondScheduleError(f"time {time} is beyond final schedule time {self.times[-1]}")

        t = time % self.period

        index = self.find_true(self.times, lambda _schedule_time:  t < _schedule_time)
        
        t_next = self.times[index]
        t_prev = self.times[index-1] % self.period

        waypoint_next = self.waypoints[index]
        waypoint_prev = self.waypoints[index-1]

        velocity = (waypoint_next - waypoint_prev) / (t_next - t_prev)

        return velocity

    def position(self, time : float) -> Point:

        if time >= self.period and not self.is_periodic:
            raise BeyondScheduleError(f"time {time} is beyond final schedule time {self.times[-1]}")

        t = time % self.period

        index = self.find_true(self.times, lambda _schedule_time:  t < _schedule_time)
        
        t_next = self.times[index]
        t_prev = self.times[index-1] % self.period

        waypoint_next = self.waypoints[index]
        waypoint_prev = self.waypoints[index-1]

        position = waypoint_prev + (waypoint_next - waypoint_prev) * ((t - t_prev) / (t_next - t_prev))

        return position

    def ee(self, time : float) -> float:

        if time >= self.period and not self.is_periodic:
            raise BeyondScheduleError(f"time {time} is beyond final schedule time {self.period}")


        t = time % self.period

        index = self.find_true(self.times, lambda _schedule_time:  t < _schedule_time)
        
        t_next = self.times[index]
        t_prev = self.times[index-1] % self.period

        ee_next = self.ees[index]
        ee_prev = self.ees[index-1]

        ee = ee_prev + (ee_next - ee_prev) * ((t - t_prev) / (t_next - t_prev))

        return float(ee)

    def __call__(self, time : float) -> SchedulePoint:

        if time >= self.period and not self.is_periodic:
            raise BeyondScheduleError(f"time {time} is beyond final schedule time {self.period}")

        t = time % self.period

        index = self.find_true(self.times, lambda _schedule_time:  t < _schedule_time)
        
        t_next = self.times[index]
        t_prev = self.times[index-1] % self.period

        waypoint_next = self.waypoints[index]
        waypoint_prev = self.waypoints[index-1]
        
        ee_next = self.ees[index]
        ee_prev = self.ees[index-1]

        waypoint = waypoint_prev + (waypoint_next - waypoint_prev) * ((t - t_prev) / (t_next - t_prev))

        ee = ee_prev + (ee_next - ee_prev) * ((t - t_prev) / (t_next - t_prev))

        return SchedulePoint(waypoint, ee, t)

    def extended(self, other : Schedule, speed=0.2, speed_ee=1) :
        is_periodic = self.is_periodic and other.is_periodic
        
        transfer_time = max(abs(other.ees[0] - self.ees[-1])/speed_ee, (other.waypoints[0] - self.waypoints[-1]).distance()/speed)
        other_time_offset = self.period - other.times[0] + transfer_time

        new_t0 = max(abs(self.ees[0] - other.ees[-1])/speed_ee, (self.waypoints[0] - other.waypoints[-1]).distance()/speed) if is_periodic else 0
        self_time_offset = new_t0 - self.times[0]

        times = [self_time + self_time_offset for self_time in self.times] + [other_time + other_time_offset + self_time_offset for other_time in other.times]

        return Schedule(self.waypoints + other.waypoints, self.ees + other.ees, times, is_periodic)

    def set_periodic(self, is_periodic : bool, speed=0.2, speed_ee=1) : 
        if is_periodic == self.is_periodic:
            return self
        else: 
            if is_periodic:
                time_offset = - self.times[0] + max(abs(self.ees[0] - self.ees[-1])/speed_ee, (self.waypoints[0] - self.waypoints[-1]).distance()/speed)

            else:
                time_offset = - self.times[0]

            return Schedule(self.waypoints, self.ees, [time + time_offset for time in self.times], is_periodic)

    def __add__(self, other : Schedule) -> Schedule:
        return Schedule(self.waypoints + other.waypoints, self.ees + other.ees, self.times + [other_time + self.period for other_time in other.times], self.is_periodic and other.is_periodic)

    def __mul__(self, n : int) -> Schedule:
        return sum([self for i in range(n)])
    
class RobotController(Node):
    """Node which takes a robot representation and a schedule"""

    def __init__(self, robot : Robot,  schedule : Schedule, timer_period = 0.1, mode = 0):
        
        super().__init__('minimal_publisher')

        self.schedule = schedule
        self.robot = robot
        self.mode = mode

        self._angles = RobotAngles.create_zero()
        self._ee = 0.
        self._now = self.get_clock().now()
        self._beginning = self._now

        self._publisher = self.create_publisher(JointTrajectory, 'joint_cmds', 10)
        
        # timer_period = 0.04  # seconds
        self._timer = self.create_timer(timer_period, self.timer_callback)

        if (self.mode):
            self._angles = min(self.robot.inverses(self.schedule.position(0)), key=self._angles.difference)
                
    def timer_callback(self):
        now = self.get_clock().now()
        t = (now - self._beginning).nanoseconds * 1e-9
        dt = (now - self._now).nanoseconds * 1e-9
        self._now = now

        msg = JointTrajectory()
        msg.header.stamp = self._now.to_msg()

        try:
            if self.mode:
                if self.mode == 1:
                    velocity = self.schedule.velocity(t)
                else: # elif self.is_velocity_control == 2:
                    velocity = (self.schedule.position(t)- self.robot.forward(self._angles) ).normalized() * self.schedule.velocity(t).distance()

                angle_velocity = self.robot.joint_velocity(velocity, self._angles)
                
                self._angles += angle_velocity * dt
                self.ee = self.schedule.ee(t)

            else:
                position, ee, _ = self.schedule(t)
                self._angles = min(self.robot.inverses(position), key=self._angles.difference)
                self._ee = ee

        except BeyondScheduleError:
            # quit
            # self._timer.stop()
            # self.destroy_node()
            # rclpy.shutdown() 
            # # do nothing (don't even send message) as you have gone beyond the schedule provided
            raise SystemExit(f"{self} exited upon completion of schedule")



        point = JointTrajectoryPoint()
        point.positions = [angle for angle in self._angles.toROS()] + [self._ee]
        msg.points = [point]

        self._publisher.publish(msg)

def main(args=None):
    waypoints = [Point(0.1, 0, 0.1), Point(0.1,-0.1,0)]
    schedule = Schedule(waypoints, is_periodic=True)
    print(schedule.times)
    for t in np.linspace(0,1):
        print(schedule(t))


if __name__ == '__main__':
    main()
