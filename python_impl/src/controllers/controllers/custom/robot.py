from __future__ import annotations
from typing import NamedTuple
import math as math
import sympy
import numpy as np

class Point(NamedTuple):
    x: float
    y: float
    z: float

    def __add__(self, other : Point) -> Point:
        return Point(self.x + other.x, self.y + other.y, self.z + other.z)

    def __neg__(self):
        return Point(-self.x, -self.y, -self.z)

    def __sub__(self, other : Point) -> Point:
        return Point(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, other : float|Point) -> float|Point:
        if type(other) is Point:
            return self.x*other.x + self.y*other.y + self.z*other.z
        else:
            return Point(self.x*other, self.y*other, self.z*other)

    def __truediv__(self, other : float) -> Point:
        return Point(self.x/other, self.y/other, self.z/other)

    def no_x(self) -> Point:
        return Point(0.0, self.y, self.z)

    def no_y(self) -> Point:
        return Point(self.x, 0.0, self.z)

    def no_z(self) -> Point:
        return Point(self.x, self.y, 0.0)

    def squared(self) -> float:
        return self.x * self.x + self.y * self.y + self.z * self.z

    def distance(self) -> float:
        return math.sqrt(self.squared())
    
    def normalized(self) -> float:
        return self/self.distance()

class Domain(NamedTuple):
    lower: float    # radians
    upper: float    # radians

    def length(self) -> float:
        return self.upper - self.lower

    def margin(self, value : float) -> float:
        return min(value - self.lower, self.upper - value)

    def margin_relative(self, value : float) -> float:
        return self.margin(value)/self.length()

    def contains(self, value: float) -> bool:
        return (value >= self.lower) and (value <= self.upper)

    def linspace(self, n : int) -> np.ndarray:
        return np.linspace(self.lower, self.upper, n)

    @classmethod
    def fromDeg(cls, lower : float, upper : float):
        return cls(lower * math.pi/180, upper * math.pi/180)

    @classmethod
    def all(cls):
        return cls(-math.inf, math.inf)

    @classmethod
    def pie(cls):
        return cls(-math.pi, math.pi)

class RobotAngles(tuple[float, float, float, float]) :
    def toDegTuple(self) -> tuple[float, float, float, float]:
        return tuple(item * 180/math.pi for item in self)

    def toROS(self) -> tuple[float, float, float, float]:
        return (float(self[0]), float(self[1])-math.pi/2, float(-self[2])-math.pi/2, float(self[3]))

    def __add__(self, other : RobotAngles) -> RobotAngles:
        return RobotAngles(a+b for a,b in zip(self, other))

    def __sub__(self, other : RobotAngles) -> RobotAngles:
        return RobotAngles(a-b for a,b in zip(self, other))

    def __mul__(self, other : float) -> RobotAngles:
        return RobotAngles(a * other for a in self)

    def __neg__(self) -> RobotAngles:
        return RobotAngles(-a for a in self)
    
    def difference(self, other : RobotAngles) -> float:
        return sum(abs(a-b) for a,b in zip(self,other))

    @classmethod
    def create_symbolic(cls) -> RobotAngles:
        return cls(tuple(sympy.symbols(['d0', 'd1', 'd2', 'd3'])))

    @classmethod
    def create_nan(cls) -> RobotAngles:
        return cls((math.nan, math.nan, math.nan, math.nan))

    @classmethod
    def create_zero(cls) -> RobotAngles:
        return cls((0.,0.,0.,0.))

class RobotLengths(tuple[float, float, float, float]) :
    pass

class RobotDomains(tuple[Domain, Domain, Domain, Domain]) :
    def contains(self, angles : RobotAngles):
        return all([domain.contains(angle) for domain,angle in zip(self,angles)])

    def margin(self, angles : RobotAngles):
        return min([domain.margin(angle) for domain, angle in zip(self, angles)])

    def margin_relative(self, angles : RobotAngles):
        return min([domain.margin_relative(angle) for domain, angle in zip(self, angles)])

class Robot:
    __slots__ = 'position', 'lengths', 'domains', 'ee_radius'
    position : Point
    lengths : RobotLengths
    domains : RobotDomains
    ee_radius : float

    @classmethod
    def create_real(cls):
        # lengths = RobotLengths((.015, .094, .107, .055))
        # position = Point(0., 0., 0.07)  # todo ensure this value is correct
        
        # lengths = RobotLengths((.015, .093, .106, .076))
        # position = Point(0., 0., 0.062)  # todo ensure this value is correct
        
        lengths = RobotLengths((.015, .094, .106, .076))
        position = Point(0., 0., 0.067)  # todo ensure this value is correct

        ee_radius = 0.015

        domains = RobotDomains((Domain.fromDeg(-95, 95), Domain.fromDeg(-14.3, 180-30.4), Domain.fromDeg(-(180-13), 0), Domain.fromDeg(-95, 100)))
        
        return cls(lengths, domains, position, ee_radius)

    @classmethod
    def create_sim(cls):
        raise NotImplementedError
        lengths = RobotLengths((1.5, 9.4, 10.7, 4.5))
        domains = RobotDomains((Domain.fromDeg(-95, 95), Domain.fromDeg(-14.3, 180-30.4), Domain.fromDeg(-(180-13), 0), Domain.fromDeg(-95, 100)))
        position = Point(0., 0., 0.07)  # todo esure this value is correct
        ee_radius = 0.01
        return cls(lengths, domains, position, ee_radius)

    @classmethod
    def create_symbolic(cls):
        lengths = tuple(sympy.symbols([f'l{i}' for i in range(4)]))
        domains = RobotDomains((Domain(sympy.Symbol(f'd{i}_min'), sympy.Symbol(f'd{i}_max')) for i in range(4)))
        position = Point(*sympy.symbols(('x0', 'y0', 'z0')))
        ee_radius = sympy.Symbol('ree')
        return cls(lengths, domains, position, ee_radius)


    def __init__(self, lengths : RobotLengths, domains : RobotDomains, position : Point, ee_radius : float) :
        self.lengths = lengths
        self.domains = domains
        self.position = position
        self.ee_radius = ee_radius

    def forward(self, angles : RobotAngles) -> Point: # tuple[float, float, float, float]) -> Point:
        d0, d1, d2, d3 = angles
        l0, l1, l2, l3 = self.lengths

        if type(angles[0]) is sympy.Symbol:
            cos = sympy.cos
            sin = sympy.sin
        elif type(angles[0]) is np.ndarray:
            cos = np.cos
            sin = np.sin
        else:
            cos = math.cos
            sin = math.sin

        c0 = cos(d0)
        c1 = cos(d1)
        c12 = cos(d1 + d2)
        c123 = cos(d1 + d2 + d3)

        s0 = sin(d0)
        s1 = sin(d1)
        s12 = sin(d1 + d2)
        s123 = sin(d1 + d2 + d3)

        x = c0 * (l0 + l1 * c1 + l2 * c12 + l3 * c123)
        y = s0 * (l0 + l1 * c1 + l2 * c12 + l3 * c123)
        z = l1 * s1 + l2 * s12 + l3 * s123

        psi = d0
        phi = 0
        theta = d1 + d2 + d3

        return self.position + Point(x,y,z)

    @staticmethod
    def _cosine_rule_alpha_angle(a: float, b: float, c: float):
        return math.acos((b * b + c * c - a * a) / (2 * b * c))

    @staticmethod
    def _cosine_rule_a_length(alpha: float, b: float, c: float):
        return math.sqrt(b*b + c*c - 2*b*c*math.cos(alpha))

    @classmethod
    def _bend1(cls, length1 : float, length2 : float, target_length : float) :
        return cls._cosine_rule_alpha_angle(length2, length1, target_length)

    @classmethod
    def _bend2(cls, length1 : float, length2 : float, target_length : float) :
        return cls._cosine_rule_alpha_angle(target_length, length1, length2) - math.pi

    def inverse_no_bend(self, point : Point) -> RobotAngles:
        point_from_center = point - self.position
        heading = math.atan2(point_from_center.y, point_from_center.x)

        # shoulder is the point at which the robot hinges up
        shoulder_from_center = Point(math.cos(heading) * self.lengths[0], math.sin(heading) * self.lengths[0], 0)
        point_from_shoulder = point_from_center - shoulder_from_center

        distance_from_shoulder = point_from_shoulder.distance()
        planar_distance_from_shoulder = point_from_shoulder.no_z().distance()

        elevation = math.atan2(point_from_shoulder.z, planar_distance_from_shoulder)

        try :
            bend1 = self._bend1(self.lengths[1], self.lengths[2] + self.lengths[3], distance_from_shoulder)
            bend2 = self._bend2(self.lengths[1], self.lengths[2] + self.lengths[3], distance_from_shoulder)
        except :
            raise ValueError(f'{point} is not within workspace of robot! Due to arm lengths')

        angles_up = RobotAngles((heading, elevation + bend1, bend2, 0.0))
        angles_down = RobotAngles((heading, elevation - bend1, -bend2, 0.0))

        if self.domains.contains(angles_up) :
            return angles_up
        elif self.domains.contains(angles_down) :
            return angles_down
        else:
            raise ValueError(f'{point} is not within workspace of robot! Due to angle limits')

    def inverse_bend(self, point : Point, wrist_bend=0.0) -> RobotAngles:
        point_from_center = point - self.position
        heading = math.atan2(point_from_center.y, point_from_center.x)

        # shoulder is the point at which the robot hinges up
        shoulder_from_center = Point(math.cos(heading) * self.lengths[0], math.sin(heading) * self.lengths[0], 0)
        point_from_shoulder = point_from_center - shoulder_from_center

        distance_from_shoulder = point_from_shoulder.distance()
        planar_distance_from_shoulder = point_from_shoulder.no_z().distance()

        elevation = math.atan2(point_from_shoulder.z, planar_distance_from_shoulder)

        #length of elbow till EE
        length_elbow_EE = self._cosine_rule_a_length(wrist_bend - math.pi, self.lengths[2], self.lengths[3])
        additional_bend2 = self._bend1(self.lengths[2], self.lengths[3], length_elbow_EE)
        if (additional_bend2 * wrist_bend) > 0:
            additional_bend2 *= -1

        try :
            bend1 = self._bend1(self.lengths[1], length_elbow_EE, distance_from_shoulder)
            bend2 = self._bend2(self.lengths[1], length_elbow_EE, distance_from_shoulder)
        except :
            raise ValueError(f'{point} is not within workspace of robot! Due to arm lengths')

        angles_up = RobotAngles((heading, elevation + bend1, bend2 + additional_bend2, wrist_bend))
        angles_down = RobotAngles((heading, elevation - bend1, -bend2 + additional_bend2, wrist_bend))

        if self.domains.contains(angles_up) :
            return angles_up
        elif self.domains.contains(angles_down) :
            return angles_down
        else:
            raise ValueError(f'{point} is not within workspace of robot! Due to angle limits')

    def inverse(self, point : Point) -> RobotAngles:

        point_from_center = point - self.position
        heading = math.atan2(point_from_center.y, point_from_center.x)

        # shoulder is the point at which the robot hinges up
        shoulder_from_center = Point(math.cos(heading) * self.lengths[0], math.sin(heading) * self.lengths[0], 0)
        point_from_shoulder = point_from_center - shoulder_from_center

        distance_from_shoulder = point_from_shoulder.distance()
        
        max_distance_from_shoulder = sum(self.lengths[1:])

        wrist_bend = math.acos(distance_from_shoulder/max_distance_from_shoulder)
    
        angles1 = self.inverse_bend(point, -wrist_bend)
        angles2 = self.inverse_bend(point, wrist_bend)

        return max([angles1, angles2], key=self.domains.margin_relative)

    def _max_dist_from_shoulder(self):
        return sum(self.lengths[1:])

    def _max_dist_elbow(self):
        return sum(self.lengths[2:])

    def _min_dist_elbow(self):
        l_upper = self._cosine_rule_a_length(math.pi - self.domains[3].upper, self.lengths[2], self.lengths[3])
        l_lower = self._cosine_rule_a_length(math.pi - self.domains[3].lower, self.lengths[2], self.lengths[3])
        return max(l_lower, l_upper)

    def inverses(self, point : Point) -> RobotAngles:
        point_from_center = point - self.position
        heading = math.atan2(point_from_center.y, point_from_center.x)        
        shoulder_from_center = Point(math.cos(heading) * self.lengths[0], math.sin(heading) * self.lengths[0], 0)
        point_from_shoulder = point_from_center - shoulder_from_center
        
        distance_from_shoulder = point_from_shoulder.distance()
        planar_distance_from_shoulder = point_from_shoulder.no_z().distance()
        elevation = math.atan2(point_from_shoulder.z, planar_distance_from_shoulder)

        angles_shoulder = RobotAngles((heading, elevation, 0., 0.))

        distance_elbow = self._min_dist_elbow() + distance_from_shoulder/self._max_dist_from_shoulder() * (self._max_dist_elbow() - self._min_dist_elbow())
        try:
            angles_wrist = RobotAngles((0.,0., self._bend1(self.lengths[2], self.lengths[3], distance_elbow), self._bend2(self.lengths[2], self.lengths[3], distance_elbow)))
            angles_elbow = RobotAngles((0., self._bend1(self.lengths[1], distance_elbow, distance_from_shoulder), self._bend2(self.lengths[1], distance_elbow, distance_from_shoulder), 0.))
        except ValueError:
            return []

        angless = [angles_shoulder + angles_elbow*i + angles_wrist*j for i in (-1,1) for j in (-1,1)]
        sorted_angless = sorted(angless, reverse=True, key=self.domains.margin_relative)
        
        while len(sorted_angless) > 0:
            if not self.domains.contains(sorted_angless[-1]):
                sorted_angless.pop()
            else:
                break

        return sorted_angless
    
   
    def inverses_unfiltered(self, point : Point) -> RobotAngles:
        point_from_center = point - self.position
        heading = math.atan2(point_from_center.y, point_from_center.x)        
        shoulder_from_center = Point(math.cos(heading) * self.lengths[0], math.sin(heading) * self.lengths[0], 0)
        point_from_shoulder = point_from_center - shoulder_from_center
        
        distance_from_shoulder = point_from_shoulder.distance()
        planar_distance_from_shoulder = point_from_shoulder.no_z().distance()
        elevation = math.atan2(point_from_shoulder.z, planar_distance_from_shoulder)

        angles_shoulder = RobotAngles((heading, elevation, 0., 0.))

        distance_elbow = self._min_dist_elbow() + distance_from_shoulder/self._max_dist_from_shoulder() * (self._max_dist_elbow() - self._min_dist_elbow())

        angles_wrist = RobotAngles((0.,0., self._bend1(self.lengths[2], self.lengths[3], distance_elbow), self._bend2(self.lengths[2], self.lengths[3], distance_elbow)))

        angles_elbow = RobotAngles((0., self._bend1(self.lengths[1], distance_elbow, distance_from_shoulder), self._bend2(self.lengths[1], distance_elbow, distance_from_shoulder), 0.))

        angless = [angles_shoulder + angles_elbow*i + angles_wrist*j for i in (-1,1) for j in (-1,1)]
        sorted_angless = sorted(angless, reverse=True, key=self.domains.margin_relative)

        return sorted_angless
    
    def jacobian(self, angles):
        d0, d1, d2, d3 = angles
        l0, l1, l2, l3 = self.lengths

        if type(d0) is sympy.Symbol:
            cos = sympy.cos
            sin = sympy.sin
        elif type(d0) is np.ndarray:
            cos = np.cos
            sin = np.sin

        else:
            cos = math.cos
            sin = math.sin

        c0 = cos(d0)
        c1 = cos(d1)
        c12 = cos(d1 + d2)
        c123 = cos(d1 + d2 + d3)

        s0 = sin(d0)
        s1 = sin(d1)
        s12 = sin(d1 + d2)
        s123 = sin(d1 + d2 + d3)

        x = c0 * (l0 + l1 * c1 + l2 * c12 + l3 * c123)
        y = s0 * (l0 + l1 * c1 + l2 * c12 + l3 * c123)
        z = l1 * s1 + l2 * s12 + l3 * s123
        psi = d0
        phi = 0
        theta = d1 + d2 + d3

        dxdd0 = -s0 * (l0 + l1 * c1 + l2 * c12 + l3 * c123)
        dxdd1 = c0 * (-l1 * s1 - l2 * s12 - l3 * s123)
        dxdd2 = c0 * (         - l2 * s12 - l3 * s123)
        dxdd3 = c0 * (                    - l3 * s123)

        dydd0 = c0 * (l0 + l1 * c1 + l2 * c12 + l3 * c123)
        dydd1 = s0 * (-l1 * s1 - l2 * s12 - l3 * s123)
        dydd2 = s0 * (         - l2 * s12 - l3 * s123)
        dydd3 = s0 * (                    - l3 * s123)

        dzdd0 = 0
        dzdd1 = l1 * c1 + l2 * c12 + l3 * c123
        dzdd2 =           l2 * c12 + l3 * c123
        dzdd3 =                      l3 * c123

        dthetadd0 = 0
        dthetadd1 = 1
        dthetadd2 = 1
        dthetadd3 = 1

        dphidd0 = 0
        dphidd1 = 0
        dphidd2 = 0
        dphidd3 = 0

        dpsidd0 = 1
        dpsidd1 = 0
        dpsidd2 = 0
        dpsidd3 = 0


        return np.array([[dxdd0, dxdd1, dxdd2, dxdd3],
                         [dydd0, dydd1, dydd2, dydd3],
                         [dzdd0, dzdd1, dzdd2, dzdd3],
                         [dthetadd0, dthetadd1, dthetadd2, dthetadd3],
                         [dphidd0, dphidd1, dphidd2, dphidd3],
                         [dpsidd0, dpsidd1, dpsidd2, dpsidd3],])

    def jacobian_square(self, angles : RobotAngles):
        return self.jacobian(angles)[:-2,:]

    def joint_velocity(self, velocity : Point, angles : RobotAngles) :
        jacobian = self.jacobian_square(angles)

        dxdt = np.array([[velocity.x],
                         [velocity.y],
                         [velocity.z],
                         [0]])
        
        # print(f'{jacobian} \n ---- \n {dxdt}')

        dqdt = np.linalg.solve(jacobian, dxdt)

        angles_velocity = RobotAngles(dqdt.reshape(-1).tolist())
        # print(angles_velocity)
        
        return angles_velocity

if __name__ == '__main__':
    robot = Robot.create_real()
    # robot_symbolic = Robot.create_symbolic()
    # print(robot_symbolic.forward(RobotAngles.create_symbolic()))
    # print(robot_symbolic.jacobian(RobotAngles.create_symbolic()))
    # print(robot.jacobian(tuple(sympy.symbols(['d0', 'd1', 'd2', 'd3']))))
    point = Point(x=0.00359319205, y=0.00359319205, z=0.19425089272)
    #point = Point(0.2,0.1,0.1)

    print(robot.inverses(point))

    def q2b():
        def tryPoint(point : Point) :
            try:
                print(f'{point} reachable with $\delta$ = {robot.inverse(point)}')
            except ValueError as error:
                print(f'{point} not reachable: {error}')

        tryPoint(Point(0.1, 0.1, 0.1))
        tryPoint(Point(0.2, 0.1, 0.3))
        tryPoint(Point(0.0, 0.0, 0.3))
        tryPoint(Point(0.0, 0.0, 0.7))


    q2b()