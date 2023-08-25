import serial 
import numpy as np 
from enum import Enum

class GripperState(Enum):
    Closed = 0
    Open = 1

class Robot(object):

    def __init__(self, l: list = [1, 1, 1, 1],
                 port: str = '/dev/ttyUSB0', 
                 baud: int = 9600,
                 speed: int = 750) -> None:
        """ Bundles the robot properties and setter and getter functions
            @param: port The string that defines the serial port
            @param: l    The list of link lengths
        """

        # Home position
        self.HOME = np.pi / 180.0 * np.array([45, 110, 180, 30])

        # Store the initialization parameters
        self.ser = serial.Serial(port, baud)
        self.l = l
        self.SPEED = speed

        # Setup the servo ticks to angle transformation
        self.min = np.array([500] * len(l))
        self.max = np.array([2500] * len(l))
        self.range = np.array([np.pi] * len(l))

        # Initialize
        self.set_des_q_rad(self.HOME)
        self.set_gripper(GripperState.Open)
        self.q = self.HOME
        self.gripper = GripperState.Open


    def RAD_2_TICKS(self, servo: int, rad: float) -> int:
        """ Function that uses the min, max and range to compute the 
            equivalent radians for a given number of ticks
            @param servo: Servo index
            @param   rad: Angle to be transformed
            
            @returns number of ticks equivalent to the rad angle
        """
        return (self.max[servo] - self.min[servo]) / self.range[servo] * rad

    def set_des_q_single_rad(self, servo: int, q: float):
        """ Set a single servo reference position
            @param servo: The servo index
            @param     q: The position in radians
        """
        assert(servo >= 0 and servo <= len(self.l))
        cmd = f"#{servo}P{int(self.RAD_2_TICKS(servo, * q) + self.min[servo]):04d}S{self.SPEED:03d}\r"
        self.ser.write(bytes(cmd, 'ascii'))
        self.q[servo] = q

    def set_des_q_single_deg(self, servo: int, q: float):
        """ Set a single servo reference position
            @param servo: The servo index
            @param     q: The position in degree
        """
        self.set_des_q_single_rad(servo, q * np.pi / 180.0)

    def set_des_q_rad(self, q: np.ndarray):
        """ Set all servo reference positions
            @param     q: np array of the positions in radians
        """
        assert(len(q) == len(self.l))
        cmd = ""
        for i in range(len(q)):
            cmd += f"#{i}P{int(self.RAD_2_TICKS(i, q[i]) + self.min[i]):04d}S{self.SPEED:03d}"
        cmd += "\r"
        print(cmd)
        self.ser.write(bytes(cmd,'ascii'))
        self.q = q

    def set_des_q_deg(self, q: np.ndarray):
        """ Set all servo reference positions
            @param q: np array of the positions in degree
        """
        self.set_des_q_rad(np.pi / 180.0 * q)

    def set_gripper(self, state: GripperState):
        """ Set the currently desired gripper state
            @param state: Currently desired gripper state (Open or Closed)
        """
        if state == GripperState.Open:
            cmd = f"#4P1300S{self.SPEED:03d}\r"
            self.ser.write(bytes(cmd, 'ascii'))
            self.gripper = GripperState.Open
        elif state == GripperState.Closed:
            cmd = f"#4P2500S{self.SPEED:03d}\r"
            self.ser.write(bytes(cmd, 'ascii'))
            self.gripper = GripperState.Closed

    def get_q(self):
        return self.q
    
    def get_gripper_State(self):
        return self.gripper