import numpy as np
import time
from python_impl.robot import Robot, GripperState

bot = Robot(speed=400)

time.sleep(2)

bot.set_des_q_rad(bot.HOME + np.pi/180 * 20 * np.ones(4))
