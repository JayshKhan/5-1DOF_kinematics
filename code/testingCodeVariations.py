''' File For Testing Multiple codes'''
import math

import numpy as np
# trajectory Generation Using RoboticsToolBox
import roboticstoolbox as rtb
from roboticstoolbox import DHRobot, RevoluteDH

'''
self.dhs = [
            {'alpha': 0, 'a': 0, 'd': 0},
            {'alpha': math.pi / 2, 'a': 0, 'd': 0},
            {'alpha': math.pi, 'a': self.a1, 'd': 0},
            {'alpha': 0, 'a': self.a2, 'd': 0},
            {'alpha': 0, 'a': 0, 'd': 0},
        ]
'''
# Create a 4-DOF robot with standard DH parameters
robot = DHRobot([
    # turn Table Joint
    RevoluteDH(a=0, alpha=0, d=0),
    # Shoulder Joint
    RevoluteDH(a=0, alpha=math.pi / 2, d=0),
    # Elbow Joint
    RevoluteDH(a=10.5, alpha=math.pi, d=0),
    # Wrist Joint
    RevoluteDH(a=10, alpha=0, d=0),
    # Gripper Joint
    RevoluteDH(a=0, alpha=0, d=0)
])


# Define the trajectory
q0 = [0, 0, 0, 0]
qf = [math.pi / 2, math.pi, math.radians(120), math.pi / 2]
q = rtb.jtraj(q0, qf, 30)

# Plot the trajectory
# plt.plot(q.q)
# plt.show()
# print 90 instead of 9*10^2
print(np.round(arr,0) for arr in q.q)
print(np.rad2deg(q.q))
