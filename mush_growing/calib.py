# three point calibration code

import numpy as np

# snapshot pos (기준이 되는 좌표)
org = np.array([[-0.280, -0.032, 0.500]])

# instances pos (인식된 객체들의 3D 좌표)
obj1 = np.array([0.117,  0.023,  0.296])
obj2 = np.array([-0.008,  0.033,  0.305])
obj3 = np.array([0.085, -0.040,  0.297])
obj4 = np.array([-0.155,   0.004,  0.290])

# teaching point (위의 객체들을 잡을 때 로봇의 위치)
pick1 = np.array([-0.31742713616983914, 0.11588656708091204, 0.37020696050405216])
pick2 = np.array([-0.3085578172508127, -0.006058533497230615, 0.3673325369894325])
pick3 = np.array([-0.38260471991282474, 0.08274379985702009, 0.36490319825755047])
pick4 = np.array([-0.336652811201147, -0.14947675080906683, 0.3707981819865231])

# XYZ 4 point calib
one = np.ones((4,1))
a = np.vstack((obj1, obj2, obj3, obj4))
a = np.hstack((a, one))
org = np.repeat(org, 4, 0)
b = np.vstack((pick1, pick2, pick3, pick4))
b = b - org

# # XY 3 point calib
# one = np.ones((3,1))
# a = np.vstack((obj1[0:2], obj2[0:2], obj3[0:2]))
# a = np.hstack((a, one))
# org = np.repeat(org[:, 0:2], 3, 0)
# b = np.vstack((pick1[0:2], pick2[0:2], pick3[0:2]))
# b = b - org
# d = [pick1[2]-org[0,2] + obj1[2], pick2[2]-org[0,2] + obj2[2], pick3[2]-org[0,2] + obj3[2]]
# d = np.mean(d)
# d = [-1, d]
# print(d)

c = np.linalg.inv(a) @ b
print(list(c)) # xy 보정 계수 (ax + by + cz + d)