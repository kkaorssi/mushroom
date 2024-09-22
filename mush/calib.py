# three point calibration code

import numpy as np

# snapshot pos (기준이 되는 좌표)
org = np.array([[0.640, -0.065, 0.450]])

# instances pos (인식된 객체들의 3D 좌표)
obj1 = np.array([0.037, -0.002,  0.353]) #0
obj2 = np.array([-0.156,  0.004,  0.352]) #1
obj3 = np.array([0.143,  0.010,  0.347]) #6
obj4 = np.array([-0.057,  0.042,  0.354]) #8

# teaching point (위의 객체들을 잡을 때 로봇의 위치)
pick1 = np.array([0.5711738320663514, -0.1298515105810451, 0.26083636240831054])
pick2 = np.array([0.7553854368617479, -0.1226554080601237, 0.2632414413139845])
pick3 = np.array([0.46434170810223185, -0.1230068359532835, 0.2586930377251709])
pick4 = np.array([0.7136367774717953, -0.1323054419472112, 0.2670105098961571])

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
# a = a * 0.001
# org = np.repeat(org[:, 0:2], 3, 0)
# b = np.vstack((pick1[0:2], pick2[0:2], pick3[0:2]))
# b = b - org
d = [pick1[2]-org[0, 2] + obj1[2], pick2[2]-org[0, 2] + obj2[2], pick3[2]-org[0, 2] + obj3[2], pick4[2]-org[0, 2] + obj4[2]]
d = np.mean(d)
d = [-1, d]
print(d)

c = np.linalg.inv(a) @ b
print(list(c)) # xy 보정 계수 (ax + by + cz + d)