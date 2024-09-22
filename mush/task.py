import time
import numpy as np
import pandas as pd

## communication message
MSG_CAM_READY = 0
MSG_TRIGGER = 1
MSG_PROG_STOP = 2
MSG_DETECTED = 3
MSG_NOTHING = 4

THRES = 150 # 버섯 간의 최소 거리

def indy_task(conn):
    import SLMP
    
    class gripper:
        def __init__(self, host, port):
            self.plc = SLMP.SLMP(host, port)
            self.trans = SLMP.transfer()
        
        def open(self):
            bw, points, w_data = self.trans.bool2bit([0, 1, 0, 1])
            self.plc.write(bw, 200, 'M', points, w_data)
            time.sleep(1)

        def close(self):
            bw, points, w_data = self.trans.bool2bit([1, 0, 1, 0])
            self.plc.write(bw, 200, 'M', points, w_data)
            time.sleep(1)

        def norm(self):
            bw, points, w_data = self.trans.bool2bit([1, 0, 0, 1])
            self.plc.write(bw, 200, 'M', points, w_data)

    host = '192.168.0.103'
    port = 9988
    plc = gripper(host, port)

    from indy_utils import indydcp_client as client

    # Calibration
    # xyz 변환 계수 (calib에서 계산된 c의 값을 여기에 입력)
    CALIXYZ = np.array([[-0.035011  ,  1.06973425,  0],
                    [0.98640077,  0.01811935, 0],
                    [-0.23470254, -0.2384869 , -1],
                    [0.07461828,  0.12601608, 0.014204332714724266]])

    # Joint Absolute
    J_HOME = [90, 0, -90, 0, -90, 0]
    J_SNAPSHOT = [119.32037605727801, -29.34634374193117, -31.454538392628624, -0.0, -119.19921270692306, 29.319844765240173]

    # Joint Relative

    # Task Absolute
    T_SNAPSHOT = [-0.095, 0.550, 0.635, 0, 180, 90]
    PRE_POS = [-0.3503389705431684, 0.3601978450100963, 0.530, 180, 0, 90]
    PLACE_POS = [-0.3503389705431684, 0.3601978450100963, 0.25775006543504225, 180, 0, 90]

    # Task Relative
    MOVE_UP = [0, 0, 0.100, 0, 0, 0] # 수직 위 방향으로 이동할 거리(100mm)

    ## Robot info
    robot_ip = "192.168.0.226" # indy 로봇의 IP
    robot_name = "NRMK-Indy7"
    indy = client.IndyDCPClient(robot_ip, robot_name)

    indy.connect()

    # 로봇의 현재 상태 확인
    status = indy.get_robot_status()
    print(status)
    if list(status.values()) == [1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0]:
        print('robot is ready')
    
        cam_data = conn.recv()
        print('Robot received data: ', cam_data)
        if cam_data == MSG_CAM_READY:
            print('cam is ready')

            indy.joint_move_to(J_SNAPSHOT)
            indy.wait_for_move_finish()
            time.sleep(0.5)

            conn.send(MSG_TRIGGER)
            cam_data = conn.recv()
            print('Robot received data: ', cam_data)
            
            if cam_data == MSG_DETECTED:
                cam_data = conn.recv()
                print('Robot received data: ', cam_data[['x3d', 'y3d', 'z3d']])
                df_list = cam_data
                order = df_list.sort_values('order', ascending=False).index
                print(order)
                for idx in order:
                    obj_pos = np.array(df_list.loc[idx, ['x3d', 'y3d', 'z3d']], dtype=float)
                    obj_pos = np.append(obj_pos, [1], axis=0)
                    dxyz = obj_pos @ CALIXYZ
                    if dxyz[2] < -0.313: # safety offset
                        print('dz error: ', dxyz[2])
                        dxyz[2] = -0.313
                    dxy = np.append(dxyz[0:2], [0, 0, 0, 0], axis=0) # 촬영 위치로부터 이동해야 할 xy 상대 task 좌표
                    dz = [0, 0, float(dxyz[2])+0.01, 0, 0, 0] # 촬영 위치로부터 이동해야 할 z 상대 task 좌표

                    print(dxyz)

                    # xy 상대 좌표 이동
                    indy.task_move_by(dxy)
                    indy.wait_for_move_finish()
                    plc.open()

                    # 버섯 위치로 접근
                    indy.task_move_by(dz)
                    indy.wait_for_move_finish()
                    plc.close()

                    # 버섯 수확
                    indy.task_move_by(MOVE_UP)
                    indy.wait_for_move_finish()

                    indy.task_move_to(PRE_POS)
                    indy.wait_for_move_finish()

                    indy.task_move_to(PLACE_POS)
                    indy.wait_for_move_finish()
                    plc.open()

                    indy.task_move_to(PRE_POS)
                    indy.wait_for_move_finish()
                    plc.norm()

                    indy.task_move_to(T_SNAPSHOT)
                    indy.wait_for_move_finish()

            indy.go_home()
            indy.wait_for_move_finish()

        else:
            print('please check cam')

    else:
        print('please check robot status')

    conn.send(MSG_PROG_STOP)
    print('Robot task stop')
    plc.plc.client.close()
    indy.disconnect()
    conn.close()


def fanuc_task(conn):
    cam_data = conn.recv()
    print('Robot received data: ', cam_data)
    time.sleep(3)
    # while True:
    for i in range(3):
        if i == 2:
            conn.send(MSG_PROG_STOP)
            continue
        conn.send(MSG_TRIGGER)
        cam_data = conn.recv()
        print('Robot received data: ', cam_data)
        if cam_data == MSG_DETECTED:
            cam_data = conn.recv()
            print('Robot received data: ', cam_data)
        time.sleep(5)
    print('Robot task stop')
    conn.close()
