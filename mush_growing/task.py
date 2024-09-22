import time
import numpy as np
import pandas as pd

## communication message
MSG_CAM_READY = 0
MSG_TRIGGER = 1
MSG_PROG_STOP = 2
MSG_DETECTED = 3
MSG_NOTHING = 4

THRES = 200 # 버섯 간의 최소 거리

def indy_task(conn):
    from indy_utils import indydcp_client as client

    # Calibration
    # xyz 변환 계수 (calib에서 계산된 c의 값을 여기에 입력)
    CALIXYZ = np.array([[-0.975998483, 0 , 0],
                    [0, 0.988629606,  0],
                    [0, 0,  -1.015693436],
                    [-0.032,  -0.070 , 0.160]]) # -0.070

    # Joint Absolute
    J_HOME = [0, 0, -90, 0, -90, 0]
    J_SNAPSHOT = [11.335529926395376, -11.571645184982685, -97.24561959259422, 0.00016316328898515926, -71.18390073871137, 101.33582389001286]

    # Joint Relative

    # Task Absolute
    T_SNAPSHOT = [0.450, -0.100, 0.400, 180, 0, 90]

    # Task Relative
    MOVE_UP = [0, 0, 0.100, 0, 0, 0] # 수직 위 방향으로 이동할 거리(100mm)

    ## Robot info
    robot_ip = "192.168.0.119" # indy 로봇의 IP
    robot_name = "NRMK-Indy7"
    indy = client.IndyDCPClient(robot_ip, robot_name)

    indy.connect()
    indy.set_joint_vel_level(1)
    indy.set_task_vel_level(1)

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
                print('Robot received data: ', cam_data[['x3d', 'y3d', 'z3d', 'dist']])
                df_list = cam_data
                order = df_list.sort_values('order', ascending=True).index
                print(order)
                for idx in order:
                    if df_list.loc[idx, 'dist'] > 300:
                        break
                    obj_pos = np.array(df_list.loc[idx, ['x3d', 'y3d', 'z3d']], dtype=float)
                    obj_pos = np.append(obj_pos, [1], axis=0)
                    dxyz = obj_pos @ CALIXYZ
                    if dxyz[2] < -0.150: # safety offset
                        print('dz error: ', dxyz[2])
                        dxyz[2] = -0.150
                    dxy = np.append(dxyz[0:2], [0, 0, 0, 0], axis=0) # 촬영 위치로부터 이동해야 할 xy 상대 task 좌표
                    dz = [0, 0, float(dxyz[2]) + 0.010, 0, 0, 0] # 촬영 위치로부터 이동해야 할 z 상대 task 좌표

                    # xy 상대 좌표 이동
                    indy.task_move_by(dxy)
                    indy.wait_for_move_finish()

                    # 버섯 위치로 접근
                    indy.task_move_by(dz)
                    indy.wait_for_move_finish()

                    # 버섯 수확
                    indy.task_move_by(MOVE_UP)
                    indy.wait_for_move_finish()

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
