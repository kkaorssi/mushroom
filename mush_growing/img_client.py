import socket
import numpy as np
import cv2
import time
import pandas as pd
import os
import json
import subprocess

import detect

## communication message
MSG_CAM_READY = 0
MSG_TRIGGER = 1
MSG_PROG_STOP = 2
MSG_DETECTED = 3
MSG_NOTHING = 4

# 작업 경로
WORK_DIR = ''
VAR_OUTPUT_DIR = WORK_DIR + 'output' # 모델이 저장된 경로
VAR_TEST_DIR = WORK_DIR + 'test' # 테스트 이미지 경로
VAR_RES_DIR = WORK_DIR + 'result' # 실행 결과를 저장할 경로
VAR_TRAIN_DIR = WORK_DIR + 'train' # 실행 결과를 저장할 경로
VAR_LABEL_DIR = WORK_DIR + 'label' # 라벨링 결과를 저장할 경로
VAR_SNAP_DIR = WORK_DIR + 'data'

s_path = VAR_TEST_DIR
m_path = VAR_OUTPUT_DIR
r_path = VAR_RES_DIR
t_path = VAR_RES_DIR
l_path = VAR_LABEL_DIR
d_path = VAR_SNAP_DIR

# 서버 주소와 포트
SERVER_ADDRESS = '127.0.0.1'  # 서버의 IP 주소
SERVER_PORT = 5000

RELATIVE_PATH = 'net8.0'

# C# 서버 프로그램 실행
csharp_process = subprocess.Popen(
    os.path.join(RELATIVE_PATH, 'AzureKinectDK.exe'),  # C# 애플리케이션 경로
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE
)

def receive_exact(sock):
    """소켓에서 데이터의 크기의 이미지 데이터를 수신하는 함수"""
    size_bytes = sock.recv(4)
    size = int.from_bytes(size_bytes, byteorder='little')

    data = bytearray()
    while len(data) < size:
        packet = sock.recv(size - len(data))
        if not packet:
            return None
        data.extend(packet)
    return data

# 픽셀 좌표와 깊이 값을 받아 3D 포인트로 변환하는 함수
def pixel_to_point(pixel_x, pixel_y, depth_value, depth_intrinsics):
    fx = depth_intrinsics[0]  # Focal length in x
    fy = depth_intrinsics[1]  # Focal length in y
    cx = depth_intrinsics[2]  # Principal point x
    cy = depth_intrinsics[3]  # Principal point y
    
    # 카메라의 좌표계에서 Z는 깊이 값 (depth_value는 밀리미터 단위)
    z = depth_value / 1000.0  # 밀리미터를 미터로 변환
    x = (pixel_x - cx) * z / fx
    y = (pixel_y - cy) * z / fy
    
    return np.array([x, y, z])

def kinect(opt, conn):
    # Release 모드에서의 상대 경로 설정
    relative_path = os.path.join(RELATIVE_PATH, 'calibration_data.json')

    # 현재 작업 디렉토리에서 상대 경로로 파일 읽기
    with open(relative_path, 'r') as f:
        calibration_data = json.load(f)
    
    # 필요한 캘리브레이션 파라미터 추출
    depth_intrinsics = calibration_data['DepthCameraCalibration']['Intrinsics']['Parameters']

    # 컬러 이미지 크기 (3840x2160, BGRA 형식이므로 4채널)
    color_image_shape = (2160, 3840, 4)

    # 깊이 이미지 크기 (1024x1024, 단일 채널)
    depth_image_shape = (2160, 3840)
            
    # 서버에 연결할 때까지 대기
    while True:
        try:
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.connect((SERVER_ADDRESS, SERVER_PORT))
            print("서버에 연결되었습니다.")
            break  # 연결 성공 시 루프 종료
        except ConnectionRefusedError:
            print("서버에 연결할 수 없습니다. 다시 시도합니다...")
            time.sleep(2)  # 2초 대기 후 재시도

    if conn:
        conn.send(MSG_CAM_READY)
    
    # load detecting model
    VS = detect.Visualization(m_path)
    
    cv2.namedWindow('Azure Kinect DK Image', cv2.WINDOW_NORMAL)
    
    try:
        idx = 0
        while True:
            # Get the color image
            color_data = receive_exact(client_socket)
            if color_data is None:
                print("컬러 이미지 수신 실패")
                continue

            # Get the colored depth
            depth_data = receive_exact(client_socket)
            if depth_data is None:
                print("깊이 이미지 수신 실패")
                continue

            # 컬러 이미지를 NumPy 배열로 변환 (BGRA 형식)
            color_img = np.frombuffer(color_data, dtype=np.uint8).reshape(color_image_shape)

            # 깊이 이미지를 NumPy 배열로 변환 (16비트 단일 채널)
            color_depth = np.frombuffer(depth_data, dtype=np.uint16).reshape(depth_image_shape)

            # 컬러 이미지 출력 (OpenCV는 기본적으로 BGR을 사용하므로, BGRA에서 BGR로 변환)
            bgr_img = cv2.cvtColor(color_img, cv2.COLOR_BGRA2BGR)

            # 깊이 이미지 출력 (16비트 이미지를 8비트로 스케일링하여 시각화)
            depth_image_normalized = cv2.normalize(color_depth, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            
            stack_img = np.hstack((bgr_img[:,:,:3], color_depth))
            
            # Plot the image
            cv2.imshow('Azure Kinect DK Image', stack_img)
            
            key = cv2.waitKey(1) & 0xFF
            if conn:
                key = None
                if conn.poll():
                    robot_msg = conn.recv()
                    if robot_msg == MSG_TRIGGER:
                        key = ord(' ')
                    elif robot_msg == MSG_PROG_STOP:
                        key = ord('q')
                else:
                    continue
            
            # Press q key to stop
            if key == ord('q'):
                break

            elif key == ord(' '):
                # 촬영된 이미지와 인식 결과 저장
                print("Saving image ", idx)
                os.makedirs(d_path, exist_ok=True)
                
                fname = 'orgimg_' + str(idx) + '.jpg'
                out_1 = os.path.join(d_path, fname)
                cv2.imwrite(out_1, color_img)

                fname = 'orgdepth_' + str(idx) + '.jpg'
                out_2 = os.path.join(d_path, fname)
                cv2.imwrite(out_2, color_depth)

                if opt == 3:
                    img = color_img[:,:,0:3]
                    df_ins = VS.run_on_image(img, idx)
                    if len(df_ins) > 0:
                        for i in range(len(df_ins)):
                            [xp, yp] = np.array(df_ins.loc[i, ['X', 'Y']], dtype=int)
                            depth_value = color_depth[xp, yp]
                            [x3d, y3d, z3d] = pixel_to_point(xp, yp, depth_value, depth_intrinsics)
                            df_ins.loc[i, ['x3d', 'y3d', 'z3d']] = [x3d, y3d, z3d]
                        if conn:
                            conn.send(MSG_DETECTED)
                            conn.send(df_ins)
                    else:
                        if conn:
                            conn.send(MSG_NOTHING)

                idx += 1

    finally:
        client_socket.close()
        cv2.destroyAllWindows()
        if conn:
            print('connection close')
            conn.close()

def realsense(opt, conn):
    import pyrealsense2 as rs

    FRAME_WIDTH = 1280
    FRAME_HEIGHT = 720

    color_intrin = None
    depth_intrin = None
    depth_to_color_extrin = None

    pipeline = rs.pipeline()
    config = rs.config()
    dev = rs.device()
    config.enable_stream(rs.stream.depth, FRAME_WIDTH, FRAME_HEIGHT, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, FRAME_WIDTH, FRAME_HEIGHT, rs.format.bgr8, 30)

    profile = pipeline.start(config)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    align_to = rs.stream.color
    align = rs.align(align_to)

    if conn:
        conn.send(MSG_CAM_READY)

    # load detecting model
    VS = detect.Visualization(m_path)

    cv2.namedWindow('Realsense2 D435 Image', cv2.WINDOW_NORMAL)
    idx = 0
    while True:
         ### Aruco marker detecting start & camera setting
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        aligned_depth_frames = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not aligned_depth_frames or not color_frame: continue
        
        color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
        depth_intrin = aligned_depth_frames.profile.as_video_stream_profile().intrinsics
        depth_to_color_extrin = aligned_depth_frames.profile.get_extrinsics_to(color_frame.profile)
        depth_frame = aligned_depth_frames.get_data()

        depth_img = np.asanyarray(depth_frame) # depth 이미지
        color_img = np.asanyarray(color_frame.get_data()) # color 이미지

        colordepth = cv2.convertScaleAbs (depth_img, alpha=0.05)
        colordepth = cv2.applyColorMap(colordepth, cv2.COLORMAP_JET)

        stack_img = np.hstack((color_img, colordepth))

        # Plot the image
        cv2.imshow('Realsense2 D435 Image', stack_img)

        key = cv2.waitKey(1) & 0xFF
        if conn:
            key = None
            if conn.poll():
                robot_msg = conn.recv()
                if robot_msg == MSG_TRIGGER:
                    key = ord(' ')
                elif robot_msg == MSG_PROG_STOP:
                    key = ord('q')
            else:
                continue

        # Press q key to stop
        if key == ord('q'):
            break

        elif key == ord(' '):
            # 촬영된 이미지와 인식 결과 저장
            print("Saving image ", idx)
            os.makedirs(d_path, exist_ok=True)

            fname = 'orgimg_' + str(idx) + '.jpg'
            out_1 = os.path.join(d_path, fname)
            cv2.imwrite(out_1, color_img)

            fname = 'orgdepth_' + str(idx) + '.jpg'
            out_2 = os.path.join(d_path, fname)
            cv2.imwrite(out_2, depth_img)

            if opt == 3:
                img = color_img
                df_ins = VS.run_on_image(img, idx)
                if len(df_ins) > 0:
                    for i in range(len(df_ins)):
                        [xp, yp] = np.array(df_ins.loc[i, ['X', 'Y']], dtype=int)
                        dist = aligned_depth_frames.get_distance(xp, yp)
                        depth_point = rs.rs2_deproject_pixel_to_point(color_intrin, [xp, yp], dist*depth_scale)
                        [x3d, y3d, z3d] = rs.rs2_transform_point_to_point(depth_to_color_extrin, depth_point)
                        [x3d, y3d, z3d] = [pos*1000 for pos in [x3d, y3d, z3d]]
                        df_ins.loc[i, ['x3d', 'y3d', 'z3d']] = [x3d, y3d, z3d]
                    conn.send(MSG_DETECTED)
                    conn.send(df_ins)
                else:
                    conn.send(MSG_NOTHING)
            idx += 1

    print('Program Stop')
    pipeline.stop()
    if conn:
        conn.close()

def img_test(prog):
    from detectron2.data.detection_utils import read_image

    # load detecting model
    VS = detect.Visualization(m_path)

    if prog == 2:
        path = s_path

    if prog == 4:
        path = l_path

    fnames = os.listdir(path)
    idx = 0
    for fname in fnames:
        f_name, f_ext = os.path.splitext(fname)
        filename = os.path.join(path, fname)

        if (f_ext != '.jpg' and f_ext != '.JPEG'): continue

        print('img ', idx)
        print('filename: ', filename)
        img = read_image(filename, format='BGR')
        df_ins = VS.run_on_image(img, idx)
        
        if os.path.isdir(d_path):
            assert os.path.isdir(d_path), d_path
        else:
            os.makedirs(d_path, exist_ok=True)

        # 인식 결과를 excel로 저장(이미 같은 이름의 excel 파일이 있는 경우 에러 발생)
        if not os.path.exists(d_path + '/result.xlsx'):
            with pd.ExcelWriter(d_path + '/result.xlsx', mode='w', engine='openpyxl') as writer:
                df_ins.to_excel(writer, index=False, sheet_name=f_name)

        else:
            with pd.ExcelWriter(d_path + '/result.xlsx', mode='a', engine='openpyxl') as writer:
                df_ins.to_excel(writer, index=False, sheet_name=f_name)

        if prog == 4:
            detect.auto_label(img, fname, df_ins)

        idx += 1

    print('test done')