from multiprocessing import Pipe, Process
import cam
import task
import argparse

def robot_task(robot_type, conn):
    if robot_type == 'indy7':
        task.indy_task(conn)

    if robot_type == 'fanuc':
        task.fanuc_task(conn)

def cam_task(prog, cam_type, conn):
    if cam_type == 'AK':
        cam.kinect(prog, conn)

    if cam_type == 'RS':
        cam.realsense(prog, conn)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="select mode")
    parser.add_argument(
        '-mode',
        default = 3,
        help = 'train only (1), image test (2), detect (3), auto label (4), screen shot(5)',
    )

    parser.add_argument(
        '-cam',
        default = 'AK',
        help = 'Azure Kinect DK (AK), Realsense D435 (RS)',
    )

    parser.add_argument(
        '-robot',
        default = 'N',
        help = 'indy7, fanuc, without Robot (N)',
    )

    args = parser.parse_args()
    prog = int(args.mode)
    cam_type = args.cam
    robot_type = args.robot

    print('prog: ', prog, ', cam: ', cam_type, ', robot: ', robot_type)

    if prog == 1:
        print('train start')

    elif prog == 2 or prog == 4:
        print('image test start')
        cam.img_test(prog)

    elif prog == 5 or robot_type == 'N':
        print('Saves snapshot from the camera. \n q to quit \n spacebar to save the snapshot')
        if cam_type == 'AK':
            cam.kinect(prog, None)

        if cam_type == 'RS':
            cam.realsense(prog, None)

    elif prog == 3:
        conn1, conn2 = Pipe()
        print('robot task start')
        p1 = Process(target=robot_task, args=(robot_type, conn1,)) # main task
        p2 = Process(target=cam_task, args=(prog, cam_type, conn2,)) # robot task
        p1.start()
        p2.start()
        p1.join()
        p2.join()