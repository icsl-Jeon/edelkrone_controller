import math
import sys
import os
import threading
import pyzed.sl as sl
from signal import signal, SIGINT
from datetime import datetime
import time
from extrinsic_edelkrone_manual import keyposeStoreNumeric, keyposeMoveFixedDuration, bundleStatus
import cv2
import numpy as np


# define zed
zed = sl.Camera()
image_zed = sl.Mat()
image_ocv = np.array([])
camera_pose = sl.Pose() # cam = ENU (optical = x / up = z)
pose_mat = np.array([]) # 4x4 SE3 matrix

path_output = datetime.today().strftime("%Y%m%d%H%M%S")
mutex = []
is_img_received = False

# define edelkrone
aimIndex = 0
linkID = "2065349D5853"
acceleration = 0.5


def get_edelkrone_state():
    status = bundleStatus(linkID, False)
    if status["result"] == 'ok':
        curSlide = float(status["data"]["readings"]["slide"])
        curPan = float(status["data"]["readings"]["headPan"])
        curTilt = float(status["data"]["readings"]["headTilt"])
        moveDuration_sec = abs(curSlide)
        return np.asarray((curSlide, curPan, curTilt)) # cm / deg / deg
    else:
        print("edelkrone not connected. State returned zero.")
        return np.asarray((0, 0, 0))


def set_edelkrone_state(desired_state): # cm / deg / deg
    cur_state = get_edelkrone_state()
    slide_diff = abs(cur_state[0] - desired_state[0]) # v = 1 cm / s
    keyposeStoreNumeric(linkID, desired_state[1], desired_state[2], desired_state[0], aimIndex,False)
    keyposeMoveFixedDuration(linkID, slide_diff, aimIndex, acceleration,False)
    time.sleep(slide_diff + 0.1)


def camera_thread():
    global image_zed, is_img_received, image_ocv, pose_mat
    init_parameters = sl.InitParameters()
    init_parameters.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD # prefer ENU system.
    init_parameters.camera_resolution = sl.RESOLUTION.HD1080
    init_parameters.camera_fps = 30
    init_parameters.coordinate_units = sl.UNIT.METER
    positional_tracking_parameters = sl.PositionalTrackingParameters()

    # open zed camera with initial setting
    status = zed.open(init_parameters)
    if status != sl.ERROR_CODE.SUCCESS:
        print("camera opening:", repr(status))
        exit(1)
    zed.enable_positional_tracking(positional_tracking_parameters)
    image_zed = sl.Mat(zed.get_camera_information().camera_resolution.width,
                       zed.get_camera_information().camera_resolution.height,
                       sl.MAT_TYPE.U8_C4)

    # recording starts
    recording_param = sl.RecordingParameters(path_output+'/record.svo', sl.SVO_COMPRESSION_MODE.H264)
    err = zed.enable_recording(recording_param)
    while err != sl.ERROR_CODE.SUCCESS:
        print("SVO recording status = ", repr(err)," retrying svo recording init.")
        err = zed.enable_recording(recording_param)
        time.sleep(0.1)


    runtime = sl.RuntimeParameters()
    print("SVO is Recording to {0}.".format(path_output))
    frames_recorded = 0

    while True:
        if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
            # Retrieving image
            zed.retrieve_image(image_zed, sl.VIEW.LEFT)
            if not mutex.locked():
                mutex.acquire()
                image_ocv = image_zed.get_data()
                tracking_state = zed.get_position(camera_pose)
                pose_mat = camera_pose.pose_data(sl.Transform()).m
                is_img_received = True
                mutex.release()

            frames_recorded += 1
            # print("Frame count: " + str(frames_recorded), end="\r")
            time.sleep(0.01)


if __name__ == '__main__':
    # 1. Open Edelkrone and set to zero
    print("first, edelkrone moves to zero.")
    set_edelkrone_state(np.array([0.0,90.0,0.0]))
    n_keypoint = 10
    slide_set = np.linspace(10,-10,n_keypoint)
    pan_set = np.linspace(90+30, 90-30,n_keypoint)
    tilt_set = np.linspace(10,-10,n_keypoint)

    # 2. Run camera thread
    threads = []
    print("Initiate camera instance")
    t = threading.Thread(target=camera_thread)
    t.setDaemon(True) # this is need for force-join
    mutex = threading.Lock()
    t.start()
    time.sleep(1) # wait for camera to load
    os.makedirs(path_output)

    print("waiting camera open..")
    while not is_img_received:
        time.sleep(0.01)
    print("now start to move the camera !")

    data_list = [] # slide (m) / pan (rad) / tilt (rad) / flatten of pose in row-wise

    # 3. Capture dataset
    for n in range(n_keypoint):
        print("pose {0}..".format(n))
        # move the camera
        target_state = np.array([slide_set[n],pan_set[n],tilt_set[n]])
        set_edelkrone_state(target_state)

        # capture image
        mutex.acquire(blocking=True)
        capture_image = image_ocv.copy()
        mutex.release()
        if is_img_received:
            cv2.imwrite(path_output+'/image_'+str(n)+'.png',capture_image)
            cv2.imshow("captured image",capture_image)
            cv2.waitKey(1)
            print("captured image {0} th".format(n))

        # save data
        cur_state = get_edelkrone_state()
        print("slide={:.3f} [cm] / pan={:.3f} [deg] / tilt={:.3f} [deg]".
              format(cur_state[0], cur_state[1],cur_state[2]))
        print("zed translation [cm] in ENU = ({:.3f}, {:.3f}, {:.3f})".
              format(pose_mat[0,3]*100, pose_mat[1,3]*100, pose_mat[2,3]*100))
        data_list.append([cur_state[0] * 0.01, cur_state[1] * math.pi / 180.0, cur_state[2] * math.pi / 180.0] +
                         list(pose_mat.flatten("C")))

        time.sleep(1)

    # 4. Terminating
    print("main loop finished.")
    t.join(timeout=1)
    zed.disable_recording()
    zed.close()
    print('saving key pose data...')
    np.savetxt(path_output+"/calibration_result.txt", np.asarray(data_list))
    print("Exiting. ")
