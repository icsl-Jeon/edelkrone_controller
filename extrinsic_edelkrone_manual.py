"""
This script calibrate the extrinsic parameters.


"""
import math
import signal
import sys
import keyboard
import numpy as np
import requests
import json
import time
import pyzed.sl as sl

httpURL = "http://127.0.0.1:32222/v1"


# ---------------------------------------------------------------------------
def keyposeStoreNumeric(linkID, pan, tilt, slide, index, verbose=True):

  # WARNING
  # =======
  # When storing a keypose by entering specific encoder values, you must
  # specify *all* axis values for the particular bundle. If you miss even one,
  # this command returns an error.

  # ...
  url = "%s/bundle/%s" % (httpURL, linkID)
  headers = {"Content-Type": "application/json"}
  payload = {
    "command": "keyposeStoreWithNumericData",
    "index": index,
    "headPan": pan,
    "headTilt": tilt,
    "slide": slide
  }

  # ...
  response = requests.request("POST", url, headers=headers, json=payload)
  response_json = json.loads(response.text)
  if verbose:
    print(json.dumps(response_json, indent=2))

# ---------------------------------------------------------------------------
def keyposeMoveFixedDuration(linkID, duration, index, accel, verbose = True):

  # WARNING
  # =======
  # With this type of fixed duration command, our motion control systems
  # can't guarantee that the motion will be completed with the given duration
  # if the given duration is shorter than what it can be achieved with %100
  # speed.
  #
  # If the given duration is shorter than what it can be achieved with %100
  # speed, system moves with %100 speed with the given acceleration.

  # ...
  url = "%s/bundle/%s" % (httpURL, linkID)
  headers = {"Content-Type": "application/json"}
  payload = {
    "command": "keyposeMoveFixedDuration",
    "index": index,
    "duration": duration,
    "acceleration": accel
  }

  # ...
  response = requests.request("POST", url, headers=headers, json=payload)
  response_json = json.loads(response.text)

  if verbose:
      print(json.dumps(response_json, indent=2))

# ---------------------------------------------------------------------------
def bundleStatus(linkID, verbose=True):

  # ...
  url = "%s/bundle/%s/status" % (httpURL, linkID)
  headers = {"Content-Type": "application/json"}
  payload = {}

  # ...
  response = requests.request(
    "GET", url, headers=headers, json=payload, timeout=1.0
  )
  response_json = json.loads(response.text)
  if verbose:
    print(json.dumps(response_json, indent=2))
  return response_json


if __name__ == "__main__":

    # 1. Opening Edelkrone
    aimIndex = 0
    linkID = "2065349D5853"

    acceleration = 0.5

    status = bundleStatus(linkID, False)
    if status["result"] == 'ok':
        status = bundleStatus(linkID, False)
        curSlide = float(status["data"]["readings"]["slide"])
        curPan = float(status["data"]["readings"]["headPan"])
        curTilt = float(status["data"]["readings"]["headTilt"])
        moveDuration_sec = abs(curSlide)

        print("For {} sec, moving toward the reference slide,pan,tilt (0 cm,90 deg,0 deg)...".format(moveDuration_sec))
        keyposeStoreNumeric(linkID, 90, 0, 0, aimIndex,False)
        keyposeMoveFixedDuration(linkID, moveDuration_sec, aimIndex, acceleration,False)
        time.sleep(moveDuration_sec + 2.0)
        print("Set to zero position")
    else:
        print("Communication with edelkrone failed. Exiting program")
        exit()

    prevSlide = 0.0
    prevPan = 90
    prevTilt = 0
    diff_thres = [0.5, 1, 1]  # thresholds for moving detection for slide / pan / tilt

    # 2. After moving, turn on zed camera
    zed = sl.Camera()

    init_parameters = sl.InitParameters()
    init_parameters.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD # prefer ENU system.
    init_parameters.camera_resolution = sl.RESOLUTION.HD1080
    init_parameters.camera_fps = 30
    init_parameters.coordinate_units = sl.UNIT.METER
    err = zed.open(init_parameters)

    positional_tracking_parameters = sl.PositionalTrackingParameters()
    zed.enable_positional_tracking(positional_tracking_parameters)

    runtime = sl.RuntimeParameters()
    camera_pose = sl.Pose()
    py_translation = sl.Translation()
    pose_data = sl.Transform()

    n_data_count = 0
    data_list = []


    while True:
        status = bundleStatus(linkID, False)
        if status["result"] == 'ok':
            status = bundleStatus(linkID, False)
            curSlide = float(status["data"]["readings"]["slide"])
            curPan = float(status["data"]["readings"]["headPan"])
            curTilt = float(status["data"]["readings"]["headTilt"])

            # read edelkrone state

            # ZED retrieving
            text_translation = ""
            text_rotation = ""
            if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
                tracking_state = zed.get_position(camera_pose)
                if tracking_state == sl.POSITIONAL_TRACKING_STATE.OK:
                    rotation = camera_pose.get_rotation_vector()
                    translation = camera_pose.get_translation(py_translation)
                    text_rotation = str((round(rotation[0], 2), round(rotation[1], 2), round(rotation[2], 2)))
                    text_translation = str((round(translation.get()[0], 2), round(translation.get()[1], 2), round(translation.get()[2], 2)))
                    pose_data = camera_pose.pose_data(sl.Transform())
                    print("zed translation (ENU coordinate): {} ".format(text_translation))

                    # save data ?
                    if abs(curSlide - prevSlide) > diff_thres[0] or abs(curPan - prevPan) > diff_thres[1] or \
                            abs(curTilt - prevTilt) > diff_thres[2]:
                        prevSlide = curSlide
                        prevPan = curPan
                        prevTilt = curTilt
                        print("="*20)
                        print("moving detected. Saving this state")
                        print("zed translation (ENU coordinate): {} ".format(text_translation))
                        print("slide={:.3f} [cm] / pan={:.3f} [deg] / tilt={:.3f} [deg]".format(curSlide, curPan,
                                                                                                curTilt))

                        # save list
                        data_list.append([curSlide * 0.01, curPan * math.pi / 180.0, curTilt * math.pi / 180.0] +
                                         list(pose_data.m.flatten("C")))

                else:
                    print("zed tracking failed.")


        if keyboard.is_pressed("q"):
            print("  pressed.  Exiting calibration process")
            break

        time.sleep(0.03)

    np.savetxt("calibration_result.txt", np.asarray(data_list))

