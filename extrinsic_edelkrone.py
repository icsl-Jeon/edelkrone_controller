"""
This script calibrate the extrinsic parameters.


"""
import math

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

    moveDuration_sec = 5
    print("For {} sec, moving toward the reference slide,pan,tilt (0 cm,90 deg,0 deg)...".format(moveDuration_sec))
    keyposeStoreNumeric(linkID, 90, 0, 0, aimIndex)
    acceleration = 0.5
    keyposeMoveFixedDuration(linkID, moveDuration_sec, aimIndex, acceleration)
    time.sleep(moveDuration_sec + 2.0)
    print("Done.")
    pan = 90
    tilt = 0
    slide = 0

    # Full range motion limits
    panFullRange = 45.0  # in degrees
    tiltFullRange = 5.0  # in degrees
    slideFullRange = 10.0  # in centimeters
    step = 10
    panStep = panFullRange / step
    tiltStep = tiltFullRange / step
    slideStep = slideFullRange / step

    # 2. After moving, turn zed camera
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

    total_data = np.zeros((step,3+16))

    for x in range(0, step):
        # Get Edelkrone data
        status = bundleStatus(linkID, False)
        if status["result"] == 'ok':
            # desired state stored
            pan += panStep
            tilt += tiltStep
            slide += slideStep
            keyposeStoreNumeric(linkID, pan, tilt, slide, aimIndex)
            time.sleep(0.250)
            # execute
            keyposeMoveFixedDuration(linkID, 1.5, aimIndex, acceleration)
            time.sleep(1)

            # read state
            status = bundleStatus(linkID, False)
            curSlide = float(status["data"]["readings"]["slide"])
            curPan = float(status["data"]["readings"]["headPan"])
            curTilt = float(status["data"]["readings"]["headTilt"])

            time.sleep(0.250)
            print("slide={:.3f} [cm] / pan={:.3f} [deg] / tilt={:.3f} [deg]".format(curSlide, curPan, curTilt))

        else:
            print("connection problem !")
            time.sleep(0.250)

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

        # Save data
        total_data[x][0:3] = [curSlide*0.01, curPan*math.pi/180.0, curTilt*math.pi/180.0]
        total_data[x][3:] = pose_data.m.flatten("C") # row-major

    np.savetxt("calibration_result.txt",total_data)