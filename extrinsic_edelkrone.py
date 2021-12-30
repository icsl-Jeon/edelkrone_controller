"""
This script calibrate the extrinsic parameters.


"""


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
def keyposeMoveFixedDuration(linkID, duration, index, accel):

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
    zed = sl.Camera()
    aimIndex = 0
    linkID = "2065349D5853"
    # startSlide = float(status["data"]["readings"]["slide"])
    # startPan = float(status["data"]["readings"]["headPan"])
    # startTilt = float(status["data"]["readings"]["headTilt"])

    while True:
        status = bundleStatus(linkID, False)
        if status["result"] == 'ok':
            startSlide = float(status["data"]["readings"]["slide"])
            startPan = float(status["data"]["readings"]["headPan"])
            startTilt = float(status["data"]["readings"]["headTilt"])

            keyposeStoreNumeric(linkID, startPan, startTilt, startSlide, aimIndex, False)
            time.sleep(0.250)
            print("slide={:.3f} [cm] / pan={:.3f} [deg] / tilt={:.3f} [deg]".format(startSlide,startPan,startTilt))

        else:
            print("connection problem !")
            time.sleep(0.250)



