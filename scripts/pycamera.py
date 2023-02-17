import picamera
from picamera.array import PiRGBAnalysis
from picamera.color import Color
import cv2
import time

pi_camera = picamera.PiCamera(resolution='1024x768', framerate=25)

with pi_camera as camera:
    # fix the camera's white-balance gains
    #camera.awb_mode = 'off'
    #camera.awb_gains = (1.4, 1.5)

    camera.shutter_speed = 35000
    camera.iso = 400

    # allow the camera to warmup
    time.sleep(2)

    camera.exposure_mode = 'off' # lock gains and disable auto exposure

    # allow the camera to warmup
    time.sleep(2)

    # construct the analysis output and start recording data to it
    camera.start_recording(detector, 'rgb')
    try:
        while True:
            camera.wait_recording(1)
    finally:
        camera.stop_recording()
