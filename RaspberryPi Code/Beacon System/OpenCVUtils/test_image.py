# This script will load an image from the camera using OpenCV, use it to check your camera is connected and setup correctly to work with OpenCV.


#import packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2

#init camera 
camera = PiCamera()
rawCapture = PiRGBArray(camera)

#camera warm up 
time.sleep(0.1)

#image grab 
camera.capture(rawCapture, format="bgr")
image = rawCapture.array

#Display on screen and wait for keypress
cv2.imshow("Image", image)
cv2.waitKey(0)
