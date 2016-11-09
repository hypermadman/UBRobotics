#import packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
from hampy import detect_markers
import os

#init camera 
camera = PiCamera()
camera.resolution = (1280,720)
camera.framerate = 32
camera.exposure_mode = 'sports'
rawCapture = PiRGBArray(camera, size=(1280,720))

#camera warm up 
time.sleep(2.5)

# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        # grab the raw NumPy array representing the image, then initialize the timestamp
        # and occupied/unoccupied text
        image = frame.array

	# Detect Markers
	markers = detect_markers(image)

	os.system('clear')

	# Draw contour and log positions
	for m in markers:
		print 'Found marker {} at {}'.format(m.id, m.center)
		m.draw_contour(image)

        # show the frame
        cv2.imshow("Markers", image)
        key = cv2.waitKey(1) & 0xFF

        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)

        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
                break
