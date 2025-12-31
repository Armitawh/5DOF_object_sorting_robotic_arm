from picamera import PiCamera
from time import sleep

camera = PiCamera()

#camera.resolution = (2592, 1944)
#camera.framerate = 15
camera.start_preview()
camera.rotation = 180
for i in range(1):
    sleep(2)
    camera.capture('/home/luna/Desktop/Pics/image%s.jpg' % i)
