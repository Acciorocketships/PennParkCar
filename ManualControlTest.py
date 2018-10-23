from threading import Thread
import picamera
import cv2

#file for i2c communication
from Message import Message

#file for joystick stuff
from ManualControl import *


#init camera
camera = picamera.PiCamera()
photoHeight = 540
camera.resolution = (16*photoHeight/9, photoHeight)

#counter for photos from trigger
photoCounter = 0

#set the joystick running on a thread
manual = ManualControl()
joystickThread = Thread(target=manual.run)
joystickThread.start()

#set up the message
message = Message(manual = True)

###
#Main Loop
###
while True:
    time.sleep(0.1)
    print(str(message.desHeading))
    print(str(message.desSpeed))
    message.desHeading = manual.servoAngle
    message.desSpeed = manual.velocity

    message.send()

   #trigger input
    if(manual.trigger == 1):
        print("Triggered")
        photoCounter += 1
        camera.capture('TriggerTest' + str(photoCounter) +'.jpg')
   










