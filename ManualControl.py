import pygame
import time

class ManualControl:
   
    def __init__(self):
        self.servoAngle = 0
        self.velocity = 0
        self.trigger = 0 
        #init joystick
        pygame.init()
        pygame.joystick.init()
        self.joystick= pygame.joystick.Joystick(0)
        self.joystick.init()
    def run(self):
        toggle = False
        while True:
            time.sleep(0.1)
    
            self.servoAngle = int(30*self.joystick.get_axis(0))
            self.velocity = int(10*max(0,self.joystick.get_axis(1)))
            pygame.event.pump()

            #trigger input
            if(self.joystick.get_button(7) == 1 and toggle == False):
                self.trigger = 1
                toggle = True
            #reset button
            elif(self.joystick.get_button(7) == 0 and toggle == True):
                self.trigger = 0
                toggle = False
  
