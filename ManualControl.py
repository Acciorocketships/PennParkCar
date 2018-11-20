import pygame
import time

class ManualControl:
   
    def __init__(self):
        self.servoAngle = 0
        self.velocity = 0
        self.trigger = 0
        self.manual = 0
        #init joystick
        pygame.init()
        pygame.joystick.init()
        self.joystick= pygame.joystick.Joystick(0)
        self.joystick.init()
        self.toggleT = False
        self.toggleM = False

    def update(self):

        pygame.event.pump()

        #manual button
        if(self.joystick.get_button(9) == 1 and self.toggleM == False):
            self.manual = 1
            self.toggleM = True
        elif(self.joystick.get_button(9) == 0 and self.toggleM == False):
            self.manual = 0
            self.toggleM = False
        elif(self.joystick.get_button(9) == 1 and self.toggleM == True):
            self.manual = 0
            self.toggleM = False

        elif(self.joystick.get_button(9) == 0 and self.toggleM == True):
            self.manual = 1
            self.toggleM = True
        pygame.event.pump()

        self.servoAngle = int(30*self.joystick.get_axis(0))
        self.velocity = int(10*max(0,self.joystick.get_axis(1)))
        pygame.event.pump()

        #trigger input
        if(self.joystick.get_button(7) == 1 and self.toggleT == False):
            self.trigger = 1
            self.toggleT = True
        #reset button
        elif(self.joystick.get_button(7) == 0 and self.toggleT == True):
            self.trigger = 0
            self.toggleT = False
            
