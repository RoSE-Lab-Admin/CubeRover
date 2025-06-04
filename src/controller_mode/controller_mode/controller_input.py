import os

os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = '1' #hides welcome prompt
import pygame

os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = '1' #allows events to be updated when window not in focus

class ControllerReader:
    def __init__(self): #initialize
        pygame.init() # initializes pygame class
        pygame.joystick.init() #initialize joystick class
        self.controller = None
        self.found_Message = False

    def _string_(self):
        return f"The {self.stick} is in position {self.pos}"

#functions
    def connect(self):
        pygame.event.pump()
        for event in pygame.event.get():
            if event.type == pygame. JOYDEVICEADDED:
                print("Controller Connected!")
                self.controller = pygame.joystick.Joystick(0) #only registers the first on connected
            return

        if pygame.joystick.get_count() == 0:
            if not self.found_Message:
                print("No controller found... Waiting...")
                self.found_Message = True
            self.controller = None
            return

    def get_input(self): #read and returns controller input
        #assigning axis'
        if self.controller is None:
            return None
        
        #left_Xaxis, left_Yaxis, right_Xaxis, right_Yaxis, left_Trig, right_Trig = [self.controller.get_axis(i) for i in range (self.controller.get_numaxes())]
        pygame.event.pump() #update 

        for event in pygame.event.get():
            if event.type == pygame.JOYDEVICEREMOVED:
                print("Controller Disconnected...")
                self.controller = None
                return
            
        #pull joystick values
        right_Xaxis = self.controller.get_axis(4) #rX is on axis 5
        right_Yaxis = self.controller.get_axis(5) #rY is on axis 2
        left_Trig = self.controller.get_axis(2) #lT is on axis 4
        right_Trig = self.controller.get_axis(1) #5
        x_button = self.controller.get_button(2) #TODO: check to see if this is correct

        return [right_Xaxis, right_Yaxis, left_Trig, right_Trig, x_button]
            

    def close(self): #closes program
        pygame.quit()








    

    
