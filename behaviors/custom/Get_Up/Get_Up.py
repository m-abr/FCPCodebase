from agent.Base_Agent import Base_Agent
from collections import deque
import numpy as np

class Get_Up():

    def __init__(self, base_agent : Base_Agent) -> None:
        self.behavior = base_agent.behavior
        self.world = base_agent.world
        self.description = "Get Up using the most appropriate skills"
        self.auto_head = False
        self.MIN_HEIGHT = 0.3 # minimum value for the head's height
        self.MAX_INCLIN = 50  # maximum torso inclination in degrees
        self.STABILITY_THRESHOLD = 4

    def reset(self):
        self.state = 0
        self.gyro_queue = deque(maxlen=self.STABILITY_THRESHOLD)
        self.watchdog = 0 # when player has the shaking bug, it is never stable enough to get up

    def execute(self,reset):

        r = self.world.robot
        execute_sub_behavior = self.behavior.execute_sub_behavior
        
        if reset:
            self.reset()

        if self.state == 0: # State 0: go to pose "Zero"

            self.watchdog += 1
            self.gyro_queue.append( max(abs(r.gyro)) ) # log last STABILITY_THRESHOLD values

            # advance to next state if behavior is complete & robot is stable
            if (execute_sub_behavior("Zero",None) and len(self.gyro_queue) == self.STABILITY_THRESHOLD 
                and all(g < 10 for g in self.gyro_queue)) or self.watchdog > 100:

                # determine how to get up
                if r.acc[0] < -4 and abs(r.acc[1]) < 2 and abs(r.acc[2]) < 3:
                    execute_sub_behavior("Get_Up_Front", True) # reset behavior
                    self.state = 1
                elif r.acc[0] > 4 and abs(r.acc[1]) < 2 and abs(r.acc[2]) < 3:
                    execute_sub_behavior("Get_Up_Back", True) # reset behavior
                    self.state = 2
                elif r.acc[2] > 8: # fail-safe if vision is not up to date: if pose is 'Zero' and torso is upright, the robot is already up
                    return True
                else:
                    execute_sub_behavior("Flip", True) # reset behavior
                    self.state = 3

        elif self.state == 1:
            if execute_sub_behavior("Get_Up_Front", False):
                return True
        elif self.state == 2:
            if execute_sub_behavior("Get_Up_Back", False):
                return True
        elif self.state == 3:
            if execute_sub_behavior("Flip", False):
                self.reset()

        return False
        

    def is_ready(self):
        ''' Returns True if the Get Up behavior is ready (= robot is down) '''
        r = self.world.robot
        # check if z < 5 and acc magnitude > 8 and any visual indicator says we fell
        return r.acc[2] < 5 and np.dot(r.acc,r.acc) > 64 and (r.loc_head_z < self.MIN_HEIGHT or r.imu_torso_inclination > self.MAX_INCLIN)
