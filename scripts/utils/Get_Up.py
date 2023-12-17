from agent.Base_Agent import Base_Agent as Agent
from itertools import count
from scripts.commons.Script import Script
import numpy as np

'''
Objective:
----------
Fall and get up
'''

class Get_Up():
    def __init__(self, script:Script) -> None:
        self.script = script
        self.player : Agent = None

    def sync(self):
        r = self.player.world.robot
        self.player.scom.commit_and_send( r.get_command() )
        self.player.scom.receive()

    def execute(self):

        a = self.script.args        
        player = self.player = Agent(a.i, a.p, a.m, a.u, a.r, a.t) # Args: Server IP, Agent Port, Monitor Port, Uniform No., Robot Type, Team Name
        behavior = player.behavior
        r = player.world.robot

        player.scom.commit_beam((-3,0),0)
        print("\nPress ctrl+c to return.")

        for i in count():
            rnd = np.random.uniform(-6,6,r.no_of_joints)

            # Fall
            while r.loc_head_z > 0.3 and r.imu_torso_inclination < 50:
                if i < 4:
                    behavior.execute(["Fall_Front","Fall_Back","Fall_Left","Fall_Right"][i % 4]) # First, fall deterministically
                else:
                    r.joints_target_speed[:] = rnd # Second, fall randomly
                self.sync()

            # Get up
            behavior.execute_to_completion("Get_Up")
            behavior.execute_to_completion("Zero_Bent_Knees")      
