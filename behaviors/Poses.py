''' 
Pose - angles in degrees for the specified joints
Note: toes positions are ignored by robots that have no toes 

Poses may control all joints or just a subgroup defined by the "indices" variable
'''

import numpy as np
from world.World import World


class Poses():
    def __init__(self, world : World) -> None:
        self.world = world
        self.tolerance = 0.05 # angle error tolerance to consider that behavior is finished

        '''
        Instruction to add new pose:
        1. add new entry to the following dictionary, using a unique behavior name
        2. that's it
        '''
        self.poses = {
            "Zero":(
                "Neutral pose, including head", # description
                False, # disable automatic head orientation
                np.array([0,1,2,3,4,5,6,7,8,9,10,11,12,13, 14, 15,16,17,18,19,20,21,22,23]), # indices
                np.array([0,0,0,0,0,0,0,0,0,0, 0, 0, 0, 0,-90,-90, 0, 0,90,90, 0, 0, 0, 0])  # values
            ),
            "Zero_Legacy":(
                "Neutral pose, including head, elbows cause collision (legacy)", # description
                False, # disable automatic head orientation
                np.array([0,1,2,3,4,5,6,7,8,9,10,11,12,13, 14, 15,16,17,18,19,20,21,22,23]), # indices
                np.array([0,0,0,0,0,0,0,0,0,0, 0, 0, 0, 0,-90,-90, 0, 0, 0, 0, 0, 0, 0, 0])  # values
            ),
            "Zero_Bent_Knees":(
                "Neutral pose, including head, bent knees", # description
                False, # disable automatic head orientation
                np.array([0,1,2,3,4,5,6,  7,  8,  9,10,11,12,13, 14, 15,16,17,18,19,20,21,22,23]), # indices
                np.array([0,0,0,0,0,0,30,30,-60,-60,30,30, 0, 0,-90,-90, 0, 0,90,90, 0, 0, 0, 0])  # values
            ),
            "Zero_Bent_Knees_Auto_Head":(
                "Neutral pose, automatic head, bent knees", # description
                True, # enable automatic head orientation
                np.array([2,3,4,5,6,  7,  8,  9,10,11,12,13, 14, 15,16,17,18,19,20,21,22,23]), # indices
                np.array([0,0,0,0,30,30,-60,-60,30,30, 0, 0,-90,-90, 0, 0,90,90, 0, 0, 0, 0])  # values
            ),
            "Fall_Back":(
                "Incline feet to fall back", # description
                True, # enable automatic head orientation
                np.array([ 10, 11]), # indices
                np.array([-20,-20])  # values
            ),
            "Fall_Front":(
                "Incline feet to fall forward", # description
                True, # enable automatic head orientation
                np.array([10,11]), # indices
                np.array([45,45])  # values
            ),
            "Fall_Left":(
                "Incline legs to fall to left", # description
                True, # enable automatic head orientation
                np.array([  4, 5]), # indices
                np.array([-20,20])  # values
            ),
            "Fall_Right":(
                "Incline legs to fall to right", # description
                True, # enable automatic head orientation
                np.array([ 4,  5]), # indices
                np.array([20,-20])  # values
            ),
        }

        # Remove toes if not robot 4
        if world.robot.type != 4:
            for key, val in self.poses.items():
                idxs = np.where(val[2] >= 22)[0] # search for joint 22 & 23
                if len(idxs) > 0:
                    self.poses[key] = (val[0], val[1], np.delete(val[2],idxs), np.delete(val[3],idxs)) # remove those joints


    def get_behaviors_callbacks(self):
        ''' 
        Returns callbacks for each pose behavior (used internally)
        
        Implementation note:
        --------------------
        Using dummy default parameters because lambda expression will remember the scope and var name.
        In the loop, the scope does not change, nor does the var name.
        However, default parameters are evaluated when the lambda is defined.
        '''
        return {key: (val[0], val[1], lambda reset, key=key: self.execute(key), lambda: True) for key, val in self.poses.items()}

    def execute(self,name) -> bool:
        _, _, indices, values = self.poses[name]
        remaining_steps = self.world.robot.set_joints_target_position_direct(indices,values,True,tolerance=self.tolerance)
        return bool(remaining_steps == -1)

