from math_ops.Math_Ops import Math_Ops as M
from os import listdir
from os.path import isfile, join
from world.World import World
import numpy as np
import xml.etree.ElementTree as xmlp

class Slot_Engine():

    def __init__(self, world : World) -> None:
        self.world = world
        self.state_slot_number = 0
        self.state_slot_start_time = 0
        self.state_slot_start_angles = None
        self.state_init_zero = True

        # ------------- Parse slot behaviors

        dir = M.get_active_directory("/behaviors/slot/")

        common_dir = f"{dir}common/"
        files =  [(f,join(common_dir, f)) for f in listdir(common_dir) if isfile(join(common_dir, f)) and f.endswith(".xml")]
        robot_dir = f"{dir}r{world.robot.type}"
        files += [(f,join(robot_dir, f)) for f in listdir(robot_dir) if isfile(join(robot_dir, f)) and f.endswith(".xml")]

        self.behaviors = dict()
        self.descriptions = dict()
        self.auto_head_flags = dict()

        for fname, file in files:
            robot_xml_root = xmlp.parse(file).getroot()
            slots = []
            bname = fname[:-4] # remove extension ".xml"

            for xml_slot in robot_xml_root:
                assert xml_slot.tag == 'slot', f"Unexpected XML element in slot behavior {fname}: '{xml_slot.tag}'"
                indices, angles = [],[]
                
                for action in xml_slot:
                    indices.append(  int(action.attrib['id'])    )
                    angles.append( float(action.attrib['angle']) )

                delta_ms = float(xml_slot.attrib['delta']) * 1000
                assert delta_ms > 0, f"Invalid delta <=0 found in Slot Behavior {fname}"
                slots.append((delta_ms, indices, angles))

            assert bname not in self.behaviors, f"Found at least 2 slot behaviors with same name: {fname}"

            self.descriptions[bname] = robot_xml_root.attrib["description"] if "description" in robot_xml_root.attrib else bname
            self.auto_head_flags[bname] = (robot_xml_root.attrib["auto_head"] == "1")
            self.behaviors[bname] = slots


    def get_behaviors_callbacks(self):
        ''' 
        Returns callbacks for each slot behavior (used internally) 

        Implementation note:
        --------------------
        Using dummy default parameters because lambda expression will remember the scope and var name.
        In the loop, the scope does not change, nor does the var name.
        However, default parameters are evaluated when the lambda is defined.
        '''
        return {key: (self.descriptions[key],self.auto_head_flags[key],
                lambda reset,key=key: self.execute(key,reset), lambda key=key: self.is_ready(key)) for key in self.behaviors}


    def is_ready(self,name) -> bool:
        return True


    def reset(self, name):
        ''' Initialize/Reset slot behavior '''

        self.state_slot_number = 0
        self.state_slot_start_time_ms = self.world.time_local_ms
        self.state_slot_start_angles = np.copy(self.world.robot.joints_position)
        assert name in self.behaviors, f"Requested slot behavior does not exist: {name}"


    def execute(self,name,reset) -> bool:
        ''' Execute one step '''

        if reset: self.reset(name)

        elapsed_ms = self.world.time_local_ms - self.state_slot_start_time_ms
        delta_ms, indices, angles = self.behaviors[name][self.state_slot_number]

        # Check slot progression
        if elapsed_ms >= delta_ms:
            self.state_slot_start_angles[indices] = angles #update start angles based on last target
             
            # Prevent 2 rare scenarios:
            # 1 - this function is called after the behavior is finished & reset==False
            # 2 - we are in the last slot, syncmode is not active, and we lost the last step
            if self.state_slot_number+1 == len(self.behaviors[name]):
                return True # So, the return indicates a finished behavior until a reset is sent via the arguments

            self.state_slot_number += 1
            elapsed_ms = 0
            self.state_slot_start_time_ms = self.world.time_local_ms
            delta_ms, indices, angles = self.behaviors[name][self.state_slot_number]

        # Execute 
        progress = (elapsed_ms+20) / delta_ms
        target = (angles - self.state_slot_start_angles[indices]) * progress + self.state_slot_start_angles[indices]
        self.world.robot.set_joints_target_position_direct(indices,target,False)

        # Return True if finished (this is the last step)
        return bool(elapsed_ms+20 >= delta_ms and self.state_slot_number + 1 == len(self.behaviors[name])) # true if next step (now+20ms) is out of bounds
