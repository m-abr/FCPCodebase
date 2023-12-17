from agent.Base_Agent import Base_Agent
from behaviors.custom.Step.Step_Generator import Step_Generator
import numpy as np

class Step():

    def __init__(self, base_agent : Base_Agent) -> None:
        self.world = base_agent.world
        self.ik = base_agent.inv_kinematics
        self.description = "Step (Skill-Set-Primitive)"
        self.auto_head = True

        nao_specs = self.ik.NAO_SPECS
        self.leg_length = nao_specs[1] + nao_specs[3] # upper leg height + lower leg height

        feet_y_dev = nao_specs[0] * 1.2 # wider step
        sample_time = self.world.robot.STEPTIME
        max_ankle_z = nao_specs[5]

        # Initialize step generator with constants
        self.step_generator = Step_Generator(feet_y_dev, sample_time, max_ankle_z)


    def execute(self,reset, ts_per_step=7, z_span=0.03, z_max=0.8):

        lfy,lfz,rfy,rfz = self.step_generator.get_target_positions(reset, ts_per_step, z_span, self.leg_length * z_max)
 
        #----------------- Apply IK to each leg + Set joint targets
          
        # Left leg 
        indices, self.values_l, error_codes = self.ik.leg((0,lfy,lfz), (0,0,0), True, dynamic_pose=False)
        for i in error_codes:
            print(f"Joint {i} is out of range!" if i!=-1 else "Position is out of reach!")

        self.world.robot.set_joints_target_position_direct(indices, self.values_l)

        # Right leg
        indices, self.values_r, error_codes = self.ik.leg((0,rfy,rfz), (0,0,0), False, dynamic_pose=False)
        for i in error_codes:
            print(f"Joint {i} is out of range!" if i!=-1 else "Position is out of reach!")

        self.world.robot.set_joints_target_position_direct(indices, self.values_r)

        # ----------------- Fixed arms

        indices = [14,16,18,20]
        values  = np.array([-80,20,90,0])
        self.world.robot.set_joints_target_position_direct(indices,values)

        indices = [15,17,19,21]
        values  = np.array([-80,20,90,0])
        self.world.robot.set_joints_target_position_direct(indices,values)

        return False
        

    def is_ready(self):
        ''' Returns True if Step Behavior is ready to start under current game/robot conditions '''
        return True