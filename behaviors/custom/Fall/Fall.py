from agent.Base_Agent import Base_Agent
from math_ops.Math_Ops import Math_Ops as M
from math_ops.Neural_Network import run_mlp
import pickle, numpy as np

class Fall():

    def __init__(self, base_agent : Base_Agent) -> None:
        self.world = base_agent.world
        self.description = "Fall example"
        self.auto_head = False

        with open(M.get_active_directory("/behaviors/custom/Fall/fall.pkl"), 'rb') as f:
            self.model = pickle.load(f)

        self.action_size = len(self.model[-1][0]) # extracted from size of Neural Network's last layer bias
        self.obs = np.zeros(self.action_size+1, np.float32)

        self.controllable_joints = min(self.world.robot.no_of_joints, self.action_size) # compatibility between different robot types

    def observe(self):
        r = self.world.robot
        
        for i in range(self.action_size):
            self.obs[i] = r.joints_position[i] / 100 # naive scale normalization

        self.obs[self.action_size] = r.cheat_abs_pos[2] # head.z (alternative: r.loc_head_z)
      
    def execute(self,reset) -> bool:
        self.observe()
        action = run_mlp(self.obs, self.model) 
        
        self.world.robot.set_joints_target_position_direct( # commit actions:
            slice(self.controllable_joints), # act on trained joints
            action*10,                       # scale actions up to motivate early exploration
            harmonize=False                  # there is no point in harmonizing actions if the targets change at every step  
        )

        return self.world.robot.loc_head_z < 0.15 # finished when head height < 0.15 m

    def is_ready(self) -> any:
        ''' Returns True if this behavior is ready to start/continue under current game/robot conditions '''
        return True
