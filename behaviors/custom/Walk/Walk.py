from agent.Base_Agent import Base_Agent
from behaviors.custom.Walk.Env import Env
from math_ops.Math_Ops import Math_Ops as M
from math_ops.Neural_Network import run_mlp
import numpy as np
import pickle

class Walk():

    def __init__(self, base_agent : Base_Agent) -> None:
        self.world = base_agent.world
        self.description = "Omnidirectional RL walk"
        self.auto_head = True
        self.env = Env(base_agent)
        self.last_executed = 0

        with open(M.get_active_directory([
            "/behaviors/custom/Walk/walk_R0.pkl",
            "/behaviors/custom/Walk/walk_R1_R3.pkl",
            "/behaviors/custom/Walk/walk_R2.pkl",
            "/behaviors/custom/Walk/walk_R1_R3.pkl",
            "/behaviors/custom/Walk/walk_R4.pkl"
            ][self.world.robot.type]), 'rb') as f:
            self.model = pickle.load(f)


    def execute(self, reset, target_2d, is_target_absolute, orientation, is_orientation_absolute, distance):
        '''
        Parameters
        ----------
        target_2d : array_like
            2D target in absolute or relative coordinates (use is_target_absolute to specify)
        is_target_absolute : bool
            True if target_2d is in absolute coordinates, False if relative to robot's torso
        orientation : float
            absolute or relative orientation of torso, in degrees
            set to None to go towards the target (is_orientation_absolute is ignored)
        is_orientation_absolute : bool
            True if orientation is relative to the field, False if relative to the robot's torso
        distance : float
            distance to final target [0,0.5] (influences walk speed when approaching the final target)
            set to None to consider target_2d the final target
        '''
        r = self.world.robot

        #------------------------ 0. Override reset (since some behaviors use this as a sub-behavior)
        if reset and self.world.time_local_ms - self.last_executed == 20:
            reset = False
        self.last_executed = self.world.time_local_ms

        #------------------------ 1. Define walk parameters 

        if is_target_absolute: # convert to target relative to (head position + torso orientation)
            raw_target = target_2d - r.loc_head_position[:2]
            self.env.walk_rel_target = M.rotate_2d_vec(raw_target, -r.imu_torso_orientation)
        else:
            self.env.walk_rel_target = target_2d

        if distance is None:
            self.env.walk_distance = np.linalg.norm(self.env.walk_rel_target)
        else:
            self.env.walk_distance = distance # MAX_LINEAR_DIST = 0.5

        # Relative orientation values are decreased to avoid overshoot
        if orientation is None:
            self.env.walk_rel_orientation = M.vector_angle(self.env.walk_rel_target) * 0.3
        elif is_orientation_absolute:
            self.env.walk_rel_orientation = M.normalize_deg( orientation - r.imu_torso_orientation )
        else:
            self.env.walk_rel_orientation = orientation * 0.3

        #------------------------ 2. Execute behavior

        obs = self.env.observe(reset)
        action = run_mlp(obs, self.model)   
        self.env.execute(action)
        
        return False


    def is_ready(self):
        ''' Returns True if Walk Behavior is ready to start under current game/robot conditions '''
        return True