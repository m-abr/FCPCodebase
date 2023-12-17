from math_ops.Math_Ops import Math_Ops as M
from world.World import World
import numpy as np

class Head():
    FIELD_FLAGS = World.FLAGS_CORNERS_POS + World.FLAGS_POSTS_POS
    HEAD_PITCH = -35

    def __init__(self, world : World) -> None:
        self.world = world
        self.look_left = True
        self.state = 0

    
    def execute(self):
        '''
        Try to compute best head orientation if possible, otherwise look around

        state:
        0            - Adjust position - ball is in FOV and robot can self-locate
        1..TIMEOUT-1 - Guided search - attempt to use recent visual/radio information to guide the search
        TIMEOUT      - Random search - look around (default mode after guided search fails by timeout)
        '''
        TIMEOUT = 30
        w = self.world
        r = w.robot
        can_self_locate = r.loc_last_update > w.time_local_ms - w.VISUALSTEP_MS
        
        #--------------------------------------- A. Ball is in FOV and robot can self-locate

        if w.ball_last_seen > w.time_local_ms - w.VISUALSTEP_MS: # ball is in FOV
            if can_self_locate:
                best_dir = self.compute_best_direction(can_self_locate, use_ball_from_vision=True)
                self.state = 0
            elif self.state < TIMEOUT:
        #--------------------------------------- B. Ball is in FOV but robot cannot currently self-locate
                best_dir = self.compute_best_direction(can_self_locate, use_ball_from_vision=True)
                self.state += 1 # change to guided search and increment time
        elif self.state < TIMEOUT:
        #--------------------------------------- C. Ball is not in FOV
            best_dir = self.compute_best_direction(can_self_locate)
            self.state += 1 # change to guided search and increment time


        if self.state == TIMEOUT: # Random search

            if w.ball_last_seen > w.time_local_ms - w.VISUALSTEP_MS: # Ball is in FOV (search 45 deg to both sides of the ball)
                ball_dir = M.vector_angle(w.ball_rel_torso_cart_pos[:2])
                targ = np.clip(ball_dir + (45 if self.look_left else -45), -119, 119)
            else: # Ball is not in FOV (search 119 deg to both sides)
                targ = 119 if self.look_left else -119

            if r.set_joints_target_position_direct([0,1], np.array([targ,Head.HEAD_PITCH]), False) <= 0:
                self.look_left = not self.look_left

        else: # Adjust position or guided search
            r.set_joints_target_position_direct([0,1], np.array([best_dir,Head.HEAD_PITCH]), False)


    def compute_best_direction(self, can_self_locate, use_ball_from_vision=False):
        FOV_MARGIN = 15 # safety margin, avoid margin horizontally
        SAFE_RANGE = 120 - FOV_MARGIN*2
        HALF_RANGE = SAFE_RANGE / 2

        w = self.world
        r = w.robot
        
        if use_ball_from_vision:
            ball_2d_dist = np.linalg.norm(w.ball_rel_torso_cart_pos[:2])
        else:
            ball_2d_dist = np.linalg.norm(w.ball_abs_pos[:2]-r.loc_head_position[:2])
                
        if ball_2d_dist > 0.12:
            if use_ball_from_vision:
                ball_dir = M.vector_angle(w.ball_rel_torso_cart_pos[:2])
            else:
                ball_dir = M.target_rel_angle(r.loc_head_position, r.imu_torso_orientation, w.ball_abs_pos)
        else: # ball is very close to robot
            ball_dir = 0

        flags_diff = dict()
            
        # iterate flags
        for f in Head.FIELD_FLAGS:
            flag_dir = M.target_rel_angle(r.loc_head_position, r.imu_torso_orientation, f)
            diff = M.normalize_deg( flag_dir - ball_dir )
            if abs(diff) < HALF_RANGE and can_self_locate:
                return ball_dir # return ball direction if robot can self-locate
            flags_diff[f] = diff

        
        closest_flag = min( flags_diff, key=lambda k: abs(flags_diff[k]) )
        closest_diff = flags_diff[closest_flag]

        if can_self_locate: # at this point, if it can self-locate, then  abs(closest_diff) > HALF_RANGE
            # return position that centers the ball as much as possible in the FOV, including the nearest flag if possible
            final_diff = min( abs(closest_diff) - HALF_RANGE, SAFE_RANGE ) * np.sign(closest_diff)
        else:
            # position that centers the flag as much as possible, until it is seen, while keeping the ball in the FOV
            final_diff = np.clip( closest_diff, -SAFE_RANGE, SAFE_RANGE )
            # saturate instead of normalizing angle to avoid a complete neck rotation
            return np.clip(ball_dir + final_diff, -119, 119)


        return M.normalize_deg( ball_dir + final_diff )