from agent.Base_Agent import Base_Agent
from math_ops.Math_Ops import Math_Ops as M
import numpy as np
import random


class Agent(Base_Agent):
    def __init__(self, host:str, agent_port:int, monitor_port:int, unum:int,
                 team_name:str, enable_log, enable_draw, wait_for_server=True, is_fat_proxy=False) -> None:
        
        # define robot type
        robot_type = 0 if unum == 1 else 4 # assume the goalkeeper uses uniform number 1 and the kicker uses any other number

        # Initialize base agent
        # Args: Server IP, Agent Port, Monitor Port, Uniform No., Robot Type, Team Name, Enable Log, Enable Draw, play mode correction, Wait for Server, Hear Callback
        super().__init__(host, agent_port, monitor_port, unum, robot_type, team_name, enable_log, enable_draw, False, wait_for_server, None)

        self.enable_draw = enable_draw
        self.state = 0  # 0-Normal, 1-Getting up, 2-Dive Left, 3-Dive Right, 4-Wait

        self.kick_dir = 0 # kick direction
        self.reset_kick = True # when True, a new random kick direction is generated
        

    def think_and_send(self):
        w = self.world
        r = self.world.robot 
        my_head_pos_2d = r.loc_head_position[:2]
        my_ori = r.imu_torso_orientation
        ball_2d = w.ball_abs_pos[:2]
        ball_vec = ball_2d - my_head_pos_2d
        ball_dir = M.vector_angle(ball_vec)
        ball_dist = np.linalg.norm(ball_vec)
        ball_speed = np.linalg.norm(w.get_ball_abs_vel(6)[:2])
        behavior = self.behavior
        PM = w.play_mode

        #--------------------------------------- 1. Decide action

        if PM in [w.M_BEFORE_KICKOFF, w.M_THEIR_GOAL, w.M_OUR_GOAL]: # beam to initial position and wait
            self.state = 0
            self.reset_kick = True
            pos = (-14,0) if r.unum == 1 else (4.9,0)
            if np.linalg.norm(pos - r.loc_head_position[:2]) > 0.1 or behavior.is_ready("Get_Up"):
                self.scom.commit_beam(pos, 0) # beam to initial position
            else:
                behavior.execute("Zero_Bent_Knees") # wait
        elif self.state == 2: # dive left
            self.state = 4 if behavior.execute("Dive_Left") else 2  # change state to wait after skill has finished
        elif self.state == 3: # dive right
            self.state = 4 if behavior.execute("Dive_Right") else 3 # change state to wait after skill has finished
        elif self.state == 4: # wait (after diving or during opposing kick)
            pass
        elif self.state == 1 or behavior.is_ready("Get_Up"): # if getting up or fallen
            self.state = 0 if behavior.execute("Get_Up") else 1 # return to normal state if get up behavior has finished
        elif PM == w.M_OUR_KICKOFF and r.unum == 1 or PM == w.M_THEIR_KICKOFF and r.unum != 1:
            self.state = 4 # wait until next beam
        elif r.unum == 1: # goalkeeper
            y_coordinate = np.clip(ball_2d[1], -1.1, 1.1)
            behavior.execute("Walk", (-14,y_coordinate), True, 0, True, None) # Args: target, is_target_abs, ori, is_ori_abs, distance
            if ball_2d[0] < -10: 
                self.state = 2 if ball_2d[1] > 0 else 3 # dive to defend
        else: # kicker
            if PM == w.M_OUR_KICKOFF and ball_2d[0] > 5: # check ball position to make sure I see it
                if self.reset_kick: 
                    self.kick_dir = random.choice([-7.5,7.5]) 
                    self.reset_kick = False
                behavior.execute("Basic_Kick", self.kick_dir)
            else:
                behavior.execute("Zero_Bent_Knees") # wait

        #--------------------------------------- 2. Broadcast
        self.radio.broadcast()

        #--------------------------------------- 3. Send to server
        self.scom.commit_and_send( r.get_command() )

        #---------------------- annotations for debugging
        if self.enable_draw: 
            d = w.draw
            if r.unum == 1:
                d.annotation((*my_head_pos_2d, 0.8), "Goalkeeper" , d.Color.yellow, "status")
            else:
                d.annotation((*my_head_pos_2d, 0.8), "Kicker" , d.Color.yellow, "status")
                if PM == w.M_OUR_KICKOFF: # draw arrow to indicate kick direction
                    d.arrow(ball_2d, ball_2d + 5*M.vector_from_angle(self.kick_dir), 0.4, 3, d.Color.cyan_light, "Target")


