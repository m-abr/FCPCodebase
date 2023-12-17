from agent.Agent import Agent
from agent.Base_Agent import Base_Agent
from scripts.commons.Script import Script
import numpy as np

'''
Objective:
----------
Dribble and score
'''

class Dribble():
    def __init__(self, script:Script) -> None:
        self.script = script

    def execute(self):

        a = self.script.args  

        # Args: Server IP, Agent Port, Monitor Port, Uniform No., [Robot Type] (for Base_Agent), Team name, Enable Log, Enable Draw
        self.script.batch_create(Base_Agent, ((a.i,a.p,a.m,a.u,a.r,a.t,True,True),)) # one dribbler
        self.script.batch_create(Agent, ((a.i,a.p,a.m,u,"Opponent",False,False) for u in range(1,2))) # 1 opponent (normal agent)

        p : Base_Agent = self.script.players[0]
        p.path_manager.draw_options(enable_obstacles=True, enable_path=True)

        behavior = p.behavior
        w = p.world
        r = w.robot
        d = w.draw
        
        p.scom.unofficial_beam((-3,0,r.beam_height),0)
        p.scom.unofficial_set_play_mode("PlayOn")
        print("\nPress ctrl+c to return.")

        while True:

            if w.play_mode == w.M_THEIR_KICKOFF:
                p.scom.unofficial_set_play_mode("PlayOn")
            
            # execute dribbler
            if behavior.is_ready("Get_Up") or w.play_mode_group in [w.MG_ACTIVE_BEAM, w.MG_PASSIVE_BEAM]:
                p.scom.unofficial_beam((*(w.ball_abs_pos[:2]-(1,0)),r.beam_height),0)
                behavior.execute("Zero_Bent_Knees")
            else:
                behavior.execute("Dribble",None,None)
            d.annotation(r.loc_head_position+(0,0,0.2),f"{np.linalg.norm(r.get_head_abs_vel(40)[:2]):.2f}",d.Color.white,"vel_annotation")
            p.scom.commit_and_send( r.get_command() ) 

            # execute opponents as normal agents
            self.script.batch_execute_agent(slice(1,None)) 
                        
            # all players wait for server to send feedback
            self.script.batch_receive()