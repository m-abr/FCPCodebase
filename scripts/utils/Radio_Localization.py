from agent.Agent import Agent
from itertools import count
from scripts.commons.Script import Script
from typing import List
from world.commons.Draw import Draw

class Radio_Localization():
    def __init__(self,script:Script) -> None:
        self.script = script

    def draw_objects(self, p:Agent, pos, is_down, was_seen, last_update, is_self=False):
        w = p.world
        me = w.robot.loc_head_position

        # get draw object from same player to always overwrite previous drawing
        # could also use team channel but with this approach we could draw for both teams
        d:Draw = self.script.players[0].world.draw 

        # VISUALSTEP_MS is the time it takes to get a visual update
        is_current = last_update > w.time_local_ms - w.VISUALSTEP_MS

        # 0.12s is the time it takes to do a full broadcast with all positions if every group is completely visible
        # here we use >= instead of > because the radio message comes with a delay of 20ms
        is_recent = last_update >= w.time_local_ms - 120

        if is_current and was_seen:
            c = d.Color.green_light  # I've seen this object in the current or previous time step
        elif is_recent and was_seen:
            c = d.Color.green        # I've seen this object in the last 0.12s
        elif is_current:
            c = d.Color.yellow       # I've heard about this object in the current or previous time step (and it was not seen in the same period)
        elif is_recent:
            c = d.Color.yellow_light # I've heard about this object in the last 0.12s (the last available info was not obtained from vision)
        else:
            c = d.Color.red          # I haven't seen or heard about this object in the last 0.12s

        if is_self:
            if w.robot.radio_fallen_state:
                d.annotation(me, "Fallen (radio)", d.Color.yellow, "objects", False)   # I heard I've fallen (but I missed the last 2 visual steps)
            elif w.robot.loc_head_z < 0.3:
                d.annotation(me, "Fallen (internal)", d.Color.white, "objects", False) # I have detected I've fallen
            d.sphere(me, 0.06, c, "objects", False)
        else:
            if is_down:
                d.annotation((me[:2]+pos[:2])/2,"Fallen",d.Color.white,"objects",False)
            d.arrow(me, pos, 0.1, 3, c, "objects", False)


    def draw(self,p:Agent):
        w = p.world
        others = w.teammates + w.opponents

        #----------------------------------------------------------- draw other players

        for o in others:
            if o.is_self or o.state_last_update==0: # do not draw self or never before seen players
                continue

            pos = o.state_abs_pos
            is_down = o.state_fallen
            # 3D head position means head is visible, 2D means some body parts are visible but not the head, or the head position comes from radio
            is_3D = pos is not None and len(pos)==3 

            self.draw_objects(p, pos, is_down, is_3D, o.state_last_update)

        #----------------------------------------------------------- draw self

        is_pos_from_vision = w.robot.loc_head_position_last_update == w.robot.loc_last_update
        self.draw_objects(p, None, None, is_pos_from_vision, w.robot.loc_head_position_last_update, True)

        #----------------------------------------------------------- draw ball and flush drawings

        self.draw_objects(p, w.ball_abs_pos, False, w.is_ball_abs_pos_from_vision, w.ball_abs_pos_last_update)
        self.script.players[0].world.draw.flush("objects")
        

    def execute(self):
        a = self.script.args

        # Args: Server IP, Agent Port, Monitor Port, Uniform No., Robot Type, Team Name, Enable Log, Enable Draw
        self.script.batch_create(Agent, ((a.i,a.p,a.m,u,a.t,       False,u==1) for u in range(1,12)))
        self.script.batch_create(Agent, ((a.i,a.p,a.m,u,"Opponent",False,False) for u in range(1,12)))
        players : List[Agent] = self.script.players

        # Beam opponents
        beam_pos = [(-(i//2)-3,(i%2*2-1)*(i//2+1),0) for i in range(11)]
        self.script.batch_commit_beam( beam_pos, slice(11,None) )
        print("\nPress ctrl+c to return.")

        # Execute
        for j in count():
            self.script.batch_execute_agent( slice(11) )        # run our agents (think and send)
            self.script.batch_commit_and_send( slice(11,None) ) # run their agents (don't think, just send)

            self.draw(players[j//15%11])                     # draw knowledge, iterate through our team, 15 time steps per player
            self.script.batch_receive(slice(11))             # receive & update our team's world state
            self.script.batch_receive(slice(11,None), False) # receive & don't update opponent's world state (to save cpu resources)
