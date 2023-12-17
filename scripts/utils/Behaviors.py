from agent.Base_Agent import Base_Agent as Agent
from scripts.commons.Script import Script
from scripts.commons.UI import UI

class Behaviors():

    def __init__(self,script:Script) -> None:
        self.script = script
        self.player : Agent = None

    def ask_for_behavior(self):
        names, descriptions = self.player.behavior.get_all_behaviors()

        UI.print_table( [names,descriptions], ["Behavior Name","Description"], numbering=[True,False])
        choice, is_str_opt = UI.read_particle('Choose behavior ("" to skip 2 time steps, "b" to beam, ctrl+c to return): ',["","b"],int,[0,len(names)])
        if is_str_opt: return choice #skip 2 time steps or quit
        return names[choice]

    def sync(self):
        self.player.scom.commit_and_send( self.player.world.robot.get_command() ) 
        self.player.scom.receive()

    def beam(self):
        self.player.scom.unofficial_beam((-2.5,0,self.player.world.robot.beam_height),0)
        for _ in range(5): 
            self.sync()

    def execute(self):

        a = self.script.args  
        self.player = Agent(a.i, a.p, a.m, a.u, a.r, a.t) # Args: Server IP, Agent Port, Monitor Port, Uniform No., Robot Type, Team Name
        behavior = self.player.behavior

        self.beam()
        self.player.scom.unofficial_set_play_mode("PlayOn")

        # Special behaviors
        special_behaviors = {"Step":(), "Basic_Kick":(0,), "Walk":((0.5,0),False,0,False,None), "Dribble":(None,None)}

        while True:
            behavior_name = self.ask_for_behavior()
            if behavior_name == 0: # skip 2 time steps (user request)
                self.sync()
                self.sync()
            elif behavior_name == 1: # beam
                self.beam() 
            else:
                if behavior_name in special_behaviors: # not using execute_to_completion to abort behavior after a timeout
                    duration = UI.read_int("For how many time steps [1,1000]? ", 1, 1001)
                    for _ in range(duration): 
                        if behavior.execute(behavior_name, *special_behaviors[behavior_name]): 
                            break # break if behavior ends
                        self.sync()
                else:
                    behavior.execute_to_completion(behavior_name)