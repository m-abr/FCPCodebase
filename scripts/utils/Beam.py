
from agent.Base_Agent import Base_Agent as Agent
from scripts.commons.Script import Script
from time import sleep


class Beam():
    def __init__(self, script:Script) -> None:
        self.script = script

    def ask_for_input(self,prompt, default):
        try:
            inp=input(prompt)
            return float(inp)
        except ValueError:
            if inp != '':
                print("Illegal input:", inp, "\n")
            return default

    def beam_and_update(self,x,y,rot):
        r = self.player.world.robot
        d = self.player.world.draw

        d.annotation((x,y,0.7), f"x:{x} y:{y} r:{rot}", d.Color.yellow, "pos_label")

        self.player.scom.unofficial_beam((x,y,r.beam_height),rot)
        for _ in range(10): # run multiple times to beam and then simulate eventual collisions (e.g. goal posts)
            sleep(0.03)
            self.player.behavior.execute("Zero")
            self.player.scom.commit_and_send( r.get_command() )
            self.player.scom.receive()

    def execute(self):

        a = self.script.args    
        self.player = Agent(a.i, a.p, a.m, a.u, a.r, a.t) # Args: Server IP, Agent Port, Monitor Port, Uniform No., Robot Type, Team Name
        d = self.player.world.draw

        self.player.scom.unofficial_set_play_mode("PlayOn")

        # Draw grid
        for x in range(-15,16):
            for y in range(-10,11):
                d.point((x,y), 6, d.Color.red, "grid", False)
        d.flush("grid")
         
        for _ in range(10): # Initialize
            self.player.scom.send()
            self.player.scom.receive()

        print("\nBeam player to coordinates + orientation:")

        x=y=a=0
        while True: # Beam self.player to given position
            x = self.ask_for_input(f"\nInput x coordinate       ('' to send {x:5} again, ctrl+c to return): ",x)
            self.beam_and_update(x,y,a)
            y = self.ask_for_input(  f"Input y coordinate       ('' to send {y:5} again, ctrl+c to return): ",y)
            self.beam_and_update(x,y,a)
            a = self.ask_for_input(  f"Orientation -180 to 180  ('' to send {a:5} again, ctrl+c to return): ",a)
            self.beam_and_update(x,y,a)