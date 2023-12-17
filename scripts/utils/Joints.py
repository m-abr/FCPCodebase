from agent.Base_Agent import Base_Agent as Agent
from scripts.commons.Script import Script
from world.commons.Draw import Draw
import numpy as np

class Joints():

    def __init__(self,script:Script) -> None:
        self.script = script
        self.agent_pos = (-3,0,0.45)
        self.enable_pos = True
        self.enable_gravity = False
        self.enable_labels = True
        self.enable_harmonize = True
        self.active_joint = 0
        self.joints_value = None #position or speed


    def _draw_joints(self,player:Agent):
        zstep = 0.05
        label_z = [3*zstep,5*zstep,0,0,zstep,zstep,2*zstep,2*zstep,0,0,0,0,zstep,zstep,0,0,zstep,zstep,4*zstep,4*zstep,5*zstep,5*zstep,0,0]
        for j, transf in enumerate(player.world.robot.joints_transform):
            rp = transf.get_translation()
            pos = player.world.robot.loc_head_to_field_transform(rp,False)
            j_id = f"{j}"
            j_name = f"{j}"
            color = Draw.Color.cyan
            if player.world.robot.joints_position[j] != 0:
                j_name += f" ({int(player.world.robot.joints_position[j])})"
                color = Draw.Color.red
            label_rp = np.array([rp[0]-0.0001,rp[1]*0.5,0]) 
            label_rp /= np.linalg.norm(label_rp) / 0.5 #labels at 0.5m from body part
            label_rp += (0,0,label_z[j])
            label = player.world.robot.loc_head_to_field_transform(rp+label_rp,False)
            player.world.draw.line( pos,label,2,Draw.Color.green_light,j_id,False)
            player.world.draw.annotation( label,j_name,color,j_id)


    def print_help(self):
        print(f"""
---------------------- Joints demonstration ----------------------
Command: {{action/actions/option}}
    action : [joint:{{int}}] value 
    actions: value0,value1,...,valueN
             e.g. if N=10, you control all joints from j0 to j10
    option:  {{h,s,g,l,w,r,"",.}}
Examples:
    "6 90"   - move joint 6 to 90deg or move joint 6 at 90deg/step
    "4"      - move last joint to 4deg or apply speed of 4deg/step
    "1,9,-35"- move joints 0,1,2 to 1deg, 9deg, -35deg (or speed)
    "h"      - help, display this message
    "s"      - toggle position/speed control ({"Posi" if self.enable_pos else "Spee"})
    "g"      - toggle gravity                ({self.enable_gravity})
    "l"      - toggle labels                 ({self.enable_labels})
    "w"      - toggle harmonize*             ({self.enable_harmonize})
    "r"      - reset (position mode + reset joints)
    ""       - advance 2 simulation step
    "."      - advance 1 simulation step
    "ctrl+c" - quit demonstration

    *all joints end moving at the same time when harmonize is True
------------------------------------------------------------------""")

    def _user_control_step(self,player:Agent):

        while True:

            inp = input("Command: ")
            if inp == "s": 
                self.enable_pos = not self.enable_pos
                print("Using", "position" if self.enable_pos else "velocity", "control.")
                if self.enable_pos:
                    self.joints_value[:] = player.world.robot.joints_position
                else:
                    self.joints_value.fill(0)
                continue
            elif inp == "g": 
                self.enable_gravity = not self.enable_gravity
                print("Using gravity:",self.enable_gravity)
                continue
            elif inp == "l":
                self.enable_labels = not self.enable_labels
                print("Using labels:",self.enable_labels)
                continue
            elif inp == "w":
                self.enable_harmonize = not self.enable_harmonize
                print("Using harmonize:",self.enable_harmonize)
                continue
            elif inp == "r":
                self.enable_pos = True
                self.joints_value.fill(0)
                print("Using position control. All joints are set to zero.")
                continue
            elif inp == "h": 
                self.print_help(); continue
            
            elif inp == "": return 1
            elif inp == ".": return 0
            
            try:
                if " " in inp:
                    self.active_joint, value = map(float, inp.split())
                    self.joints_value[int(self.active_joint)] = value
                elif "," in inp:
                    values = inp.split(",")
                    self.joints_value[0:len(values)] = values
                else:
                    self.joints_value[self.active_joint] = float(inp)
            except:
                print("Illegal command!")
                continue



    def execute(self):

        a = self.script.args
        player = Agent(a.i, a.p, a.m, a.u, a.r, a.t) # Args: Server IP, Agent Port, Monitor Port, Uniform No., Robot Type, Team Name

        self.joints_no = player.world.robot.no_of_joints
        self.joints_value = np.zeros(self.joints_no) # initialize

        player.scom.commit_beam(self.agent_pos[0:2],0)

        self.print_help()

        # initialize (+beam)
        for _ in range(8):
            player.scom.commit_and_send()
            player.scom.receive()
        self._draw_joints(player)

        skip_next = 0 # variable to advance more than 1 step

        while True:
            if skip_next == 0:
                skip_next = self._user_control_step(player)
            else:
                skip_next -= 1

            if self.enable_labels:
                self._draw_joints(player)

            if self.enable_pos:
                player.world.robot.set_joints_target_position_direct(slice(self.joints_no), self.joints_value, harmonize=self.enable_harmonize)
            else:
                player.world.robot.joints_target_speed[:]=self.joints_value * 0.87266463 #deg/step to rad/s

            if not self.enable_gravity: player.scom.unofficial_beam(self.agent_pos,0)
            player.scom.commit_and_send( player.world.robot.get_command() ) 
            player.scom.receive()