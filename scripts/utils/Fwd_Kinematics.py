from agent.Base_Agent import Base_Agent as Agent
from scripts.commons.Script import Script
from world.commons.Draw import Draw
import numpy as np

class Fwd_Kinematics():

    def __init__(self,script:Script) -> None:
        self.script = script
        self.cycle_duration = 200 #steps

    def draw_cycle(self):

        #Draw position of body parts
        for _ in range(self.cycle_duration):
            self.script.batch_execute_behavior("Squat")
            self.script.batch_commit_and_send()
            self.script.batch_receive()

            p : Agent
            for p in self.script.players:
                if p.world.vision_is_up_to_date and not p.world.robot.loc_is_up_to_date:
                    p.world.draw.annotation(p.world.robot.cheat_abs_pos, "Not enough visual data! Using IMU", Draw.Color.red,"localization")

                for key, val in p.world.robot.body_parts.items():
                    rp = val.transform.get_translation()
                    pos = p.world.robot.loc_head_to_field_transform(rp,False)
                    label_rp = np.array([rp[0]-0.0001,rp[1]*0.5,0])
                    label_rp /= np.linalg.norm(label_rp) / 0.4 #labels at 0.4m from body part
                    label = p.world.robot.loc_head_to_field_transform(rp+label_rp,False)
                    p.world.draw.line(pos,label,2,Draw.Color.green_light,key,False)
                    p.world.draw.annotation(label,key,Draw.Color.red,key)

                rp = p.world.robot.body_parts['lfoot'].transform((0.08,0,0))
                ap = p.world.robot.loc_head_to_field_transform(rp,False)
                p.world.draw.line(ap,ap+(0,0,0.1),1,Draw.Color.red,"soup",False)  
                rp = p.world.robot.body_parts['lfoot'].transform((-0.08,0,0))
                ap = p.world.robot.loc_head_to_field_transform(rp,False)
                p.world.draw.line(ap,ap+(0,0,0.1),1,Draw.Color.red,"soup",False) 
                rp = p.world.robot.body_parts['lfoot'].transform((0,0.04,0))
                ap = p.world.robot.loc_head_to_field_transform(rp,False)
                p.world.draw.line(ap,ap+(0,0,0.1),1,Draw.Color.red,"soup",False)
                rp = p.world.robot.body_parts['lfoot'].transform((0,-0.04,0))
                ap = p.world.robot.loc_head_to_field_transform(rp,False)
                p.world.draw.line(ap,ap+(0,0,0.1),1,Draw.Color.red,"soup",True)         

        Draw.clear_all()

        #Draw position of joints
        for _ in range(self.cycle_duration):
            self.script.batch_execute_behavior("Squat")
            self.script.batch_commit_and_send()
            self.script.batch_receive()

            for p in self.script.players:
                if p.world.vision_is_up_to_date and not p.world.robot.loc_is_up_to_date:
                    p.world.draw.annotation(p.world.robot.cheat_abs_pos, "Not enough visual data! Using IMU", Draw.Color.red,"localization")

                zstep = 0.05
                label_z = [0,0,0,0,zstep,zstep,2*zstep,2*zstep,0,0,0,0,zstep,zstep,0,0,zstep,zstep,2*zstep,2*zstep,3*zstep,3*zstep,0,0]
                for j, transf in enumerate(p.world.robot.joints_transform):
                    rp = transf.get_translation()
                    pos = p.world.robot.loc_head_to_field_transform(rp,False)
                    j_name = str(j)
                    label_rp = np.array([rp[0]-0.0001,rp[1]*0.5,0]) 
                    label_rp /= np.linalg.norm(label_rp) / 0.4 #labels at 0.4m from body part
                    label_rp += (0,0,label_z[j])
                    label = p.world.robot.loc_head_to_field_transform(rp+label_rp,False)
                    p.world.draw.line( pos,label,2,Draw.Color.green_light,j_name,False)
                    p.world.draw.annotation( label,j_name,Draw.Color.cyan,j_name)

                
        Draw.clear_all()

        #Draw orientation of body parts
        for _ in range(self.cycle_duration):
            self.script.batch_execute_behavior("Squat")
            self.script.batch_commit_and_send()
            self.script.batch_receive()

            p : Agent
            for p in self.script.players:
                if p.world.vision_is_up_to_date and not p.world.robot.loc_is_up_to_date:
                    p.world.draw.annotation(p.world.robot.cheat_abs_pos, "Not enough visual data! Using IMU", Draw.Color.red,"localization")

                for key in p.world.robot.body_parts:
                    #Select only some body parts
                    if key not in ['head', 'torso', 'llowerarm', 'rlowerarm', 'lthigh', 'rthigh', 'lshank', 'rshank', 'lfoot', 'rfoot']: continue
                    bpart_abs_pos = p.world.robot.get_body_part_to_field_transform(key).translate((0.1,0,0)) #10cm in front of body part
                    x_axis = bpart_abs_pos((0.05,0,0),False)
                    y_axis = bpart_abs_pos((0,0.05,0),False)
                    z_axis = bpart_abs_pos((0,0,0.05),False)
                    axes_0 = bpart_abs_pos.get_translation()
                    p.world.draw.line( axes_0,x_axis,2,Draw.Color.green_light,key,False)
                    p.world.draw.line( axes_0,y_axis,2,Draw.Color.blue,key,False)
                    p.world.draw.line( axes_0,z_axis,2,Draw.Color.red,key)

        Draw.clear_all()

        
        
    def execute(self):

        a = self.script.args

        # Args: Server IP, Agent Port, Monitor Port, Uniform No., Robot Type, Team Name, Enable Log, Enable Draw
        self.script.batch_create(Agent, ((a.i,a.p,a.m,u,u-1,a.t,True,True) for u in range(1,6)) )

        #Beam players
        self.script.batch_unofficial_beam( [(-2,i*4-10,0.5,i*45) for i in range(5)] )

        print("\nPress ctrl+c to return.")

        while True:
            self.draw_cycle()