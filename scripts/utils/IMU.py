from agent.Base_Agent import Base_Agent as Agent
from math_ops.Matrix_3x3 import Matrix_3x3
from math_ops.Matrix_4x4 import Matrix_4x4
from scripts.commons.Script import Script
from world.commons.Draw import Draw
from world.Robot import Robot
import numpy as np

'''
Objective:
----------
Demonstrate the accuracy of the IMU
Robot.imu_(...) variables are based on the visual localizer algorithm and the IMU when no visual data is available.
If visual data is not available for longer than 0.2 seconds, the robot's position is frozen and the velocity decays to zero.
The rotation computed by the IMU is so accurate that it is never frozen, no matter how long the robot goes without visual data.
It is almost always safe to use IMU data for rotation.
Known issues: the accelerometer is not reliable in the presence of "instant" acceleration peaks, due to its low sample rate (50Hz)
              this limitation impacts the translation estimation during crashes (e.g. falling, crashing against other players)
'''

class IMU():

    def __init__(self,script:Script) -> None:
        self.script = script
        self.player : Agent = None
        self.cycle = 0

        self.imu_torso_to_field_rotation  = [Matrix_3x3() for _ in range(3)]
        self.imu_torso_to_field_transform = [Matrix_4x4() for _ in range(3)]
        self.imu_head_to_field_transform  = [Matrix_4x4() for _ in range(3)]
        self.imu_torso_position = np.zeros((3,3))
        self.imu_torso_velocity = np.zeros((3,3))
        self.imu_torso_acceleration = np.zeros((3,3))
        self.imu_torso_next_position = np.zeros((3,3))
        self.imu_torso_next_velocity = np.zeros((3,3))
        self.imu_CoM_position = np.zeros((3,3))
        self.colors = [Draw.Color.green_light, Draw.Color.yellow, Draw.Color.red]

    def act(self):
        r = self.player.world.robot
        joint_indices = [r.J_LLEG_PITCH, 
                         r.J_LKNEE, 
                         r.J_LFOOT_PITCH,
                         r.J_LARM_ROLL,
                         r.J_RLEG_PITCH, 
                         r.J_RKNEE, 
                         r.J_RFOOT_PITCH,
                         r.J_RARM_ROLL]
        
        amplitude = [1,0.93,1,1,1][r.type]

        self.cycle += 1
        if self.cycle < 50:
            r.set_joints_target_position_direct(joint_indices, np.array([32+10,-64,32, 45,  40+10,-80,40,  0])*amplitude)        
        elif self.cycle < 100:
            r.set_joints_target_position_direct(joint_indices, np.array([  -10,  0, 0,  0,    -10,  0, 0,  0])*amplitude)
        elif self.cycle < 150:
            r.set_joints_target_position_direct(joint_indices, np.array([40+10,-80,40,  0,  32+10,-64,32, 45])*amplitude)
        elif self.cycle < 200:
            r.set_joints_target_position_direct(joint_indices, np.array([  -10,  0, 0,  0,    -10,  0, 0,  0])*amplitude)
        else:
            self.cycle = 0

        self.player.scom.commit_and_send( r.get_command() )
        self.player.scom.receive()

    def act2(self):
        r = self.player.world.robot
        self.player.behavior.execute("Walk", (0.2,0), False, 5, False, None ) # Args: target, is_target_abs, ori, is_ori_abs, distance
        self.player.scom.commit_and_send( r.get_command() )
        self.player.scom.receive()

    def draw_player_reference_frame(self,i):
        pos = self.imu_torso_position[i]
        xvec = self.imu_torso_to_field_rotation[i].multiply((1,0,0)) + pos
        yvec = self.imu_torso_to_field_rotation[i].multiply((0,1,0)) + pos
        zvec = self.imu_torso_to_field_rotation[i].multiply((0,0,1)) + pos
        self.player.world.draw.arrow(pos, xvec, 0.2, 2, self.colors[i], "IMU"+str(i), False)
        self.player.world.draw.arrow(pos, yvec, 0.2, 2, self.colors[i], "IMU"+str(i), False)
        self.player.world.draw.arrow(pos, zvec, 0.2, 2, self.colors[i], "IMU"+str(i), False)
        self.player.world.draw.annotation(xvec, "x", Draw.Color.white, "IMU"+str(i), False)
        self.player.world.draw.annotation(yvec, "y", Draw.Color.white, "IMU"+str(i), False)
        self.player.world.draw.annotation(zvec, "z", Draw.Color.white, "IMU"+str(i), False)
        self.player.world.draw.sphere(self.imu_CoM_position[i],0.04,self.colors[i],"IMU"+str(i), True)


    def compute_local_IMU(self):
        r = self.player.world.robot
        g = r.gyro / 50 # convert degrees per second to degrees per step
        self.imu_torso_to_field_rotation[2].multiply( Matrix_3x3.from_rotation_deg(g), in_place=True, reverse_order=True)
        self.imu_torso_position[2][:] = self.imu_torso_next_position[2]
        if self.imu_torso_position[2][2] < 0: self.imu_torso_position[2][2] = 0 #limit z coordinate to positive values
        self.imu_torso_velocity[2][:] = self.imu_torso_next_velocity[2]

        # convert proper acceleration to coordinate acceleration and fix rounding bias
        self.imu_torso_acceleration[2] = self.imu_torso_to_field_rotation[2].multiply(r.acc) + Robot.GRAVITY
        self.imu_torso_to_field_transform[2] = Matrix_4x4.from_3x3_and_translation(self.imu_torso_to_field_rotation[2],self.imu_torso_position[2])
        self.imu_head_to_field_transform[2] = self.imu_torso_to_field_transform[2].multiply(r.body_parts["torso"].transform.invert())
        self.imu_CoM_position[2][:] = self.imu_head_to_field_transform[2](r.rel_cart_CoM_position)

        # Next Position = x0 + v0*t + 0.5*a*t^2,   Next velocity = v0 + a*t
        self.imu_torso_next_position[2] = self.imu_torso_position[2] + self.imu_torso_velocity[2] * 0.02 + self.imu_torso_acceleration[2] * 0.0002
        self.imu_torso_next_velocity[2] = self.imu_torso_velocity[2] + self.imu_torso_acceleration[2] * 0.02
        self.imu_torso_next_velocity[2] *= Robot.IMU_DECAY #stability tradeoff

    def compute_local_IMU_rotation_only(self):
        r = self.player.world.robot
        g = r.gyro / 50 # convert degrees per second to degrees per step
        self.imu_torso_to_field_rotation[1].multiply( Matrix_3x3.from_rotation_deg(g), in_place=True, reverse_order=True)
        self.imu_torso_position[1][:] = r.loc_torso_position
        self.imu_torso_to_field_transform[1] = Matrix_4x4.from_3x3_and_translation(self.imu_torso_to_field_rotation[1],self.imu_torso_position[1])
        self.imu_head_to_field_transform[1] = self.imu_torso_to_field_transform[1].multiply(r.body_parts["torso"].transform.invert())
        self.imu_CoM_position[1][:] = self.imu_head_to_field_transform[1](r.rel_cart_CoM_position)
        

    def update_local_IMU(self, i):
        r = self.player.world.robot
        self.imu_torso_to_field_rotation[i].m[:] = r.imu_torso_to_field_rotation.m
        self.imu_torso_to_field_transform[i].m[:] = r.imu_weak_torso_to_field_transform.m
        self.imu_head_to_field_transform[i].m[:] = r.imu_weak_head_to_field_transform.m
        self.imu_torso_position[i][:] = r.imu_weak_torso_position
        self.imu_torso_velocity[i][:] = r.imu_weak_torso_velocity
        self.imu_torso_acceleration[i][:] = r.imu_weak_torso_acceleration
        self.imu_torso_next_position[i] = self.imu_torso_position[i] + self.imu_torso_velocity[i] * 0.02 + self.imu_torso_acceleration[i] * 0.0002
        self.imu_torso_next_velocity[i] = self.imu_torso_velocity[i] + self.imu_torso_acceleration[i] * 0.02
        self.imu_CoM_position[i][:] = r.imu_weak_CoM_position

    def execute(self):

        a = self.script.args    
        self.player = Agent(a.i, a.p, a.m, a.u, a.r, a.t) # Args: Server IP, Agent Port, Monitor Port, Uniform No., Robot Type, Team Name

        self.player.scom.unofficial_beam((-3,0,self.player.world.robot.beam_height),15)

        for _ in range(10): #beam to place
            self.player.scom.commit_and_send()
            self.player.scom.receive()

        self.player.world.draw.annotation((-3,1,1.1), "IMU + Localizer", self.colors[0], "note_IMU_1", True)

        for _ in range(150):
            self.act()
            self.update_local_IMU(0)
            self.draw_player_reference_frame(0)

        self.player.world.draw.annotation((-3,1,0.9), "IMU for rotation", self.colors[1], "note_IMU_2", True)
        self.update_local_IMU(1)

        for _ in range(200):
            self.act()   
            self.update_local_IMU(0)
            self.draw_player_reference_frame(0)
            self.compute_local_IMU_rotation_only()
            self.draw_player_reference_frame(1)

        self.player.world.draw.annotation((-3,1,0.7), "IMU for rotation & position", self.colors[2], "note_IMU_3", True)
        self.update_local_IMU(2)

        for _ in range(200):
            self.act()
            self.update_local_IMU(0)
            self.draw_player_reference_frame(0)
            self.compute_local_IMU_rotation_only()
            self.draw_player_reference_frame(1)
            self.compute_local_IMU()
            self.draw_player_reference_frame(2)

        print("\nPress ctrl+c to return.")

        # Still "IMU for rotation & position" but now start walking
        self.update_local_IMU(2)
        while True:
            self.act2()
            self.update_local_IMU(0)
            self.draw_player_reference_frame(0)
            self.compute_local_IMU_rotation_only()
            self.draw_player_reference_frame(1)
            self.compute_local_IMU()
            self.draw_player_reference_frame(2)            