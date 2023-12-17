from agent.Base_Agent import Base_Agent as Agent
from behaviors.custom.Step.Step import Step
from world.commons.Draw import Draw
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv
from scripts.commons.Server import Server
from scripts.commons.Train_Base import Train_Base
from time import sleep
import os, gym
import numpy as np

'''
Objective:
Learn how to run forward using step primitive
----------
- class Basic_Run: implements an OpenAI custom gym
- class Train:  implements algorithms to train a new model or test an existing model
'''

class Basic_Run(gym.Env):
    def __init__(self, ip, server_p, monitor_p, r_type, enable_draw) -> None:

        self.robot_type = r_type

        # Args: Server IP, Agent Port, Monitor Port, Uniform No., Robot Type, Team Name, Enable Log, Enable Draw
        self.player = Agent(ip, server_p, monitor_p, 1, self.robot_type, "Gym", True, enable_draw)
        self.step_counter = 0 # to limit episode size

        self.step_obj : Step = self.player.behavior.get_custom_behavior_object("Step") # Step behavior object

        # State space
        obs_size = 70
        self.obs = np.zeros(obs_size, np.float32)
        self.observation_space = gym.spaces.Box(low=np.full(obs_size,-np.inf,np.float32), high=np.full(obs_size,np.inf,np.float32), dtype=np.float32)

        # Action space
        MAX = np.finfo(np.float32).max
        self.no_of_actions = act_size = 22
        self.action_space = gym.spaces.Box(low=np.full(act_size,-MAX,np.float32), high=np.full(act_size,MAX,np.float32), dtype=np.float32)

        # Step behavior defaults
        self.step_default_dur = 7
        self.step_default_z_span = 0.035
        self.step_default_z_max = 0.70

        # Place ball far away to keep landmarks in FoV (head follows ball while using Step behavior)
        self.player.scom.unofficial_move_ball((14, 0, 0.042))
        

    def observe(self, init=False):

        r = self.player.world.robot

        # index       observation              naive normalization
        self.obs[0] = self.step_counter        /100  # simple counter: 0,1,2,3...
        self.obs[1] = r.loc_head_z             *3    # z coordinate (torso)
        self.obs[2] = r.loc_head_z_vel         /2    # z velocity (torso)  
        self.obs[3] = r.imu_torso_orientation  /50   # absolute orientation in deg
        self.obs[4] = r.imu_torso_roll         /15   # absolute torso roll  in deg
        self.obs[5] = r.imu_torso_pitch        /15   # absolute torso pitch in deg
        self.obs[6:9] = r.gyro                 /100  # gyroscope
        self.obs[9:12] = r.acc                 /10   # accelerometer

        self.obs[12:18] = r.frp.get('lf', (0,0,0,0,0,0)) #  left foot: relative point of origin (p) and force vector (f) -> (px,py,pz,fx,fy,fz)*
        self.obs[18:24] = r.frp.get('rf', (0,0,0,0,0,0)) # right foot: relative point of origin (p) and force vector (f) -> (px,py,pz,fx,fy,fz)*
        self.obs[15:18] /= 100 # naive normalization of force vector
        self.obs[21:24] /= 100 # naive normalization of force vector
        self.obs[24:44] = r.joints_position[2:22] /100    # position of all joints except head & toes (for robot type 4)
        self.obs[44:64] = r.joints_speed[2:22]    /6.1395 # speed of    all joints except head & toes (for robot type 4)
        # *if foot is not touching the ground, then (px=0,py=0,pz=0,fx=0,fy=0,fz=0)

        if init: # the walking parameters refer to the last parameters in effect (after a reset, they are pointless)
            self.obs[64] = self.step_default_dur    /10 # step duration in time steps
            self.obs[65] = self.step_default_z_span *20 # vertical movement span
            self.obs[66] = self.step_default_z_max      # relative extension of support leg
            self.obs[67] = 1 # step progress
            self.obs[68] = 1 # 1 if left  leg is active
            self.obs[69] = 0 # 1 if right leg is active
        else:
            self.obs[64] = self.step_obj.step_generator.ts_per_step   /10 # step duration in time steps
            self.obs[65] = self.step_obj.step_generator.swing_height  *20 # vertical movement span
            self.obs[66] = self.step_obj.step_generator.max_leg_extension / self.step_obj.leg_length # relative extension of support leg
            self.obs[67] = self.step_obj.step_generator.external_progress # step progress
            self.obs[68] = float(self.step_obj.step_generator.state_is_left_active)     # 1 if left  leg is active
            self.obs[69] = float(not self.step_obj.step_generator.state_is_left_active) # 1 if right leg is active

        '''
        Expected observations for walking parameters/state (example):
        Time step        R  0  1  2  0   1   2   3  4
        Progress         1  0 .5  1  0 .25  .5 .75  1
        Left leg active  T  F  F  F  T   T   T   T  T
        Parameters       A  A  A  B  B   B   B   B  C
        Example note: (A) has a step duration of 3ts, (B) has a step duration of 5ts
        '''

        return self.obs

    def sync(self):
        ''' Run a single simulation step '''
        r = self.player.world.robot
        self.player.scom.commit_and_send( r.get_command() )
        self.player.scom.receive()


    def reset(self):
        '''
        Reset and stabilize the robot
        Note: for some behaviors it would be better to reduce stabilization or add noise
        '''

        self.step_counter = 0
        r = self.player.world.robot
        
        for _ in range(25): 
            self.player.scom.unofficial_beam((-14,0,0.50),0) # beam player continuously (floating above ground)
            self.player.behavior.execute("Zero_Bent_Knees")
            self.sync()

        # beam player to ground
        self.player.scom.unofficial_beam((-14,0,r.beam_height),0) 
        r.joints_target_speed[0] = 0.01 # move head to trigger physics update (rcssserver3d bug when no joint is moving)
        self.sync()

        # stabilize on ground
        for _ in range(7): 
            self.player.behavior.execute("Zero_Bent_Knees")
            self.sync()

        # memory variables
        self.lastx = r.cheat_abs_pos[0]
        self.act = np.zeros(self.no_of_actions,np.float32)

        return self.observe(True)

    def render(self, mode='human', close=False):
        return

    def close(self):
        Draw.clear_all()
        self.player.terminate()

    def step(self, action):
        
        r = self.player.world.robot

        # exponential moving average
        self.act = 0.4 * self.act + 0.6 * action

        # execute Step behavior to extract the target positions of each leg (we will override these targets)
        if self.step_counter == 0:
            '''
            The first time step will change the parameters of the next footstep
            It uses default parameters so that the agent can anticipate the next generated pose
            Reason: the agent decides the parameters during the previous footstep
            '''
            self.player.behavior.execute("Step", self.step_default_dur, self.step_default_z_span, self.step_default_z_max)
        else:
            
            step_zsp =     np.clip(self.step_default_z_span + self.act[20]/300,   0,     0.07)
            step_zmx =     np.clip(self.step_default_z_max  + self.act[21]/30,    0.6,   0.9)

            self.player.behavior.execute("Step", self.step_default_dur, step_zsp, step_zmx)

        
        # add action as residuals to Step behavior (the index of these actions is not the typical index because both head joints are excluded)
        new_action = self.act[:20] * 2 # scale up actions to motivate exploration
        new_action[[0,2,4,6,8,10]] += self.step_obj.values_l
        new_action[[1,3,5,7,9,11]] += self.step_obj.values_r
        new_action[12] -= 90 # arms down
        new_action[13] -= 90 # arms down
        new_action[16] += 90 # untwist arms
        new_action[17] += 90 # untwist arms
        new_action[18] += 90 # elbows at 90 deg
        new_action[19] += 90 # elbows at 90 deg

        r.set_joints_target_position_direct( # commit actions:
            slice(2,22),        # act on all joints except head & toes (for robot type 4)
            new_action,         # target joint positions 
            harmonize=False     # there is no point in harmonizing actions if the targets change at every step  
        )

        self.sync() # run simulation step
        self.step_counter += 1
         
        reward = r.cheat_abs_pos[0] - self.lastx
        self.lastx = r.cheat_abs_pos[0]

        # terminal state: the robot is falling or timeout
        terminal = r.cheat_abs_pos[2] < 0.3 or self.step_counter > 300

        return self.observe(), reward, terminal, {}





class Train(Train_Base):
    def __init__(self, script) -> None:
        super().__init__(script)


    def train(self, args):

        #--------------------------------------- Learning parameters
        n_envs = min(16, os.cpu_count())
        n_steps_per_env = 1024  # RolloutBuffer is of size (n_steps_per_env * n_envs)
        minibatch_size = 64    # should be a factor of (n_steps_per_env * n_envs)
        total_steps = 30000000
        learning_rate = 3e-4
        folder_name = f'Basic_Run_R{self.robot_type}'
        model_path = f'./scripts/gyms/logs/{folder_name}/'

        print("Model path:", model_path)

        #--------------------------------------- Run algorithm
        def init_env(i_env):
            def thunk():
                return Basic_Run( self.ip , self.server_p + i_env, self.monitor_p_1000 + i_env, self.robot_type, False )
            return thunk

        servers = Server( self.server_p, self.monitor_p_1000, n_envs+1 ) #include 1 extra server for testing

        env = SubprocVecEnv( [init_env(i) for i in range(n_envs)] )
        eval_env = SubprocVecEnv( [init_env(n_envs)] )

        try:
            if "model_file" in args: # retrain
                model = PPO.load( args["model_file"], env=env, device="cpu", n_envs=n_envs, n_steps=n_steps_per_env, batch_size=minibatch_size, learning_rate=learning_rate )
            else: # train new model
                model = PPO( "MlpPolicy", env=env, verbose=1, n_steps=n_steps_per_env, batch_size=minibatch_size, learning_rate=learning_rate, device="cpu" )

            model_path = self.learn_model( model, total_steps, model_path, eval_env=eval_env, eval_freq=n_steps_per_env*20, save_freq=n_steps_per_env*200, backup_env_file=__file__ )
        except KeyboardInterrupt:
            sleep(1) # wait for child processes
            print("\nctrl+c pressed, aborting...\n")
            servers.kill()
            return
    
        env.close()
        eval_env.close()
        servers.kill()
        

    def test(self, args):

        # Uses different server and monitor ports
        server = Server( self.server_p-1, self.monitor_p, 1 )
        env = Basic_Run( self.ip, self.server_p-1, self.monitor_p, self.robot_type, True )
        model = PPO.load( args["model_file"], env=env )

        try:
            self.export_model( args["model_file"], args["model_file"]+".pkl", False )  # Export to pkl to create custom behavior
            self.test_model( model, env, log_path=args["folder_dir"], model_path=args["folder_dir"] )
        except KeyboardInterrupt:
            print()

        env.close()
        server.kill()


'''
The learning process takes several hours.
A video with the results can be seen at:
https://imgur.com/a/dC2V6Et

Stats:
- Avg. reward:     7.7 
- Avg. ep. length: 5.5s (episode is limited to 6s)
- Max. reward:     9.3  (speed: 1.55m/s)    

State space:
- Composed of all joint positions + torso height
- Stage of the underlying Step behavior

Reward:
- Displacement in the x-axis (it can be negative)
- Note that cheat and visual data is only updated every 3 steps
'''
