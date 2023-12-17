from agent.Base_Agent import Base_Agent as Agent
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
Learn how to fall (simplest example)
----------
- class Fall: implements an OpenAI custom gym
- class Train:  implements algorithms to train a new model or test an existing model
'''

class Fall(gym.Env):
    def __init__(self, ip, server_p, monitor_p, r_type, enable_draw) -> None:

        self.robot_type = r_type

        # Args: Server IP, Agent Port, Monitor Port, Uniform No., Robot Type, Team Name, Enable Log, Enable Draw
        self.player = Agent(ip, server_p, monitor_p, 1, self.robot_type, "Gym", True, enable_draw)
        self.step_counter = 0 # to limit episode size

        # State space
        self.no_of_joints = self.player.world.robot.no_of_joints
        self.obs = np.zeros(self.no_of_joints + 1, np.float32) # joints + torso height
        self.observation_space = gym.spaces.Box(low=np.full(len(self.obs),-np.inf,np.float32), high=np.full(len(self.obs),np.inf,np.float32), dtype=np.float32)

        # Action space
        MAX = np.finfo(np.float32).max
        no_of_actions = self.no_of_joints
        self.action_space = gym.spaces.Box(low=np.full(no_of_actions,-MAX,np.float32), high=np.full(no_of_actions,MAX,np.float32), dtype=np.float32)

        # Check if cheats are enabled
        assert np.any(self.player.world.robot.cheat_abs_pos), "Cheats are not enabled! Run_Utils.py -> Server -> Cheats"
        

    def observe(self):

        r = self.player.world.robot
        
        for i in range(self.no_of_joints):
            self.obs[i] = r.joints_position[i] / 100 # naive scale normalization

        self.obs[self.no_of_joints] = r.cheat_abs_pos[2] # head.z (alternative: r.loc_head_z)

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
            self.player.scom.unofficial_beam((-3,0,0.50),0) # beam player continuously (floating above ground)
            self.player.behavior.execute("Zero")
            self.sync()

        # beam player to ground
        self.player.scom.unofficial_beam((-3,0,r.beam_height),0) 
        r.joints_target_speed[0] = 0.01 # move head to trigger physics update (rcssserver3d bug when no joint is moving)
        self.sync()

        # stabilize on ground
        for _ in range(7): 
            self.player.behavior.execute("Zero")
            self.sync()

        return self.observe()

    def render(self, mode='human', close=False):
        return

    def close(self):
        Draw.clear_all()
        self.player.terminate()

    def step(self, action):
        
        r = self.player.world.robot
        r.set_joints_target_position_direct( # commit actions:
            slice(self.no_of_joints),        # act on all available joints
            action*10,                       # scale actions up to motivate early exploration
            harmonize=False                  # there is no point in harmonizing actions if the targets change at every step  
        )

        self.sync() # run simulation step
        self.step_counter += 1
        self.observe() 

        if self.obs[-1] < 0.15:           # terminal state: the robot has fallen successfully
            return self.obs, 1, True, {}  # Reward: 1 (this reward will motivate a fast reaction if the return is discounted)
        elif self.step_counter > 150:     # terminal state: 3s passed and robot has not fallen (may be stuck)
            return self.obs, 0, True, {}
        else:
            return self.obs, 0, False, {} # Reward: 0





class Train(Train_Base):
    def __init__(self, script) -> None:
        super().__init__(script)


    def train(self, args):

        #--------------------------------------- Learning parameters
        n_envs = min(4, os.cpu_count())
        n_steps_per_env = 128   # RolloutBuffer is of size (n_steps_per_env * n_envs) (*RV: >=2048)
        minibatch_size = 64     # should be a factor of (n_steps_per_env * n_envs)
        total_steps = 50000     # (*RV: >=10M)
        learning_rate = 30e-4   # (*RV: 3e-4)
        # *RV -> Recommended value for more complex environments
        folder_name = f'Fall_R{self.robot_type}'
        model_path = f'./scripts/gyms/logs/{folder_name}/'

        print("Model path:", model_path)

        #--------------------------------------- Run algorithm
        def init_env(i_env):
            def thunk():
                return Fall( self.ip , self.server_p + i_env, self.monitor_p_1000 + i_env, self.robot_type, False )
            return thunk

        servers = Server( self.server_p, self.monitor_p_1000, n_envs+1 ) #include 1 extra server for testing

        env = SubprocVecEnv( [init_env(i) for i in range(n_envs)] )
        eval_env = SubprocVecEnv( [init_env(n_envs)] )

        try:
            if "model_file" in args: # retrain
                model = PPO.load( args["model_file"], env=env, n_envs=n_envs, n_steps=n_steps_per_env, batch_size=minibatch_size, learning_rate=learning_rate )
            else: # train new model
                model = PPO( "MlpPolicy", env=env, verbose=1, n_steps=n_steps_per_env, batch_size=minibatch_size, learning_rate=learning_rate )

            model_path = self.learn_model( model, total_steps, model_path, eval_env=eval_env, eval_freq=n_steps_per_env*10, save_freq=n_steps_per_env*20, backup_env_file=__file__ )
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
        env = Fall( self.ip, self.server_p-1, self.monitor_p, self.robot_type, True )
        model = PPO.load( args["model_file"], env=env )

        try:
            self.export_model( args["model_file"], args["model_file"]+".pkl", False )  # Export to pkl to create custom behavior
            self.test_model( model, env, log_path=args["folder_dir"], model_path=args["folder_dir"] )
        except KeyboardInterrupt:
            print()

        env.close()
        server.kill()


'''
The learning process takes about 5 minutes.
A video with the results can be seen at:
https://imgur.com/a/KvpXS41

State space:
- Composed of all joint positions + torso height
- The number of joint positions is different for robot type 4, so the models are not interchangeable
- For this example, this problem can be avoided by using only the first 22 joints and actuators

Reward:
- The reward for falling is 1, which means that after a while every episode will have a r=1.
- What is the incetive for the robot to fall faster? Discounted return.
  In every state, the algorithm will seek short-term rewards.
- During training, the best model is saved according to the average return, which is almost always 1.
  Therefore, the last model will typically be superior for this example.

Expected evolution of episode length:
    3s|o
      |o
      | o
      |  o
      |   oo
      |     ooooo
  0.4s|          oooooooooooooooo
      |------------------------------> time


This example scales poorly with the number of CPUs because:
- It uses a small rollout buffer (n_steps_per_env * n_envs)
- The simulation workload is light
- For these reasons, the IPC overhead is significant
'''
