from agent.Base_Agent import Base_Agent as Agent
from pathlib import Path
from scripts.commons.Server import Server
from scripts.commons.Train_Base import Train_Base
from stable_baselines3 import PPO
from stable_baselines3.common.base_class import BaseAlgorithm
from stable_baselines3.common.vec_env import SubprocVecEnv
from time import sleep
from world.commons.Draw import Draw
import gym
import numpy as np
import os

'''
Objective:
Learn how to get up (4 variants, see line 157)
Optimize each keyframe of existing slot behaviors
----------
- class Get_Up: implements an OpenAI custom gym
- class Train:  implements algorithms to train a new model or test an existing model
'''

class Get_Up(gym.Env):
    def __init__(self, ip, server_p, monitor_p, r_type, fall_direction, enable_draw) -> None:
        self.robot_type = r_type
        self.fall_direction = fall_direction # 0:front, 1:left side, 2:right side, 3:back
        self.player = Agent(ip, server_p, monitor_p, 1, self.robot_type, "Gym", True, enable_draw, [])
        self.get_up_names = {0:"Get_Up_Front", 1:"Get_Up_Side_Left", 2:"Get_Up_Side_Right", 3:"Get_Up_Back"}

        # Backup original slots
        self.original_slots = []
        for delta_ms, indices, angles in self.player.behavior.slot_engine.behaviors[self.get_up_names[self.fall_direction]]:
            self.original_slots.append((delta_ms, indices, np.array(angles)))

        self.obs = np.identity(len(self.original_slots)) # one-hot encoding for each slot
        self.current_slot = 0

        MAX = np.finfo(np.float32).max
        self.action_space = gym.spaces.Box(low=np.full(11,-MAX,np.float32), high=np.full(11,MAX,np.float32), dtype=np.float32)
        self.observation_space = gym.spaces.Box(low=np.zeros(len(self.obs),np.float32), high=np.ones(len(self.obs),np.float32), dtype=np.float32)


    def fall(self):
        r = self.player.world.robot
        joint_indices = [r.J_LFOOT_PITCH,
                         r.J_RFOOT_PITCH,
                         r.J_LLEG_ROLL,
                         r.J_RLEG_ROLL]

        if self.fall_direction == 0:
            r.set_joints_target_position_direct(joint_indices, np.array([50,50,0,0]))
        elif self.fall_direction == 1:
            r.set_joints_target_position_direct(joint_indices, np.array([0,0,-20,20]))
        elif self.fall_direction == 2:
            r.set_joints_target_position_direct(joint_indices, np.array([0,0,20,-20]))
        elif self.fall_direction == 3:
            r.set_joints_target_position_direct(joint_indices, np.array([-20,-20,0,0]))
        else:
            raise ValueError

        self.player.scom.commit_and_send( r.get_command() )
        self.player.scom.receive()


    def get_up(self):
        r = self.player.world.robot
        finished = self.player.behavior.execute(self.get_up_names[self.fall_direction])

        self.player.scom.commit_and_send( r.get_command() )
        self.player.scom.receive()
        return finished


    def other(self, behavior_name):
        r = self.player.world.robot
        self.player.behavior.execute(behavior_name)
        self.player.scom.commit_and_send( r.get_command() )
        self.player.scom.receive()


    def reset(self):
        self.player.scom.commit_beam((-3,0),0)

        for _ in range(30): self.fall()
        while self.player.world.robot.cheat_abs_pos[2] > 0.32: self.fall()
        
        import random
        t = random.randint(7,17) if self.fall_direction==0 else random.randint(10,20)
        for _ in range(t):  self.other("Zero")

        self.current_slot = 0

        return self.obs[self.current_slot]


    def render(self, mode='human', close=False):
        return


    def close(self):
        Draw.clear_all()
        self.player.scom.close()


    @staticmethod
    def scale_action(action : np.ndarray):
        new_action = np.zeros(len(action)*2-1,action.dtype) 
        new_action[0]  = action[0] * 10
        new_action[1:] = np.repeat(action[1:] * 3,2) # expand symmetrical actions

        return new_action


    @staticmethod
    def get_22_angles(angles, indices):
        new_angles = np.zeros(22, np.float32) # all joints except for toes
        new_angles[indices] = angles          # get all joints that are defined in the XML (otherwise, assume 0)

        return new_angles


    def step(self, action):
        #action: 1 delta + 10 joints
        r = self.player.world.robot
        action = Get_Up.scale_action(action)

        delta, indices, angles = self.original_slots[self.current_slot]
        angles = Get_Up.get_22_angles(angles, indices)

        angles[2:] += action[1:] # exclude head
        new_delta = max((delta + action[0])//20*20, 20)

        self.player.behavior.slot_engine.behaviors[self.get_up_names[self.fall_direction]][self.current_slot] = ( new_delta, slice(0,22), angles )

        self.current_slot += 1
        terminal = bool(self.current_slot == len(self.obs))
        reward = 0
        

        if terminal: # network has set values for all keyframes, now run behavior and evaluate
            
            while not self.get_up():
                reward -= 0.05

            for _ in range(50): 
                self.other("Zero_Bent_Knees")
                reward += r.cheat_abs_pos[2] * 0.95**abs(r.gyro[1])

            print("rew:", reward)
            obs = self.obs[0] # dummy observation
        else:
            obs = self.obs[self.current_slot]


        return obs, reward, terminal, {}




class Train(Train_Base):
    def __init__(self, script) -> None:
        super().__init__(script)

        self.fall_direction = 0 # 0:front, 1:left side, 2:right side, 3:back


    def train(self, args):

        n_envs = min(15, os.cpu_count())
        n_steps_per_env = 72
        minibatch_size = 72 # should be a factor of n_steps_per_env * n_envs
        total_steps = 1000
        learning_rate = 2e-4
        folder_name = f'GetUp_R{self.robot_type}_Direction{self.fall_direction}'
        model_path = f'./scripts/gyms/logs/{folder_name}/'

        print("Model path:", model_path)

        def init_env(i_env):
            def thunk():
                return Get_Up( self.ip , self.server_p + i_env, self.monitor_p_1000 + i_env, self.robot_type, self.fall_direction, False )
            return thunk

        servers = Server( self.server_p, self.monitor_p_1000, n_envs+1 ) #include 1 extra server for testing

        env = SubprocVecEnv( [init_env(i) for i in range(n_envs)] )
        eval_env = SubprocVecEnv( [init_env(n_envs)] )

        try:
            if "model_file" in args:
                model = PPO.load( args["model_file"], env=env, n_envs=n_envs, n_steps=n_steps_per_env, batch_size=minibatch_size, learning_rate=learning_rate )
            else:
                model = PPO( "MlpPolicy", env=env, verbose=1, n_steps=n_steps_per_env, batch_size=minibatch_size, learning_rate=learning_rate )

            model_path = self.learn_model( model, total_steps, model_path, eval_env=eval_env, eval_freq=n_steps_per_env*10, backup_env_file=__file__ )
        except KeyboardInterrupt:
            sleep(1) # wait for child processes
            print("\nctrl+c pressed, aborting...\n")
            servers.kill()
            return

        
        # Generate slot behavior XML
        self.generate_get_up_behavior(model, model_path, eval_env.get_attr('original_slots')[0], "last_model.xml")
        
        env.close()
        eval_env.close()
        servers.kill()
        

    def test(self, args):

        # Uses different server and monitor ports
        server = Server( self.server_p-1, self.monitor_p, 1 )
        env = Get_Up( self.ip, self.server_p-1, self.monitor_p, self.robot_type, self.fall_direction, True )
        model = PPO.load( args["model_file"], env=env )

        # Generate slot behavior XML
        XML_name = Path(args["model_file"]).stem + ".xml"
        if not os.path.isfile(os.path.join( args["folder_dir"], XML_name )):
            self.generate_get_up_behavior(model, args["folder_dir"], env.original_slots, XML_name )

        self.test_model( model, env, log_path=args["folder_dir"], model_path=args["folder_dir"] )

        env.close()
        server.kill()


    def generate_get_up_behavior(self, model : BaseAlgorithm, folder_dir, original_slots, XML_name):

        predictions = model.predict(np.identity(len(original_slots)),deterministic=True)[0]
        slots = []

        for i in range(len(predictions)):
            pred = Get_Up.scale_action(predictions[i])
            delta = max((original_slots[i][0] + pred[0])//20*20, 20)
            angles = Get_Up.get_22_angles(original_slots[i][2], original_slots[i][1])
            angles[2:] += pred[1:]
            slots.append((delta, range(22), angles))

        self.generate_slot_behavior( folder_dir, slots, False, XML_name )
