import numpy as np


class Behavior():

    def __init__(self, base_agent) -> None:
        from agent.Base_Agent import Base_Agent # for type hinting
        self.base_agent : Base_Agent = base_agent
        self.world = self.base_agent.world
        self.state_behavior_name = None
        self.state_behavior_init_ms = 0
        self.previous_behavior = None
        self.previous_behavior_duration = None

        #Initialize standard behaviors
        from behaviors.Poses import Poses
        from behaviors.Slot_Engine import Slot_Engine
        from behaviors.Head import Head

        self.poses = Poses(self.world)
        self.slot_engine = Slot_Engine(self.world)
        self.head = Head(self.world)


    def create_behaviors(self):   
        '''
        Behaviors dictionary:
            creation:   key: ( description, auto_head, lambda reset[,a,b,c,..]: self.execute(...), lambda: self.is_ready(...) )
            usage:      key: ( description, auto_head, execute_func(reset, *args), is_ready_func )
        '''
        self.behaviors = self.poses.get_behaviors_callbacks()
        self.behaviors.update(self.slot_engine.get_behaviors_callbacks())
        self.behaviors.update(self.get_custom_callbacks())


    def get_custom_callbacks(self):
        '''
        Searching custom behaviors could be implemented automatically
        However, for code distribution, loading code dynamically is not ideal (unless we load byte code or some other import solution)
        Currently, adding custom behaviors is a manual process:
            1. Add import statement below
            2. Add class to 'classes' list
        '''

        # Declaration of behaviors
        from behaviors.custom.Basic_Kick.Basic_Kick import Basic_Kick
        from behaviors.custom.Dribble.Dribble import Dribble
        from behaviors.custom.Fall.Fall import Fall
        from behaviors.custom.Get_Up.Get_Up import Get_Up
        from behaviors.custom.Step.Step import Step
        from behaviors.custom.Walk.Walk import Walk
        classes = [Basic_Kick,Dribble,Fall,Get_Up,Step,Walk]

        '''---- End of manual declarations ----'''

        # Prepare callbacks
        self.objects = {cls.__name__ : cls(self.base_agent) for cls in classes}

        return {name: (o.description,o.auto_head,
                       lambda reset,*args,o=o: o.execute(reset,*args), lambda *args,o=o: o.is_ready(*args)) for name, o in self.objects.items()}


    def get_custom_behavior_object(self, name):
        ''' Get unique object from class "name" ("name" must represent a custom behavior) '''
        assert name in self.objects, f"There is no custom behavior called {name}"
        return self.objects[name]
        

    def get_all_behaviors(self):
        ''' Get name and description of all behaviors '''
        return [ key for key in self.behaviors ], [ val[0] for val in self.behaviors.values() ]


    def get_current(self):
        ''' Get name and duration (in seconds) of current behavior '''
        duration = (self.world.time_local_ms - self.state_behavior_init_ms) / 1000.0
        return self.state_behavior_name, duration

    
    def get_previous(self):
        ''' Get name and duration (in seconds) of previous behavior '''
        return self.previous_behavior, self.previous_behavior_duration


    def force_reset(self):
        ''' Force reset next executed behavior '''
        self.state_behavior_name = None


    def execute(self, name, *args) -> bool:
        ''' 
        Execute one step of behavior `name` with arguments `*args`
        - Automatically resets behavior on first call
        - Call get_current() to get the current behavior (and its duration)

        Returns
        -------
        finished : bool
            True if behavior has finished
        '''

        assert name in self.behaviors, f"Behavior {name} does not exist!"

        # Check if transitioning from other behavior
        reset = bool(self.state_behavior_name != name)
        if reset: 
            if self.state_behavior_name is not None:
                self.previous_behavior = self.state_behavior_name # Previous behavior was interrupted (did not finish)
            self.previous_behavior_duration = (self.world.time_local_ms - self.state_behavior_init_ms) / 1000.0
            self.state_behavior_name = name
            self.state_behavior_init_ms = self.world.time_local_ms

        # Control head orientation if behavior allows it
        if self.behaviors[name][1]:
            self.head.execute()

        # Execute behavior
        if not self.behaviors[name][2](reset,*args):
            return False

        # The behavior has finished
        self.previous_behavior = self.state_behavior_name # Store current behavior name
        self.state_behavior_name = None
        return True


    def execute_sub_behavior(self, name, reset, *args):
        '''
        Execute one step of behavior `name` with arguments `*args`
        Useful for custom behaviors that call other behaviors
        - Behavior reset is performed manually
        - Calling get_current() will return the main behavior (not the sub behavior)
        - Poses ignore the reset argument

        Returns
        -------
        finished : bool
            True if behavior has finished
        '''

        assert name in self.behaviors, f"Behavior {name} does not exist!"

        # Control head orientation if behavior allows it
        if self.behaviors[name][1]:
            self.head.execute()

        # Execute behavior
        return self.behaviors[name][2](reset,*args)

    
    def execute_to_completion(self, name, *args):
        ''' 
        Execute steps and communicate with server until completion 
        - Slot behaviors indicate that the behavior has finished when sending the last command (which is promptly sent)
        - Poses are finished when the server returns the desired robot state (so the last command is irrelevant)
        - For custom behaviors, we assume the same logic, and so, the last command is ignored

        Notes
        -----
        - Before exiting, the `Robot.joints_target_speed` array is reset to avoid polluting the next command
        - For poses and custom behaviors that indicate a finished behavior on the 1st call, nothing is committed or sent
        - Warning: this function may get stuck in an infinite loop if the behavior never ends
        '''

        r = self.world.robot
        skip_last = name not in self.slot_engine.behaviors

        while True:
            done = self.execute(name, *args)
            if done and skip_last: break # Exit here if last command is irrelevant
            self.base_agent.scom.commit_and_send( r.get_command() ) 
            self.base_agent.scom.receive()
            if done: break # Exit here if last command is part of the behavior

        # reset to avoid polluting the next command
        r.joints_target_speed = np.zeros_like(r.joints_target_speed)


    def is_ready(self, name, *args) -> bool:
        ''' Checks if behavior is ready to start under current game/robot conditions '''

        assert name in self.behaviors, f"Behavior {name} does not exist!"
        return self.behaviors[name][3](*args)