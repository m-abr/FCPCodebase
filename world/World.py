from collections import deque
from cpp.ball_predictor import ball_predictor
from cpp.localization import localization
from logs.Logger import Logger
from math import atan2, pi
from math_ops.Matrix_4x4 import Matrix_4x4
from world.commons.Draw import Draw
from world.commons.Other_Robot import Other_Robot
from world.Robot import Robot
import numpy as np


class World():
    STEPTIME = 0.02    # Fixed step time
    STEPTIME_MS = 20   # Fixed step time in milliseconds
    VISUALSTEP = 0.04  # Fixed visual step time
    VISUALSTEP_MS = 40 # Fixed visual step time in milliseconds

    # play modes in our favor
    M_OUR_KICKOFF = 0
    M_OUR_KICK_IN = 1
    M_OUR_CORNER_KICK = 2
    M_OUR_GOAL_KICK = 3
    M_OUR_FREE_KICK = 4
    M_OUR_PASS = 5
    M_OUR_DIR_FREE_KICK = 6
    M_OUR_GOAL = 7
    M_OUR_OFFSIDE = 8

    # play modes in their favor
    M_THEIR_KICKOFF = 9
    M_THEIR_KICK_IN = 10
    M_THEIR_CORNER_KICK = 11
    M_THEIR_GOAL_KICK = 12
    M_THEIR_FREE_KICK = 13
    M_THEIR_PASS = 14
    M_THEIR_DIR_FREE_KICK = 15
    M_THEIR_GOAL = 16
    M_THEIR_OFFSIDE = 17

    # neutral play modes
    M_BEFORE_KICKOFF = 18
    M_GAME_OVER = 19
    M_PLAY_ON = 20

    # play mode groups
    MG_OUR_KICK = 0
    MG_THEIR_KICK = 1
    MG_ACTIVE_BEAM = 2
    MG_PASSIVE_BEAM = 3
    MG_OTHER = 4 # play on, game over

    FLAGS_CORNERS_POS = ((-15,-10,0), (-15,+10,0), (+15,-10,0), (+15,+10,0))
    FLAGS_POSTS_POS = ((-15,-1.05,0.8),(-15,+1.05,0.8),(+15,-1.05,0.8),(+15,+1.05,0.8))

    def __init__(self,robot_type:int, team_name:str, unum:int, apply_play_mode_correction:bool, 
                 enable_draw:bool, logger:Logger, host:str) -> None:

        self.team_name = team_name               # Name of our team
        self.team_name_opponent : str = None     # Name of opponent team
        self.apply_play_mode_correction = apply_play_mode_correction # True to adjust ball position according to play mode
        self.step = 0           # Total number of received simulation steps (always in sync with self.time_local_ms)
        self.time_server = 0.0  # Time, in seconds, as indicated by the server (this time is NOT reliable, use only for synchronization between agents)
        self.time_local_ms = 0  # Reliable simulation time in milliseconds, use this when possible (it is incremented 20ms for every TCP message)
        self.time_game = 0.0    # Game time, in seconds, as indicated by the server
        self.goals_scored = 0   # Goals score by our team
        self.goals_conceded = 0 # Goals conceded by our team
        self.team_side_is_left : bool = None # True if our team plays on the left side (this value is later changed by the world parser)
        self.play_mode = None                # Play mode of the soccer game, provided by the server  
        self.play_mode_group = None          # Certain play modes share characteristics, so it makes sense to group them
        self.flags_corners : dict = None     # corner flags, key=(x,y,z), always assume we play on the left side
        self.flags_posts : dict = None       # goal   posts, key=(x,y,z), always assume we play on the left side
        self.ball_rel_head_sph_pos = np.zeros(3)     # Ball position relative to head  (spherical coordinates) (m, deg, deg)
        self.ball_rel_head_cart_pos = np.zeros(3)    # Ball position relative to head  (cartesian coordinates) (m)
        self.ball_rel_torso_cart_pos = np.zeros(3)   # Ball position relative to torso (cartesian coordinates) (m)
        self.ball_rel_torso_cart_pos_history = deque(maxlen=20) # Ball position relative to torso history (queue with up to 20 old positions at intervals of 0.04s, where index 0 is the previous position)
        self.ball_abs_pos = np.zeros(3)              # Ball absolute position (up to date if self.ball_is_visible and self.robot.loc_is_up_to_date) (m)
        self.ball_abs_pos_history = deque(maxlen=20) # Ball absolute position history (queue with up to 20 old positions at intervals of 0.04s, where index 0 is the previous position)
        self.ball_abs_pos_last_update = 0        # World.time_local_ms when self.ball_abs_pos was last updated by vision or radio
        self.ball_abs_vel = np.zeros(3)          # Ball velocity vector based on the last 2 known values of self.ball_abs_pos (m/s) (Warning: noisy if ball is distant, use instead get_ball_abs_vel)
        self.ball_abs_speed = 0                  # Ball scalar speed based on the last 2 known values of self.ball_abs_pos (m/s)    (Warning: noisy if ball is distant, use instead ||get_ball_abs_vel||)
        self.ball_is_visible = False             # True if the last server message contained vision information related to the ball
        self.is_ball_abs_pos_from_vision = False # True if ball_abs_pos originated from vision, False if it originated from radio
        self.ball_last_seen = 0                  # World.time_local_ms when ball was last seen (note: may be different from self.ball_abs_pos_last_update)
        self.ball_cheat_abs_pos = np.zeros(3)    # Absolute ball position provided by the server as cheat (m)
        self.ball_cheat_abs_vel = np.zeros(3)    # Absolute velocity vector based on the last 2 values of self.ball_cheat_abs_pos (m/s)
        self.ball_2d_pred_pos = np.zeros((1,2))  # prediction of current and future 2D ball positions*
        self.ball_2d_pred_vel = np.zeros((1,2))  # prediction of current and future 2D ball velocities*
        self.ball_2d_pred_spd = np.zeros(1)      # prediction of current and future 2D ball linear speeds*
        # *at intervals of 0.02 s until ball comes to a stop or gets out of bounds (according to prediction)
        self.lines = np.zeros((30,6))            # Position of visible lines, relative to head, start_pos+end_pos (spherical coordinates) (m, deg, deg, m, deg, deg)
        self.line_count = 0                      # Number of visible lines
        self.vision_last_update = 0                                   # World.time_local_ms when last vision update was received
        self.vision_is_up_to_date = False                             # True if the last server message contained vision information
        self.teammates = [Other_Robot(i, True ) for i in range(1,12)] # List of teammates, ordered by unum
        self.opponents = [Other_Robot(i, False) for i in range(1,12)] # List of opponents, ordered by unum
        self.teammates[unum-1].is_self = True                         # This teammate is self
        self.draw = Draw(enable_draw, unum, host, 32769)              # Draw object for current player
        self.team_draw = Draw(enable_draw, 0, host, 32769)            # Draw object shared with teammates
        self.logger = logger
        self.robot = Robot(unum, robot_type)


    def log(self, msg:str):
        '''
        Shortcut for:

        self.logger.write(msg, True, self.step)

        Parameters
        ----------
        msg : str
            message to be written after the simulation step
        ''' 
        self.logger.write(msg, True, self.step)

    def get_ball_rel_vel(self, history_steps:int):
        '''
        Get ball velocity, relative to torso (m/s)

        Parameters
        ----------
        history_steps : int
            number of history steps to consider [1,20]

        Examples
        --------
        get_ball_rel_vel(1) is equivalent to (current rel pos - last rel pos)      / 0.04
        get_ball_rel_vel(2) is equivalent to (current rel pos - rel pos 0.08s ago) / 0.08
        get_ball_rel_vel(3) is equivalent to (current rel pos - rel pos 0.12s ago) / 0.12
        '''
        assert 1 <= history_steps <= 20, "Argument 'history_steps' must be in range [1,20]"

        if len(self.ball_rel_torso_cart_pos_history) == 0:
            return np.zeros(3)

        h_step = min(history_steps, len(self.ball_rel_torso_cart_pos_history))
        t = h_step * World.VISUALSTEP

        return (self.ball_rel_torso_cart_pos - self.ball_rel_torso_cart_pos_history[h_step-1]) / t

    def get_ball_abs_vel(self, history_steps:int):
        '''
        Get ball absolute velocity (m/s)

        Parameters
        ----------
        history_steps : int
            number of history steps to consider [1,20]

        Examples
        --------
        get_ball_abs_vel(1) is equivalent to (current abs pos - last abs pos)      / 0.04
        get_ball_abs_vel(2) is equivalent to (current abs pos - abs pos 0.08s ago) / 0.08
        get_ball_abs_vel(3) is equivalent to (current abs pos - abs pos 0.12s ago) / 0.12
        '''
        assert 1 <= history_steps <= 20, "Argument 'history_steps' must be in range [1,20]"

        if len(self.ball_abs_pos_history) == 0:
            return np.zeros(3)

        h_step = min(history_steps, len(self.ball_abs_pos_history))
        t = h_step * World.VISUALSTEP

        return (self.ball_abs_pos - self.ball_abs_pos_history[h_step-1]) / t

    def get_predicted_ball_pos(self, max_speed):
        '''
        Get predicted 2D ball position when its predicted speed is equal to or less than `max_speed`
        In case that position exceeds the prediction horizon, the last available prediction is returned

        Parameters
        ----------
        max_speed : float
            maximum speed at which the ball will be moving at returned future position
        '''
        b_sp = self.ball_2d_pred_spd
        index = len(b_sp) - max( 1, np.searchsorted(b_sp[::-1], max_speed, side='right') )
        return self.ball_2d_pred_pos[index]
    
    def get_intersection_point_with_ball(self, player_speed):
        '''
        Get 2D intersection point with moving ball, based on `self.ball_2d_pred_pos`

        Parameters
        ----------
        player_speed : float
            average speed at which the robot will chase the ball

        Returns
        -------
        2D intersection point : ndarray
            2D intersection point with moving ball, assuming the robot moves at an avg. speed of `player_speed`
        intersection distance : float
            distance between current robot position and intersection point
        '''
        
        params = np.array([*self.robot.loc_head_position[:2], player_speed*0.02, *self.ball_2d_pred_pos.flat], np.float32)
        pred_ret  = ball_predictor.get_intersection(params)
        return pred_ret[:2], pred_ret[2]
    
    def update(self):
        r = self.robot
        PM = self.play_mode
        W = World

        # reset variables
        r.loc_is_up_to_date = False                   
        r.loc_head_z_is_up_to_date = False

        # update play mode groups
        if PM in (W.M_PLAY_ON, W.M_GAME_OVER): # most common group
            self.play_mode_group = W.MG_OTHER
        elif PM in (W.M_OUR_KICKOFF, W.M_OUR_KICK_IN, W.M_OUR_CORNER_KICK, W.M_OUR_GOAL_KICK,
                    W.M_OUR_OFFSIDE, W.M_OUR_PASS, W.M_OUR_DIR_FREE_KICK, W.M_OUR_FREE_KICK):
            self.play_mode_group = W.MG_OUR_KICK
        elif PM in (W.M_THEIR_KICK_IN, W.M_THEIR_CORNER_KICK, W.M_THEIR_GOAL_KICK, W.M_THEIR_OFFSIDE,
                    W.M_THEIR_PASS, W.M_THEIR_DIR_FREE_KICK, W.M_THEIR_FREE_KICK, W.M_THEIR_KICKOFF):
            self.play_mode_group = W.MG_THEIR_KICK
        elif PM in (W.M_BEFORE_KICKOFF, W.M_THEIR_GOAL):
            self.play_mode_group = W.MG_ACTIVE_BEAM
        elif PM in (W.M_OUR_GOAL,):
            self.play_mode_group = W.MG_PASSIVE_BEAM
        elif PM is not None:
            raise ValueError(f'Unexpected play mode ID: {PM}')

        r.update_pose() # update forward kinematics

        if self.ball_is_visible:
            # Compute ball position, relative to torso
            self.ball_rel_torso_cart_pos = r.head_to_body_part_transform("torso",self.ball_rel_head_cart_pos)

        if self.vision_is_up_to_date: # update vision based localization 

            # Prepare all variables for localization

            feet_contact = np.zeros(6)

            lf_contact = r.frp.get('lf', None)
            rf_contact = r.frp.get('rf', None)
            if lf_contact is not None:
                feet_contact[0:3] = Matrix_4x4( r.body_parts["lfoot"].transform ).translate( lf_contact[0:3] , True).get_translation()
            if rf_contact is not None:
                feet_contact[3:6] = Matrix_4x4( r.body_parts["rfoot"].transform ).translate( rf_contact[0:3] , True).get_translation()

            ball_pos = np.concatenate(( self.ball_rel_head_cart_pos, self.ball_cheat_abs_pos))
            
            corners_list = [[key in self.flags_corners, 1.0, *key, *self.flags_corners.get(key,(0,0,0))] for key in World.FLAGS_CORNERS_POS]
            posts_list   = [[key in self.flags_posts  , 0.0, *key, *self.flags_posts.get(  key,(0,0,0))] for key in World.FLAGS_POSTS_POS]
            all_landmarks = np.array(corners_list + posts_list, float)

            # Compute localization

            loc = localization.compute(
                r.feet_toes_are_touching['lf'],
                r.feet_toes_are_touching['rf'],
                feet_contact,
                self.ball_is_visible,
                ball_pos,
                r.cheat_abs_pos,
                all_landmarks,
                self.lines[0:self.line_count])  

            r.update_localization(loc, self.time_local_ms)

            # Update self in teammates list (only the most useful parameters, add as needed)
            me = self.teammates[r.unum-1]
            me.state_last_update = r.loc_last_update
            me.state_abs_pos = r.loc_head_position
            me.state_fallen = r.loc_head_z < 0.3 # uses same criterion as for other teammates - not as reliable as player.behavior.is_ready("Get_Up")
            me.state_orientation = r.loc_torso_orientation
            me.state_ground_area = (r.loc_head_position[:2],0.2) # relevant for localization demo

            # Save last ball position to history at every vision cycle (even if not up to date) 
            self.ball_abs_pos_history.appendleft(self.ball_abs_pos) # from vision or radio
            self.ball_rel_torso_cart_pos_history.appendleft(self.ball_rel_torso_cart_pos)

            '''
            Get ball position based on vision or play mode
            Sources:
            Corner kick position - rcssserver3d/plugin/soccer/soccerruleaspect/soccerruleaspect.cpp:1927 (May 2022)
            Goal   kick position - rcssserver3d/plugin/soccer/soccerruleaspect/soccerruleaspect.cpp:1900 (May 2022)
            '''
            ball = None
            if self.apply_play_mode_correction:
                if PM == W.M_OUR_CORNER_KICK:
                    ball = np.array([15, 5.483 if self.ball_abs_pos[1] > 0 else -5.483, 0.042], float)
                elif PM == W.M_THEIR_CORNER_KICK:
                    ball = np.array([-15, 5.483 if self.ball_abs_pos[1] > 0 else -5.483, 0.042], float)
                elif PM in [W.M_OUR_KICKOFF, W.M_THEIR_KICKOFF, W.M_OUR_GOAL, W.M_THEIR_GOAL]:
                    ball = np.array([0, 0, 0.042], float)
                elif PM == W.M_OUR_GOAL_KICK:
                    ball = np.array([-14, 0, 0.042], float)
                elif PM == W.M_THEIR_GOAL_KICK:
                    ball = np.array([14, 0, 0.042], float)

                # Discard hard-coded ball position if robot is near that position (in favor of its own vision)
                if ball is not None and np.linalg.norm(r.loc_head_position[:2] - ball[:2]) < 1:
                    ball = None

            if ball is None and self.ball_is_visible and r.loc_is_up_to_date:
                ball = r.loc_head_to_field_transform( self.ball_rel_head_cart_pos )
                ball[2] = max(ball[2], 0.042) # lowest z = ball radius
                if PM != W.M_BEFORE_KICKOFF: # for compatibility with tests without active soccer rules
                    ball[:2] = np.clip(ball[:2], [-15,-10], [15,10]) # force ball position to be inside field

            # Update internal ball position (also updated by Radio)
            if ball is not None:
                time_diff = (self.time_local_ms - self.ball_abs_pos_last_update) / 1000
                self.ball_abs_vel = (ball - self.ball_abs_pos) / time_diff
                self.ball_abs_speed = np.linalg.norm(self.ball_abs_vel)
                self.ball_abs_pos_last_update = self.time_local_ms
                self.ball_abs_pos = ball
                self.is_ball_abs_pos_from_vision = True

            # Velocity decay for teammates and opponents (it is later neutralized if the velocity is updated)
            for p in self.teammates:
                p.state_filtered_velocity *= p.vel_decay
            for p in self.opponents:
                p.state_filtered_velocity *= p.vel_decay

            # Update teammates and opponents
            if r.loc_is_up_to_date:
                for p in self.teammates:
                    if not p.is_self:                     # if teammate is not self
                        if p.is_visible:                  # if teammate is visible, execute full update
                            self.update_other_robot(p)
                        elif p.state_abs_pos is not None: # otherwise update its horizontal distance (assuming last known position)
                            p.state_horizontal_dist = np.linalg.norm(r.loc_head_position[:2] - p.state_abs_pos[:2])

                for p in self.opponents:
                    if p.is_visible:                  # if opponent is visible, execute full update
                        self.update_other_robot(p)
                    elif p.state_abs_pos is not None: # otherwise update its horizontal distance (assuming last known position)
                        p.state_horizontal_dist = np.linalg.norm(r.loc_head_position[:2] - p.state_abs_pos[:2])

        # Update prediction of ball position/velocity
        if self.play_mode_group != W.MG_OTHER: # not 'play on' nor 'game over', so ball must be stationary
            self.ball_2d_pred_pos = self.ball_abs_pos[:2].copy().reshape(1, 2)
            self.ball_2d_pred_vel = np.zeros((1,2))
            self.ball_2d_pred_spd = np.zeros(1)

        elif self.ball_abs_pos_last_update == self.time_local_ms: # make new prediction for new ball position (from vision or radio)

            params = np.array([*self.ball_abs_pos[:2], *np.copy(self.get_ball_abs_vel(6)[:2])], np.float32)
            pred_ret  = ball_predictor.predict_rolling_ball(params)
            sample_no = len(pred_ret) // 5 * 2
            self.ball_2d_pred_pos = pred_ret[:sample_no].reshape(-1, 2)
            self.ball_2d_pred_vel = pred_ret[sample_no:sample_no*2].reshape(-1, 2)
            self.ball_2d_pred_spd = pred_ret[sample_no*2:]

        elif len(self.ball_2d_pred_pos) > 1: # otherwise, advance to next predicted step, if available 
            self.ball_2d_pred_pos = self.ball_2d_pred_pos[1:]
            self.ball_2d_pred_vel = self.ball_2d_pred_vel[1:]
            self.ball_2d_pred_spd = self.ball_2d_pred_spd[1:]

        r.update_imu(self.time_local_ms)      # update imu (must be executed after localization)


    def update_other_robot(self,other_robot : Other_Robot):
        ''' 
        Update other robot state based on the relative position of visible body parts
        (also updated by Radio, with the exception of state_orientation)
        '''
        o = other_robot
        r = self.robot

        # update body parts absolute positions
        o.state_body_parts_abs_pos = o.body_parts_cart_rel_pos.copy()
        for bp, pos in o.body_parts_cart_rel_pos.items():
            # Using the IMU could be beneficial if we see other robots but can't self-locate
            o.state_body_parts_abs_pos[bp] = r.loc_head_to_field_transform( pos, False )

        # auxiliary variables 
        bps_apos = o.state_body_parts_abs_pos                 # read-only shortcut
        bps_2d_apos_list = [v[:2] for v in bps_apos.values()] # list of body parts' 2D absolute positions
        avg_2d_pt = np.average(bps_2d_apos_list, axis=0)      # 2D avg pos of visible body parts
        head_is_visible = 'head' in bps_apos

        # evaluate robot's state (unchanged if head is not visible)
        if head_is_visible:
            o.state_fallen = bps_apos['head'][2] < 0.3

        # compute velocity if head is visible
        if o.state_abs_pos is not None:
            time_diff = (self.time_local_ms - o.state_last_update) / 1000
            if head_is_visible:
                # if last position is 2D, we assume that the z coordinate did not change, so that v.z=0
                old_p = o.state_abs_pos if len(o.state_abs_pos)==3 else np.append(o.state_abs_pos, bps_apos['head'][2])            
                velocity = (bps_apos['head'] - old_p) / time_diff
                decay = o.vel_decay # neutralize decay in all axes
            else: # if head is not visible, we only update the x & y components of the velocity
                velocity = np.append( (avg_2d_pt - o.state_abs_pos[:2]) / time_diff, 0)
                decay = (o.vel_decay,o.vel_decay,1) # neutralize decay (except in the z-axis)
            # apply filter
            if np.linalg.norm(velocity - o.state_filtered_velocity) < 4: # otherwise assume it was beamed
                o.state_filtered_velocity /= decay # neutralize decay
                o.state_filtered_velocity += o.vel_filter * (velocity-o.state_filtered_velocity)

        # compute robot's position (preferably based on head)  
        if head_is_visible:  
            o.state_abs_pos = bps_apos['head'] # 3D head position, if head is visible
        else:   
            o.state_abs_pos = avg_2d_pt # 2D avg pos of visible body parts

        # compute robot's horizontal distance (head distance, or avg. distance of visible body parts)
        o.state_horizontal_dist = np.linalg.norm(r.loc_head_position[:2] - o.state_abs_pos[:2])
        
        # compute orientation based on pair of lower arms or feet, or average of both
        lr_vec = None
        if 'llowerarm' in bps_apos and 'rlowerarm' in bps_apos:
            lr_vec = bps_apos['rlowerarm'] - bps_apos['llowerarm']
            
        if 'lfoot' in bps_apos and 'rfoot' in bps_apos:
            if lr_vec is None:
                lr_vec = bps_apos['rfoot'] - bps_apos['lfoot']
            else:
                lr_vec = (lr_vec + (bps_apos['rfoot'] - bps_apos['lfoot'])) / 2
        
        if lr_vec is not None:
            o.state_orientation = atan2(lr_vec[1],lr_vec[0]) * 180 / pi + 90

        # compute projection of player area on ground (circle) 
        if o.state_horizontal_dist < 4: # we don't need precision if the robot is farther than 4m 
            max_dist = np.max(np.linalg.norm(bps_2d_apos_list - avg_2d_pt, axis=1))
        else:
            max_dist = 0.2
        o.state_ground_area = (avg_2d_pt,max_dist)

        # update timestamp
        o.state_last_update = self.time_local_ms