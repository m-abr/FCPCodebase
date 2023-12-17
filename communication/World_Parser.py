from math_ops.Math_Ops import Math_Ops as M
from world.Robot import Robot
from world.World import World
import math
import numpy as np


class World_Parser():
    def __init__(self, world:World, hear_callback) -> None:
        self.LOG_PREFIX = "World_Parser.py: "
        self.world = world
        self.hear_callback = hear_callback
        self.exp = None
        self.depth = None
        self.LEFT_SIDE_FLAGS = {b'F2L':(-15,-10,0),
                                b'F1L':(-15,+10,0),
                                b'F2R':(+15,-10,0),
                                b'F1R':(+15,+10,0),
                                b'G2L':(-15,-1.05,0.8),
                                b'G1L':(-15,+1.05,0.8),
                                b'G2R':(+15,-1.05,0.8),
                                b'G1R':(+15,+1.05,0.8)} #mapping between flag names and their corrected location, when playing on the left side
        self.RIGHT_SIDE_FLAGS = {b'F2L':(+15,+10,0),
                                 b'F1L':(+15,-10,0),
                                 b'F2R':(-15,+10,0),
                                 b'F1R':(-15,-10,0),
                                 b'G2L':(+15,+1.05,0.8),
                                 b'G1L':(+15,-1.05,0.8),
                                 b'G2R':(-15,+1.05,0.8),
                                 b'G1R':(-15,-1.05,0.8)}  
        self.play_mode_to_id = None
        self.LEFT_PLAY_MODE_TO_ID = {"KickOff_Left":World.M_OUR_KICKOFF, "KickIn_Left":World.M_OUR_KICK_IN, "corner_kick_left":World.M_OUR_CORNER_KICK,
                                    "goal_kick_left":World.M_OUR_GOAL_KICK, "free_kick_left":World.M_OUR_FREE_KICK, "pass_left":World.M_OUR_PASS,
                                    "direct_free_kick_left": World.M_OUR_DIR_FREE_KICK, "Goal_Left": World.M_OUR_GOAL, "offside_left": World.M_OUR_OFFSIDE,
                                    "KickOff_Right":World.M_THEIR_KICKOFF, "KickIn_Right":World.M_THEIR_KICK_IN, "corner_kick_right":World.M_THEIR_CORNER_KICK,
                                    "goal_kick_right":World.M_THEIR_GOAL_KICK, "free_kick_right":World.M_THEIR_FREE_KICK, "pass_right":World.M_THEIR_PASS,
                                    "direct_free_kick_right": World.M_THEIR_DIR_FREE_KICK, "Goal_Right": World.M_THEIR_GOAL, "offside_right": World.M_THEIR_OFFSIDE,
                                    "BeforeKickOff": World.M_BEFORE_KICKOFF, "GameOver": World.M_GAME_OVER, "PlayOn": World.M_PLAY_ON }
        self.RIGHT_PLAY_MODE_TO_ID = {"KickOff_Left":World.M_THEIR_KICKOFF, "KickIn_Left":World.M_THEIR_KICK_IN, "corner_kick_left":World.M_THEIR_CORNER_KICK,
                                    "goal_kick_left":World.M_THEIR_GOAL_KICK, "free_kick_left":World.M_THEIR_FREE_KICK, "pass_left":World.M_THEIR_PASS,
                                    "direct_free_kick_left": World.M_THEIR_DIR_FREE_KICK, "Goal_Left": World.M_THEIR_GOAL, "offside_left": World.M_THEIR_OFFSIDE,
                                    "KickOff_Right":World.M_OUR_KICKOFF, "KickIn_Right":World.M_OUR_KICK_IN, "corner_kick_right":World.M_OUR_CORNER_KICK,
                                    "goal_kick_right":World.M_OUR_GOAL_KICK, "free_kick_right":World.M_OUR_FREE_KICK, "pass_right":World.M_OUR_PASS,
                                    "direct_free_kick_right": World.M_OUR_DIR_FREE_KICK, "Goal_Right": World.M_OUR_GOAL, "offside_right": World.M_OUR_OFFSIDE,
                                    "BeforeKickOff": World.M_BEFORE_KICKOFF, "GameOver": World.M_GAME_OVER, "PlayOn": World.M_PLAY_ON }


    def find_non_digit(self,start):
        while True:
            if (self.exp[start] < ord('0') or self.exp[start] > ord('9')) and self.exp[start] != ord('.'): return start
            start+=1

    def find_char(self,start,char):
        while True:
            if self.exp[start] == char : return start
            start+=1

    def read_float(self, start):
        if self.exp[start:start+3] == b'nan': return float('nan'), start+3 #handle nan values (they exist)
        end = self.find_non_digit(start+1) #we assume the first one is a digit or minus sign
        try:
            retval = float(self.exp[start:end])
        except:
            self.world.log(f"{self.LOG_PREFIX}String to float conversion failed: {self.exp[start:end]} at msg[{start},{end}], \nMsg: {self.exp.decode()}")
            retval = 0
        return retval, end

    def read_int(self, start):
        end = self.find_non_digit(start+1) #we assume the first one is a digit or minus sign
        return int(self.exp[start:end]), end

    def read_bytes(self, start):
        end = start
        while True:
            if self.exp[end] == ord(' ') or self.exp[end] == ord(')'): break
            end+=1

        return self.exp[start:end], end

    def read_str(self, start):
        b, end = self.read_bytes(start)
        return b.decode(), end  

    def get_next_tag(self, start):
        min_depth = self.depth
        while True:
            if self.exp[start] == ord(")") :  #monitor xml element depth
                self.depth -= 1
                if min_depth > self.depth: min_depth = self.depth
            elif self.exp[start] == ord("(") : break
            start+=1
            if start >= len(self.exp): return None, start, 0

        self.depth += 1
        start += 1
        end = self.find_char(start, ord(" ")) 
        return self.exp[start:end], end, min_depth


    def parse(self, exp):

        self.exp = exp #used by other member functions
        self.depth = 0 #xml element depth
        self.world.step += 1
        self.world.line_count = 0
        self.world.robot.frp = dict()
        self.world.flags_posts = dict()
        self.world.flags_corners = dict()
        self.world.vision_is_up_to_date = False
        self.world.ball_is_visible = False
        self.world.robot.feet_toes_are_touching = dict.fromkeys(self.world.robot.feet_toes_are_touching, False)
        self.world.time_local_ms += World.STEPTIME_MS

        for p in self.world.teammates: p.is_visible = False
        for p in self.world.opponents: p.is_visible = False

        tag, end, _ = self.get_next_tag(0)

        while end < len(exp):

            if tag==b'time':
                while True:
                    tag, end, min_depth = self.get_next_tag(end)
                    if min_depth == 0: break 

                    if tag==b'now':
                        #last_time = self.world.time_server
                        self.world.time_server, end = self.read_float(end+1)
                        
                        #Test server time reliability
                        #increment = self.world.time_server - last_time
                        #if increment < 0.019: print ("down",last_time,self.world.time_server)
                        #if increment > 0.021: print ("up",last_time,self.world.time_server)
                    else:
                        self.world.log(f"{self.LOG_PREFIX}Unknown tag inside 'time': {tag} at {end}, \nMsg: {exp.decode()}")


            elif tag==b'GS':
                while True:
                    tag, end, min_depth = self.get_next_tag(end)
                    if min_depth == 0: break

                    if tag==b'unum':
                        _, end = self.read_int(end+1) #We already know our unum
                    elif tag==b'team':
                        aux, end = self.read_str(end+1)
                        is_left = bool(aux == "left")
                        if self.world.team_side_is_left != is_left:
                            self.world.team_side_is_left = is_left
                            self.play_mode_to_id = self.LEFT_PLAY_MODE_TO_ID if is_left else self.RIGHT_PLAY_MODE_TO_ID
                            self.world.draw.set_team_side(not is_left)
                            self.world.team_draw.set_team_side(not is_left)
                    elif tag==b'sl':
                        if self.world.team_side_is_left:
                            self.world.goals_scored, end  = self.read_int(end+1)
                        else:
                            self.world.goals_conceded, end  = self.read_int(end+1)
                    elif tag==b'sr':
                        if self.world.team_side_is_left:
                            self.world.goals_conceded, end  = self.read_int(end+1)
                        else:
                            self.world.goals_scored, end  = self.read_int(end+1)
                    elif tag==b't':
                        self.world.time_game, end = self.read_float(end+1)
                    elif tag==b'pm':
                        aux, end = self.read_str(end+1)
                        if self.play_mode_to_id is not None:
                            self.world.play_mode = self.play_mode_to_id[aux]
                    else:
                        self.world.log(f"{self.LOG_PREFIX}Unknown tag inside 'GS': {tag} at {end}, \nMsg: {exp.decode()}")


            elif tag==b'GYR':
                while True:
                    tag, end, min_depth = self.get_next_tag(end)
                    if min_depth == 0: break

                    '''
                    The gyroscope measures the robot's torso angular velocity (rotation rate vector)
                    The angular velocity's orientation is given by the right-hand rule.

                    Original reference frame:
                        X:left(-)/right(+)      Y:back(-)/front(+)      Z:down(-)/up(+)

                    New reference frame:
                        X:back(-)/front(+)      Y:right(-)/left(+)      Z:down(-)/up(+)

                    '''

                    if tag==b'n':
                        pass
                    elif tag==b'rt':
                        self.world.robot.gyro[1], end = self.read_float(end+1)
                        self.world.robot.gyro[0], end = self.read_float(end+1)
                        self.world.robot.gyro[2], end = self.read_float(end+1)
                        self.world.robot.gyro[1] *= -1
                    else:
                        self.world.log(f"{self.LOG_PREFIX}Unknown tag inside 'GYR': {tag} at {end}, \nMsg: {exp.decode()}")


            elif tag==b'ACC':
                while True:
                    tag, end, min_depth = self.get_next_tag(end)
                    if min_depth == 0: break

                    '''
                    The accelerometer measures the acceleration relative to freefall. It will read zero during any type of free fall.
                    When at rest relative to the Earth's surface, it will indicate an upwards acceleration of 9.81m/s^2 (in SimSpark).

                    Original reference frame:
                        X:left(-)/right(+)      Y:back(-)/front(+)      Z:down(-)/up(+)

                    New reference frame:
                        X:back(-)/front(+)      Y:right(-)/left(+)      Z:down(-)/up(+)
                    '''

                    if tag==b'n':
                        pass
                    elif tag==b'a':
                        self.world.robot.acc[1], end = self.read_float(end+1)
                        self.world.robot.acc[0], end = self.read_float(end+1)
                        self.world.robot.acc[2], end = self.read_float(end+1)
                        self.world.robot.acc[1] *= -1
                    else:
                        self.world.log(f"{self.LOG_PREFIX}Unknown tag inside 'ACC': {tag} at {end}, \nMsg: {exp.decode()}")


            elif tag==b'HJ':
                while True:
                    tag, end, min_depth = self.get_next_tag(end)
                    if min_depth == 0: break

                    if tag==b'n':
                        joint_name, end = self.read_str(end+1)
                        joint_index = Robot.MAP_PERCEPTOR_TO_INDEX[joint_name]
                    elif tag==b'ax':
                        joint_angle, end = self.read_float(end+1)

                        #Fix symmetry issues 2/4 (perceptors)
                        if joint_name in Robot.FIX_PERCEPTOR_SET: joint_angle = -joint_angle

                        old_angle = self.world.robot.joints_position[joint_index] 
                        self.world.robot.joints_speed[joint_index] = (joint_angle - old_angle) / World.STEPTIME * math.pi / 180
                        self.world.robot.joints_position[joint_index] = joint_angle
                    else:
                        self.world.log(f"{self.LOG_PREFIX}Unknown tag inside 'HJ': {tag} at {end}, \nMsg: {exp.decode()}")

            elif tag==b'FRP':
                while True:
                    tag, end, min_depth = self.get_next_tag(end)
                    if min_depth == 0: break

                    '''
                    The reference frame is used for the contact point and force vector applied to that point
                        Note: The force vector is applied to the foot, so it usually points up

                    Original reference frame:
                        X:left(-)/right(+)      Y:back(-)/front(+)      Z:down(-)/up(+)

                    New reference frame:
                        X:back(-)/front(+)      Y:right(-)/left(+)      Z:down(-)/up(+)

                    '''

                    if tag==b'n':
                        foot_toe_id, end = self.read_str(end+1)
                        self.world.robot.frp[foot_toe_id] = foot_toe_ref = np.empty(6)
                        self.world.robot.feet_toes_last_touch[foot_toe_id] = self.world.time_local_ms
                        self.world.robot.feet_toes_are_touching[foot_toe_id] = True
                    elif tag==b'c':
                        foot_toe_ref[1], end = self.read_float(end+1)
                        foot_toe_ref[0], end = self.read_float(end+1)
                        foot_toe_ref[2], end = self.read_float(end+1)
                        foot_toe_ref[1] *= -1
                    elif tag==b'f':
                        foot_toe_ref[4], end = self.read_float(end+1)
                        foot_toe_ref[3], end = self.read_float(end+1)
                        foot_toe_ref[5], end = self.read_float(end+1)
                        foot_toe_ref[4] *= -1
                    else:
                        self.world.log(f"{self.LOG_PREFIX}Unknown tag inside 'FRP': {tag} at {end}, \nMsg: {exp.decode()}")


            elif tag==b'See':
                self.world.vision_is_up_to_date = True
                self.world.vision_last_update = self.world.time_local_ms

                while True:
                    tag, end, min_depth = self.get_next_tag(end)
                    if min_depth == 0: break

                    tag_bytes = bytes(tag) #since bytearray is not hashable, it cannot be used as key for dictionaries

                    if tag==b'G1R' or tag==b'G2R' or tag==b'G1L' or tag==b'G2L':
                        _, end, _ = self.get_next_tag(end)

                        c1, end = self.read_float(end+1)
                        c2, end = self.read_float(end+1)
                        c3, end = self.read_float(end+1)

                        aux = self.LEFT_SIDE_FLAGS[tag_bytes] if self.world.team_side_is_left else self.RIGHT_SIDE_FLAGS[tag_bytes]
                        self.world.flags_posts[aux] = (c1,c2,c3)

                    elif tag==b'F1R' or tag==b'F2R' or tag==b'F1L' or tag==b'F2L':
                        _, end, _ = self.get_next_tag(end)

                        c1, end = self.read_float(end+1)
                        c2, end = self.read_float(end+1)
                        c3, end = self.read_float(end+1)

                        aux = self.LEFT_SIDE_FLAGS[tag_bytes] if self.world.team_side_is_left else self.RIGHT_SIDE_FLAGS[tag_bytes]
                        self.world.flags_corners[aux] = (c1,c2,c3)

                    elif tag==b'B':
                        _, end, _ = self.get_next_tag(end)

                        self.world.ball_rel_head_sph_pos[0], end = self.read_float(end+1)
                        self.world.ball_rel_head_sph_pos[1], end = self.read_float(end+1)
                        self.world.ball_rel_head_sph_pos[2], end = self.read_float(end+1)
                        self.world.ball_rel_head_cart_pos = M.deg_sph2cart(self.world.ball_rel_head_sph_pos)
                        self.world.ball_is_visible = True
                        self.world.ball_last_seen = self.world.time_local_ms

                    elif tag==b'mypos':

                        self.world.robot.cheat_abs_pos[0], end = self.read_float(end+1)
                        self.world.robot.cheat_abs_pos[1], end = self.read_float(end+1)
                        self.world.robot.cheat_abs_pos[2], end = self.read_float(end+1)

                    elif tag==b'myorien':

                        self.world.robot.cheat_ori, end = self.read_float(end+1)

                    elif tag==b'ballpos':

                        c1, end = self.read_float(end+1)
                        c2, end = self.read_float(end+1)
                        c3, end = self.read_float(end+1)

                        self.world.ball_cheat_abs_vel[0] = (c1 - self.world.ball_cheat_abs_pos[0]) / World.VISUALSTEP
                        self.world.ball_cheat_abs_vel[1] = (c2 - self.world.ball_cheat_abs_pos[1]) / World.VISUALSTEP
                        self.world.ball_cheat_abs_vel[2] = (c3 - self.world.ball_cheat_abs_pos[2]) / World.VISUALSTEP

                        self.world.ball_cheat_abs_pos[0] = c1
                        self.world.ball_cheat_abs_pos[1] = c2
                        self.world.ball_cheat_abs_pos[2] = c3

                    elif tag==b'P':

                        while True:
                            previous_depth = self.depth
                            previous_end = end
                            tag, end, min_depth = self.get_next_tag(end)
                            if min_depth < 2: #if =1 we are still inside 'See', if =0 we are already outside 'See'
                                end = previous_end #The "P" tag is special because it's the only variable particle inside 'See'
                                self.depth = previous_depth
                                break # we restore the previous tag, and let 'See' handle it

                            if tag==b'team':
                                player_team, end = self.read_str(end+1)
                                is_teammate = bool(player_team == self.world.team_name)
                                if self.world.team_name_opponent is None and not is_teammate: #register opponent team name
                                    self.world.team_name_opponent = player_team
                            elif tag==b'id':
                                player_id, end = self.read_int(end+1)
                                player = self.world.teammates[player_id-1] if is_teammate else self.world.opponents[player_id-1]
                                player.body_parts_cart_rel_pos = dict() #reset seen body parts
                                player.is_visible = True
                            elif tag==b'llowerarm' or tag==b'rlowerarm' or tag==b'lfoot' or tag==b'rfoot' or tag==b'head':
                                tag_str = tag.decode()
                                _, end, _ = self.get_next_tag(end)

                                c1, end = self.read_float(end+1)
                                c2, end = self.read_float(end+1)
                                c3, end = self.read_float(end+1)

                                if is_teammate: 
                                    self.world.teammates[player_id-1].body_parts_sph_rel_pos[tag_str] = (c1,c2,c3)
                                    self.world.teammates[player_id-1].body_parts_cart_rel_pos[tag_str] = M.deg_sph2cart((c1,c2,c3))
                                else:
                                    self.world.opponents[player_id-1].body_parts_sph_rel_pos[tag_str] = (c1,c2,c3)
                                    self.world.opponents[player_id-1].body_parts_cart_rel_pos[tag_str] = M.deg_sph2cart((c1,c2,c3))
                            else:
                                self.world.log(f"{self.LOG_PREFIX}Unknown tag inside 'P': {tag} at {end}, \nMsg: {exp.decode()}")
                        
                    elif tag==b'L':
                        l = self.world.lines[self.world.line_count]
                        
                        _, end, _ = self.get_next_tag(end)
                        l[0], end = self.read_float(end+1)
                        l[1], end = self.read_float(end+1)
                        l[2], end = self.read_float(end+1)
                        _, end, _ = self.get_next_tag(end)
                        l[3], end = self.read_float(end+1)
                        l[4], end = self.read_float(end+1)
                        l[5], end = self.read_float(end+1)

                        if np.isnan(l).any():
                            self.world.log(f"{self.LOG_PREFIX}Received field line with NaNs {l}")
                        else:
                            self.world.line_count += 1 #accept field line if there are no NaNs
                        
                    else:
                        self.world.log(f"{self.LOG_PREFIX}Unknown tag inside 'see': {tag} at {end}, \nMsg: {exp.decode()}")


            elif tag==b'hear':

                team_name, end = self.read_str(end+1)

                if team_name == self.world.team_name:   # discard message if it's not from our team
                    
                    timestamp, end = self.read_float(end+1)

                    if self.exp[end+1] == ord('s'):     # this message was sent by oneself
                        direction, end = "self", end+5
                    else:                               # this message was sent by teammate
                        direction, end = self.read_float(end+1)

                    msg, end = self.read_bytes(end+1)
                    self.hear_callback(msg, direction, timestamp)


                tag, end, _ = self.get_next_tag(end)


            else:
                self.world.log(f"{self.LOG_PREFIX}Unknown root tag: {tag} at {end}, \nMsg: {exp.decode()}")
                tag, end, min_depth = self.get_next_tag(end)

