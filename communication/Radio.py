from typing import List
from world.commons.Other_Robot import Other_Robot
from world.World import World
import numpy as np

class Radio():
    '''
    map limits are hardcoded:
        teammates/opponents positions (x,y) in ([-16,16],[-11,11])
        ball position (x,y) in ([-15,15],[-10,10])
    known server limitations:
        claimed: all ascii from 0x20 to 0x7E except ' ', '(', ')'
        bugs: 
            - ' or " clip the message 
            - '\' at the end or near another '\'
            - ';' at beginning of message
    '''
    # map limits are hardcoded:

    # lines, columns, half lines index, half cols index, (lines-1)/x_span, (cols-1)/y_span, combinations, combinations*2states, 
    TP = 321,221,160,110,10,  10,70941,141882 # teammate position
    OP = 201,111,100,55, 6.25,5, 22311,44622  # opponent position
    BP = 301,201,150,100,10,  10,60501        # ball position
    SYMB = "!#$%&*+,-./0123456789:<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[]^_`abcdefghijklmnopqrstuvwxyz{|}~;"
    SLEN = len(SYMB)
    SYMB_TO_IDX = {ord(s):i for i,s in enumerate(SYMB)}


    def __init__(self, world : World, commit_announcement) -> None:
        self.world = world
        self.commit_announcement = commit_announcement
        r = world.robot
        t = world.teammates
        o = world.opponents
        self.groups = ( # player team/unum, group has ball?, self in group?
            [(t[9],t[10],o[6],o[7],o[8],o[9],o[10]),     True ], # 2 teammates, 5 opponents, ball
            [(t[0],t[1], t[2],t[3],t[4],t[5],t[6] ),     False], # 7 teammates
            [(t[7],t[8], o[0],o[1],o[2],o[3],o[4],o[5]), False]  # 2 teammates, 6 opponents
        )
        for g in self.groups: # add 'self in group?'
            g.append(any(i.is_self for i in g[0]))

    def get_player_combination(self, pos, is_unknown, is_down, info):
        ''' Returns combination (0-based) and number of possible combinations '''

        if is_unknown: 
            return info[7]+1, info[7]+2 # return unknown combination

        x,y = pos[:2]

        if x < -17 or x > 17 or y < -12 or y > 12:
            return info[7], info[7]+2 # return out of bounds combination (if it exceeds 1m in any axis)

        # convert to int to avoid overflow later
        l = int(np.clip( round(info[4]*x+info[2]), 0, info[0]-1 )) # absorb out of bounds positions (up to 1m in each axis)
        c = int(np.clip( round(info[5]*y+info[3]), 0, info[1]-1 ))

        return (l*info[1]+c)+(info[6] if is_down else 0), info[7]+2 # return valid combination


    def get_ball_combination(self, x, y):
        ''' Returns combination (0-based) and number of possible combinations '''

        # if ball is out of bounds, we force it in
        l = int(np.clip( round(Radio.BP[4]*x+Radio.BP[2]), 0, Radio.BP[0]-1 ))
        c = int(np.clip( round(Radio.BP[5]*y+Radio.BP[3]), 0, Radio.BP[1]-1 ))

        return l*Radio.BP[1]+c, Radio.BP[6] # return valid combination

    def get_ball_position(self,comb):

        l = comb // Radio.BP[1]
        c = comb %  Radio.BP[1]

        return np.array([l/Radio.BP[4]-15, c/Radio.BP[5]-10, 0.042]) # assume ball is on ground

    def get_player_position(self,comb, info):

        if comb == info[7]:   return -1  # player is out of bounds
        if comb == info[7]+1: return -2  # player is in unknown location

        is_down = comb >= info[6]
        if is_down:
            comb -= info[6]
            
        l = comb // info[1]
        c = comb %  info[1]

        return l/info[4]-16, c/info[5]-11, is_down


    def check_broadcast_requirements(self):
        '''
        Check if broadcast group is valid

        Returns
        -------
        ready : bool
            True if all requirements are met

        Sequence: g0,g1,g2, ig0,ig1,ig2, iig0,iig1,iig2  (whole cycle: 0.36s)
            igx  means      'incomplete group', where <=1 element  can be MIA recently
            iigx means 'very incomplete group', where <=2 elements can be MIA recently
            Rationale: prevent incomplete messages from monopolizing the broadcast space 

        However:
        - 1st round: when 0 group  members are missing,          that group will update 3 times every 0.36s
        - 2nd round: when 1 group  member  is  recently missing, that group will update 2 times every 0.36s
        - 3rd round: when 2 group  members are recently missing, that group will update 1 time  every 0.36s
        -            when >2 group members are recently missing, that group will not be updated

        Players that have never been seen or heard are not considered for the 'recently missing'.
        If there is only 1 group member since the beginning, the respective group can be updated, except in the 1st round.
        In this way, the 1st round cannot be monopolized by clueless agents, which is important during games with 22 players.
        '''

        w = self.world
        r = w.robot
        ago40ms = w.time_local_ms - 40
        ago370ms = w.time_local_ms - 370 # maximum delay (up to 2 MIAs) is 360ms because radio has a delay of 20ms (otherwise max delay would be 340ms)
        group : List[Other_Robot]

        idx9 = int((w.time_server * 25)+0.1) % 9 # sequence of 9 phases
        max_MIA = idx9 // 3                      # maximum number of MIA players (based on server time)
        group_idx = idx9 % 3                     # group number                  (based on server time)
        group, has_ball, is_self_included = self.groups[group_idx]

        #============================================ 0. check if group is valid

        if has_ball and w.ball_abs_pos_last_update < ago40ms: # Ball is included and not up to date
            return False

        if is_self_included and r.loc_last_update < ago40ms: # Oneself is included and unable to self-locate
            return False

        # Get players that have been previously seen or heard but not recently
        MIAs = [not ot.is_self and ot.state_last_update < ago370ms and ot.state_last_update > 0 for ot in group]
        self.MIAs = [ot.state_last_update == 0 or MIAs[i] for i,ot in enumerate(group)] # add players that have never been seen

        if sum(MIAs) > max_MIA: # checking if number of recently missing members is not above threshold
            return False

        # never seen before players are always ignored except when:
        # - this is the 0 MIAs round (see explanation above)
        # - all are MIA
        if (max_MIA == 0 and any(self.MIAs)) or all(self.MIAs): 
            return False

        # Check for invalid members. Conditions:
        # - Player is other and not MIA and:  
        #      - last update was >40ms ago OR
        #      - last update did not include the head (head is important to provide state and accurate position)

        if any(
            (not ot.is_self and not self.MIAs[i] and                   
                (ot.state_last_update < ago40ms or ot.state_last_update==0 or len(ot.state_abs_pos)<3)#  (last update: has no head or is old)                          
            ) for i,ot in enumerate(group)
        ):
            return False

        return True


    def broadcast(self):
        '''
        Commit messages to teammates if certain conditions are met
        Messages contain: positions/states of every moving entity
        '''

        if not self.check_broadcast_requirements():
            return
            
        w = self.world
        ot : Other_Robot

        group_idx = int((w.time_server * 25)+0.1) % 3 # group number based on server time
        group, has_ball, _ = self.groups[group_idx]

        #============================================ 1. create combination

        # add message number
        combination = group_idx
        no_of_combinations = 3

        # add ball combination
        if has_ball: 
            c, n = self.get_ball_combination(w.ball_abs_pos[0], w.ball_abs_pos[1])   
            combination += c * no_of_combinations
            no_of_combinations *= n


        # add group combinations
        for i,ot in enumerate(group):
            c, n = self.get_player_combination(ot.state_abs_pos,                           # player position
                                               self.MIAs[i], ot.state_fallen,              # is unknown, is down
                                               Radio.TP if ot.is_teammate else Radio.OP)   # is teammate
            combination += c * no_of_combinations
            no_of_combinations *= n


        assert(no_of_combinations < 9.61e38) # 88*89^19 (first character cannot be ';')

        #============================================ 2. create message

        # 1st msg symbol: ignore ';' due to server bug
        msg = Radio.SYMB[combination % (Radio.SLEN-1)]
        combination //= (Radio.SLEN-1)

        # following msg symbols
        while combination:
            msg += Radio.SYMB[combination % Radio.SLEN]
            combination //= Radio.SLEN

        #============================================ 3. commit message

        self.commit_announcement(msg.encode()) # commit message


    def receive(self, msg:bytearray):
        w = self.world
        r = w.robot
        ago40ms = w.time_local_ms - 40
        ago110ms = w.time_local_ms - 110
        msg_time = w.time_local_ms - 20 # message was sent in the last step

        #============================================ 1. get combination

        # read first symbol, which cannot be ';' due to server bug
        combination = Radio.SYMB_TO_IDX[msg[0]]
        total_combinations = Radio.SLEN-1

        if len(msg)>1:
            for m in msg[1:]:
                combination += total_combinations * Radio.SYMB_TO_IDX[m]
                total_combinations *= Radio.SLEN

        #============================================ 2. get msg ID

        message_no = combination % 3
        combination //= 3
        group, has_ball, _ = self.groups[message_no]

        #============================================ 3. get data

        if has_ball:
            ball_comb = combination % Radio.BP[6]
            combination //= Radio.BP[6]

        players_combs = []
        ot : Other_Robot
        for ot in group:
            info = Radio.TP if ot.is_teammate else Radio.OP
            players_combs.append( combination % (info[7]+2) )
            combination //= info[7]+2

        #============================================ 4. update world

        if has_ball and w.ball_abs_pos_last_update < ago40ms: # update ball if it was not seen
            time_diff = (msg_time - w.ball_abs_pos_last_update) / 1000
            ball = self.get_ball_position(ball_comb)
            w.ball_abs_vel = (ball - w.ball_abs_pos) / time_diff
            w.ball_abs_speed = np.linalg.norm(w.ball_abs_vel)
            w.ball_abs_pos_last_update = msg_time # (error: 0-40 ms)
            w.ball_abs_pos = ball
            w.is_ball_abs_pos_from_vision = False
            
        for c, ot in zip(players_combs, group):
            
            # handle oneself case
            if ot.is_self:
                # the ball's position has a fair amount of noise, whether seen by us or other players
                # but our self-locatization mechanism is usually much better than how others perceive us
                if r.loc_last_update < ago110ms: # so we wait until we miss 2 visual steps
                    data = self.get_player_position(c, Radio.TP)
                    if type(data)==tuple:
                        x,y,is_down = data
                        r.loc_head_position[:2] = x,y # z is kept unchanged
                        r.loc_head_position_last_update = msg_time
                        r.radio_fallen_state = is_down
                        r.radio_last_update = msg_time
                continue

            # do not update if other robot was recently seen
            if ot.state_last_update >= ago40ms:
                continue

            info = Radio.TP if ot.is_teammate else Radio.OP
            data = self.get_player_position(c, info)  
            if type(data)==tuple:
                x,y,is_down = data
                p = np.array([x,y])
                
                if ot.state_abs_pos is not None:  # update the x & y components of the velocity
                    time_diff = (msg_time - ot.state_last_update) / 1000
                    velocity = np.append( (p - ot.state_abs_pos[:2]) / time_diff, 0) # v.z = 0
                    vel_diff = velocity - ot.state_filtered_velocity
                    if np.linalg.norm(vel_diff) < 4: # otherwise assume it was beamed
                        ot.state_filtered_velocity /= (ot.vel_decay,ot.vel_decay,1) # neutralize decay (except in the z-axis)
                        ot.state_filtered_velocity += ot.vel_filter * vel_diff
                    
                ot.state_fallen = is_down             
                ot.state_last_update = msg_time    
                ot.state_body_parts_abs_pos = {"head":p}
                ot.state_abs_pos = p
                ot.state_horizontal_dist = np.linalg.norm(p - r.loc_head_position[:2])
                ot.state_ground_area = (p, 0.3 if is_down else 0.2)  # not very precise, but we cannot see the robot