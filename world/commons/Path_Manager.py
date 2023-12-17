from cpp.a_star import a_star
from math_ops.Math_Ops import Math_Ops as M
from world.World import World
import math
import numpy as np


class Path_Manager():
    MODE_CAUTIOUS = 0
    MODE_DRIBBLE = 1    # safety margins are increased
    MODE_AGGRESSIVE = 2 # safety margins are reduced for opponents

    STATUS_SUCCESS = 0 # the pathfinding algorithm was executed normally
    STATUS_TIMEOUT = 1 # timeout before the target was reached (may be impossible)
    STATUS_IMPOSSIBLE = 2 # impossible to reach target (all options were tested)
    STATUS_DIRECT = 3 # no obstacles between start and target (path contains only 2 points: the start and target)

    HOT_START_DIST_WALK = 0.05    # hot start prediction distance (when walking) 
    HOT_START_DIST_DRIBBLE = 0.10 # hot start prediction distance (when dribbling) 

    def __init__(self, world : World) -> None:
        self.world = world

        self._draw_obstacles = False   # enabled by function 'draw_options'
        self._draw_path = False        # enabled by function 'draw_options'
        self._use_team_channel = False # enabled by function 'draw_options'

        # internal variables to bootstrap the path to start from a prediction (to reduce path instability)
        self.last_direction_rad = None
        self.last_update = 0
        self.last_start_dist = None

    def draw_options(self, enable_obstacles, enable_path, use_team_drawing_channel=False):
        '''
        Enable or disable drawings, and change drawing channel
        If self.world.draw.enable is False, these options are ignored

        Parameters
        ----------
        enable_obstacles : bool
            draw relevant obstacles for path planning
        enable_path : bool
            draw computed path
        use_team_drawing_channel : bool
            True to use team drawing channel, otherwise use individual channel
            Using individual channels for each player means that drawings with the same name can coexist
            With the team channel, drawings with the same name will replace previous drawings, even if drawn by a teammate
        '''
        self._draw_obstacles = enable_obstacles
        self._draw_path = enable_path
        self._use_team_channel = use_team_drawing_channel
        
    def get_obstacles(self, include_teammates, include_opponents, include_play_mode_restrictions, max_distance = 4, max_age = 500, 
                      ball_safety_margin = 0, goalpost_safety_margin = 0, mode = MODE_CAUTIOUS, priority_unums=[]):
        '''
        Parameters
        ----------
        include_teammates : bool
            include teammates in the returned list of obstacles
        include_opponents : bool
            include opponents in the returned list of obstacles
        max_distance : float
            teammates or opponents are only considered if they are closer than `max_distance` (meters)
        max_age : float
            teammates or opponents are only considered if they were seen in the last `max_age` (milliseconds)
        ball_safety_margin : float
            minimum value for the ball's soft repulsion radius
            this value is increased when the game is stopped, and when the ball is almost out of bounds
            default is zero, the ball is ignored
        goalpost_safety_margin : float
            hard repulsion radius around the opponents' goalposts
            default is zero, uses the minimum margin
        mode : int
            overall attitude towards safety margins (concerns teammates and opponents)
        priority_unums : list
            list of teammates to avoid (since their role is more important)

        Returns
        -------
        obstacles : list
            list of obstacles, where each obstacle is a tuple of 5 floats (x, y, hard radius, soft radius, repulsive force)
        '''
        w = self.world

        ball_2d = w.ball_abs_pos[:2]
        obstacles = []

        # 'comparator' is a variable local to the lambda, which captures the current value of (w.time_local_ms - max_age)
        check_age = lambda last_update, comparator = w.time_local_ms - max_age : last_update > 0 and last_update >= comparator
        
        #---------------------------------------------- Get recently seen close teammates
        if include_teammates:
            soft_radius = 1.1 if mode == Path_Manager.MODE_DRIBBLE else 0.6 # soft radius: repulsive force is max at center and fades

            def get_hard_radius(t):
                if t.unum in priority_unums:
                    return 1.0 # extra distance for priority roles
                else:
                    return t.state_ground_area[1]+0.2

            # Get close teammates (center, hard radius, soft radius, force)
            obstacles.extend( (*t.state_ground_area[0],
                            get_hard_radius(t),
                            1.5 if t.unum in priority_unums else soft_radius, 
                            1.0) # repulsive force 
            for t in w.teammates if not t.is_self and check_age(t.state_last_update) and t.state_horizontal_dist < max_distance)

        #---------------------------------------------- Get recently seen close opponents
        if include_opponents: 

            # soft radius: repulsive force is max at center and fades
            if mode == Path_Manager.MODE_AGGRESSIVE:
                soft_radius = 0.6
                hard_radius = lambda o : 0.2
            elif mode == Path_Manager.MODE_DRIBBLE:
                soft_radius = 2.3
                hard_radius = lambda o : o.state_ground_area[1]+0.9
            else:
                soft_radius = 1.0
                hard_radius = lambda o : o.state_ground_area[1]+0.2

            # Get close opponents (center, hard radius, soft radius, force)
            obstacles.extend( (*o.state_ground_area[0],
                            hard_radius(o),
                            soft_radius,
                            1.5 if o.unum == 1 else 1.0) # repulsive force (extra for their GK)
            for o in w.opponents if o.state_last_update > 0 and w.time_local_ms - o.state_last_update <= max_age and o.state_horizontal_dist < max_distance)

        #---------------------------------------------- Get play mode restrictions
        if include_play_mode_restrictions:
            if w.play_mode == World.M_THEIR_GOAL_KICK:
                obstacles.extend((15,i,2.1,0,0) for i in range(-2,3)) # 5 circular obstacles to cover their goal area
            elif w.play_mode == World.M_THEIR_PASS:
                obstacles.append((*ball_2d, 1.2, 0, 0))
            elif w.play_mode in [World.M_THEIR_KICK_IN,World.M_THEIR_CORNER_KICK,World.M_THEIR_FREE_KICK,World.M_THEIR_DIR_FREE_KICK, World.M_THEIR_OFFSIDE]:
                obstacles.append((*ball_2d, 2.5, 0, 0))

        #---------------------------------------------- Get ball
        if ball_safety_margin > 0:

            # increase ball safety margin in certain game scenarios
            if (w.play_mode_group != w.MG_OTHER) or abs(ball_2d[1])>9.5 or abs(ball_2d[0])>14.5:
                ball_safety_margin += 0.12

            obstacles.append((*ball_2d, 0, ball_safety_margin, 8))

        #---------------------------------------------- Get goal posts
        if goalpost_safety_margin > 0:
            obstacles.append((14.75, 1.10,goalpost_safety_margin,0,0))
            obstacles.append((14.75,-1.10,goalpost_safety_margin,0,0))

        #---------------------------------------------- Draw obstacles
        if self._draw_obstacles:
            d = w.team_draw if self._use_team_channel else w.draw
            if d.enabled:     
                for o in obstacles:
                    if o[3] > 0: d.circle(o[:2],o[3],o[4]/2, d.Color.orange, "path_obstacles", False)
                    if o[2] > 0: d.circle(o[:2],o[2],1, d.Color.red, "path_obstacles", False)
                d.flush("path_obstacles")

        return obstacles

    def _get_hot_start(self, start_distance):
        '''
        Get hot start position for path (considering the previous path)
        (as opposed to a cold start, where the path starts at the player)
        '''
        if self.last_update > 0 and self.world.time_local_ms - self.last_update == 20 and self.last_start_dist == start_distance: 
            return self.world.robot.loc_head_position[:2] + M.vector_from_angle(self.last_direction_rad, is_rad = True) * start_distance
        else:
            return self.world.robot.loc_head_position[:2] # return cold start if start_distance was different or the position was not updated in the last step

    def _update_hot_start(self, next_dir_rad, start_distance):
        ''' Update hot start position for next run '''
        self.last_direction_rad = next_dir_rad
        self.last_update = self.world.time_local_ms
        self.last_start_dist = start_distance

    def _extract_target_from_path(self, path, path_len, ret_segments):
        ret_seg_ceil = math.ceil(ret_segments)

        if path_len >= ret_seg_ceil:
            i = ret_seg_ceil * 2 # path index of ceil point (x)
            if ret_seg_ceil == ret_segments:
                return path[i:i+2]
            else:
                floor_w = ret_seg_ceil-ret_segments
                return path[i-2:i] * floor_w + path[i:i+2] * (1-floor_w)
        else:
            return path[-2:] # path end


    def get_path_to_ball(self, x_ori = None, x_dev = -0.2, y_dev = 0, torso_ori = None, torso_ori_thrsh = 1,
                         priority_unums:list=[], is_aggressive=True, safety_margin = 0.25, timeout = 3000):
        '''
        Get next target from path to ball (next absolute position + next absolute orientation)
        If the robot is an active player, and close to the ball, it makes sense to be aggressive
        If the robot is far, it should follow the role_position instead to predict the intersection with ball


        Parameters
        ----------
        x_ori : float
            (This variable allows the specification of a target position, relative to the ball, in a custom reference frame.)
            absolute orientation of the custom reference frame's x-axis
            if None, the orientation is given by the vector (robot->ball)
        x_dev : float
            (This variable allows the specification of a target position, relative to the ball, in a custom reference frame.)
            target position deviation, in the custom reference frame's x-axis
        y_dev : float
            (This variable allows the specification of a target position, relative to the ball, in a custom reference frame.)
            target position deviation, in the custom reference frame's y-axis
        torso_ori : float
            torso's target absolute orientation (see `torso_ori_thrsh`)
            if None, the orientation is given by the vector (robot->target)
        torso_ori_thrsh :  float
            `torso_ori` will only be applied when the distance between robot and final target is < `torso_ori_thrsh` meters
            otherwise, the robot will orient itself towards the final target
        priority_unums : list
            list of teammates to avoid (since their role is more important)
        is_aggressive : bool
            if True, safety margins are reduced for opponents
        safety_margin : float
            repulsion radius around ball to avoid colliding with it
        timeout : float
            maximum execution time (in microseconds)

        Returns
        -------
        next_pos : ndarray
            next absolute position from path to ball
        next_ori : float
            next absolute orientation
        distance : float
            minimum between (distance to final target) and (distance to ball) 


        Example
        -------
        ----------------------------------------------------------------------------------------------
        x_ori        |  x_dev  |  y_dev  |  torso_ori  |  OBS
        -------------+---------+---------+-------------+----------------------------------------------
        None =>      |    -    |   !0    |      -      |  Not recommended. Will not converge.
        (orient. of: |    0    |    0    |     None    |  Frontal  ball chase, expected* slow approach
        robot->ball) |    0    |    0    |    value    |  Oriented ball chase, expected* slow approach
                     |   >0    |    0    |      -      |  Not recommended. Will not converge.
                     |   <0    |    0    |     None    |  Frontal  ball chase until distance == x_dev
                     |   <0    |    0    |    value    |  Oriented ball chase until distance == x_dev
        -------------+---------+---------+-------------+----------------------------------------------
        value        |    -    |    -    |     None    |  Frontal point chase
                     |    -    |    -    |    value    |  Oriented point chase
        ----------------------------------------------------------------------------------------------
        * it depends on the caller function (expected slow walking near target)
        `torso_ori` will only be applied when the distance between robot and final target is < `torso_ori_thrsh` meters
        '''

        w = self.world
        r = w.robot
        dev = np.array([x_dev,y_dev])
        dev_len = np.linalg.norm(dev)
        dev_mult = 1

        # use ball prediction if we are further than 0.5 m and in PlayOn
        if np.linalg.norm(w.ball_abs_pos[:2] - r.loc_head_position[:2]) > 0.5 and w.play_mode_group == w.MG_OTHER:
            ball_2d = w.get_intersection_point_with_ball(0.4)[0] # intersection point, while moving at 0.4 m/s
        else:
            ball_2d = w.ball_abs_pos[:2]

        # custom reference frame orientation
        vec_me_ball = ball_2d - r.loc_head_position[:2]
        if x_ori is None:
            x_ori = M.vector_angle(vec_me_ball)

        distance_boost = 0 # boost for returned distance to target
        if torso_ori is not None and dev_len > 0:
            approach_ori_diff = abs(M.normalize_deg( r.imu_torso_orientation - torso_ori ))
            if approach_ori_diff > 15: # increase walking speed near target if robot is far from approach orientation
                distance_boost = 0.15
            if approach_ori_diff > 30: # increase target distance to ball   if robot is far from approach orientation
                dev_mult = 1.3
            if approach_ori_diff > 45: # increase safety margin around ball if robot is far from approach orientation
                safety_margin = max(0.32,safety_margin)

        #------------------------------------------- get target

        front_unit_vec = M.vector_from_angle(x_ori)
        left_unit_vec = np.array([-front_unit_vec[1], front_unit_vec[0]]) 

        rel_target = front_unit_vec * dev[0] + left_unit_vec * dev[1]
        target = ball_2d + rel_target * dev_mult
        target_vec = target - r.loc_head_position[:2]
        target_dist = np.linalg.norm(target_vec)

        if self._draw_path:
            d = self.world.team_draw if self._use_team_channel else self.world.draw
            d.point(target, 4, d.Color.red, "path_target") # will not draw if drawing object is internally disabled

        #------------------------------------------- get obstacles 

        # Ignore ball if we are on the same side of the target (with small margin)
        if dev_len>0 and np.dot(vec_me_ball, rel_target) < -0.10:
            safety_margin = 0

        obstacles = self.get_obstacles(include_teammates = True, include_opponents = True, include_play_mode_restrictions = True,
                           ball_safety_margin = safety_margin,
                           mode = Path_Manager.MODE_AGGRESSIVE if is_aggressive else Path_Manager.MODE_CAUTIOUS, 
                           priority_unums = priority_unums)
        
        # Add obstacle on the side opposite to the target 
        if dev_len>0 and safety_margin > 0:
            center = ball_2d - M.normalize_vec( rel_target ) * safety_margin
            obstacles.append((*center, 0, safety_margin*0.9, 5))
            if self._draw_obstacles:
                d = w.team_draw if self._use_team_channel else w.draw
                if d.enabled:   
                    d.circle(center,safety_margin*0.8,2.5, d.Color.orange, "path_obstacles_1")

        #------------------------------------------- get path

        # see explanation for the context at the hot start update section below
        start_pos = self._get_hot_start(Path_Manager.HOT_START_DIST_WALK) if target_dist > 0.4 else self.world.robot.loc_head_position[:2]

        path, path_len, path_status, path_cost = self.get_path(start_pos, True, obstacles, target, timeout)
        path_end = path[-2:] # last position allowed by A*

        #------------------------------------------- get relevant distances

        if w.ball_last_seen > w.time_local_ms - w.VISUALSTEP_MS:          # ball is in FOV
            raw_ball_dist = np.linalg.norm(w.ball_rel_torso_cart_pos[:2]) # - distance between torso center and ball center
        else:                                                             # otherwise use absolute coordinates to compute distance
            raw_ball_dist = np.linalg.norm(vec_me_ball)                   # - distance between head center and ball center

        avoid_touching_ball = (w.play_mode_group != w.MG_OTHER)
        distance_to_final_target = np.linalg.norm(path_end - r.loc_head_position[:2])
        distance_to_ball = max(0.07 if avoid_touching_ball else 0.14, raw_ball_dist - 0.13)
        caution_dist = min(distance_to_ball,distance_to_final_target)

        #------------------------------------------- get next target position
  
        next_pos = self._extract_target_from_path( path, path_len, ret_segments=1 if caution_dist < 1 else 2 )

        #------------------------------------------ get next target orientation

        # use given orientation if it exists, else target's orientation if far enough, else current orientation
        if torso_ori is not None:
            
            if caution_dist > torso_ori_thrsh:
                next_ori = M.vector_angle(target_vec)
            else:
                mid_ori = M.normalize_deg( M.vector_angle(vec_me_ball) - M.vector_angle(-dev) - x_ori + torso_ori )
                mid_ori_diff = abs(M.normalize_deg(mid_ori - r.imu_torso_orientation))
                final_ori_diff = abs(M.normalize_deg(torso_ori - r.imu_torso_orientation))
                next_ori = mid_ori if mid_ori_diff + 10 < final_ori_diff else torso_ori
            
        elif target_dist > 0.1:
            next_ori = M.vector_angle(target_vec)
        else:
            next_ori = r.imu_torso_orientation

        #------------------------------------------ update hot start for next run

        ''' Defining the hot start distance:
        - if path_len is zero, there is no hot start, because we are already there (dist=0)
        - if the target is close, the hot start is not applied (see above)
        - if the next pos is very close (due to hard obstacle), the hot start is the next pos (dist<Path_Manager.HOT_START_DIST_WALK)
        - otherwise, the hot start distance is constant (dist=Path_Manager.HOT_START_DIST_WALK)
        '''
        if path_len != 0:
            next_pos_vec = next_pos - self.world.robot.loc_head_position[:2]
            next_pos_dist = np.linalg.norm(next_pos_vec)
            self._update_hot_start(M.vector_angle(next_pos_vec, is_rad=True), min(Path_Manager.HOT_START_DIST_WALK,next_pos_dist))

        return next_pos, next_ori, min(distance_to_ball, distance_to_final_target + distance_boost)


    def get_path_to_target(self, target, ret_segments = 1.0, torso_ori = None, priority_unums:list=[], is_aggressive=True, timeout = 3000):
        '''
        Get next position from path to target (next absolute position + next absolute orientation)

        Parameters
        ----------
        ret_segments : float
            returned target's maximum distance (measured in path segments from hot start position)
            actual distance: min(ret_segments,path_length)
            each path segment has 0.10 m or 0.1*sqrt(2) m (if diagonal)
            if `ret_segments` is 0, the current position is returned
        torso_ori : float
            torso's target absolute orientation
            if None, the orientation is given by the vector (robot->target)
        priority_unums : list
            list of teammates to avoid (since their role is more important)
        is_aggressive : bool
            if True, safety margins are reduced for opponents
        timeout : float
            maximum execution time (in microseconds)
        '''

        w = self.world

        #------------------------------------------- get target

        target_vec = target - w.robot.loc_head_position[:2]
        target_dist = np.linalg.norm(target_vec)

         #------------------------------------------- get obstacles 

        obstacles = self.get_obstacles(include_teammates = True, include_opponents = True, include_play_mode_restrictions = True,
                           mode = Path_Manager.MODE_AGGRESSIVE if is_aggressive else Path_Manager.MODE_CAUTIOUS, priority_unums = priority_unums)

        #------------------------------------------- get path

        # see explanation for the context at the hot start update section below
        start_pos = self._get_hot_start(Path_Manager.HOT_START_DIST_WALK) if target_dist > 0.4 else self.world.robot.loc_head_position[:2]

        path, path_len, path_status, path_cost = self.get_path(start_pos, True, obstacles, target, timeout)
        path_end = path[-2:] # last position allowed by A*

        #------------------------------------------- get next target position
        next_pos = self._extract_target_from_path(path, path_len, ret_segments)

        #------------------------------------------ get next target orientation

        # use given orientation if it exists, else target's orientation if far enough, else current orientation
        if torso_ori is not None:
            next_ori = torso_ori
        elif target_dist > 0.1:
            next_ori = M.vector_angle(target_vec)
        else:
            next_ori = w.robot.imu_torso_orientation

        #------------------------------------------ update hot start for next run

        ''' Defining the hot start distance:
        - if path_len is zero, there is no hot start, because we are already there (dist=0)
        - if the target is close, the hot start is not applied (see above)
        - if the next pos is very close (due to hard obstacle), the hot start is the next pos (dist<Path_Manager.HOT_START_DIST_WALK)
        - otherwise, the hot start distance is constant (dist=Path_Manager.HOT_START_DIST_WALK)
        '''
        if path_len != 0:
            next_pos_vec = next_pos - self.world.robot.loc_head_position[:2]
            next_pos_dist = np.linalg.norm(next_pos_vec)
            self._update_hot_start(M.vector_angle(next_pos_vec, is_rad=True), min(Path_Manager.HOT_START_DIST_WALK,next_pos_dist))


        distance_to_final_target = np.linalg.norm(path_end - w.robot.loc_head_position[:2])

        return next_pos, next_ori, distance_to_final_target


    def get_dribble_path(self, ret_segments = None, optional_2d_target = None, goalpost_safety_margin=0.4, timeout = 3000):
        '''
        Get next position from path to target (next relative orientation)
        Path is optimized for dribble

        Parameters
        ----------
        ret_segments : float
            returned target's maximum distance (measured in path segments from hot start position)
            actual distance: min(ret_segments,path_length)
            each path segment has 0.10 m or 0.1*sqrt(2) m (if diagonal)
            if `ret_segments` is 0, the current position is returned
            if `ret_segments` is None, it is dynamically set according to the robot's speed
        optional_2d_target : float
            2D target
            if None, the target is the opponent's goal (the specific goal point is decided by the A* algorithm)
        goalpost_safety_margin : float
            hard repulsion radius around the opponents' goalposts
            if zero, the minimum margin is used
        timeout : float
            maximum execution time (in microseconds)
        '''

        r = self.world.robot
        ball_2d = self.world.ball_abs_pos[:2]

        #------------------------------------------- get obstacles 

        obstacles = self.get_obstacles(include_teammates = True, include_opponents = True, include_play_mode_restrictions = False,
                                       max_distance=5, max_age=1000, goalpost_safety_margin=goalpost_safety_margin, mode = Path_Manager.MODE_DRIBBLE)

        #------------------------------------------- get path

        start_pos = self._get_hot_start(Path_Manager.HOT_START_DIST_DRIBBLE)

        path, path_len, path_status, path_cost = self.get_path(start_pos, False, obstacles, optional_2d_target, timeout)

        #------------------------------------------- get next target position & orientation

        if ret_segments is None:
            ret_segments = 2.0

        next_pos = self._extract_target_from_path(path, path_len, ret_segments)
        next_rel_ori = M.normalize_deg(M.vector_angle(next_pos - ball_2d) - r.imu_torso_orientation)

        #------------------------------------------ update hot start for next run

        if path_len != 0:
            self._update_hot_start(M.deg_to_rad(r.imu_torso_orientation), Path_Manager.HOT_START_DIST_DRIBBLE)

        #------------------------------------------ draw
        if self._draw_path and path_status != Path_Manager.STATUS_DIRECT:
            d = self.world.team_draw if self._use_team_channel else self.world.draw
            d.point(next_pos, 2, d.Color.pink, "path_next_pos",False)   # will not draw if drawing object is internally disabled
            d.line(ball_2d, next_pos, 2, d.Color.pink, "path_next_pos") # will not draw if drawing object is internally disabled

        return next_pos, next_rel_ori

    
    def get_push_path(self, ret_segments = 1.5, optional_2d_target = None, avoid_opponents=False, timeout = 3000):
        '''
        Get next position from path ball to target (next absolute position)
        Path is optimized for critical push (no obstacles, also for preparation stability)

        Parameters
        ----------
        ret_segments : float
            returned target's maximum distance (measured in path segments from hot start position)
            actual distance: min(ret_segments,path_length)
            each path segment has 0.10 m or 0.1*sqrt(2) m (if diagonal)
            if `ret_segments` is 0, the current position is returned
        optional_2d_target : float
            2D target
            if None, the target is the opponent's goal (the specific goal point is decided by the A* algorithm)
        timeout : float
            maximum execution time (in microseconds)
        '''

        ball_2d = self.world.ball_abs_pos[:2]

        #------------------------------------------- get obstacles 

        obstacles = self.get_obstacles(include_teammates = False, include_opponents = avoid_opponents, include_play_mode_restrictions = False)

        #------------------------------------------- get path

        path, path_len, path_status, path_cost = self.get_path(ball_2d, False, obstacles, optional_2d_target, timeout)

        #------------------------------------------- get next target position & orientation

        next_pos = self._extract_target_from_path(path, path_len, ret_segments)

        return next_pos

    def get_path(self, start, allow_out_of_bounds, obstacles=[], optional_2d_target = None, timeout = 3000):
        '''
        Parameters
        ----------
        allow_out_of_bounds : bool
            allow path to go out of bounds, should be False when dribbling
        obstacles : list
            list of obstacles, where each obstacle is a tuple of 5 floats (x, y, hard radius, soft radius, repulsive force)
        optional_2d_target : float
            2D target
            if None, the target is the opponent's goal (the specific goal point is decided by the A* algorithm)
        timeout : float
            maximum execution time (in microseconds)
        '''

        go_to_goal = int(optional_2d_target is None)

        if optional_2d_target is None:
            optional_2d_target = (0,0)

        # flatten obstacles
        obstacles = sum(obstacles, tuple())
        assert len(obstacles) % 5 == 0, "Each obstacle should be characterized by exactly 5 float values"

        # Path parameters: start, allow_out_of_bounds, go_to_goal, optional_target, timeout (us), obstacles
        params = np.array([*start, int(allow_out_of_bounds), go_to_goal, *optional_2d_target, timeout, *obstacles], np.float32)
        path_ret  = a_star.compute(params)
        path = path_ret[:-2]
        path_status = path_ret[-2]

        #---------------------------------------------- Draw path segments
        if self._draw_path:
            d = self.world.team_draw if self._use_team_channel else self.world.draw
            if d.enabled:     
                c = {0: d.Color.green_lawn, 1: d.Color.yellow, 2: d.Color.red, 3: d.Color.cyan}[path_status]
                for j in range(0, len(path)-2, 2):
                    d.line((path[j],path[j+1]),(path[j+2],path[j+3]), 1, c, "path_segments", False)
                d.flush("path_segments")

        return path, len(path)//2-1, path_status, path_ret[-1] # path, path_len (number of segments), path_status, path_cost (A* cost)