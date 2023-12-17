from math import asin, atan, atan2, pi, sqrt
from math_ops.Matrix_3x3 import Matrix_3x3
from math_ops.Math_Ops import Math_Ops as M
import numpy as np

class Inverse_Kinematics():

    # leg y deviation, upper leg height, upper leg depth, lower leg length, knee extra angle, max ankle z
    NAO_SPECS_PER_ROBOT = ((0.055,      0.12,        0.005, 0.1,         atan(0.005/0.12),        -0.091),
                           (0.055,      0.13832,     0.005, 0.11832,     atan(0.005/0.13832),     -0.106),
                           (0.055,      0.12,        0.005, 0.1,         atan(0.005/0.12),        -0.091),
                           (0.072954143,0.147868424, 0.005, 0.127868424, atan(0.005/0.147868424), -0.114),
                           (0.055,      0.12,        0.005, 0.1,         atan(0.005/0.12),        -0.091))

    TORSO_HIP_Z = 0.115 # distance in the z-axis, between the torso and each hip (same for all robots)
    TORSO_HIP_X = 0.01  # distance in the x-axis, between the torso and each hip (same for all robots) (hip is 0.01m to the back)

    def __init__(self, robot) -> None:
        self.robot = robot
        self.NAO_SPECS = Inverse_Kinematics.NAO_SPECS_PER_ROBOT[robot.type]

    def torso_to_hip_transform(self, coords, is_batch=False):
        '''
        Convert cartesian coordinates that are relative to torso to coordinates that are relative the center of both hip joints
        
        Parameters
        ----------
        coords : array_like
            One 3D position or list of 3D positions
        is_batch : `bool`
            Indicates if coords is a batch of 3D positions

        Returns
        -------
        coord : `list` or ndarray
            A numpy array is returned if is_batch is False, otherwise, a list of arrays is returned   
        '''
        if is_batch:
            return [c + (Inverse_Kinematics.TORSO_HIP_X, 0, Inverse_Kinematics.TORSO_HIP_Z) for c in coords]
        else:
            return coords + (Inverse_Kinematics.TORSO_HIP_X, 0, Inverse_Kinematics.TORSO_HIP_Z)
        

    def head_to_hip_transform(self, coords, is_batch=False):
        '''
        Convert cartesian coordinates that are relative to head to coordinates that are relative the center of both hip joints
        
        Parameters
        ----------
        coords : array_like
            One 3D position or list of 3D positions
        is_batch : `bool`
            Indicates if coords is a batch of 3D positions

        Returns
        -------
        coord : `list` or ndarray
            A numpy array is returned if is_batch is False, otherwise, a list of arrays is returned   
        '''
        coords_rel_torso = self.robot.head_to_body_part_transform( "torso", coords, is_batch )
        return self.torso_to_hip_transform(coords_rel_torso, is_batch)

    def get_body_part_pos_relative_to_hip(self, body_part_name):
        ''' Get body part position relative to the center of both hip joints '''
        bp_rel_head = self.robot.body_parts[body_part_name].transform.get_translation()
        return self.head_to_hip_transform(bp_rel_head)

    def get_ankle_pos_relative_to_hip(self, is_left):
        ''' Internally calls get_body_part_pos_relative_to_hip() '''
        return self.get_body_part_pos_relative_to_hip("lankle" if is_left else "rankle")

    def get_linear_leg_trajectory(self, is_left:bool, p1, p2=None, foot_ori3d=(0,0,0), dynamic_pose:bool=True, resolution=100):
        ''' 
        Compute leg trajectory so that the ankle moves linearly between two 3D points (relative to hip) 
        
        Parameters
        ----------
        is_left : `bool`
            set to True to select left leg, False to select right leg
        p1 : array_like, length 3
            if p2 is None:     
                p1 is the target position (relative to hip), and the initial point is given by the ankle's current position
            if p2 is not None: 
                p1 is the initial point (relative to hip)
        p2 : array_like, length 3 / `None`
            target position (relative to hip) or None (see p1)
        foot_ori3d : array_like, length 3
            rotation around x,y,z (rotation around x & y are biases, relative to a vertical pose, or dynamic pose, if enabled)
        dynamic_pose : `bool`
            enable dynamic feet rotation to be parallel to the ground, based on IMU
        resolution : int
            interpolation resolution; more resolution is always better, but it takes more time to compute;
            having more points does not make the movement slower, because if there are excessive points they are removed
            during the analytical optimization

        Returns
        -------
        trajecory : `tuple`
            indices, [[values_1,error_codes_1], [values_2,error_codes_2], ...]
            See leg() for further details
        '''

        if p2 is None:
            p2 = np.asarray(p1, float)
            p1 = self.get_body_part_pos_relative_to_hip('lankle' if is_left else 'rankle')
        else:
            p1 = np.asarray(p1, float)
            p2 = np.asarray(p2, float)

        vec = (p2 - p1) / resolution


        hip_points = [p1 + vec * i for i in range(1,resolution+1)]
        interpolation = [self.leg(p, foot_ori3d, is_left, dynamic_pose) for p in hip_points]

        indices = [2,4,6,8,10,12] if is_left else [3,5,7,9,11,13]

        last_joint_values = self.robot.joints_position[indices[0:4]] #exclude feet joints to compute ankle trajectory
        next_step = interpolation[0]
        trajectory = []

        for p in interpolation[1:-1]:
            if np.any(np.abs(p[1][0:4]-last_joint_values) > 7.03): 
                trajectory.append(next_step[1:3])
                last_joint_values = next_step[1][0:4]
                next_step = p
            else:
                next_step = p

        trajectory.append(interpolation[-1][1:3])

        return indices, trajectory



    def leg(self, ankle_pos3d, foot_ori3d, is_left:bool, dynamic_pose:bool):
        '''
        Compute inverse kinematics for the leg, considering as input the relative 3D position of the ankle and 3D orientation* of the foot
        *the yaw can be controlled directly, but the pitch and roll are biases (see below)

        Parameters
        ----------
        ankle_pos3d : array_like, length 3
            (x,y,z) position of ankle in 3D, relative to the center of both hip joints
        foot_ori3d : array_like, length 3
            rotation around x,y,z (rotation around x & y are biases, relative to a vertical pose, or dynamic pose, if enabled)
        is_left : `bool`
            set to True to select left leg, False to select right leg
        dynamic_pose : `bool`
            enable dynamic feet rotation to be parallel to the ground, based on IMU

        Returns
        -------
        indices : `list`
            indices of computed joints
        values : `list`
            values of computed joints
        error_codes : `list`
            list of error codes
                Error codes:
                    (-1) Foot is too far (unreachable)
                    (x)  Joint x is out of range
        '''

        error_codes = []
        leg_y_dev, upper_leg_height, upper_leg_depth, lower_leg_len, knee_extra_angle, _ = self.NAO_SPECS
        sign = -1 if is_left else 1

        # Then we translate to origin of leg by shifting the y coordinate
        ankle_pos3d = np.asarray(ankle_pos3d) + (0,sign*leg_y_dev,0)

        # First we rotate the leg, then we rotate the coordinates to abstract from the rotation
        ankle_pos3d = Matrix_3x3().rotate_z_deg(-foot_ori3d[2]).multiply(ankle_pos3d)

        # Use geometric solution to compute knee angle and foot pitch
        dist = np.linalg.norm(ankle_pos3d)  #dist hip <-> ankle
        sq_dist = dist * dist
        sq_upper_leg_h = upper_leg_height * upper_leg_height
        sq_lower_leg_l = lower_leg_len * lower_leg_len
        sq_upper_leg_l = upper_leg_depth * upper_leg_depth + sq_upper_leg_h
        upper_leg_len = sqrt(sq_upper_leg_l)
        knee = M.acos((sq_upper_leg_l + sq_lower_leg_l - sq_dist)/(2 * upper_leg_len * lower_leg_len)) + knee_extra_angle # Law of cosines
        foot = M.acos((sq_lower_leg_l + sq_dist - sq_upper_leg_l)/(2 * lower_leg_len * dist)) # foot perpendicular to vec(origin->ankle_pos)

        # Check if target is reachable
        if dist > upper_leg_len + lower_leg_len: 
            error_codes.append(-1)

        # Knee and foot
        knee_angle = pi - knee
        foot_pitch = foot - atan(ankle_pos3d[0] / np.linalg.norm(ankle_pos3d[1:3]))
        foot_roll = atan(ankle_pos3d[1] / min(-0.05, ankle_pos3d[2])) * -sign  # avoid instability of foot roll (not relevant above -0.05m)

        # Raw hip angles if all joints were straightforward
        raw_hip_yaw = foot_ori3d[2]
        raw_hip_pitch = foot_pitch - knee_angle
        raw_hip_roll = -sign * foot_roll

        # Rotate 45deg due to yaw joint orientation, then rotate yaw, roll and pitch
        m = Matrix_3x3().rotate_y_rad(raw_hip_pitch).rotate_x_rad(raw_hip_roll).rotate_z_deg(raw_hip_yaw).rotate_x_deg(-45*sign)

        # Get actual hip angles considering the yaw joint orientation
        hip_roll = (pi/4) - (sign * asin(m.m[1,2])) #Add pi/4 due to 45deg rotation
        hip_pitch = - atan2(m.m[0,2],m.m[2,2])
        hip_yaw = sign * atan2(m.m[1,0],m.m[1,1])

        # Convert rad to deg
        values = np.array([hip_yaw,hip_roll,hip_pitch,-knee_angle,foot_pitch,foot_roll]) * 57.2957795 #rad to deg

        # Set feet rotation bias (based on vertical pose, or dynamic_pose)
        values[4] -= foot_ori3d[1]
        values[5] -= foot_ori3d[0] * sign

        indices = [2,4,6,8,10,12] if is_left else [3,5,7,9,11,13]

        if dynamic_pose:

            # Rotation of torso in relation to foot
            m : Matrix_3x3 = Matrix_3x3.from_rotation_deg((self.robot.imu_torso_roll, self.robot.imu_torso_pitch, 0))
            m.rotate_z_deg(foot_ori3d[2], True)

            roll =  m.get_roll_deg()
            pitch = m.get_pitch_deg()

            # Simple balance algorithm
            correction = 1 #correction to motivate a vertical torso (in degrees)
            roll  = 0 if abs(roll)  < correction else roll  - np.copysign(correction,roll)
            pitch = 0 if abs(pitch) < correction else pitch - np.copysign(correction,pitch)
     
            values[4] += pitch
            values[5] += roll * sign


        # Check and limit range of joints
        for i in range(len(indices)):
            if values[i] < self.robot.joints_info[indices[i]].min or values[i] > self.robot.joints_info[indices[i]].max: 
                error_codes.append(indices[i])
                values[i] = np.clip(values[i], self.robot.joints_info[indices[i]].min, self.robot.joints_info[indices[i]].max)


        return indices, values, error_codes

