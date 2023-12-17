from collections import deque
from math import atan, pi, sqrt, tan
from math_ops.Math_Ops import Math_Ops as M
from math_ops.Matrix_3x3 import Matrix_3x3
from math_ops.Matrix_4x4 import Matrix_4x4
from world.commons.Body_Part import Body_Part
from world.commons.Joint_Info import Joint_Info
import numpy as np
import xml.etree.ElementTree as xmlp

class Robot():
    STEPTIME = 0.02   # Fixed step time
    VISUALSTEP = 0.04 # Fixed visual step time
    SQ_STEPTIME = STEPTIME * STEPTIME
    GRAVITY = np.array([0,0,-9.81])
    IMU_DECAY = 0.996 #IMU's velocity decay
            
    #------------------ constants to force symmetry in joints/effectors

    MAP_PERCEPTOR_TO_INDEX = {"hj1":0,  "hj2":1,  "llj1":2, "rlj1":3,
                              "llj2":4, "rlj2":5, "llj3":6, "rlj3":7,
                              "llj4":8, "rlj4":9, "llj5":10,"rlj5":11,
                              "llj6":12,"rlj6":13,"laj1":14,"raj1":15,
                              "laj2":16,"raj2":17,"laj3":18,"raj3":19,
                              "laj4":20,"raj4":21,"llj7":22,"rlj7":23 }

    # Fix symmetry issues 1a/4 (identification)                                  
    FIX_PERCEPTOR_SET = {'rlj2','rlj6','raj2','laj3','laj4'}
    FIX_INDICES_LIST = [5,13,17,18,20]

    # Recommended height for unofficial beam (near ground)
    BEAM_HEIGHTS = [0.4, 0.43, 0.4, 0.46, 0.4]


    def __init__(self, unum:int, robot_type:int) -> None:
        robot_xml = "nao"+str(robot_type)+".xml" # Typical NAO file name
        self.type = robot_type
        self.beam_height = Robot.BEAM_HEIGHTS[robot_type]
        self.no_of_joints = 24 if robot_type == 4 else 22 

        #Fix symmetry issues 1b/4 (identification) 
        self.FIX_EFFECTOR_MASK = np.ones(self.no_of_joints)
        self.FIX_EFFECTOR_MASK[Robot.FIX_INDICES_LIST] = -1

        self.body_parts = dict()    # keys='body part names' (given by the robot's XML),  values='Body_Part objects'
        self.unum = unum            # Robot's uniform number
        self.gyro = np.zeros(3)     # Angular velocity along the three axes of freedom of the robot's torso (deg/s)
        self.acc  = np.zeros(3)     # Proper acceleration along the three axes of freedom of the robot's torso (m/s2)
        self.frp = dict() # foot "lf"/"rf", toe "lf1"/"rf1" resistance perceptor (relative [p]oint of origin + [f]orce vector) e.g. {"lf":(px,py,pz,fx,fy,fz)}
        self.feet_toes_last_touch = {"lf":0,"rf":0,"lf1":0,"rf1":0} # foot "lf"/"rf", toe "lf1"/"rf1" World.time_local_ms when foot/toe last touched any surface
        self.feet_toes_are_touching = {"lf":False,"rf":False,"lf1":False,"rf1":False} # foot "lf"/"rf", toe "lf1"/"rf1" True if touching in last received server message
        self.fwd_kinematics_list = None             # List of body parts, ordered according to dependencies
        self.rel_cart_CoM_position = np.zeros(3)    # Center of Mass position, relative to head, in cartesian coordinates (m)

        # Joint variables are optimized for performance / array operations
        self.joints_position =          np.zeros(self.no_of_joints)                      # Joints' angular position  (deg)
        self.joints_speed =             np.zeros(self.no_of_joints)                      # Joints' angular speed     (rad/s)
        self.joints_target_speed =      np.zeros(self.no_of_joints)                      # Joints' target speed      (rad/s) (max: 6.1395 rad/s, see rcssserver3d/data/rsg/agent/nao/hingejoint.rsg)
        self.joints_target_last_speed = np.zeros(self.no_of_joints)                      # Joints' last target speed (rad/s) (max: 6.1395 rad/s, see rcssserver3d/data/rsg/agent/nao/hingejoint.rsg)
        self.joints_info =              [None] * self.no_of_joints                       # Joints' constant information (see class Joint_Info)
        self.joints_transform =         [Matrix_4x4() for _ in range(self.no_of_joints)] # Joints' transformation matrix

        # Localization variables relative to head
        self.loc_head_to_field_transform = Matrix_4x4()  # Transformation matrix from head to field
        self.loc_field_to_head_transform = Matrix_4x4()  # Transformation matrix from field to head
        self.loc_rotation_head_to_field = Matrix_3x3()   # Rotation matrix from head to field
        self.loc_rotation_field_to_head = Matrix_3x3()   # Rotation matrix from field to head
        self.loc_head_position = np.zeros(3)             # Absolute head position (m)
        self.loc_head_position_history = deque(maxlen=40)# Absolute head position history (queue with up to 40 old positions at intervals of 0.04s, where index 0 is the previous position)
        self.loc_head_velocity = np.zeros(3)             # Absolute head velocity (m/s) (Warning: possibly noisy)
        self.loc_head_orientation = 0                    # Head orientation (deg)
        self.loc_is_up_to_date = False                   # False if this is not a visual step, or not enough elements are visible
        self.loc_last_update = 0                         # World.time_local_ms when the localization was last updated
        self.loc_head_position_last_update = 0           # World.time_local_ms when loc_head_position was last updated by vision or radio
        self.radio_fallen_state = False                  # True if (radio says we fell) and (radio is significantly more recent than loc)
        self.radio_last_update = 0                       # World.time_local_ms when radio_fallen_state was last updated (and possibly loc_head_position)

        # Localization variables relative to torso
        self.loc_torso_to_field_rotation = Matrix_3x3()  # Rotation matrix from torso to field  
        self.loc_torso_to_field_transform = Matrix_4x4() # Transformation matrix from torso to field
        self.loc_torso_roll = 0                          # Torso roll        (deg)
        self.loc_torso_pitch = 0                         # Torso pitch       (deg) 
        self.loc_torso_orientation = 0                   # Torso orientation (deg)
        self.loc_torso_inclination = 0                   # Torso inclination (deg) (inclination of z-axis in relation to field z-axis)
        self.loc_torso_position = np.zeros(3)            # Absolute torso position (m)
        self.loc_torso_velocity = np.zeros(3)            # Absolute torso velocity (m/s)
        self.loc_torso_acceleration = np.zeros(3)        # Absolute Coordinate acceleration (m/s2)

        # Other localization variables
        self.cheat_abs_pos = np.zeros(3)                 # Absolute head position    provided by the server as cheat (m)
        self.cheat_ori = 0.0                             # Absolute head orientation provided by the server as cheat (deg)
        self.loc_CoM_position = np.zeros(3)              # Absolute CoM position (m)
        self.loc_CoM_velocity = np.zeros(3)              # Absolute CoM velocity (m/s)

        # Localization special variables
        '''
        self.loc_head_z is often equivalent to self.loc_head_position[2], but sometimes it differs.
        There are situations in which the rotation and translation cannot be computed, 
        but the z-coordinate can still be found through vision, in which case:
            self.loc_is_up_to_date          is False
            self.loc_head_z_is_up_to_date   is True
        It should be used in applications which rely on z as an independent coordinate, such
        as detecting if the robot has fallen, or as an observation for machine learning.
        It should NEVER be used for 3D transformations.
        '''
        self.loc_head_z = 0                     # Absolute head position (z) - see above for explanation (m)
        self.loc_head_z_is_up_to_date = False   # False if this is not a visual step, or not enough elements are visible
        self.loc_head_z_last_update = 0         # World.time_local_ms when loc_head_z was last computed
        self.loc_head_z_vel = 0                 # Absolute head velocity (z) (m/s)

        # Localization + Gyroscope
        # These variables are reliable. The gyroscope is used to update the rotation when waiting for the next visual cycle
        self.imu_torso_roll = 0                          # Torso roll        (deg)                    (src: Localization + Gyro)
        self.imu_torso_pitch = 0                         # Torso pitch       (deg)                    (src: Localization + Gyro)
        self.imu_torso_orientation = 0                   # Torso orientation (deg)                    (src: Localization + Gyro)
        self.imu_torso_inclination = 0                   # Torso inclination (deg)                    (src: Localization + Gyro)
        self.imu_torso_to_field_rotation = Matrix_3x3()  # Rotation matrix from torso to field        (src: Localization + Gyro)
        self.imu_last_visual_update = 0                  # World.time_local_ms when the IMU data was last updated with visual information 

        # Localization + Gyroscope + Accelerometer
        # Warning: these variables are unreliable, since small errors in the Localization Orientation lead to 
        #          wrong acceleration -> wrong velocity -> wrong position
        self.imu_weak_torso_to_field_transform = Matrix_4x4() # Transformation matrix from torso to field  (src: Localization + Gyro + Acc)
        self.imu_weak_head_to_field_transform  = Matrix_4x4() # Transformation matrix from head to field   (src: Localization + Gyro + Acc)
        self.imu_weak_field_to_head_transform  = Matrix_4x4() # Transformation matrix from field to head   (src: Localization + Gyro + Acc)
        self.imu_weak_torso_position = np.zeros(3)        # Absolute torso position (m)                    (src: Localization + Gyro + Acc)
        self.imu_weak_torso_velocity = np.zeros(3)        # Absolute torso velocity (m/s)                  (src: Localization + Gyro + Acc)
        self.imu_weak_torso_acceleration = np.zeros(3)    # Absolute torso acceleration (m/s2)             (src: Localization + Gyro + Acc)
        self.imu_weak_torso_next_position = np.zeros(3)   # Absolute position in next step estimate (m)    (src: Localization + Gyro + Acc)
        self.imu_weak_torso_next_velocity = np.zeros(3)   # Absolute velocity in next step estimate (m/s)  (src: Localization + Gyro + Acc)
        self.imu_weak_CoM_position = np.zeros(3)          # Absolute CoM position (m)                      (src: Localization + Gyro + Acc)
        self.imu_weak_CoM_velocity = np.zeros(3)          # Absolute CoM velocity (m/s)                    (src: Localization + Gyro + Acc)


        #Using explicit variables to enable IDE suggestions
        self.J_HEAD_YAW = 0
        self.J_HEAD_PITCH = 1
        self.J_LLEG_YAW_PITCH = 2
        self.J_RLEG_YAW_PITCH = 3
        self.J_LLEG_ROLL = 4
        self.J_RLEG_ROLL = 5
        self.J_LLEG_PITCH = 6
        self.J_RLEG_PITCH = 7
        self.J_LKNEE = 8
        self.J_RKNEE = 9
        self.J_LFOOT_PITCH = 10
        self.J_RFOOT_PITCH = 11
        self.J_LFOOT_ROLL = 12
        self.J_RFOOT_ROLL = 13
        self.J_LARM_PITCH = 14
        self.J_RARM_PITCH = 15
        self.J_LARM_ROLL = 16
        self.J_RARM_ROLL = 17
        self.J_LELBOW_YAW = 18
        self.J_RELBOW_YAW = 19
        self.J_LELBOW_ROLL = 20
        self.J_RELBOW_ROLL = 21
        self.J_LTOE_PITCH = 22
        self.J_RTOE_PITCH = 23


        #------------------ parse robot xml

        dir = M.get_active_directory("/world/commons/robots/")
        robot_xml_root = xmlp.parse(dir + robot_xml).getroot()

        joint_no = 0
        for child in robot_xml_root:
            if child.tag == "bodypart":
                self.body_parts[child.attrib['name']] = Body_Part(child.attrib['mass'])
            elif child.tag == "joint":
                self.joints_info[joint_no] = Joint_Info(child)
                self.joints_position[joint_no] = 0.0
                ji = self.joints_info[joint_no]

                #save joint if body part is 1st anchor (to simplify model traversal in a single direction)
                self.body_parts[ji.anchor0_part].joints.append(Robot.MAP_PERCEPTOR_TO_INDEX[ji.perceptor]) 

                joint_no += 1
                if joint_no == self.no_of_joints: break #ignore extra joints

            else:
                raise NotImplementedError

        assert joint_no == self.no_of_joints, "The Robot XML and the robot type don't match!"


    def get_head_abs_vel(self, history_steps:int):
        '''
        Get robot's head absolute velocity (m/s)

        Parameters
        ----------
        history_steps : int
            number of history steps to consider [1,40]

        Examples
        --------
        get_head_abs_vel(1) is equivalent to (current abs pos - last abs pos)      / 0.04
        get_head_abs_vel(2) is equivalent to (current abs pos - abs pos 0.08s ago) / 0.08
        get_head_abs_vel(3) is equivalent to (current abs pos - abs pos 0.12s ago) / 0.12
        '''
        assert 1 <= history_steps <= 40, "Argument 'history_steps' must be in range [1,40]"

        if len(self.loc_head_position_history) == 0:
            return np.zeros(3)

        h_step = min(history_steps, len(self.loc_head_position_history))
        t = h_step * Robot.VISUALSTEP

        return (self.loc_head_position - self.loc_head_position_history[h_step-1]) / t
        

    def _initialize_kinematics(self):

        #starting with head
        parts={"head"}
        sequential_body_parts = ["head"]

        while len(parts) > 0:
            part = parts.pop()

            for j in self.body_parts[part].joints:

                p = self.joints_info[j].anchor1_part

                if len(self.body_parts[p].joints) > 0: #add body part if it is the 1st anchor of some joint
                    parts.add(p)
                    sequential_body_parts.append(p)

        self.fwd_kinematics_list = [(self.body_parts[part],j, self.body_parts[self.joints_info[j].anchor1_part] ) 
                                     for part in sequential_body_parts for j in self.body_parts[part].joints]

        #Fix symmetry issues 4/4 (kinematics)
        for i in Robot.FIX_INDICES_LIST:
            self.joints_info[i].axes *= -1
            aux = self.joints_info[i].min
            self.joints_info[i].min = -self.joints_info[i].max
            self.joints_info[i].max = -aux


    def update_localization(self, localization_raw, time_local_ms): 

        # parse raw data
        loc = localization_raw.astype(float) #32bits to 64bits for consistency
        self.loc_is_up_to_date = bool(loc[32])
        self.loc_head_z_is_up_to_date = bool(loc[34])

        if self.loc_head_z_is_up_to_date:
            time_diff = (time_local_ms - self.loc_head_z_last_update) / 1000 
            self.loc_head_z_vel = (loc[33] - self.loc_head_z) / time_diff
            self.loc_head_z = loc[33]
            self.loc_head_z_last_update = time_local_ms

        # Save last position to history at every vision cycle (even if not up to date) (update_localization is only called at vision cycles)
        self.loc_head_position_history.appendleft(np.copy(self.loc_head_position))

        if self.loc_is_up_to_date:
            time_diff = (time_local_ms - self.loc_last_update) / 1000
            self.loc_last_update = time_local_ms
            self.loc_head_to_field_transform.m[:] = loc[0:16].reshape((4,4))
            self.loc_field_to_head_transform.m[:] = loc[16:32].reshape((4,4))
        
            # extract data (related to the robot's head)
            self.loc_rotation_head_to_field = self.loc_head_to_field_transform.get_rotation()
            self.loc_rotation_field_to_head = self.loc_field_to_head_transform.get_rotation()
            p = self.loc_head_to_field_transform.get_translation()
            self.loc_head_velocity = (p - self.loc_head_position) / time_diff
            self.loc_head_position = p
            self.loc_head_position_last_update = time_local_ms
            self.loc_head_orientation = self.loc_head_to_field_transform.get_yaw_deg()
            self.radio_fallen_state = False

            # extract data (related to the center of mass)
            p = self.loc_head_to_field_transform(self.rel_cart_CoM_position)
            self.loc_CoM_velocity = (p - self.loc_CoM_position) / time_diff
            self.loc_CoM_position = p

            # extract data (related to the robot's torso)
            t = self.get_body_part_to_field_transform('torso')
            self.loc_torso_to_field_transform = t
            self.loc_torso_to_field_rotation = t.get_rotation()
            self.loc_torso_orientation = t.get_yaw_deg()
            self.loc_torso_pitch = t.get_pitch_deg()
            self.loc_torso_roll = t.get_roll_deg()
            self.loc_torso_inclination = t.get_inclination_deg()
            p = t.get_translation()
            self.loc_torso_velocity = (p - self.loc_torso_position) / time_diff
            self.loc_torso_position = p
            self.loc_torso_acceleration = self.loc_torso_to_field_rotation.multiply(self.acc) + Robot.GRAVITY


    def head_to_body_part_transform(self, body_part_name, coords, is_batch=False):
        '''
        If coord is a vector or list of vectors:
        Convert cartesian coordinates that are relative to head to coordinates that are relative to a body part 

        If coord is a Matrix_4x4 or a list of Matrix_4x4:
        Convert pose that is relative to head to a pose that is relative to a body part 
        
        Parameters
        ----------
        body_part_name : `str`
            name of body part (given by the robot's XML)
        coords : array_like
            One 3D position or list of 3D positions
        is_batch : `bool`
            Indicates if coords is a batch of 3D positions

        Returns
        -------
        coord : `list` or ndarray
            A numpy array is returned if is_batch is False, otherwise, a list of arrays is returned
        '''
        head_to_bp_transform : Matrix_4x4 = self.body_parts[body_part_name].transform.invert()
        
        if is_batch:
            return [head_to_bp_transform(c) for c in coords]
        else:
            return head_to_bp_transform(coords)



    def get_body_part_to_field_transform(self, body_part_name) -> Matrix_4x4:
        '''
        Computes the transformation matrix from body part to field, from which we can extract its absolute position and rotation.
        For best results, use this method when self.loc_is_up_to_date is True. Otherwise, the forward kinematics
        will not be synced with the localization data and strange results may occur.
        '''
        return self.loc_head_to_field_transform.multiply(self.body_parts[body_part_name].transform)

    def get_body_part_abs_position(self, body_part_name) -> np.ndarray:
        '''
        Computes the absolute position of a body part considering the localization data and forward kinematics.
        For best results, use this method when self.loc_is_up_to_date is True. Otherwise, the forward kinematics
        will not be synced with the localization data and strange results may occur.
        '''
        return self.get_body_part_to_field_transform(body_part_name).get_translation()

    def get_joint_to_field_transform(self, joint_index) -> Matrix_4x4:
        '''
        Computes the transformation matrix from joint to field, from which we can extract its absolute position and rotation.
        For best results, use this method when self.loc_is_up_to_date is True. Otherwise, the forward kinematics
        will not be synced with the localization data and strange results may occur.
        '''
        return self.loc_head_to_field_transform.multiply(self.joints_transform[joint_index])

    def get_joint_abs_position(self, joint_index) -> np.ndarray:
        '''
        Computes the absolute position of a joint considering the localization data and forward kinematics.
        For best results, use this method when self.loc_is_up_to_date is True. Otherwise, the forward kinematics
        will not be synced with the localization data and strange results may occur.
        '''
        return self.get_joint_to_field_transform(joint_index).get_translation()

    def update_pose(self):

        if self.fwd_kinematics_list is None:
            self._initialize_kinematics()

        for body_part, j, child_body_part in self.fwd_kinematics_list:
            ji = self.joints_info[j]
            self.joints_transform[j].m[:] = body_part.transform.m
            self.joints_transform[j].translate(ji.anchor0_axes, True)
            child_body_part.transform.m[:] = self.joints_transform[j].m
            child_body_part.transform.rotate_deg(ji.axes, self.joints_position[j], True)
            child_body_part.transform.translate(ji.anchor1_axes_neg, True)

        self.rel_cart_CoM_position = np.average([b.transform.get_translation() for b in self.body_parts.values()], 0,
                                                [b.mass                        for b in self.body_parts.values()])


    def update_imu(self, time_local_ms):

        # update IMU
        if self.loc_is_up_to_date:
            self.imu_torso_roll = self.loc_torso_roll
            self.imu_torso_pitch = self.loc_torso_pitch   
            self.imu_torso_orientation = self.loc_torso_orientation
            self.imu_torso_inclination = self.loc_torso_inclination
            self.imu_torso_to_field_rotation.m[:] = self.loc_torso_to_field_rotation.m
            self.imu_weak_torso_to_field_transform.m[:] = self.loc_torso_to_field_transform.m
            self.imu_weak_head_to_field_transform.m[:] = self.loc_head_to_field_transform.m
            self.imu_weak_field_to_head_transform.m[:] = self.loc_field_to_head_transform.m
            self.imu_weak_torso_position[:] = self.loc_torso_position
            self.imu_weak_torso_velocity[:] = self.loc_torso_velocity
            self.imu_weak_torso_acceleration[:] = self.loc_torso_acceleration
            self.imu_weak_torso_next_position = self.loc_torso_position + self.loc_torso_velocity * Robot.STEPTIME + self.loc_torso_acceleration * (0.5 * Robot.SQ_STEPTIME)
            self.imu_weak_torso_next_velocity = self.loc_torso_velocity + self.loc_torso_acceleration * Robot.STEPTIME
            self.imu_weak_CoM_position[:] = self.loc_CoM_position
            self.imu_weak_CoM_velocity[:] = self.loc_CoM_velocity
            self.imu_last_visual_update = time_local_ms
        else:
            g = self.gyro / 50 # convert degrees per second to degrees per step

            self.imu_torso_to_field_rotation.multiply( Matrix_3x3.from_rotation_deg(g), in_place=True, reverse_order=True)

            self.imu_torso_orientation = self.imu_torso_to_field_rotation.get_yaw_deg()
            self.imu_torso_pitch = self.imu_torso_to_field_rotation.get_pitch_deg()
            self.imu_torso_roll = self.imu_torso_to_field_rotation.get_roll_deg()

            self.imu_torso_inclination = atan(sqrt(tan(self.imu_torso_roll/180*pi)**2+tan(self.imu_torso_pitch/180*pi)**2))*180/pi

            # Update position and velocity until 0.2 seconds has passed since last visual update
            if time_local_ms < self.imu_last_visual_update + 200:
                self.imu_weak_torso_position[:] = self.imu_weak_torso_next_position
                if self.imu_weak_torso_position[2] < 0: self.imu_weak_torso_position[2] = 0 # limit z coordinate to positive values
                self.imu_weak_torso_velocity[:] = self.imu_weak_torso_next_velocity * Robot.IMU_DECAY # stability tradeoff
            else:
                self.imu_weak_torso_velocity *= 0.97 # without visual updates for 0.2s, the position is locked, and the velocity decays to zero

            # convert proper acceleration to coordinate acceleration and fix rounding bias
            self.imu_weak_torso_acceleration = self.imu_torso_to_field_rotation.multiply(self.acc) + Robot.GRAVITY
            self.imu_weak_torso_to_field_transform = Matrix_4x4.from_3x3_and_translation(self.imu_torso_to_field_rotation,self.imu_weak_torso_position)
            self.imu_weak_head_to_field_transform = self.imu_weak_torso_to_field_transform.multiply(self.body_parts["torso"].transform.invert())
            self.imu_weak_field_to_head_transform = self.imu_weak_head_to_field_transform.invert()
            p = self.imu_weak_head_to_field_transform(self.rel_cart_CoM_position)
            self.imu_weak_CoM_velocity = (p-self.imu_weak_CoM_position)/Robot.STEPTIME
            self.imu_weak_CoM_position = p

            # Next Position = x0 + v0*t + 0.5*a*t^2,   Next velocity = v0 + a*t
            self.imu_weak_torso_next_position = self.imu_weak_torso_position + self.imu_weak_torso_velocity * Robot.STEPTIME + self.imu_weak_torso_acceleration * (0.5 * Robot.SQ_STEPTIME)
            self.imu_weak_torso_next_velocity = self.imu_weak_torso_velocity + self.imu_weak_torso_acceleration * Robot.STEPTIME



    def set_joints_target_position_direct(self,indices,values:np.ndarray,harmonize=True,max_speed=7.03,tolerance=0.012,limit_joints=True) -> int:
        '''
        Computes the speed of a list of joints, taking as argument the target position

        Parameters
        ----------
        indices : `int`/`list`/`slice`/numpy array
            joint indices
        values : numpy array 
            target position for each listed joint index
        harmonize : `bool`
            if True, all joints reach target at same time
        max_speed : `float`
            max. speed for all joints in deg/step
            Most joints have a maximum speed of 351.77 deg/s according to rcssserver3d/data/rsg/agent/nao/hingejoint.rsg
            That translates as 7.0354 deg/step or 6.1395 rad/s
        tolerance : `float`
            angle error tolerance (in degrees) to return that target was reached (returns -1)
        limit_joints : `bool`
            limit values to the joints' range of motion

        Returns
        -------
        remaining_steps : `int`
            predicted number of remaining steps or -1 if target was already reached

        Examples
        -------
        (let p[tx] be the joint position at t=x)

        Example for return value: moving joint[0] from 0deg to 10deg
                pos[t0]: 0,  speed[t0]: 7deg/step,  ret=2   # target will predictedly be reached in 2 steps
                pos[t1]: 7,  speed[t1]: 3deg/step,  ret=1   # target will predictedly be reached in 1 step (send final action)
                pos[t2]: 10, speed[t2]: 0deg/step,  ret=0   # target was predictedly already reached 
                pos[t3]: 10, speed[t3]: 0deg/step,  ret=-1  # (best case scenario) server reported with delay, that target was reached (see tolerance)
                pos[t?]: 10, speed[t?]: 0deg/step,  ret=-1  # if there is friction, it may take some additional steps 

                If everything worked as predicted we could stop calling this function when ret==1
                If we need precision, it is recommended to wait for ret==-1

        Example 1:
            set_joints_target_position_direct(range(2,4),np.array([10.0,5.0]),harmonize=True)    
                Joint[2]   p[t0]: 0  target pos: 10  ->  p[t1]=5,   p[t2]=10
                Joint[3]   p[t0]: 0  target pos: 5   ->  p[t1]=2.5, p[t2]=5

        Example 2:
            set_joints_target_position_direct([2,3],np.array([10.0,5.0]),harmonize=False)  
                Joint[2]   p[t0]: 0  target pos: 10  ->  p[t1]=7,   p[t2]=10
                Joint[3]   p[t0]: 0  target pos: 5   ->  p[t1]=5,   p[t2]=5  
        '''

        assert type(values) == np.ndarray, "'values' argument must be a numpy array"
        np.nan_to_num(values, copy=False) # Replace NaN with zero and infinity with large finite numbers

        # limit range of joints
        if limit_joints:     
            if type(indices) == list or type(indices) == np.ndarray:
                for i in range(len(indices)):
                    values[i] = np.clip(values[i], self.joints_info[indices[i]].min, self.joints_info[indices[i]].max)
            elif type(indices) == slice:
                info = self.joints_info[indices]
                for i in range(len(info)):
                    values[i] = np.clip(values[i], info[i].min, info[i].max)
            else: # int
                values[0] = np.clip(values[0], self.joints_info[indices].min, self.joints_info[indices].max)

        #predicted_diff: predicted difference between reported position and actual position

        predicted_diff = self.joints_target_last_speed[indices] * 1.1459156 #rad/s to deg/step
        predicted_diff = np.asarray(predicted_diff)
        np.clip(predicted_diff,-7.03,7.03,out=predicted_diff) #saturate predicted movement in-place

        #reported_dist: difference between reported position and target position

        reported_dist = values - self.joints_position[indices]
        if np.all((np.abs(reported_dist) < tolerance)) and np.all((np.abs(predicted_diff) < tolerance)):
            self.joints_target_speed[indices] = 0
            return -1
       
        deg_per_step = reported_dist - predicted_diff

        relative_max = np.max( np.abs(deg_per_step) ) / max_speed
        remaining_steps = np.ceil( relative_max  )

        if remaining_steps == 0:
            self.joints_target_speed[indices] = 0
            return 0

        if harmonize:   
            deg_per_step /= remaining_steps
        else:
            np.clip(deg_per_step,-max_speed,max_speed,out=deg_per_step) #limit maximum speed

        self.joints_target_speed[indices] = deg_per_step * 0.87266463 #convert to rad/s

        return remaining_steps



    def get_command(self) -> bytes:
        '''
        Builds commands string from self.joints_target_speed
        '''
        j_speed = self.joints_target_speed * self.FIX_EFFECTOR_MASK #Fix symmetry issues 3/4 (effectors)
        cmd = "".join(f"({self.joints_info[i].effector} {j_speed[i]:.5f})" for i in range(self.no_of_joints)).encode('utf-8')

        self.joints_target_last_speed = self.joints_target_speed           #1. both point to the same array
        self.joints_target_speed = np.zeros_like(self.joints_target_speed) #2. create new array for joints_target_speed
        return cmd
