import math


class Step_Generator():
    GRAVITY = 9.81
    Z0 = 0.2
    
    def __init__(self, feet_y_dev, sample_time, max_ankle_z) -> None:
        self.feet_y_dev = feet_y_dev
        self.sample_time = sample_time
        self.state_is_left_active = False
        self.state_current_ts = 0
        self.switch = False # switch legs
        self.external_progress = 0 # non-overlaped progress
        self.max_ankle_z = max_ankle_z


    def get_target_positions(self, reset, ts_per_step, z_span, z_extension):
        '''
        Get target positions for each foot

        Returns
        -------
        target : `tuple`
            (Left leg y, Left leg z, Right leg y, Right leg z)
        '''

        assert type(ts_per_step)==int and ts_per_step > 0, "ts_per_step must be a positive integer!"

        #-------------------------- Advance 1ts
        if reset:
            self.ts_per_step = ts_per_step        # step duration in time steps
            self.swing_height = z_span
            self.max_leg_extension = z_extension  # maximum distance between ankle to center of both hip joints
            self.state_current_ts = 0
            self.state_is_left_active = False 
            self.switch = False
        elif self.switch:
            self.state_current_ts = 0
            self.state_is_left_active = not self.state_is_left_active # switch leg
            self.switch = False
        else:
            self.state_current_ts += 1

        #-------------------------- Compute COM.y
        W = math.sqrt(self.Z0/self.GRAVITY)

        step_time = self.ts_per_step * self.sample_time
        time_delta = self.state_current_ts * self.sample_time
 
        y0 = self.feet_y_dev # absolute initial y value
        y_swing = y0 + y0 * (  math.sinh((step_time - time_delta)/W) + math.sinh(time_delta/W)  ) / math.sinh(-step_time/W)

        #-------------------------- Cap maximum extension and swing height
        z0 = min(-self.max_leg_extension, self.max_ankle_z) #  capped initial z value
        zh = min(self.swing_height, self.max_ankle_z - z0) # capped swing height

        #-------------------------- Compute Z Swing
        progress = self.state_current_ts / self.ts_per_step
        self.external_progress = self.state_current_ts / (self.ts_per_step-1)
        active_z_swing = zh * math.sin(math.pi * progress)

        #-------------------------- Accept new parameters after final step
        if self.state_current_ts + 1 >= self.ts_per_step:
            self.ts_per_step = ts_per_step        # step duration in time steps
            self.swing_height = z_span
            self.max_leg_extension = z_extension  # maximum distance between ankle to center of both hip joints
            self.switch = True

        #-------------------------- Distinguish active leg
        if self.state_is_left_active:
            return y0+y_swing, active_z_swing+z0, -y0+y_swing, z0
        else:
            return y0-y_swing, z0, -y0-y_swing, active_z_swing+z0

