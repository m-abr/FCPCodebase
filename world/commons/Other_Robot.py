import numpy as np

#Note: When other robot is seen, all previous body part positions are deleted
# E.g. we see 5 body parts at 0 seconds -> body_parts_cart_rel_pos contains 5 elements
#      we see 1 body part  at 1 seconds -> body_parts_cart_rel_pos contains 1 element


class Other_Robot():
    def __init__(self, unum, is_teammate) -> None:
        self.unum = unum                # convenient variable to indicate uniform number (same as other robot's index + 1)
        self.is_self = False            # convenient flag to indicate if this robot is self
        self.is_teammate = is_teammate  # convenient variable to indicate if this robot is from our team
        self.is_visible = False # True if this robot was seen in the last message from the server (it doesn't mean we know its absolute location)
        self.body_parts_cart_rel_pos = dict()  # cartesian relative position of the robot's visible body parts
        self.body_parts_sph_rel_pos = dict()   # spherical relative position of the robot's visible body parts
        self.vel_filter = 0.3                  # EMA filter coefficient applied to self.state_filtered_velocity
        self.vel_decay  = 0.95                 # velocity decay at every vision cycle (neutralized if velocity is updated)


        # State variables: these are computed when this robot is visible and when the original robot is able to self-locate
        self.state_fallen = False              # true if the robot is lying down  (updated when head is visible)
        self.state_last_update = 0             # World.time_local_ms when the state was last updated
        self.state_horizontal_dist = 0         # horizontal head distance if head is visible, otherwise, average horizontal distance of visible body parts (the distance is updated by vision or radio when state_abs_pos gets a new value, but also when the other player is not visible, by assuming its last position)
        self.state_abs_pos = None              # 3D head position if head is visible, otherwise, 2D average position of visible body parts, or, 2D radio head position
        self.state_orientation = 0             # orientation based on pair of lower arms or feet, or average of both (WARNING: may be older than state_last_update)  
        self.state_ground_area = None          # (pt_2d,radius) projection of player area on ground (circle), not precise if farther than 3m (for performance), useful for obstacle avoidance when it falls
        self.state_body_parts_abs_pos = dict() # 3D absolute position of each body part
        self.state_filtered_velocity = np.zeros(3) # 3D filtered velocity (m/s) (if the head is not visible, the 2D part is updated and v.z decays)
