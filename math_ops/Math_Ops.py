from math import acos, asin, atan2, cos, pi, sin, sqrt
import numpy as np
import sys

try:
    GLOBAL_DIR = sys._MEIPASS # temporary folder with libs & data files
except:
    GLOBAL_DIR = "."


class Math_Ops():
    '''
    This class provides general mathematical operations that are not directly available through numpy 
    '''
  
    @staticmethod
    def deg_sph2cart(spherical_vec):
        ''' Converts SimSpark's spherical coordinates in degrees to cartesian coordinates '''
        r = spherical_vec[0]
        h = spherical_vec[1] * pi / 180
        v = spherical_vec[2] * pi / 180
        return np.array([r * cos(v) * cos(h), r * cos(v) * sin(h), r * sin(v)])

    @staticmethod
    def deg_sin(deg_angle):
        ''' Returns sin of degrees '''
        return sin(deg_angle * pi / 180)

    @staticmethod
    def deg_cos(deg_angle):
        ''' Returns cos of degrees '''
        return cos(deg_angle * pi / 180)

    @staticmethod
    def to_3d(vec_2d, value=0) -> np.ndarray:
        ''' Returns new 3d vector from 2d vector '''
        return np.append(vec_2d,value)

    @staticmethod
    def to_2d_as_3d(vec_3d) -> np.ndarray:
        ''' Returns new 3d vector where the 3rd dimension is zero '''
        vec_2d_as_3d = np.copy(vec_3d)
        vec_2d_as_3d[2] = 0
        return vec_2d_as_3d

    @staticmethod
    def normalize_vec(vec) -> np.ndarray:
        ''' Divides vector by its length '''
        size = np.linalg.norm(vec)
        if size == 0: return vec
        return vec / size

    @staticmethod
    def get_active_directory(dir:str) -> str:
        global GLOBAL_DIR
        return GLOBAL_DIR + dir

    @staticmethod
    def acos(val):
        ''' arccosine function that limits input '''
        return acos( np.clip(val,-1,1) )
    
    @staticmethod
    def asin(val):
        ''' arcsine function that limits input '''
        return asin( np.clip(val,-1,1) )

    @staticmethod
    def normalize_deg(val):
        ''' normalize val in range [-180,180[ '''
        return (val + 180.0) % 360 - 180

    @staticmethod
    def normalize_rad(val):
        ''' normalize val in range [-pi,pi[ '''
        return (val + pi) % (2*pi) - pi

    @staticmethod
    def deg_to_rad(val):
        ''' convert degrees to radians '''
        return val * 0.01745329251994330

    @staticmethod
    def rad_to_deg(val):
        ''' convert radians to degrees '''
        return val * 57.29577951308232

    @staticmethod
    def vector_angle(vector, is_rad=False):
        ''' angle (degrees or radians) of 2D vector '''
        if is_rad:
            return atan2(vector[1], vector[0])
        else:
            return atan2(vector[1], vector[0]) * 180 / pi

    @staticmethod
    def vectors_angle(vec1, vec2, is_rad=False):
        ''' get angle between vectors (degrees or radians) '''
        ang_rad = acos(np.dot(Math_Ops.normalize_vec(vec1),Math_Ops.normalize_vec(vec2)))
        return ang_rad if is_rad else ang_rad * 180 / pi

    @staticmethod
    def vector_from_angle(angle, is_rad=False):
        ''' unit vector with direction given by `angle` '''
        if is_rad:
            return np.array([cos(angle), sin(angle)], float)
        else:
            return np.array([Math_Ops.deg_cos(angle), Math_Ops.deg_sin(angle)], float)

    @staticmethod
    def target_abs_angle(pos2d, target, is_rad=False):
        ''' angle (degrees or radians) of vector (target-pos2d) '''
        if is_rad:
            return atan2(target[1]-pos2d[1], target[0]-pos2d[0])
        else:
            return atan2(target[1]-pos2d[1], target[0]-pos2d[0]) * 180 / pi

    @staticmethod
    def target_rel_angle(pos2d, ori, target, is_rad=False):
        ''' relative angle (degrees or radians) of target if we're located at 'pos2d' with orientation 'ori' (degrees or radians) '''
        if is_rad:
            return Math_Ops.normalize_rad( atan2(target[1]-pos2d[1], target[0]-pos2d[0]) - ori )
        else:
            return Math_Ops.normalize_deg( atan2(target[1]-pos2d[1], target[0]-pos2d[0]) * 180 / pi - ori )

    @staticmethod
    def rotate_2d_vec(vec, angle, is_rad=False):
        ''' rotate 2D vector anticlockwise around the origin by `angle` '''
        cos_ang = cos(angle) if is_rad else cos(angle * pi / 180)
        sin_ang = sin(angle) if is_rad else sin(angle * pi / 180)
        return np.array([cos_ang*vec[0]-sin_ang*vec[1], sin_ang*vec[0]+cos_ang*vec[1]])

    @staticmethod
    def distance_point_to_line(p:np.ndarray, a:np.ndarray, b:np.ndarray):
        ''' 
        Distance between point p and 2d line 'ab' (and side where p is)

        Parameters
        ----------
        a : ndarray
            2D point that defines line
        b : ndarray
            2D point that defines line
        p : ndarray
            2D point

        Returns
        -------
        distance : float
            distance between line and point
        side : str
            if we are at a, looking at b, p may be at our "left" or "right"
        '''
        line_len = np.linalg.norm(b-a)

        if line_len == 0: # assumes vertical line
            dist = sdist = np.linalg.norm(p-a)
        else:
            sdist = np.cross(b-a,p-a)/line_len
            dist = abs(sdist)

        return dist, "left" if sdist>0 else "right"

    @staticmethod
    def distance_point_to_segment(p:np.ndarray, a:np.ndarray, b:np.ndarray):
        ''' Distance from point p to 2d line segment 'ab' '''
        
        ap = p-a
        ab = b-a

        ad = Math_Ops.vector_projection(ap,ab)

        # Is d in ab? We can find k in (ad = k * ab) without computing any norm
        # we use the largest dimension of ab to avoid division by 0
        k = ad[0]/ab[0] if abs(ab[0])>abs(ab[1]) else ad[1]/ab[1]

        if   k <= 0: return np.linalg.norm(ap)
        elif k >= 1: return np.linalg.norm(p-b)
        else:        return np.linalg.norm(p-(ad + a)) # p-d

    @staticmethod
    def distance_point_to_ray(p:np.ndarray, ray_start:np.ndarray, ray_direction:np.ndarray):
        ''' Distance from point p to 2d ray '''
        
        rp = p-ray_start
        rd = Math_Ops.vector_projection(rp,ray_direction)

        # Is d in ray? We can find k in (rd = k * ray_direction) without computing any norm
        # we use the largest dimension of ray_direction to avoid division by 0
        k = rd[0]/ray_direction[0] if abs(ray_direction[0])>abs(ray_direction[1]) else rd[1]/ray_direction[1]

        if   k <= 0: return np.linalg.norm(rp)
        else:        return np.linalg.norm(p-(rd + ray_start)) # p-d

    @staticmethod
    def closest_point_on_ray_to_point(p:np.ndarray, ray_start:np.ndarray, ray_direction:np.ndarray):
        ''' Point on ray closest to point p '''
        
        rp = p-ray_start
        rd = Math_Ops.vector_projection(rp,ray_direction)

        # Is d in ray? We can find k in (rd = k * ray_direction) without computing any norm
        # we use the largest dimension of ray_direction to avoid division by 0
        k = rd[0]/ray_direction[0] if abs(ray_direction[0])>abs(ray_direction[1]) else rd[1]/ray_direction[1]

        if   k <= 0: return ray_start
        else:        return rd + ray_start

    @staticmethod
    def does_circle_intersect_segment(p:np.ndarray, r, a:np.ndarray, b:np.ndarray):
        ''' Returns true if circle (center p, radius r) intersect 2d line segment '''

        ap = p-a
        ab = b-a

        ad = Math_Ops.vector_projection(ap,ab)

        # Is d in ab? We can find k in (ad = k * ab) without computing any norm
        # we use the largest dimension of ab to avoid division by 0
        k = ad[0]/ab[0] if abs(ab[0])>abs(ab[1]) else ad[1]/ab[1]

        if   k <= 0: return np.dot(ap,ap)   <= r*r
        elif k >= 1: return np.dot(p-b,p-b) <= r*r
        
        dp = p-(ad + a)
        return np.dot(dp,dp) <= r*r

    @staticmethod
    def vector_projection(a:np.ndarray, b:np.ndarray):
        ''' Vector projection of a onto b '''
        b_dot = np.dot(b,b)
        return b * np.dot(a,b) / b_dot if b_dot != 0 else b

    @staticmethod
    def do_noncollinear_segments_intersect(a,b,c,d):
        ''' 
        Check if 2d line segment 'ab' intersects with noncollinear 2d line segment 'cd' 
        Explanation: https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/ 
        '''

        ccw = lambda a,b,c: (c[1]-a[1]) * (b[0]-a[0]) > (b[1]-a[1]) * (c[0]-a[0])
        return ccw(a,c,d) != ccw(b,c,d) and ccw(a,b,c) != ccw(a,b,d)

    @staticmethod
    def intersection_segment_opp_goal(a:np.ndarray, b:np.ndarray):
        ''' Computes the intersection point of 2d segment 'ab' and the opponents' goal (front line) '''
        vec_x = b[0]-a[0]

        # Collinear intersections are not accepted
        if vec_x == 0: return None
        
        k = (15.01-a[0])/vec_x

        # No collision
        if k < 0 or k > 1: return None

        intersection_pt = a + (b-a) * k

        if -1.01 <= intersection_pt[1] <= 1.01:
            return intersection_pt
        else:
            return None

    @staticmethod
    def intersection_circle_opp_goal(p:np.ndarray, r):
        ''' 
        Computes the intersection segment of circle (center p, radius r) and the opponents' goal (front line)
        Only the y coordinates are returned since the x coordinates are always equal to 15
        '''

        x_dev = abs(15-p[0])

        if x_dev > r:
            return None # no intersection with x=15

        y_dev = sqrt(r*r - x_dev*x_dev)

        p1 = max(p[1] - y_dev, -1.01)
        p2 = min(p[1] + y_dev,  1.01)

        if p1 == p2:
            return p1 # return the y coordinate of a single intersection point
        elif p2 < p1:
            return None # no intersection
        else:
            return p1, p2 # return the y coordinates of the intersection segment


    @staticmethod
    def distance_point_to_opp_goal(p:np.ndarray):
        ''' Distance between point 'p' and the opponents' goal (front line) '''

        if p[1] < -1.01:
            return np.linalg.norm( p-(15,-1.01) )
        elif p[1] > 1.01:
            return np.linalg.norm( p-(15, 1.01) )
        else:
            return abs(15-p[0])


    @staticmethod
    def circle_line_segment_intersection(circle_center, circle_radius, pt1, pt2, full_line=True, tangent_tol=1e-9):
        """ Find the points at which a circle intersects a line-segment.  This can happen at 0, 1, or 2 points.

        :param circle_center: The (x, y) location of the circle center
        :param circle_radius: The radius of the circle
        :param pt1: The (x, y) location of the first point of the segment
        :param pt2: The (x, y) location of the second point of the segment
        :param full_line: True to find intersections along full line - not just in the segment.  False will just return intersections within the segment.
        :param tangent_tol: Numerical tolerance at which we decide the intersections are close enough to consider it a tangent
        :return Sequence[Tuple[float, float]]: A list of length 0, 1, or 2, where each element is a point at which the circle intercepts a line segment.

        Note: We follow: http://mathworld.wolfram.com/Circle-LineIntersection.html
        """

        (p1x, p1y), (p2x, p2y), (cx, cy) = pt1, pt2, circle_center
        (x1, y1), (x2, y2) = (p1x - cx, p1y - cy), (p2x - cx, p2y - cy)
        dx, dy = (x2 - x1), (y2 - y1)
        dr = (dx ** 2 + dy ** 2)**.5
        big_d = x1 * y2 - x2 * y1
        discriminant = circle_radius ** 2 * dr ** 2 - big_d ** 2

        if discriminant < 0:  # No intersection between circle and line
            return []
        else:  # There may be 0, 1, or 2 intersections with the segment
            intersections = [
                (cx + (big_d * dy + sign * (-1 if dy < 0 else 1) * dx * discriminant**.5) / dr ** 2,
                cy + (-big_d * dx + sign * abs(dy) * discriminant**.5) / dr ** 2)
                for sign in ((1, -1) if dy < 0 else (-1, 1))]  # This makes sure the order along the segment is correct
            if not full_line:  # If only considering the segment, filter out intersections that do not fall within the segment
                fraction_along_segment = [
                    (xi - p1x) / dx if abs(dx) > abs(dy) else (yi - p1y) / dy for xi, yi in intersections]
                intersections = [pt for pt, frac in zip(
                    intersections, fraction_along_segment) if 0 <= frac <= 1]
            # If line is tangent to circle, return just one point (as both intersections have same location)
            if len(intersections) == 2 and abs(discriminant) <= tangent_tol:
                return [intersections[0]]
            else:
                return intersections




    # adapted from https://stackoverflow.com/questions/3252194/numpy-and-line-intersections
    @staticmethod
    def get_line_intersection(a1, a2, b1, b2):
        """ 
        Returns the point of intersection of the lines passing through a2,a1 and b2,b1.
        a1: [x, y] a point on the first line
        a2: [x, y] another point on the first line
        b1: [x, y] a point on the second line
        b2: [x, y] another point on the second line
        """
        s = np.vstack([a1,a2,b1,b2])        # s for stacked
        h = np.hstack((s, np.ones((4, 1)))) # h for homogeneous
        l1 = np.cross(h[0], h[1])           # get first line
        l2 = np.cross(h[2], h[3])           # get second line
        x, y, z = np.cross(l1, l2)          # point of intersection
        if z == 0:                          # lines are parallel
            return np.array([float('inf'), float('inf')])
        return np.array([x/z, y/z],float)
