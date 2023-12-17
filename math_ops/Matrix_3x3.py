from math import asin, atan2, pi, sqrt
import numpy as np

class Matrix_3x3():

    def __init__(self, matrix = None) -> None:
        '''
        Constructor examples:
        a = Matrix_3x3( )                           # create identity matrix
        b = Matrix_3x3( [[1,1,1],[2,2,2],[3,3,3]] ) # manually initialize matrix
        c = Matrix_3x3( [1,1,1,2,2,2,3,3,3] )       # manually initialize matrix
        d = Matrix_3x3( b )                         # copy constructor
        '''
        if matrix is None:
            self.m = np.identity(3)
        elif type(matrix) == Matrix_3x3: 
            self.m = np.copy(matrix.m)
        else:
            self.m = np.asarray(matrix)
            self.m.shape = (3,3) #reshape if needed, throw error if impossible


        self.rotation_shortcuts={(1,0,0):self.rotate_x_rad, (-1, 0, 0):self._rotate_x_neg_rad,
                                 (0,1,0):self.rotate_y_rad, ( 0,-1, 0):self._rotate_y_neg_rad,
                                 (0,0,1):self.rotate_z_rad, ( 0, 0,-1):self._rotate_z_neg_rad}

    @classmethod
    def from_rotation_deg(cls, euler_vec):
        '''
        Create rotation matrix from Euler angles, in degrees.
        Rotation order: RotZ*RotY*RotX

        Parameters
        ----------
        euler_vec : array_like, length 3
            vector with Euler angles (x,y,z) aka (roll, pitch, yaw)

        Example
        ----------
        Matrix_3x3.from_rotation_deg((roll,pitch,yaw))    # Creates: RotZ(yaw)*RotY(pitch)*RotX(roll)
        '''
        mat = cls().rotate_z_deg(euler_vec[2], True).rotate_y_deg(euler_vec[1], True).rotate_x_deg(euler_vec[0], True)
        return mat

    def get_roll_deg(self):
        ''' Get angle around the x-axis in degrees, Rotation order: RotZ*RotY*RotX=Rot '''
        if self.m[2,1] == 0 and self.m[2,2] == 0: 
            return 180
        return atan2(self.m[2,1], self.m[2,2]) * 180 / pi

    def get_pitch_deg(self):
        ''' Get angle around the y-axis in degrees, Rotation order: RotZ*RotY*RotX=Rot '''
        return atan2(-self.m[2,0], sqrt(self.m[2,1]*self.m[2,1] + self.m[2,2]*self.m[2,2])) * 180 / pi

    def get_yaw_deg(self):
        ''' Get angle around the z-axis in degrees, Rotation order: RotZ*RotY*RotX=Rot '''
        if self.m[1,0] == 0 and self.m[0,0] == 0: 
            return atan2(self.m[0,1], self.m[1,1]) * 180 / pi
        return atan2(self.m[1,0], self.m[0,0]) * 180 / pi

    def get_inclination_deg(self):
        ''' Get inclination of z-axis in relation to reference z-axis '''
        return 90 - (asin(self.m[2,2]) * 180 / pi)


    def rotate_deg(self, rotation_vec, rotation_deg, in_place=False):
        '''
        Rotates the current rotation matrix

        Parameters
        ----------
        rotation_vec : array_like, length 3
            rotation vector
        rotation_rad : float
            rotation in degrees
        in_place: bool, optional
            * True: the internal matrix is changed in-place (default)
            * False: a new matrix is returned and the current one is not changed 
        
        Returns
        -------
        result : Matrix_3x3 
            self is returned if in_place is True
        '''
        return self.rotate_rad(rotation_vec, rotation_deg * (pi/180) , in_place)

        
    def rotate_rad(self, rotation_vec, rotation_rad, in_place=False):
        '''
        Rotates the current rotation matrix

        Parameters
        ----------
        rotation_vec : array_like, length 3
            rotation vector
        rotation_rad : float
            rotation in radians
        in_place: bool, optional
            * True: the internal matrix is changed in-place (default)
            * False: a new matrix is returned and the current one is not changed 
        
        Returns
        -------
        result : Matrix_3x3 
            self is returned if in_place is True
        '''

        if rotation_rad == 0: return

        shortcut = self.rotation_shortcuts.get(tuple(a for a in rotation_vec))
        if shortcut:
            return shortcut(rotation_rad, in_place)
            
        c = np.math.cos(rotation_rad)
        c1 = 1 - c
        s = np.math.sin(rotation_rad)
        x = rotation_vec[0]
        y = rotation_vec[1]
        z = rotation_vec[2]
        xxc1 = x * x * c1
        yyc1 = y * y * c1
        zzc1 = z * z * c1
        xyc1 = x * y * c1
        xzc1 = x * z * c1
        yzc1 = y * z * c1
        xs = x * s
        ys = y * s
        zs = z * s

        mat = np.array([
        [xxc1 +  c,  xyc1 - zs,  xzc1 + ys],
        [xyc1 + zs,  yyc1 +  c,  yzc1 - xs],
        [xzc1 - ys,  yzc1 + xs,  zzc1 +  c]])

        return self.multiply(mat, in_place)


    def _rotate_x_neg_rad(self, rotation_rad, in_place=False):
        self.rotate_x_rad(-rotation_rad, in_place)

    def _rotate_y_neg_rad(self, rotation_rad, in_place=False):
        self.rotate_y_rad(-rotation_rad, in_place)

    def _rotate_z_neg_rad(self, rotation_rad, in_place=False):
        self.rotate_z_rad(-rotation_rad, in_place)

    def rotate_x_rad(self, rotation_rad, in_place=False):
        '''
        Rotates the current rotation matrix around the x-axis

        Parameters
        ----------
        rotation_rad : float
            rotation in radians
        in_place: bool, optional
            * True: the internal matrix is changed in-place (default)
            * False: a new matrix is returned and the current one is not changed 
        
        Returns
        -------
        result : Matrix_3x3 
            self is returned if in_place is True
        '''
        if rotation_rad == 0: 
            return self if in_place else Matrix_3x3(self)
 
        c = np.math.cos(rotation_rad)
        s = np.math.sin(rotation_rad)

        mat = np.array([
        [1, 0, 0],
        [0, c,-s],
        [0, s, c]])

        return self.multiply(mat, in_place)

    def rotate_y_rad(self, rotation_rad, in_place=False):
        '''
        Rotates the current rotation matrix around the y-axis

        Parameters
        ----------
        rotation_rad : float
            rotation in radians
        in_place: bool, optional
            * True: the internal matrix is changed in-place (default)
            * False: a new matrix is returned and the current one is not changed 
        
        Returns
        -------
        result : Matrix_3x3 
            self is returned if in_place is True
        '''
        if rotation_rad == 0: 
            return self if in_place else Matrix_3x3(self)
 
        c = np.math.cos(rotation_rad)
        s = np.math.sin(rotation_rad)

        mat = np.array([
        [ c, 0, s],
        [ 0, 1, 0],
        [-s, 0, c]])

        return self.multiply(mat, in_place)

    def rotate_z_rad(self, rotation_rad, in_place=False):
        '''
        Rotates the current rotation matrix around the z-axis

        Parameters
        ----------
        rotation_rad : float
            rotation in radians
        in_place: bool, optional
            * True: the internal matrix is changed in-place (default)
            * False: a new matrix is returned and the current one is not changed 
        
        Returns
        -------
        result : Matrix_3x3 
            self is returned if in_place is True
        '''
        if rotation_rad == 0: 
            return self if in_place else Matrix_3x3(self)
 
        c = np.math.cos(rotation_rad)
        s = np.math.sin(rotation_rad)

        mat = np.array([
        [ c,-s, 0],
        [ s, c, 0],
        [ 0, 0, 1]])

        return self.multiply(mat, in_place)

    def rotate_x_deg(self, rotation_deg, in_place=False):
        '''
        Rotates the current rotation matrix around the x-axis

        Parameters
        ----------
        rotation_rad : float
            rotation in degrees
        in_place: bool, optional
            * True: the internal matrix is changed in-place (default)
            * False: a new matrix is returned and the current one is not changed 
        
        Returns
        -------
        result : Matrix_3x3 
            self is returned if in_place is True
        '''
        return self.rotate_x_rad(rotation_deg * (pi/180), in_place)

    def rotate_y_deg(self, rotation_deg, in_place=False):
        '''
        Rotates the current rotation matrix around the y-axis

        Parameters
        ----------
        rotation_rad : float
            rotation in degrees
        in_place: bool, optional
            * True: the internal matrix is changed in-place (default)
            * False: a new matrix is returned and the current one is not changed 
        
        Returns
        -------
        result : Matrix_3x3 
            self is returned if in_place is True
        '''
        return self.rotate_y_rad(rotation_deg * (pi/180), in_place)

    def rotate_z_deg(self, rotation_deg, in_place=False):
        '''
        Rotates the current rotation matrix around the z-axis

        Parameters
        ----------
        rotation_rad : float
            rotation in degrees
        in_place: bool, optional
            * True: the internal matrix is changed in-place (default)
            * False: a new matrix is returned and the current one is not changed 
        
        Returns
        -------
        result : Matrix_3x3 
            self is returned if in_place is True
        '''
        return self.rotate_z_rad(rotation_deg * (pi/180), in_place)

    def invert(self, in_place=False):
        '''
        Inverts the current rotation matrix

        Parameters
        ----------
        in_place: bool, optional
            * True: the internal matrix is changed in-place (default)
            * False: a new matrix is returned and the current one is not changed 
        
        Returns
        -------
        result : Matrix_3x3 
            self is returned if in_place is True
        '''

        if in_place:
            self.m = np.linalg.inv(self.m)
            return self
        else:
            return Matrix_3x3(np.linalg.inv(self.m))

    def multiply(self,mat, in_place=False, reverse_order=False):
        '''
        Multiplies the current rotation matrix by mat

        Parameters
        ----------
        mat : Matrix_3x3 or array_like
            multiplier matrix or 3D vector
        in_place: bool, optional
            - True: the internal matrix is changed in-place
            - False: a new matrix is returned and the current one is not changed (default) 
        reverse_order: bool, optional
            - False: self * mat
            - True:  mat * self
        
        Returns
        -------
        result : Matrix_3x3 | array_like
            Matrix_3x3 is returned if mat is a matrix (self is returned if in_place is True); 
            a 3D vector is returned if mat is a vector
        '''
        # get array from matrix object or convert to numpy array (if needed) 
        mat = mat.m if type(mat) == Matrix_3x3 else np.asarray(mat)

        a,b = (mat, self.m) if reverse_order else (self.m, mat)

        if mat.ndim == 1: 
            return np.matmul(a, b)  # multiplication by 3D vector
        elif in_place:
            np.matmul(a, b, self.m) # multiplication by matrix, in place
            return self
        else:                       # multiplication by matrix, return new Matrix_3x3
            return Matrix_3x3(np.matmul(a, b))
    

