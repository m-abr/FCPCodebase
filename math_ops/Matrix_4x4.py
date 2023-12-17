from math import asin, atan2, pi, sqrt
from math_ops.Math_Ops import Math_Ops as M
from math_ops.Matrix_3x3 import Matrix_3x3
import numpy as np

class Matrix_4x4():

    def __init__(self, matrix = None) -> None:
        '''
        Constructor examples:
        a = Matrix_4x4( )                                           # create identity matrix
        b = Matrix_4x4( [[1,1,1,1],[2,2,2,2],[3,3,3,3],[4,4,4,4]] ) # manually initialize matrix
        c = Matrix_4x4( [1,1,1,1,2,2,2,2,3,3,3,3,4,4,4,4] )         # manually initialize matrix
        d = Matrix_4x4( b )                                         # copy constructor
        '''
        if matrix is None:
            self.m = np.identity(4)
        elif type(matrix) == Matrix_4x4: 
            self.m = np.copy(matrix.m)
        elif type(matrix) == Matrix_3x3: 
            self.m = np.identity(4)
            self.m[0:3,0:3] = matrix.m
        else:
            self.m = np.asarray(matrix)
            self.m.shape = (4,4) #reshape if needed, throw error if impossible


    @classmethod
    def from_translation(cls, translation_vec):
        '''
        Create transformation matrix from translation_vec translation
        e.g. Matrix_4x4.from_translation((a,b,c))
            output: [[1,0,0,a],[0,1,0,b],[0,0,1,c],[0,0,0,1]]
        '''
        mat = np.identity(4)
        mat[0:3,3] = translation_vec
        return cls(mat)

    @classmethod
    def from_3x3_and_translation(cls, mat3x3:Matrix_3x3, translation_vec):
        '''
        Create transformation matrix from rotation matrix (3x3) and translation
        e.g. Matrix_4x4.from_3x3_and_translation(r,(a,b,c))    
            output: [[r00,r01,r02,a],[r10,r11,r12,b],[r20,r21,r22,c],[0,0,0,1]]
        '''
        mat = np.identity(4)
        mat[0:3,0:3] = mat3x3.m
        mat[0:3,3] = translation_vec
        return cls(mat)

    def translate(self, translation_vec, in_place=False):
        '''
        Translates the current transformation matrix

        Parameters
        ----------
        translation_vec : array_like, length 3
            translation vector
        in_place: bool, optional
            * True: the internal matrix is changed in-place
            * False: a new matrix is returned and the current one is not changed 

        Returns
        -------
        result : Matrix_4x4 
            self is returned if in_place is True
        '''
        vec = np.array([*translation_vec,1])# conversion to 4D vector
        np.matmul(self.m, vec, out=vec)     # compute only 4th column

        if in_place:
            self.m[:,3] = vec
            return self
        else:
            ret = Matrix_4x4(self.m)
            ret.m[:,3] = vec
            return ret


    def get_translation(self):
        ''' Get translation vector (x,y,z) '''
        return self.m[0:3,3] # return view

    def get_x(self):
        return self.m[0,3]

    def get_y(self):
        return self.m[1,3]

    def get_z(self):
        return self.m[2,3]

    def get_rotation_4x4(self):
        ''' Get Matrix_4x4 without translation ''' 
        mat = Matrix_4x4(self)
        mat.m[0:3,3] = 0
        return mat

    def get_rotation(self):
        ''' Get rotation Matrix_3x3 '''
        return Matrix_3x3(self.m[0:3,0:3])

    def get_distance(self):
        ''' Get translation vector length '''
        return np.linalg.norm(self.m[0:3,3])

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
        return 90 - (asin(np.clip(self.m[2,2],-1,1)) * 180 / pi)

    def rotate_deg(self, rotation_vec, rotation_deg, in_place=False):
        '''
        Rotates the current transformation matrix

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
        result : Matrix_4x4 
            self is returned if in_place is True
        '''
        return self.rotate_rad(rotation_vec, rotation_deg * (pi/180) , in_place)

        
    def rotate_rad(self, rotation_vec, rotation_rad, in_place=False):
        '''
        Rotates the current transformation matrix

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
        result : Matrix_4x4 
            self is returned if in_place is True
        '''

        if rotation_rad == 0: 
            return self if in_place else Matrix_4x4(self)

        # shortcuts for rotation around 1 axis
        if rotation_vec[0]==0:
            if rotation_vec[1]==0:
                if rotation_vec[2]==1:
                    return self.rotate_z_rad(rotation_rad, in_place)
                elif rotation_vec[2]==-1:
                    return self.rotate_z_rad(-rotation_rad, in_place)
            elif rotation_vec[2]==0:
                if rotation_vec[1]==1:
                    return self.rotate_y_rad(rotation_rad, in_place)
                elif rotation_vec[1]==-1:
                    return self.rotate_y_rad(-rotation_rad, in_place)
        elif rotation_vec[1]==0 and rotation_vec[2]==0:
            if rotation_vec[0]==1:
                return self.rotate_x_rad(rotation_rad, in_place)
            elif rotation_vec[0]==-1:
                return self.rotate_x_rad(-rotation_rad, in_place)
            
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
        [xxc1 +  c,  xyc1 - zs,  xzc1 + ys, 0],
        [xyc1 + zs,  yyc1 +  c,  yzc1 - xs, 0],
        [xzc1 - ys,  yzc1 + xs,  zzc1 +  c, 0],
        [0, 0, 0, 1]])

        return self.multiply(mat, in_place)


    def rotate_x_rad(self, rotation_rad, in_place=False):
        '''
        Rotates the current transformation matrix around the x-axis

        Parameters
        ----------
        rotation_rad : float
            rotation in radians
        in_place: bool, optional
            * True: the internal matrix is changed in-place (default)
            * False: a new matrix is returned and the current one is not changed 
        
        Returns
        -------
        result : Matrix_4x4 
            self is returned if in_place is True
        '''
        if rotation_rad == 0: 
            return self if in_place else Matrix_4x4(self)
 
        c = np.math.cos(rotation_rad)
        s = np.math.sin(rotation_rad)

        mat = np.array([
        [1, 0, 0, 0],
        [0, c,-s, 0],
        [0, s, c, 0],
        [0, 0, 0, 1]])

        return self.multiply(mat, in_place)

    def rotate_y_rad(self, rotation_rad, in_place=False):
        '''
        Rotates the current transformation matrix around the y-axis

        Parameters
        ----------
        rotation_rad : float
            rotation in radians
        in_place: bool, optional
            * True: the internal matrix is changed in-place (default)
            * False: a new matrix is returned and the current one is not changed 
        
        Returns
        -------
        result : Matrix_4x4 
            self is returned if in_place is True
        '''
        if rotation_rad == 0: 
            return self if in_place else Matrix_4x4(self)
 
        c = np.math.cos(rotation_rad)
        s = np.math.sin(rotation_rad)

        mat = np.array([
        [ c, 0, s, 0],
        [ 0, 1, 0, 0],
        [-s, 0, c, 0],
        [ 0, 0, 0, 1]])

        return self.multiply(mat, in_place)

    def rotate_z_rad(self, rotation_rad, in_place=False):
        '''
        Rotates the current transformation matrix around the z-axis

        Parameters
        ----------
        rotation_rad : float
            rotation in radians
        in_place: bool, optional
            * True: the internal matrix is changed in-place (default)
            * False: a new matrix is returned and the current one is not changed 
        
        Returns
        -------
        result : Matrix_4x4 
            self is returned if in_place is True
        '''
        if rotation_rad == 0: 
            return self if in_place else Matrix_4x4(self)
 
        c = np.math.cos(rotation_rad)
        s = np.math.sin(rotation_rad)

        mat = np.array([
        [ c,-s, 0, 0],
        [ s, c, 0, 0],
        [ 0, 0, 1, 0],
        [ 0, 0, 0, 1]])

        return self.multiply(mat, in_place)

    def rotate_x_deg(self, rotation_deg, in_place=False):
        '''
        Rotates the current transformation matrix around the x-axis

        Parameters
        ----------
        rotation_rad : float
            rotation in degrees
        in_place: bool, optional
            * True: the internal matrix is changed in-place (default)
            * False: a new matrix is returned and the current one is not changed 
        
        Returns
        -------
        result : Matrix_4x4 
            self is returned if in_place is True
        '''
        return self.rotate_x_rad(rotation_deg * (pi/180), in_place)

    def rotate_y_deg(self, rotation_deg, in_place=False):
        '''
        Rotates the current transformation matrix around the y-axis

        Parameters
        ----------
        rotation_rad : float
            rotation in degrees
        in_place: bool, optional
            * True: the internal matrix is changed in-place (default)
            * False: a new matrix is returned and the current one is not changed 
        
        Returns
        -------
        result : Matrix_4x4 
            self is returned if in_place is True
        '''
        return self.rotate_y_rad(rotation_deg * (pi/180), in_place)

    def rotate_z_deg(self, rotation_deg, in_place=False):
        '''
        Rotates the current transformation matrix around the z-axis

        Parameters
        ----------
        rotation_rad : float
            rotation in degrees
        in_place: bool, optional
            * True: the internal matrix is changed in-place (default)
            * False: a new matrix is returned and the current one is not changed 
        
        Returns
        -------
        result : Matrix_4x4 
            self is returned if in_place is True
        '''
        return self.rotate_z_rad(rotation_deg * (pi/180), in_place)

    def invert(self, in_place=False):
        '''
        Inverts the current transformation matrix

        Parameters
        ----------
        in_place: bool, optional
            * True: the internal matrix is changed in-place (default)
            * False: a new matrix is returned and the current one is not changed 
        
        Returns
        -------
        result : Matrix_4x4 
            self is returned if in_place is True
        '''

        if in_place:
            self.m = np.linalg.inv(self.m)
            return self
        else:
            return Matrix_4x4(np.linalg.inv(self.m))

    def multiply(self,mat, in_place=False):
        '''
        Multiplies the current transformation matrix by mat

        Parameters
        ----------
        mat : Matrix_4x4 or array_like
            multiplier matrix or 3D vector
        in_place: bool, optional
            * True: the internal matrix is changed in-place (default)
            * False: a new matrix is returned and the current one is not changed (if mat is a 4x4 matrix)
        
        Returns
        -------
        result : Matrix_4x4 | array_like
            Matrix_4x4 is returned if mat is a matrix (self is returned if in_place is True); 
            a 3D vector is returned if mat is a vector
        '''
        if type(mat) == Matrix_4x4:  
            mat = mat.m
        else:
            mat = np.asarray(mat)                   # conversion to array, if needed
            if mat.ndim == 1:                       # multiplication by 3D vector
                vec = np.append(mat,1)              # conversion to 4D vector
                return np.matmul(self.m, vec)[0:3]  # conversion to 3D vector

        if in_place:
            np.matmul(self.m, mat, self.m)
            return self
        else:
            return Matrix_4x4(np.matmul(self.m, mat))

    def __call__(self,mat, is_spherical=False):
        '''
        Multiplies the current transformation matrix by mat and returns a new matrix or vector

        Parameters
        ----------
        mat : Matrix_4x4 or array_like
            multiplier matrix or 3D vector
        is_spherical : bool
            only relevant if mat is a 3D vector, True if it uses spherical coordinates
        
        Returns
        -------
        result : Matrix_4x4 | array_like
            Matrix_4x4 is returned if mat is a matrix; 
            a 3D vector is returned if mat is a vector
        '''

        if is_spherical and mat.ndim == 1: mat = M.deg_sph2cart(mat)
        return self.multiply(mat,False)

    