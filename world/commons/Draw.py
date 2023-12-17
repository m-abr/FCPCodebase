import socket
from math_ops.Math_Ops import Math_Ops as M
import numpy as np

class Draw():
    _socket = None

    def __init__(self, is_enabled:bool, unum:int, host:str, port:int) -> None:
        self.enabled = is_enabled  
        self._is_team_right = None
        self._unum = unum   
        self._prefix = f'?{unum}_'.encode() # temporary prefix that should never be used in normal circumstances
     
        #Create one socket for all instances
        if Draw._socket is None:
            Draw._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM )
            Draw._socket.connect((host, port))
            Draw.clear_all()


    def set_team_side(self, is_right):
        ''' Called by world parser to switch side '''
        '''
        Generate an appropriate player ID
        RoboViz has a bug/feature: we send "swap buffers for player: 'l_1' and RoboViz
            will swap every buffer that contains 'l_1' in the name, including 
            'l_10' and 'l_11'. To avoid that, we swap the separator to 'l-10', 'l-11'
        '''
        self._is_team_right = is_right
        self._prefix = f"{'r' if is_right else 'l'}{'_' if self._unum < 10 else '-'}{self._unum}_".encode() #e.g. b'l_5', b'l-10'


    @staticmethod
    def _send(msg, id, flush):
        ''' Private method to send message if RoboViz is accessible '''
        try:
            if flush:
                Draw._socket.send(msg + id + b'\x00\x00\x00' + id + b'\x00')
            else:
                Draw._socket.send(msg + id + b'\x00')
        except ConnectionRefusedError:
            pass

        
    def circle(self, pos2d, radius, thickness, color:bytes, id:str, flush=True):
        ''' 
        Draw circle

        Examples
        ----------
        Circle in 2D (z=0): circle((-1,2), 3, 2, Draw.Color.red, "my_circle")
        '''
        if not self.enabled: return
        assert type(color)==bytes, "The RGB color must be a bytes object, e.g. red: b'\xFF\x00\x00'"
        assert not np.isnan(pos2d).any(), "Argument 'pos2d' contains 'nan' values"

        if self._is_team_right:
            pos2d = (-pos2d[0],-pos2d[1]) 

        msg = b'\x01\x00' + (
        f'{f"{pos2d[0]  :.4f}":.6s}'
        f'{f"{pos2d[1]  :.4f}":.6s}'
        f'{f"{radius    :.4f}":.6s}'
        f'{f"{thickness :.4f}":.6s}').encode() + color
        
        Draw._send(msg, self._prefix + id.encode(), flush)


    def line(self, p1, p2, thickness, color:bytes, id:str, flush=True):
        ''' 
        Draw line

        Examples
        ----------
        Line in 3D: line((0,0,0), (0,0,2), 3, Draw.Color.red, "my_line") 
        Line in 2D (z=0): line((0,0), (0,1),   3, Draw.Color.red, "my_line") 
        '''
        if not self.enabled: return
        assert type(color)==bytes, "The RGB color must be a bytes object, e.g. red: b'\xFF\x00\x00'"
        assert not np.isnan(p1).any(), "Argument 'p1' contains 'nan' values"
        assert not np.isnan(p2).any(), "Argument 'p2' contains 'nan' values"

        z1 = p1[2] if len(p1)==3 else 0
        z2 = p2[2] if len(p2)==3 else 0

        if self._is_team_right: 
            p1 = (-p1[0],-p1[1],p1[2]) if len(p1)==3 else (-p1[0],-p1[1])
            p2 = (-p2[0],-p2[1],p2[2]) if len(p2)==3 else (-p2[0],-p2[1])

        msg = b'\x01\x01' + (
        f'{f"{p1[0]  :.4f}":.6s}'
        f'{f"{p1[1]  :.4f}":.6s}'
        f'{f"{z1     :.4f}":.6s}'
        f'{f"{p2[0]  :.4f}":.6s}'
        f'{f"{p2[1]  :.4f}":.6s}'
        f'{f"{z2     :.4f}":.6s}'
        f'{f"{thickness :.4f}":.6s}').encode() + color

        Draw._send(msg, self._prefix + id.encode(), flush)
        

    def point(self, pos, size, color:bytes, id:str, flush=True):
        ''' 
        Draw point

        Examples
        ----------
        Point in 3D: point((1,1,1), 3, Draw.Color.red, "my_point")
        Point in 2D (z=0): point((1,1), 3, Draw.Color.red, "my_point")
        '''
        if not self.enabled: return
        assert type(color)==bytes, "The RGB color must be a bytes object, e.g. red: b'\xFF\x00\x00'"
        assert not np.isnan(pos).any(), "Argument 'pos' contains 'nan' values"

        z = pos[2] if len(pos)==3 else 0

        if self._is_team_right: 
            pos = (-pos[0],-pos[1],pos[2]) if len(pos)==3 else (-pos[0],-pos[1])

        msg = b'\x01\x02' + (
        f'{f"{pos[0]  :.4f}":.6s}'
        f'{f"{pos[1]  :.4f}":.6s}'
        f'{f"{z       :.4f}":.6s}'
        f'{f"{size      :.4f}":.6s}').encode() + color
        
        Draw._send(msg, self._prefix + id.encode(), flush)


    def sphere(self, pos, radius, color:bytes, id:str, flush=True):
        ''' 
        Draw sphere

        Examples
        ----------
        Sphere in 3D: sphere((1,1,1), 3, Draw.Color.red, "my_sphere")
        Sphere in 2D (z=0): sphere((1,1), 3, Draw.Color.red, "my_sphere")
        '''
        if not self.enabled: return
        assert type(color)==bytes, "The RGB color must be a bytes object, e.g. red: b'\xFF\x00\x00'"
        assert not np.isnan(pos).any(), "Argument 'pos' contains 'nan' values"

        z = pos[2] if len(pos)==3 else 0

        if self._is_team_right: 
            pos = (-pos[0],-pos[1],pos[2]) if len(pos)==3 else (-pos[0],-pos[1])

        msg = b'\x01\x03' + (
        f'{f"{pos[0]  :.4f}":.6s}'
        f'{f"{pos[1]  :.4f}":.6s}'
        f'{f"{z       :.4f}":.6s}'
        f'{f"{radius    :.4f}":.6s}').encode() + color
        
        Draw._send(msg, self._prefix + id.encode(), flush)


    def polygon(self, vertices, color:bytes, alpha:int, id:str, flush=True):
        ''' 
        Draw polygon

        Examples
        ----------
        Polygon in 3D: polygon(((0,0,0),(1,0,0),(0,1,0)), Draw.Color.red, 255, "my_polygon")
        '''
        if not self.enabled: return
        assert type(color)==bytes, "The RGB color must be a bytes object, e.g. red: b'\xFF\x00\x00'"
        assert 0<=alpha<=255, "The alpha channel (degree of opacity) must be in range [0,255]"

        if self._is_team_right: 
            vertices = [(-v[0],-v[1],v[2]) for v in vertices]

        msg = b'\x01\x04' + bytes([len(vertices)]) + color + alpha.to_bytes(1,'big')

        for v in vertices:
            msg += (
                f'{f"{v[0]  :.4f}":.6s}'
                f'{f"{v[1]  :.4f}":.6s}'
                f'{f"{v[2]  :.4f}":.6s}').encode()
        
        Draw._send(msg, self._prefix + id.encode(), flush)


    def annotation(self, pos, text, color:bytes, id:str, flush=True):
        ''' 
        Draw annotation

        Examples
        ----------
        Annotation in 3D: annotation((1,1,1), "SOMEtext!", Draw.Color.red, "my_annotation")
        Annotation in 2D (z=0): annotation((1,1), "SOMEtext!", Draw.Color.red, "my_annotation")
        '''
        if not self.enabled: return
        if type(text) != bytes: text = str(text).encode()
        assert type(color)==bytes, "The RGB color must be a bytes object, e.g. red: b'\xFF\x00\x00'"
        z = pos[2] if len(pos)==3 else 0

        if self._is_team_right: 
            pos = (-pos[0],-pos[1],pos[2]) if len(pos)==3 else (-pos[0],-pos[1])

        msg = b'\x02\x00' + (
        f'{f"{pos[0]  :.4f}":.6s}'
        f'{f"{pos[1]  :.4f}":.6s}'
        f'{f"{z       :.4f}":.6s}').encode() + color + text + b'\x00'
        
        Draw._send(msg, self._prefix + id.encode(), flush)

    
    def arrow(self, p1, p2, arrowhead_size, thickness, color:bytes, id:str, flush=True):
        ''' 
        Draw arrow

        Examples
        ----------
        Arrow in 3D: arrow((0,0,0), (0,0,2), 0.1, 3, Draw.Color.red, "my_arrow")
        Arrow in 2D (z=0): arrow((0,0), (0,1), 0.1, 3, Draw.Color.red, "my_arrow")
        '''
        if not self.enabled: return
        assert type(color)==bytes, "The RGB color must be a bytes object, e.g. red: b'\xFF\x00\x00'"

        # No need to invert sides, the called shapes will handle that
        if len(p1)==2: p1 = M.to_3d(p1) 
        else: p1 = np.asarray(p1)
        if len(p2)==2: p2 = M.to_3d(p2) 
        else: p2 = np.asarray(p2)

        vec  = p2-p1
        vec_size = np.linalg.norm(vec)
        if vec_size == 0: return #return without warning/error
        if arrowhead_size > vec_size: arrowhead_size = vec_size

        ground_proj_perpendicular = np.array([ vec[1], -vec[0], 0 ])

        if np.all(ground_proj_perpendicular == 0): #vertical arrow
            ground_proj_perpendicular = np.array([ arrowhead_size/2, 0, 0 ])
        else:
            ground_proj_perpendicular *= arrowhead_size/2 / np.linalg.norm(ground_proj_perpendicular)

        head_start = p2 - vec * (arrowhead_size/vec_size)
        head_pt1 = head_start + ground_proj_perpendicular
        head_pt2 = head_start - ground_proj_perpendicular

        self.line(p1,p2,thickness,color,id,False)
        self.line(p2,head_pt1,thickness,color,id,False)
        self.line(p2,head_pt2,thickness,color,id,flush)


    def flush(self, id):
        ''' Flush specific drawing by ID '''
        if not self.enabled: return

        Draw._send(b'\x00\x00', self._prefix + id.encode(), False)

    def clear(self, id):
        ''' Clear specific drawing by ID '''
        if not self.enabled: return

        Draw._send(b'\x00\x00', self._prefix + id.encode(), True) #swap buffer twice


    def clear_player(self):
        ''' Clear all drawings made by this player '''
        if not self.enabled: return

        Draw._send(b'\x00\x00', self._prefix, True) #swap buffer twice


    @staticmethod
    def clear_all():
        ''' Clear all drawings of all players '''
        if Draw._socket is not None:
            Draw._send(b'\x00\x00\x00\x00\x00',b'',False) #swap buffer twice using no id


    class Color():
        '''
        Based on X11 colors
        The names are restructured to make better suggestions
        '''
        pink_violet = b'\xC7\x15\x85'
        pink_hot = b'\xFF\x14\x93'
        pink_violet_pale = b'\xDB\x70\x93'
        pink = b'\xFF\x69\xB4'
        pink_pale = b'\xFF\xB6\xC1'
        
        red_dark = b'\x8B\x00\x00'
        red = b'\xFF\x00\x00'
        red_brick = b'\xB2\x22\x22'
        red_crimson = b'\xDC\x14\x3C'
        red_indian = b'\xCD\x5C\x5C'
        red_salmon = b'\xFA\x80\x72'

        orange_red = b'\xFF\x45\x00'
        orange = b'\xFF\x8C\x00'
        orange_ligth = b'\xFF\xA5\x00'

        yellow_gold = b'\xFF\xD7\x00'
        yellow = b'\xFF\xFF\x00'
        yellow_light = b'\xBD\xB7\x6B'

        brown_maroon =b'\x80\x00\x00'
        brown_dark = b'\x8B\x45\x13'
        brown = b'\xA0\x52\x2D'
        brown_gold = b'\xB8\x86\x0B'
        brown_light = b'\xCD\x85\x3F'
        brown_pale = b'\xDE\xB8\x87'

        green_dark = b'\x00\x64\x00' 
        green = b'\x00\x80\x00' 
        green_lime = b'\x32\xCD\x32' 
        green_light = b'\x00\xFF\x00' 
        green_lawn = b'\x7C\xFC\x00' 
        green_pale = b'\x90\xEE\x90' 

        cyan_dark = b'\x00\x80\x80' 
        cyan_medium = b'\x00\xCE\xD1'  
        cyan = b'\x00\xFF\xFF' 
        cyan_light = b'\xAF\xEE\xEE'

        blue_dark = b'\x00\x00\x8B' 
        blue = b'\x00\x00\xFF' 
        blue_royal = b'\x41\x69\xE1' 
        blue_medium = b'\x1E\x90\xFF' 
        blue_light = b'\x00\xBF\xFF'
        blue_pale = b'\x87\xCE\xEB'

        purple_violet = b'\x94\x00\xD3' 
        purple_magenta = b'\xFF\x00\xFF' 
        purple_light = b'\xBA\x55\xD3' 
        purple_pale = b'\xDD\xA0\xDD'

        white = b'\xFF\xFF\xFF'
        gray_10 = b'\xE6\xE6\xE6'
        gray_20 = b'\xCC\xCC\xCC'
        gray_30 = b'\xB2\xB2\xB2'   
        gray_40 = b'\x99\x99\x99'
        gray_50 = b'\x80\x80\x80'
        gray_60 = b'\x66\x66\x66'
        gray_70 = b'\x4C\x4C\x4C'
        gray_80 = b'\x33\x33\x33'
        gray_90 = b'\x1A\x1A\x1A'
        black = b'\x00\x00\x00' 

        @staticmethod
        def get(r,g,b):
            ''' Get RGB color (0-255) '''
            return bytes([int(r),int(g),int(b)])
