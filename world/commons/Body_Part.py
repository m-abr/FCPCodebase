from math_ops.Matrix_4x4 import Matrix_4x4

class Body_Part():
    def __init__(self, mass) -> None:
        self.mass = float(mass)
        self.joints = []
        self.transform = Matrix_4x4() # body part to head transformation matrix