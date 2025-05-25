import math

class Vector:
    
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        
    def update_x(self, x: float) -> None:
        self.x += x
        
    def update_y(self, y: float) -> None:
        self.y += y
    
    def update_z(self, z: float) -> None:
        self.y += z
        
    def update(self, x: float, y: float, z: float) -> None:
        self.update_x(x)
        self.update_y(y)
        self.update_z(z)
        
    def set_x(self, x: float) -> None:
        self.x = x
        
    def set_y(self, y: float) -> None:
        self.y = y
    
    def set_z(self, z: float) -> None:
        self.y = z
        
    def set(self, x: float, y: float, z: float) -> None:
        self.set_x(x)
        self.set_y(y)
        self.set_z(z)
        
    def fill_vector(self, v) -> None:
        self.x = v.x
        self.y = v.y
        self.z = v.z
        
    def calculate_size(self):
        return math.sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2)
    
    def reset(self, value: float = 0.0):
        self.set(value, value, value)