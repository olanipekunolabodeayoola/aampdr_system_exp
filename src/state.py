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

class State:
    
    def __init__(self):
        
        self.gyro_offset = Vector(0.0, 0.0, 0.0)
        self.gyro_raw = Vector(0.0, 0.0, 0.0)
        self.acc_raw = Vector(0.0, 0.0, 0.0)
        
        self.measurements = Vector(0.0, 0.0, 0.0)
        self.gyro_angle = Vector(0.0, 0.0, 0.0)
        self.acc_angle = Vector(0.0, 0.0, 0.0)
        
        self.pitch = 0.0
        self.yaw = 0.0
        self.throttle = 0.0
        
    def update_state(self, gyro_raw_x: float, gyro_raw_y: float, gyro_raw_z: float, acc_raw_x: float, acc_raw_y: float, acc_raw_z: float)->None:
        
        self.gyro_raw.set(gyro_raw_x, gyro_raw_y, gyro_raw_z)
        self.acc_raw.set(acc_raw_x, acc_raw_y, acc_raw_z)
        
        self.calculate_gyro_angles()
        self.calculate_acce_angles()
        
        self.calculate_angles()
        
    def calculate_angles(self) -> None:
        
        self.gyro_raw.set_x(self.gyro_angle.x *  0.9996 + self.acc_angle.x * 0.0004)
        self.gyro_raw.set_y(self.gyro_angle.y *  0.9996 + self.acc_angle.y * 0.0004)
        
        self.measurements.set(
            self.measurements.x * 0.9 + self.gyro_angle.x * 0.1,
            self.measurements.y * 0.9 + self.gyro_angle.y * 0.1,
            self.gyro_angle.z
        )
        
        self.row = 0.7 * self.row  + 0.3 * self.gyro_raw.x
        self.pitch = 0.7 * self.pitch  + 0.3 * self.gyro_raw.y
        self.yaw = 0.7 * self.yaw  + 0.3 * self.gyro_raw.z
    
    def calculate_gyro_angles(self)->None:
        
        self.gyro_raw.update_x(-self.gyro_offset.x)
        self.gyro_raw.update_y(-self.gyro_offset.y)
        self.gyro_raw.update_z(-self.gyro_offset.z)
        
        # TODO Check the values and confirm 
        self.gyro_angle.update_x(self.gyro_raw.x)
        self.gyro_angle.update_y(-self.gyro_raw.y)
        
        self.acc_angle.set_x(self.acc_angle.x)
        self.acc_angle.set_y(-self.acc_angle.y)
        
        # // Angle calculation using integration
        # gyro_angle[X] += (gyro_raw[X] / (FREQ * SSF_GYRO));
        # gyro_angle[Y] += (-gyro_raw[Y] / (FREQ * SSF_GYRO)); // Change sign to match the accelerometer's one

        # // Transfer roll to pitch if IMU has yawed
        # gyro_angle[Y] += gyro_angle[X] * sin(gyro_raw[Z] * (PI / (FREQ * SSF_GYRO * 180)));
        # gyro_angle[X] -= gyro_angle[Y] * sin(gyro_raw[Z] * (PI / (FREQ * SSF_GYRO * 180)));
        
    def calculate_acce_angles(self) -> None:
        
        acceloration = self.acc_raw.calculate_size()
        
        if abs(self.acc_raw.x) < acceloration:
            self.acc_angle.set_x(math.asin(self.acc_raw.y / acceloration) * (180 / math.pi))
            
        if abs(self.acc_raw.y) < acceloration:
            self.acc_angle.set_y(math.asin(self.acc_raw.x / acceloration) * (180 / math.pi))

        
    
