class State:
    
    def __init__(self):
        
        self.gyro_raw_x = 0.0
        self.gyro_raw_y = 0.0
        self.gyro_raw_z = 0.0
        
        self.acc_raw_x = 0.0
        self.acc_raw_y = 0.0
        self.acc_raw_z = 0.0
        
        self.acc_angle_x = 0.0
        self.acc_angle_y = 0.0
        self.acc_angle_z = 0.0
        
        self.gyro_angle_x = 0.0
        self.gyro_angle_y = 0.0
        self.gyro_angle_z = 0.0
        
        self.pitch = 0.0
        self.yaw = 0.0
        self.throttle = 0.0
        
    def update_state(self, gyro_raw_x: float, gyro_raw_y: float, gyro_raw_z: float, acc_raw_x: float, acc_raw_y: float, acc_raw_z: float)->None:
        
        self.gyro_raw_x = gyro_raw_x
        self.gyro_raw_y = gyro_raw_y
        self.gyro_raw_z = gyro_raw_z
        
        self.acc_raw_x = acc_raw_x
        self.acc_raw_y = acc_raw_y
        self.acc_raw_z = acc_raw_z
    
        
    
