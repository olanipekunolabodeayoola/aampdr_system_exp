import navio.mpu9250
from vector import Vector

class RawSensorData:
    def __init__(self, acc, gyro, mag):
        self.acc = Vector(acc[0], acc[1], acc[2])
        self.gyro = Vector(gyro[0], gyro[1], gyro[2])
        self.mag = Vector(mag[0], mag[1], mag[2])

class Sensors:
    
    def __init__(self):
        self.imu = navio.mpu9250.MPU9250()
        
    def initialize(self) -> None:
        self.imu.initialize()
        
    def read_data(self) -> RawSensorData:
        m9a, m9g, m9m = self.imu.getMotion9()
        return RawSensorData(m9a, m9g, m9m)
    