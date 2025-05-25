import navio.util
from state import State
from control import Control
from sensors import RawSensorData, Sensors
from pid_controller import PID
from motor_control import MotorController
from vector import Vector
class Drone:
    
    def __init__(self):
        
        navio.util.check_apm()
        
        self.state: State = State()
        self.sensors: Sensors = Sensors()
        self.control: Control = Control()
        self.pid: PID = PID(
            Vector(4.0, 1.3, 1.3),
            Vector(0.02, 0.04, 0.04),
            Vector(0, 18, 18)
        )
        self.motor: MotorController = MotorController(0, 1, 2, 3)
        
        self.sensors.initialize()
        self.motor.initialize()
    
    def is_drone_started(self) -> None:
        
        if self.state.is_drone_stopped() and self.control.yaw <= self.pid.ARM_BOUNDARY and self.control.throttle <= self.pid.ARM_BOUNDARY:
            self.state.set_drone_status("STARTING")
            
        if self.state.is_drone_starting() and self.control.yaw == 1500 and self.control.throttle <= self.pid.ARM_BOUNDARY:
            self.state.set_drone_status("STARTED")
            
            self.pid.reset()
            
            self.state.gyro_angle.fill_vector(self.state.acc_angle)
            
        if self.state.is_drone_started() and self.control.yaw >= 1988 and self.control.throttle <= self.pid.ARM_BOUNDARY:
            self.state.set_drone_status("STOPPED")
            self.pid.stop_all_esc()
            
        return self.state.is_drone_started()
        
    def execute(self):
        
        while True:
            
            sensor_data: RawSensorData = self.sensors.read_data()
            
            self.state.update_state(sensor_data)
            self.control.update()
            
            self.pid.calculate_set_points(self.state, self.control)
            
            self.pid.calculate_errors(self.state)
            
            if self.is_drone_started():
                
                self.pid.run()
                
                # Check Battery
                
            self.motor.apply_roter_speed_a(self.pid.esc_a * 0.001)
            self.motor.apply_roter_speed_b(self.pid.esc_b * 0.001)
            self.motor.apply_roter_speed_c(self.pid.esc_c * 0.001)
            self.motor.apply_roter_speed_d(self.pid.esc_d * 0.001)
            