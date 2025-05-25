import navio.util
from state import State
from control import Control
from sensors import RawSensorData, Sensors
from pid_controller import PID

class Drone:
    
    def __init__(self):
        
        navio.util.check_apm()
        
        self.state: State = State()
        self.sensors: Sensors = Sensors()
        self.control: Control = Control()
        self.pid: PID = PID()
        
        self.sensors.initialize()
    
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
            