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
        
    def execute(self):
        
        while True:
            
            sensor_data: RawSensorData = self.sensors.read_data()
            
            self.state.update_state(sensor_data)
            self.control.update()
            
            self.pid.calculate_set_points(self.state, self.control)
            
            self.pid.calculate_errors(self.state)
            