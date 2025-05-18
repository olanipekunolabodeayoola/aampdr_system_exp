"""
This is PID controller Class
"""
from state import State
from control import Control

adjest_level = lambda x: x * 15

class PID:
    
    def __init__(self):
        self.MIN_SETPOINT_BOUNDARY = 1492
        self.MAX_SETPOINT_BOUNDARY = 1508
        self.TROTTLE_UP_BLOUDARY = 1050
        
        self.set_points = State()
    
    def calculate_set_point_angle(self, angle: float, channel_pulse: float) -> float:
        
        set_point = 0.0
        if channel_pulse > self.MAX_SETPOINT_BOUNDARY:
            set_point = channel_pulse - self.MAX_SETPOINT_BOUNDARY;
        elif channel_pulse < self.MIN_SETPOINT_BOUNDARY:
            set_point = channel_pulse - self.MIN_SETPOINT_BOUNDARY;
        else:
            set_point = 0.0
            
        set_point -= adjest_level(angle)
        set_point /= 3
        return set_point
    
    def calculate_yaw_set_point(self, throttle_pulse: float, yaw_pulse: float) -> float:
        
        if throttle_pulse > self.TROTTLE_UP_BLOUDARY:
            return self.calculate_set_point_angle(0, yaw_pulse)
        else:
            return 0.0
    
    def calculate_set_points(self, drone_state: State, control_signals: Control) -> State:
        
        self.set_points.roll = self.calculate_set_point_angle(drone_state.roll, control_signals.roll)
        self.set_points.pitch = self.calculate_set_point_angle(drone_state.pitch, control_signals.pitch)
        self.set_points.yaw = self.calculate_yaw_set_point(drone_state.throttle, control_signals.yaw)
        
        return self.set_points
    
    def get_last_set_points(self) -> State:
        return self.set_points