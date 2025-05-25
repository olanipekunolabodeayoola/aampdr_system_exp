"""
This is PID controller Class
"""
from vector import Vector
from state import State
from control import Control
from utils import clip

adjest_level = lambda x: x * 15

class PID:
    
    def __init__(self, KP: Vector, KI: Vector, KD: Vector):
        self.MIN_SETPOINT_BOUNDARY = 1492
        self.MAX_SETPOINT_BOUNDARY = 1508
        self.TROTTLE_UP_BLOUDARY = 1050
        
        self.set_points = Vector()
        self.error = Vector()
        self.error_sum = Vector()
        self.clip_values = Vector()
        self.delta_error = Vector()
        self.prev_error = Vector()
        
        self.clip_values.update(KI.x/400, KI.y/400, KI.z/400)
    
    def calculate_set_point_angle(self, angle: float, channel_pulse: float) -> float:
        
        set_point = 0.0
        if channel_pulse > self.MAX_SETPOINT_BOUNDARY:
            set_point = channel_pulse - self.MAX_SETPOINT_BOUNDARY
        elif channel_pulse < self.MIN_SETPOINT_BOUNDARY:
            set_point = channel_pulse - self.MIN_SETPOINT_BOUNDARY
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
    
    def calculate_set_points(self, drone_state: State, control_signals: Control) -> Vector:
        
        self.set_points.x = self.calculate_set_point_angle(drone_state.roll, control_signals.roll)
        self.set_points.y = self.calculate_set_point_angle(drone_state.pitch, control_signals.pitch)
        self.set_points.z = self.calculate_yaw_set_point(drone_state.throttle, control_signals.yaw)
        
        return self.set_points
    
    def get_last_set_points(self) -> Vector:
        return self.set_points
    
    def calculate_errors(self, state: State) -> None:
        
        self.error.set(
            state.roll - self.set_points.x,
            state.pitch - self.set_points.y,
            state.yaw - self.set_points.z
        )
        
        self.error_sum.update(
            self.error.x,
            self.error.y,
            self.error.z
        )
        
        self.error_sum.set(
            clip(self.error_sum.x, -self.clip_values.x, self.clip_values.x),
            clip(self.error_sum.y, -self.clip_values.y, self.clip_values.y),
            clip(self.error_sum.z, -self.clip_values.z, self.clip_values.z)
        )
        
        self.delta_error.set(
            self.error.x - self.prev_error.x,
            self.error.y - self.prev_error.y,
            self.error.z - self.prev_error.z
        )
        
        self.prev_error.fill_vector(self.error)
        
        