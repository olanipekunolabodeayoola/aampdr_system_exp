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
        self.ARM_BOUNDARY = 1012
        self.LOWER_ESC = 1100
        self.HIGHER_ESC = 2000
        
        self.set_points = Vector()
        self.error = Vector()
        self.error_sum = Vector()
        self.clip_values = Vector()
        self.delta_error = Vector()
        self.prev_error = Vector()
        
        self.KP = KP
        self.KI = KI
        self.KD = KD
        
        self.clip_values.update(KI.x/400, KI.y/400, KI.z/400)
        
        self.esc_a: float = 0.0
        self.esc_b: float = 0.0
        self.esc_c: float = 0.0
        self.esc_d: float = 0.0
        
        """
        A    B
          \/
          /\
        C    D
        
        A, D : Clockwise
        B, C : Counter Clockwise
        
        ^
        | = Forward
        """
    
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
        
    def reset(self):
        self.error.reset()
        self.error_sum.reset()
        self.prev_error.reset()
        self.delta_error.reset()
        
    def stop_all_esc(self):
        self.esc_a = 1000
        self.esc_b = 1000
        self.esc_c = 1000
        self.esc_d = 1000
        
    def run(self, control: Control) -> None:
        
        pid_values = Vector()
        
        self.esc_a = control.throttle
        self.esc_b = control.throttle
        self.esc_c = control.throttle
        self.esc_d = control.throttle
        
        if control.throttle > self.ARM_BOUNDARY:
            
            pid_values.set(
                (self.error.x * self.KP.x) + (self.error_sum.x * self.KI.x) + (self.delta_error.x * self.KD.x),
                (self.error.y * self.KP.y) + (self.error_sum.y * self.KI.y) + (self.delta_error.y * self.KD.y),
                (self.error.z * self.KP.z) + (self.error_sum.z * self.KI.z) + (self.delta_error.z * self.KD.z)
            )
            
            pid_values.set(clip(pid_values.x, -400, 400), clip(pid_values.y, -400, 400), clip(pid_values.z, -400, 400))
            
            self.esc_a = control.throttle - pid_values.x - pid_values.y + pid_values.z
            self.esc_b = control.throttle + pid_values.x - pid_values.y - pid_values.z
            self.esc_c = control.throttle - pid_values.x + pid_values.y - pid_values.z
            self.esc_d = control.throttle + pid_values.x + pid_values.y + pid_values.z
            
        
        self.esc_a = clip(self.esc_a, self.LOWER_ESC, self.HIGHER_ESC)
        self.esc_b = clip(self.esc_b, self.LOWER_ESC, self.HIGHER_ESC)
        self.esc_c = clip(self.esc_c, self.LOWER_ESC, self.HIGHER_ESC)
        self.esc_d = clip(self.esc_d, self.LOWER_ESC, self.HIGHER_ESC)
        