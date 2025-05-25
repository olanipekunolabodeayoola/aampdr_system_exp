import navio.pwm

class MotorController:
    
    def __init__(self, A:int, B:int, C:int, D:int):
        
        self.motor_a = navio.pwm.PWM(A)
        self.motor_b = navio.pwm.PWM(B)
        self.motor_c = navio.pwm.PWM(C)
        self.motor_d = navio.pwm.PWM(D)
        
    def initialize(self, period:int=50):
        
        self.motor_a.set_period(period)
        self.motor_a.enable()
        
        self.motor_b.set_period(period)
        self.motor_b.enable()
        
        self.motor_c.set_period(period)
        self.motor_c.enable()
        
        self.motor_d.set_period(period)
        self.motor_d.enable()
        
    def apply_roter_speed_a(self, speed: float) -> None:
        self.motor_a.set_duty_cycle(speed)
        
    def apply_roter_speed_b(self, speed: float) -> None:
        self.motor_b.set_duty_cycle(speed)
        
    def apply_roter_speed_c(self, speed: float) -> None:
        self.motor_c.set_duty_cycle(speed)
        
    def apply_roter_speed_d(self, speed: float) -> None:
        self.motor_d.set_duty_cycle(speed)