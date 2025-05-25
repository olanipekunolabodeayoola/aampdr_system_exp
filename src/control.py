import navio.rcinput

class Control:
    
    def __init__(self, ch_throttle:int, ch_roll:int, ch_pitch:int, ch_yaw:int):
        self.throttle = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        self.rcin = navio.rcinput.RCInput()
        
        self.ch_throttle = ch_throttle
        self.ch_roll = ch_roll
        self.ch_pitch = ch_pitch
        self.ch_yaw = ch_yaw
        
    def update(self)->None:
        
        self.throttle = self.rcin.read(self.ch_throttle)
        self.roll = self.rcin.read(self.ch_roll)
        self.pitch = self.rcin.read(self.ch_pitch)
        self.yaw = self.rcin.read(self.ch_yaw)