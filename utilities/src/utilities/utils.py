#!/usr/bin/env python


# Sign number to name dictionary and name list
signDict = { 
    20 : 'STOP',    # Stop sign
    74 : 'GO',      # Traffic light sign
    7  : 'LEFT',    # One way pointing left
    6  : 'RIGHT',   # One way pointing right
    96 : 'SLOW',    # Duck crossing
    2  : 'FAST',    # Yield ~ welcome to Massachusetts
    -1 : 'NONE'     # No signs spotted
}

signNames = signDict.values()


# Generic PID controller class
class PidController:
    def __init__(self, kp, ki, kd, init_time, init_error, init_integ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = init_error
        self.integ = init_integ
        self.prev_time = init_time

    def calibrateTime(self, curr_time):
        # This may need tuning
        if curr_time - self.prev_time >= 2:
            self.prev_time = curr_time
        
    def pid(self, curr_time, curr_error):
        dt = self.prev_time - curr_time
        self.prev_time = curr_time
        
        self.integ += curr_error * dt
        p = curr_error * self.kp
        i = self.integ * self.ki
        d = ((curr_error - self.prev_error) / dt ) * self.kd
        self.prev_error = curr_error
        return  p + i + d

