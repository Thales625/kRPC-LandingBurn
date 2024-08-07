from time import time

from controllers import PIController

class ThrottleControl:
    def __init__(self, ut_func=time) -> None:
        self.ut  = ut_func
        self.pi = PIController()
        self.pi.limit_output(0, 1)
    
    def linear_control(self, dv, a_eng, a_g, factor=5):
        return (dv*factor + a_g) / a_eng

    def pid_control(self, dv, a_eng):
        return self.pi(-dv/a_eng, 0, self.ut())