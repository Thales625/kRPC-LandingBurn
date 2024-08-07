from numpy import interp, linspace
from scipy.integrate import odeint

class LandingCalculator:
    def __init__(self, touchdown_vel, body, thrust_func, area_cd=0):
        self.GM      = body.gravitational_parameter
        self.r_body  = body.equatorial_radius
        self.has_atm = body.has_atmosphere
        self.thrust_func = thrust_func
        self.thrust_multiplier = 1

        if self.has_atm:
            self.rho = body.density_at
            self.p   = body.pressure_at
            self.p0  = 1 / body.pressure_at(0)
            self.k   = area_cd * 0.5

        self.touchdown_vel = touchdown_vel

        self.resolution = 5000
        self.h_eval = None
        self.v_h_target = None
        self.calculated = False

    def dVdh(self, h, v):
        if v == 0:
            return 0 # dvdh
        else:
            dvdt = -self.GM / (self.r_body + h)**2 # Gravity

            if self.has_atm:
                f_eng = self.thrust_func(self.p(h) * self.p0) * self.thrust_multiplier
                f_drag = -self.k * self.rho(h) * v*abs(v)
                dvdt += (f_eng + f_drag) * self.mass_const # dvdt with drag
            else:
                dvdt += self.thrust_func(0) * self.thrust_multiplier * self.mass_const # engine

            return dvdt / v

    def calculate(self, h_final, mass):
        self.mass_const = 1 / mass

        self.h_eval = linspace(0, h_final, self.resolution)

        # V(h)
        self.v_h_target = [i[0] for i in odeint(self.dVdh, t=self.h_eval, y0=[self.touchdown_vel if self.touchdown_vel != 0 else -1e-3], tfirst=True)]

        del self.mass_const

        self.calculated = True

    def get_v_target(self, h):
        if not self.calculated:
            print("V_target was not calculated!")
            return -1
        
        return interp(h, self.h_eval, self.v_h_target)