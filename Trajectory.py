import krpc
from math import exp, pi, ceil
from threading import Thread
from time import sleep
#from numba import jit

from Vector import Vector3

class Trajectory:
    def __init__(self) -> None:
        self.conn = krpc.connect("Trajectory")
        self.space_center = self.conn.space_center
        self.vessel = self.space_center.active_vessel
        self.body = self.vessel.orbit.body

        self.body_ref = self.body.reference_frame

        self.drawing = self.conn.drawing
        self.GM = self.body.gravitational_parameter
        self.rb = self.body.equatorial_radius
        self.has_atm = self.body.has_atmosphere
        self.atm_depth = self.body.atmosphere_depth

        # Streams
        self.stream_mass = self.conn.add_stream(getattr, self.vessel, "mass")
        self.stream_pos_body = self.conn.add_stream(self.vessel.position, self.body_ref)
        self.stream_vel_body = self.conn.add_stream(self.vessel.velocity, self.body_ref)

        # Initializing
        self.land_pos = Vector3()
        self.draw_trajectory = False

        # drag
        self.radius = 2.5
        self.area = pi * self.radius**2
        self.drag_coefficient = 0.15
        self.c = 0.5 * self.area * self.drag_coefficient

        # rk4
        self.step_check_ground = 2 # steps to check ground collision
        self.step_size = 1 # todo: dynamic step size
        self.total_time = 1000
        self.steps = ceil(self.total_time / self.step_size)

        self.RUNNING = True
        self.thread = Thread(target=self.calc)

    def start(self): # Start loop calculation Thread
        self.thread.start()

    def end(self):
        self.RUNNING = False

    def alt_at_pos(self, pos):
        lat = self.body.latitude_at_position(pos, self.body_ref)
        lon = self.body.longitude_at_position(pos, self.body_ref)
        
        return self.body.altitude_at_position(pos, self.body_ref) - self.body.surface_height(lat, lon)

    #@jit()
    def rk4(self, pos):
        dist = pos.magnitude()

        accel = -(self.GM * pos / dist**3) # gravity

        # drag force
        if self.has_atm and 0 < (dist - self.rb) < self.atm_depth:
            alt = self.body.altitude_at_position(pos, self.body_ref)
            pho = 1.113 * exp(-1.24 / 10000 * alt) # kerbin atm density
            v_mag = self.v[-1].magnitude()
            f_drag = self.c*pho*v_mag**2
            accel -= (f_drag/self.m) * (self.v[-1]/v_mag)

        return accel
    
    def calc(self):
        while self.RUNNING:
            self.r = [Vector3(self.stream_pos_body())]
            self.v = [Vector3(self.stream_vel_body())]
            self.m = self.stream_mass()

            for step in range(self.steps):
                k1v = self.rk4(self.r[-1])
                k1r = self.v[-1]

                k2v = self.rk4(self.r[-1]+k1r*self.step_size*.5)
                k2r = self.v[-1]+k1v*self.step_size*.5

                k3v = self.rk4(self.r[-1]+k2r*self.step_size*.5)
                k3r = self.v[-1]+k2v*self.step_size*.5

                k4v = self.rk4(self.r[-1]+k3r*self.step_size)
                k4r = self.v[-1]+k3v*self.step_size

                dv = (self.step_size/6)*(k1v + 2*k2v + 2*k3v + k4v)
                dr = (self.step_size/6)*(k1r + 2*k2r + 2*k3r + k4r)

                self.v.append(self.v[-1] + dv)
                self.r.append(self.r[-1] + dr)

                if step % self.step_check_ground == 0 and (self.r[-1].magnitude() < self.rb*1.5 and self.alt_at_pos(self.r[-1]) <= 0):
                    i = -2
                    alt = self.alt_at_pos(self.r[i])
                    while alt < 0:
                        i -= 1
                        alt = self.alt_at_pos(self.r[i])
                    a = self.r[i]
                    b = self.r[i+1]
                    
                    ab_dir = (b-a).normalize()
                    _ = (-a).normalize()
                    self.land_pos = a + ab_dir * (alt / _.dot(ab_dir))
                    break
                
            if self.draw_trajectory:
                self.drawing.clear()
                for i in range(len(self.r)-1):
                    self.drawing.add_line(self.r[i], self.r[i+1], self.body_ref)
                    
            sleep(0.1)

        self.conn.close()