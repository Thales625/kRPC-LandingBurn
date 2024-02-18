import krpc
from math import pi, ceil
from time import sleep

from threading import Thread

from PyVecs import Vector3

class Trajectory:
    def __init__(self, radius, cd, draw_trajectory=False) -> None:
        self.conn = krpc.connect("Trajectory")
        self.space_center = self.conn.space_center
        self.vessel = self.space_center.active_vessel
        self.body = self.vessel.orbit.body
        self.drawing = self.conn.drawing

        self.body_ref = self.body.reference_frame

        self.GM = self.body.gravitational_parameter
        self.rb = self.body.equatorial_radius
        self.has_atm = self.body.has_atmosphere
        self.atm_depth = self.body.atmosphere_depth
        self.rho = self.body.density_at

        # Streams
        self.stream_mass = self.conn.add_stream(getattr, self.vessel, "mass")
        self.stream_pos_body = self.conn.add_stream(self.vessel.position, self.body_ref)
        self.stream_vel_body = self.conn.add_stream(self.vessel.velocity, self.body_ref)

        # Initializing
        self.draw_trajectory = draw_trajectory
        self.land_pos = Vector3()
        self.delay = 0.2

        # drag
        self.k = 0.5 * pi * radius**2 * cd

        # rk4
        self.step_check_ground = 4 # steps to check ground collision
        self.step_size = 0.1 # todo: dynamic step size
        self.total_time = 1000
        self.steps = ceil(self.total_time / self.step_size)

        self.RUNNING = False
        self.CALCULATED = False

        self.thread = Thread(target=self.calc)

    def start(self): # Start loop calculation Thread
        self.RUNNING = True
        self.thread.start()

    def end(self):
        self.RUNNING = False

    def alt_at_pos(self, pos):
        alt = self.body.altitude_at_position(pos, self.body_ref)

        lat = self.body.latitude_at_position(pos, self.body_ref)
        lon = self.body.longitude_at_position(pos, self.body_ref)
        height = self.body.surface_height(lat, lon)

        if height <= 0:
            return alt

        return alt - height
    
    def rk4(self, pos):
        dist = pos.magnitude()

        accel = -(self.GM * pos / dist**3) # gravity

        # drag force 
        if self.has_atm and 0 < (dist - self.rb) < self.atm_depth:
            alt = self.body.altitude_at_position(pos, self.body_ref)
            v_mag = self.v[-1].magnitude()
            f_drag = self.k*self.rho(alt)*v_mag**2
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

                if step % self.step_check_ground == 0 and self.r[-1].magnitude() < self.rb*1.5 and self.alt_at_pos(self.r[-1]) <= 0:
                    
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
                    line = self.drawing.add_line(self.r[i], self.r[i+1], self.body_ref)
                    line.color = (0, 0, 1)
                    line.thickness = 1
                l = self.drawing.add_line(self.land_pos, self.land_pos*1.2, self.body_ref)
                l.color = (1, 0, 0)
                l.thickness = 5
                    
            self.CALCULATED = True

            sleep(self.delay)

        self.conn.close()


if __name__ == "__main__":
    traj = Trajectory(1.2, 0.6, True)
    traj.start()

    try:
        while True:
            sleep(1)
    except:
        traj.end()

    
    traj.end()