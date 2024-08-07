import krpc
from math import ceil
from time import sleep
import numpy as np

from threading import Thread

from PyVecs import Vector3

class Trajectory:
    def __init__(self, use_thread=True, draw_trajectory=False) -> None:
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

        self.atm_alt_list = []
        self.atm_density_list = []
        self.rho = lambda h: np.interp(h, self.atm_alt_list, self.atm_density_list)

        if self.has_atm:
            self.get_atm_data()

        # Streams
        self.stream_mass = self.conn.add_stream(getattr, self.vessel, "mass")
        self.stream_pos_body = self.conn.add_stream(self.vessel.position, self.body_ref)
        self.stream_vel_body = self.conn.add_stream(self.vessel.velocity, self.body_ref)

        # rk4
        self.step_check_ground = 4 # steps to check ground collision
        self._step_size = 1
        self._total_time = 1000
        self.steps = ceil(self._total_time / self._step_size)

        # Initializing
        self.draw_trajectory = draw_trajectory
        self.land_pos = Vector3()
        self.timeout = 0
        self.k = 0

        self.RUNNING = False
        self.CALCULATED = False

        # Thread
        if use_thread:
            self.thread = Thread(target=self.calc)
        else:
            self.start = lambda k: (setattr(self, "k", k), setattr(self, "RUNNING", True), self.calc())

        # Consts
        self.rb_margin = self.rb * 1.5
        
    @property
    def step_size(self):
        return self._step_size
    @step_size.setter
    def step_size(self, value):
        self._step_size = value
        self.steps = ceil(self._total_time / self._step_size)

    @property
    def total_time(self):
        return self._total_time
    @total_time.setter
    def total_time(self, value):
        self._total_time = value
        self.steps = ceil(self._total_time / self._step_size)

    def get_atm_data(self):
        from os import path, makedirs
        from json import dumps, loads

        body_name = self.body.name.lower().replace(' ', '_')

        if path.isdir(f"./data/{body_name}"):
            print("Reading data")

            with open(f"./data/{body_name}/altitude.txt", 'r') as f:
                self.atm_alt_list     = loads(f.read())

            with open(f"./data/{body_name}/density.txt", 'r') as f:
                self.atm_density_list = loads(f.read())

            print("Atm data readed")
        else:
            print("Getting and saving data")

            self.atm_alt_list     = [round(h, 2) for h in np.linspace(0, self.body.atmosphere_depth, 5000)]
            self.atm_density_list = [round(self.body.density_at(h), 2) for h in self.atm_alt_list]

            makedirs(f"./data/{body_name}")

            with open(f"./data/{body_name}/altitude.txt", 'w+') as f:
                f.write(dumps(self.atm_alt_list).replace(' ', ''))

            with open(f"./data/{body_name}/density.txt", 'w+') as f:
                f.write(dumps(self.atm_density_list).replace(' ', ''))

            print("Data saved")

    def start(self, area_cd): # Start loop calculation Thread
        self.k = area_cd # area * Cd
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
    
    def dSdt(self, pos):
        dist = pos.magnitude()

        accel = -(self.GM / dist**3) * pos # gravity

        # drag force 
        if self.has_atm and 0 < (dist - self.rb) < self.atm_depth:
            alt = dist - self.rb
            if alt < 0: alt = 0
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
                k1v = self.dSdt(self.r[-1])
                k1r = self.v[-1]

                k2v = self.dSdt(self.r[-1]+k1r*self.step_size*.5)
                k2r = self.v[-1]+k1v*self.step_size*.5

                k3v = self.dSdt(self.r[-1]+k2r*self.step_size*.5)
                k3r = self.v[-1]+k2v*self.step_size*.5

                k4v = self.dSdt(self.r[-1]+k3r*self.step_size)
                k4r = self.v[-1]+k3v*self.step_size

                dv = (self.step_size/6)*(k1v + 2*k2v + 2*k3v + k4v)
                dr = (self.step_size/6)*(k1r + 2*k2r + 2*k3r + k4r)

                self.v.append(self.v[-1] + dv)
                self.r.append(self.r[-1] + dr)

                if step % self.step_check_ground == 0 and self.r[-1].magnitude() < self.rb_margin and self.alt_at_pos(self.r[-1]) <= 0:
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

            if self.timeout != 0: sleep(self.timeout)

        self.conn.close()


if __name__ == "__main__":
    Trajectory(False)
    exit()
    """
    conn = krpc.connect()
    drawing = conn.drawing
    vessel = conn.space_center.active_vessel
    body = vessel.orbit.body

    body_ref = body.reference_frame

    drag_area = 1.2**2 * 3.14
    cd = 0.5

    traj = Trajectory(False, True)
    traj.timeout = 0
    traj.step_size = 2

    traj.start(cd * drag_area)

    line = traj.drawing.add_line((0, 0, 0), (0, 0, 0), traj.body_ref)

    try:
        while True:
            sleep(1)
            line.start = traj.vessel.position(traj.body_ref)
            line.end = traj.land_pos
    except:
        traj.end()

    
    traj.end()
    """