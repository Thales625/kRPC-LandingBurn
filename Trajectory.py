import krpc
from math import exp, pi
from threading import Thread

from sys import path
path.append('D:\Codes\Python\www\Vector')
from Vector import Vector3

class Trajectory:
    def __init__(self) -> None:
        self.conn = krpc.connect('Trajectory')
        self.space_center = self.conn.space_center
        self.vessel = self.space_center.active_vessel
        self.body = self.vessel.orbit.body
        self.body_ref = self.body.reference_frame
        self.flight_body = self.vessel.flight(self.body_ref)
        self.GM = self.body.gravitational_parameter
        #self.drawing = self.conn.drawing

        # Streams
        self.stream_mass = self.conn.add_stream(getattr, self.vessel, "mass")
        self.stream_position_body = self.conn.add_stream(self.vessel.position, self.body_ref)
        self.stream_vel_body = self.conn.add_stream(getattr, self.flight_body, "velocity")

        # Initializing
        self.land_pos = Vector3() # Body Ref

        # line
        #self.line = self.drawing.add_line((0, 0 ,0), (0, 0 ,0), self.body_ref)

        # drag
        self.radius = 2.6
        self.area = pi * self.radius**2
        self.drag_coefficient = 0.14
        self.c = 0.5 * self.area * self.drag_coefficient

        self.dt = 0.1

        self.RUNNING = True
        self.thread = Thread(target=self.calc)

    def start(self): # Start loop calculation Thread
        self.thread.start()

    def end(self):
        self.RUNNING = False

    
    def calc(self):
        while self.RUNNING:
            pos = Vector3(self.stream_position_body())
            vel = Vector3(self.stream_vel_body())

            alt = self.body.altitude_at_position(tuple(pos), self.body_ref)

            while alt > 0:
                a = (-self.GM / pos.magnitude()**2) * pos.normalize() # gravity
                v_mag = vel.magnitude()
                v_norm = vel / v_mag
 
                # drag
                if self.body.has_atmosphere and alt < self.body.atmosphere_depth:
                    pho = 1.113 * exp(-1.24 / 10000*alt)
                    
                    a -= (self.c * pho * v_mag**2 / self.stream_mass()) * v_norm

                vel += a * self.dt
                pos += vel * self.dt

                alt = self.body.altitude_at_position(tuple(pos), self.body_ref)
                alt -= self.body.surface_height(self.body.latitude_at_position(tuple(pos), self.body_ref), self.body.longitude_at_position(tuple(pos), self.body_ref))

            self.land_pos = pos

        self.conn.close()