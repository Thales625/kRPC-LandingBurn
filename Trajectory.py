from threading import Thread

from sys import path
path.append('/home/thales/Documentos/Codes/www/Vector')
from Vector import Vector3

class Trajectory:
    def __init__(self, body) -> None:
        self.body = body
        self.ref = self.body.reference_frame

        self.GM = self.body.gravitational_parameter

        self.initial_pos = Vector3() # Body Ref
        self.initial_vel = Vector3() # Body Ref

        self.land_pos = Vector3()    # Body Ref

        self.dt = 0.1

        self.RUNNING = True

        self.thread = Thread(target=self.calc)

    def start(self): # Start loop calculation Thread
        self.thread.start()
    
    def calc(self):
        while self.RUNNING:
            pos = self.initial_pos
            vel = self.initial_vel

            while True:
                a_g = (-self.GM / pos.magnitude()**2) * pos.normalize()

                vel += a_g * self.dt
                pos += vel * self.dt

                alt = self.body.altitude_at_position(tuple(pos), self.ref)
                #print(alt)
                if alt <= 0:
                    self.land_pos = pos
                    break