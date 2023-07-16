import krpc
from time import sleep
from math import sqrt

from sys import path
path.append('/www/Vector')
from Vector import Vector3


class LandingBurn:
    def __init__(self, vessel=None, land_func=None):
        self.conn = krpc.connect('LandingBurn')
        self.space_center = self.conn.space_center
        self.vessel = self.space_center.active_vessel if vessel == None else vessel
        self.body = self.vessel.orbit.body
        self.body_ref = self.body.reference_frame
        self.surface_ref = self.vessel.surface_reference_frame
        self.flight = self.vessel.flight(self.body_ref)
        self.drawing = self.conn.drawing

        # Streams
        self.stream_mass = self.conn.add_stream(getattr, self.vessel, "mass")
        self.stream_av_thrust = self.conn.add_stream(getattr, self.vessel, "available_thrust")
        self.stream_vel_body = self.conn.add_stream(getattr, self.flight, "velocity")
        self.stream_surface_altitude = self.conn.add_stream(getattr, self.flight, "surface_altitude")
        self.stream_pitch = self.conn.add_stream(getattr, self.vessel.flight(self.surface_ref), "pitch")

        # ===================
        if land_func is None:
            self.land_func = lambda self: (
                self.progressive_engine_cut(),
                self.finish())
        else:
            self.land_func = land_func

        # body properties
        self.surface_gravity = self.body.surface_gravity
        
        # Params
        self.tilted_engines = False
        self.max_twr = 4
        self.final_speed = -2
        self.final_altitude = 5
        self.gear_delay = 4

        # Initializing
        self.vessel.control.throttle = 0
        self.vessel.control.brakes = True
        self.vessel.control.rcs = True
        self.vessel.auto_pilot.engage()
        self.vessel.auto_pilot.target_roll = -90
        self.vessel.auto_pilot.reference_frame = self.body_ref

        while self.get_velocity().x > 0 or self.get_altitude() > 10000: # WAIT
            self.vessel.auto_pilot.target_direction = self.space_center.transform_direction((1, 0, 0), self.surface_ref, self.body_ref)
            sleep(0.5)

        # Thrust de acordo com ângulo de montagem do motor
        self.thrust = 0
        for engine in self.vessel.parts.engines:
            if engine.active and engine.has_fuel:
                self.thrust += engine.available_thrust * engine.part.direction(self.vessel.reference_frame)[1]

        while True:
            # Check Landed
            if self.vessel.situation == self.vessel.situation.landed or self.vessel.situation == self.vessel.situation.splashed:
                self.land_func(self)
                break

            # Get Streams
            vel = self.get_velocity()
            alt = self.get_altitude()
            mass = self.stream_mass()
            av_thrust = self.thrust if self.tilted_engines else self.stream_av_thrust()
            pitch = self.stream_pitch()

            vert_speed = vel.x
            mag_speed = vel.magnitude()
            
            a_eng = av_thrust / mass
            eng_threshold = min(self.max_twr * self.surface_gravity / a_eng, 1)
            a_eng_l = a_eng * eng_threshold
            a_net = max(a_eng_l - self.surface_gravity, .1)
            
            target_dir = vel * -1
            target_dir.x = abs(target_dir.x)

            time_fall = (vert_speed + sqrt(vert_speed**2 + 2*self.surface_gravity*alt)) / self.surface_gravity

            burn_altitude = (mag_speed**2 - self.final_speed**2 + 2*self.surface_gravity*alt) / (2 * a_eng_l)
            t_to_burn = (vert_speed + sqrt(vert_speed**2 + 2*self.surface_gravity*abs(alt - burn_altitude))) / self.surface_gravity
            t_burning = sqrt(2*abs(burn_altitude) / a_net)
            t_fall = t_to_burn + t_burning

            # Throttle Controller
            if vert_speed < 0 and pitch > 0:

                land_prediction = Vector3(-alt, vel.y*time_fall, vel.z*time_fall)
                dist = land_prediction.magnitude()
                
                #target_speed = a_net * sqrt(2*dist / a_net)
                target_speed = abs(self.final_speed)
                if alt > self.final_altitude:
                    target_speed = sqrt(self.final_speed**2 + 2*a_net*abs(dist-self.final_altitude))
                    delta_speed = mag_speed - target_speed
                    print(delta_speed)
                    throttle = (delta_speed * 18 + self.surface_gravity) / a_eng
                else: # Final Burn
                    target_dir.x *= 40
                    delta_speed = self.final_speed - vert_speed
                    throttle = (delta_speed * 10 + self.surface_gravity) / a_eng

                self.vessel.control.throttle = throttle
        
                #print(f'Speed: {vert_speed:.2f} | Target: {target_speed:.2f} | Delta: {delta_speed:.2f} | Alt: {alt:.2f} | TFall: {(t_fall):.2f}')

                # Check Gears
                if t_fall < self.gear_delay: self.vessel.control.gear = True
            else:
                self.vessel.control.throttle = 0

            # Aim Vessel
            self.vessel.auto_pilot.target_direction = self.space_center.transform_direction(tuple(target_dir), self.surface_ref, self.body_ref)

            sleep(0.01)

    def finish(self):
        self.vessel.control.brakes = False
        print(f'{self.vessel.name} Pousou!')
        self.vessel.auto_pilot.disengage()
        self.vessel.control.rcs = True
        self.vessel.control.sas = True
        sleep(0.1)
        try:
            self.vessel.control.sas_mode = self.vessel.control.sas_mode.radial
        except:
            self.vessel.auto_pilot.engage()
            self.vessel.auto_pilot.target_direction = self.space_center.transform_direction((1, 0, 0), self.surface_ref, self.body_ref)
        sleep(4)
        self.vessel.control.rcs = False
        self.vessel.auto_pilot.disengage()
        self.conn.close()

    def progressive_engine_cut(self):
        a_eng = (self.thrust if self.tilted_engines else self.stream_av_thrust()) / self.stream_mass()

        throttle = min((self.surface_gravity / a_eng) *.9, 1)
        total_time = 1
        dt = 0.1
        
        delta_throttle = (throttle * dt) / total_time

        while throttle > 0:
            throttle -= delta_throttle
            self.vessel.control.throttle = throttle
            sleep(dt)

        self.vessel.control.throttle = 0

    def get_velocity(self):
        return Vector3(self.space_center.transform_direction(self.stream_vel_body(), self.body_ref, self.surface_ref))
    
    def get_altitude(self):
        return max(0, self.stream_surface_altitude() + self.vessel.bounding_box(self.surface_ref)[0][0])
    

if __name__ == '__main__':
    LandingBurn()