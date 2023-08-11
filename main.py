import krpc
from time import sleep
from math import sqrt

from Vector import Vector3

from Trajectory import Trajectory

MULTILANDING = False

class LandingBurn:
    def __init__(self, vessel_tag=None, land_func=None):
        self.conn = krpc.connect('LandingBurn' + ((': ' + vessel_tag) if vessel_tag else ''))
        self.space_center = self.conn.space_center
        self.vessel = self.space_center.active_vessel
        if vessel_tag:
            for vessel in self.space_center.vessels:
                if len(vessel.parts.with_tag(vessel_tag)) > 0:
                    self.vessel = vessel
                    break
        self.body = self.vessel.orbit.body
        self.body_ref = self.body.reference_frame
        self.surface_ref = self.vessel.surface_reference_frame
        self.flight_body = self.vessel.flight(self.body_ref)
        self.flight_surface = self.vessel.flight(self.surface_ref)

        # Streams
        self.stream_mass = self.conn.add_stream(getattr, self.vessel, "mass")
        self.stream_av_thrust = self.conn.add_stream(getattr, self.vessel, "available_thrust")
        self.stream_vel_body = self.conn.add_stream(getattr, self.flight_body, "velocity")
        self.stream_position_body = self.conn.add_stream(self.vessel.position, self.body_ref)
        self.stream_surface_altitude = self.conn.add_stream(getattr, self.flight_body, "surface_altitude")
        self.stream_pitch = self.conn.add_stream(getattr, self.flight_surface, "pitch")

        # ===================
        if land_func is None:
            self.land_func = lambda self: (
                self.progressive_engine_cut(),
                self.finish())
        else:
            self.land_func = land_func

        # consts
        self.a_g = self.body.surface_gravity
        
        # Params
        self.tilted_engines = False
        self.gear_delay = 4
        self.max_twr = 4
        self.final_speed = -2
        self.final_altitude = 10

        self.line_land = self.conn.drawing.add_line((0, 0, 0), (0, 0, 0), self.body_ref)
        self.line_land.thickness = 10

        # Trajectory
        if not MULTILANDING:
            self.trajectory = Trajectory()

        # Initializing
        self.vessel.control.throttle = 0
        self.vessel.control.brakes = True
        self.vessel.control.rcs = True
        self.vessel.auto_pilot.engage()
        self.vessel.auto_pilot.reference_frame = self.body_ref
        self.vessel.auto_pilot.stopping_time = (0.5, 0.5, 0.5)
        self.vessel.auto_pilot.deceleration_time = (5, 5, 5)
        self.vessel.auto_pilot.target_roll = 0

        # Waiting
        while self.get_velocity().x > 0:
            self.vessel.auto_pilot.target_direction = self.space_center.transform_direction((1, 0, 0), self.surface_ref, self.body_ref)
            sleep(0.5)
        
        while self.stream_surface_altitude() > 15000:
            self.vessel.auto_pilot.target_direction = tuple(Vector3(self.stream_vel_body()) * -1)
            sleep(0.5)

        # Initializing Trajectory module
        if not MULTILANDING: self.trajectory.start()

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

            mag_speed = vel.magnitude()
            
            a_eng = av_thrust / mass
            eng_threshold = min(self.max_twr * self.a_g / a_eng, 1)
            a_eng_l = a_eng * eng_threshold

            a_net = max(a_eng_l - self.a_g, .1)
            
            target_dir = vel * -1
            target_dir.x = abs(target_dir.x)

            burn_altitude = (mag_speed**2 - self.final_speed**2 + 2*self.a_g*alt) / (2 * a_eng_l)
            t_to_burn = (vel.x + sqrt(vel.x**2 + 2*self.a_g*abs(alt - burn_altitude))) / self.a_g
            t_burning = sqrt(2*abs(burn_altitude) / a_net)
            t_fall = t_to_burn + t_burning

            #required_dv = t_burning / a_eng # dv = Ve * ln(m0/mf) | dv/Ve = ln(m0/mf) | e^(dv/Ve) = m0/mf  |  m0 = e^(dv/Ve) / mf

            # Throttle Controller
            if pitch > 0:
                if not MULTILANDING and vel.y**2 + vel.z**2 > 800: # 30**2
                    dist = Vector3(self.stream_position_body()).distance(self.trajectory.land_pos)
                    print(dist)
                    self.line_land.start = self.stream_position_body()
                    self.line_land.end = tuple(self.trajectory.land_pos)
                else:
                    t_free_fall = (vel.x + sqrt(vel.x**2 + 2*self.a_g*alt)) / self.a_g
                    dist = Vector3(-alt, vel.y*t_free_fall, vel.z*t_free_fall).magnitude()
                
                #print(dist)
                
                if alt > self.final_altitude:
                    target_speed = sqrt(self.final_speed**2 + 2*a_net*abs(dist-self.final_altitude))
                    delta_speed = mag_speed - target_speed
                    #print(delta_speed)
                    throttle = (delta_speed*5 + self.a_g) / a_eng
                else: # Final Burn
                    target_dir.x *= 15
                    delta_speed = self.final_speed - vel.x
                    throttle = (delta_speed*2 + self.a_g) / a_eng

                self.vessel.control.throttle = throttle
        
                # print(f'Speed: {vel.x:.2f} | Delta: {delta_speed:.2f} | Alt: {alt:.2f} | TFall: {(t_fall):.2f}')

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

        if not MULTILANDING: self.trajectory.end()
        self.conn.close()

    def progressive_engine_cut(self):
        a_eng = (self.thrust if self.tilted_engines else self.stream_av_thrust()) / self.stream_mass()

        throttle = min((self.a_g / a_eng) *.8, 1)
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
    if MULTILANDING:
        from threading import Thread

        Thread(target=LandingBurn, args=['booster1']).start()
        Thread(target=LandingBurn, args=['booster2']).start()
    LandingBurn()