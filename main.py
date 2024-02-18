import krpc
from time import sleep
from math import sqrt

from PyVecs import Vector3

from Trajectory import Trajectory
from DataLogger import DataLogger

MULTILANDING = False
USE_TRAJECTORY = True
LOG_DATA = True

GEAR_DELAY = 4

MAX_TWR = 4
ENG_THRESHOLD = .9

MAX_FINAL_HOR_SPEED = 8
FINAL_SPEED = -1
FINAL_ALTITUDE = 5



class LandingBurn:
    def __init__(self, vessel_tag=None, land_func=None):
        self.conn = krpc.connect("LandingBurn" + ((": " + vessel_tag) if vessel_tag else ""))
        self.space_center = self.conn.space_center
        self.vessel = self.space_center.active_vessel
        if vessel_tag:
            for vessel in self.space_center.vessels:
                if len(vessel.parts.with_tag(vessel_tag)) > 0:
                    self.vessel = vessel
                    break
        self.control = self.vessel.control
        self.auto_pilot = self.vessel.auto_pilot
        self.body = self.vessel.orbit.body

        self.body_ref = self.body.reference_frame
        self.surface_ref = self.vessel.surface_reference_frame
        self.hybrid_ref = self.space_center.ReferenceFrame.create_hybrid(position=self.body_ref, rotation=self.surface_ref)

        self.flight_body = self.vessel.flight(self.body_ref)
        self.flight_surface = self.vessel.flight(self.surface_ref)

        # Streams
        self.stream_mass = self.conn.add_stream(getattr, self.vessel, "mass")
        self.stream_av_thrust = self.conn.add_stream(getattr, self.vessel, "available_thrust")
        self.stream_vel = self.conn.add_stream(self.vessel.velocity, self.hybrid_ref)
        self.stream_position_body = self.conn.add_stream(self.vessel.position, self.body_ref)
        self.stream_surface_altitude = self.conn.add_stream(getattr, self.flight_body, "surface_altitude")
        self.stream_bbox = self.conn.add_stream(self.vessel.bounding_box, self.surface_ref)
        self.stream_pitch = self.conn.add_stream(getattr, self.flight_surface, "pitch")
        self.stream_situation = self.conn.add_stream(getattr, self.vessel, "situation")
        if LOG_DATA: self.stream_ut = self.conn.add_stream(getattr, self.space_center, "ut")

        # ===================
        
        if land_func is None:
            self.land_func = lambda self: (
                self.end_modules(),
                self.progressive_engine_cut(),
                self.sas_aim_up(),
                self.finish())
        else:
            self.land_func = land_func

        # Thrust correction
        self.thrust_correction = min(1, sum([eng.available_thrust * eng.part.direction(self.vessel.reference_frame)[1] for eng in self.vessel.parts.engines if (eng.active and eng.has_fuel)]) / self.stream_av_thrust())

        # Consts
        self.a_g = self.body.surface_gravity
        self.landed_situation = self.vessel.situation.landed
        self.splashed_situation = self.vessel.situation.splashed
        self.vf_2 = FINAL_SPEED*abs(FINAL_SPEED)

        # Initializing
        self.control.throttle = 0
        self.control.brakes = True
        self.control.rcs = True
        self.auto_pilot.engage()
        self.auto_pilot.reference_frame = self.body_ref
        self.auto_pilot.stopping_time = (0.5, 0.5, 0.5)
        self.auto_pilot.deceleration_time = (5, 5, 5)
        self.auto_pilot.target_roll = -90

        # Waiting
        while self.stream_vel()[0] > 0:
            self.auto_pilot.target_direction = self.space_center.transform_direction((1, 0, 0), self.surface_ref, self.body_ref)
            sleep(0.5)
        
        while self.stream_surface_altitude() > 15000:
            self.auto_pilot.target_direction = Vector3(self.flight_body.velocity) * -1
            sleep(0.5)

        # Trajectory
        vessel_radius = 1.5
        cd = 0.4
        if USE_TRAJECTORY:
            self.trajectory = Trajectory(vessel_radius, cd)
            self.trajectory.start()
            
        # Data Logger
        if LOG_DATA:
            self.data_log = DataLogger()
            self.data_log.ut0 = self.stream_ut()

        try:
            while True:
                # Get Streams
                vel = Vector3(self.stream_vel())
                alt = self.get_altitude()
                mass = self.stream_mass()
                av_thrust = self.stream_av_thrust() * self.thrust_correction
                pitch = self.stream_pitch()

                hor_speed = sqrt(vel.y**2 + vel.z**2)
                mag_speed = vel.magnitude()
                delta_speed = 0
                
                a_eng = av_thrust / mass
                a_eng_l = a_eng * ENG_THRESHOLD * min(MAX_TWR * self.a_g / a_eng, 1)

                a_net = max(a_eng_l - self.a_g, .1) # used to calculate v_target
                
                target_dir = -vel
                target_dir.x = abs(target_dir.x)

                burn_altitude = abs((mag_speed**2 + self.vf_2 + 2*self.a_g*alt) / (2 * a_eng_l))

                v_2 = vel.x*abs(vel.x) # auxiliar math variable
                t_to_burn = (vel.x + sqrt(max(0, 2*self.a_g*(alt - burn_altitude) - v_2))) / self.a_g
                t_burning = sqrt(2 * burn_altitude / a_net)
                #t_hovering = min(FINAL_ALTITUDE, alt) / abs(FINAL_SPEED)
                t_fall = t_to_burn + t_burning# + t_hovering

                #required_dv = t_burning * a_eng + t_hovering * self.a_g

                #print(f'TF: {t_fall:.1f}' | TB: {t_burning:.1f} | TTB: {t_to_burn:.1f} | TH: {t_hovering:.1f}')
                
                # Throttle Controller
                if pitch > 0:
                    if USE_TRAJECTORY and hor_speed > 30:
                        dist = Vector3(self.stream_position_body()).distance(self.trajectory.land_pos)
                    else:
                        t_free_fall = (vel.x + sqrt(max(0, 2*self.a_g*alt - v_2))) / self.a_g
                        dist = Vector3(-alt, vel.y*t_free_fall, vel.z*t_free_fall).magnitude()
                        if USE_TRAJECTORY: self.trajectory.end()

                    if alt > FINAL_ALTITUDE:
                        _ = 2*a_net*abs(dist-FINAL_ALTITUDE) - self.vf_2
                        target_speed = sqrt(_) if _ > 0 else 0
                        delta_speed = mag_speed - target_speed
                        throttle = (delta_speed*5 + self.a_g) / a_eng
                    else: # Final Burn
                        if hor_speed > MAX_FINAL_HOR_SPEED:
                            target_dir.x *= 8
                            delta_h = FINAL_ALTITUDE - alt
                            v_target = sqrt(2*self.a_g*abs(delta_h)) * (delta_h / abs(delta_h))
                        else:
                            target_dir.x *= 10
                            v_target = FINAL_SPEED

                        delta_speed = v_target - vel.x
                        throttle = (delta_speed*2 + self.a_g) / a_eng

                        # Check Landed
                        situation = self.stream_situation()
                        if situation == self.landed_situation or situation == self.splashed_situation:
                            self.land_func(self)
                            break

                    self.control.throttle = throttle
            
                    #print(f'Alt: {alt:.2f} | Delta: {delta_speed:.2f} | TFall: {t_fall:.2f}')

                    # Check Gears
                    if t_fall < GEAR_DELAY: self.control.gear = True
                else:
                    self.control.throttle = 0

                # Aim Vessel
                self.auto_pilot.target_direction = self.space_center.transform_direction(target_dir, self.surface_ref, self.body_ref)
        
                # Log Data
                if LOG_DATA:
                    self.data_log.log(self.stream_ut(), dist, alt, vel.x, hor_speed, delta_speed, pitch)

                sleep(0.05)

        except KeyboardInterrupt as e:
            self.end_modules()

    def end_modules(self):
        if USE_TRAJECTORY: self.trajectory.end()
        if LOG_DATA: self.data_log.close()

    def progressive_engine_cut(self):
        a_eng = self.stream_av_thrust() * self.thrust_correction / self.stream_mass()

        throttle = min((self.a_g / a_eng) *.8, 1)
        total_time = 1
        dt = 0.1
        
        delta_throttle = (throttle * dt) / total_time

        while throttle > 0:
            throttle -= delta_throttle
            self.control.throttle = throttle
            sleep(dt)

        self.control.throttle = 0

    def sas_aim_up(self):
        self.auto_pilot.disengage()
        self.control.rcs = True
        self.control.sas = True
        sleep(0.1)
        try:
            self.control.sas_mode = self.control.sas_mode.radial
        except:
            self.control.sas = False
            self.auto_pilot.engage()
            self.auto_pilot.target_direction = self.space_center.transform_direction((1, 0, 0), self.surface_ref, self.body_ref)
            input("Press enter to finish program.")
            self.auto_pilot.disengage()

    def finish(self):
        self.control.rcs = True
        self.control.brakes = False
        print(f"{self.vessel.name} has landed!")
        self.conn.close()

    def get_altitude(self):
        return max(0, self.stream_surface_altitude() + self.stream_bbox()[0][0])
    

if __name__ == "__main__":
    if MULTILANDING:
        USE_TRAJECTORY = False
        LOG_DATA = False
        
        from threading import Thread
        Thread(target=LandingBurn, args=["sb1"]).start()
        Thread(target=LandingBurn, args=["sb2"]).start()

    LandingBurn()
