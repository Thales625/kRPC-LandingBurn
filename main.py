import krpc
from time import sleep
from math import sqrt
from os import system
from PyVecs import Vector3

from utils import sqrt_, sign, smooth_func
from trajectory          import Trajectory
from phase_controller    import PhaseControl
from landing_calculator  import LandingCalculator
from throttle_controller import ThrottleControl

from config import *

class LandingBurn:
    def __init__(self, vessel_tag=None):
        self.conn = krpc.connect("LandingBurn" + ((": " + vessel_tag) if vessel_tag else ""))
        self.space_center = self.conn.space_center
        self.vessel = self.space_center.active_vessel
        if vessel_tag:
            print("MultiLandingBurn WIP")
            exit()
            for vessel in self.space_center.vessels:
                if len(vessel.parts.with_tag(vessel_tag)) > 0:
                    self.vessel = vessel
                    break
        self.body = self.vessel.orbit.body
        self.control = self.vessel.control
        self.auto_pilot = self.vessel.auto_pilot

        self.vessel_ref  = self.vessel.reference_frame
        self.surface_ref = self.vessel.surface_reference_frame
        self.body_ref    = self.body.reference_frame
        self.hybrid_ref  = self.space_center.ReferenceFrame.create_hybrid(position=self.body_ref, rotation=self.surface_ref)

        # Streams
        self.stream_mass             = self.conn.add_stream(getattr, self.vessel, "mass")
        self.stream_av_thrust        = self.conn.add_stream(getattr, self.vessel, "available_thrust")
        self.stream_situation        = self.conn.add_stream(getattr, self.vessel, "situation")
        self.stream_ut               = self.conn.add_stream(getattr, self.space_center, "ut")
        self.stream_surface_altitude = self.conn.add_stream(getattr, self.vessel.flight(self.body_ref), "surface_altitude")
        self.stream_vel              = self.conn.add_stream(self.vessel.velocity, self.hybrid_ref)
        self.stream_position_body    = self.conn.add_stream(self.vessel.position, self.body_ref)
        self.stream_bbox             = self.conn.add_stream(self.vessel.bounding_box, self.surface_ref)

        # ===================
        
        # Params
        self.max_twr          = 5 if self.body.has_atmosphere else 10
        self.thrust_threshold = 1 if self.body.has_atmosphere else 0.98

        # Drag
        self.k = DRAG_AREA * DRAG_COEFFICIENT

        # Consts
        self.a_g = self.body.surface_gravity
        self.landed_situation = self.vessel.situation.landed
        self.splashed_situation = self.vessel.situation.splashed

        self.const_g_2   = self.a_g * 2
        self.const_g_inv = 1 / self.a_g

        # Initializing
        self.control.throttle = 0
        self.control.brakes = True
        self.control.rcs = True

        # AutoPilot
        self.auto_pilot.engage()
        self.auto_pilot.reference_frame = self.body_ref
        self.auto_pilot.stopping_time = (0.5, 0.5, 0.5)
        self.auto_pilot.deceleration_time = (5, 5, 5)
        self.auto_pilot.target_roll = -90

        # Calculator
        self.land_calculator = LandingCalculator(FINAL_SPEED, self.body, self.vessel.available_thrust_at, self.k)

        # Throttle controller
        self.throttle_controller = ThrottleControl(self.stream_ut)
        self.throttle_controller.pi.adjust_gains(5, 15)

        # Phase Controller
        self.phase_controller = PhaseControl([
            (self.waiting,      None),
            (self.coasting,     self.coasting_transition),
            (self.landing_burn, self.landing_burn_transition),
            (self.final_burn,   None),
            (self.touch_down,   None)
        ])

        # Trajectory
        if USE_TRAJECTORY:
            self.trajectory_min_vel_hor = 10
            self.trajectory = Trajectory(draw_trajectory=DRAW_TRAJECTORY)
            self.trajectory.step_size = 2

        # Thrust correction
        self.thrust_correction = abs(min(1, sum([eng.available_thrust * eng.part.direction(self.vessel_ref)[1] for eng in self.vessel.parts.engines if (eng.active and eng.has_fuel)]) / self.stream_av_thrust()))

        # Variables - Inputs
        self._throttle = 0
        self._aim_direction = Vector3(1, 0, 0)

        try:
            # Main loop
            while True:
                self.phase_controller.loop()

                self.control.throttle = self._throttle
                self.auto_pilot.target_direction = self.space_center.transform_direction(self._aim_direction, self.surface_ref, self.body_ref)

                sleep(INTERVAL)

        except KeyboardInterrupt as e:
            self.end_modules()

    def waiting(self):
        system(CLEAR_CMD)
        print("WAITING")

        if self.stream_vel()[0] > 0: # Going up
            self._aim_direction = (1, 0, 0)
            sleep(0.5)
            return
        
        self.phase_controller.next_phase()

    def coasting(self):
        system(CLEAR_CMD)
        print("COASTING")

        self._aim_direction = -Vector3(self.stream_vel())

        # END CHECK
        alt = self.stream_surface_altitude()
        if self.body.has_atmosphere:
            if alt < 0.5 * self.body.atmosphere_depth:
                self.phase_controller.next_phase()
            return
        
        if alt < 10000:
            self.phase_controller.next_phase()

    def coasting_transition(self):
        # Calculate velocity profile
        if USE_TRAJECTORY:
            self.trajectory.start(self.k)
            while not self.trajectory.CALCULATED:
                sleep(0.2)
            height = Vector3(self.vessel.position(self.body_ref)).distance(self.trajectory.land_pos)
        else:
            height = self.get_altitude()

        vel_mag = Vector3(self.stream_vel()).magnitude()
        
        thrust_multiplier = self.thrust_correction * self.thrust_threshold * max(0.8, 0.95 ** (len([eng for eng in self.vessel.parts.engines if eng.active]) - 1)) * min(self.max_twr * self.a_g / (self.stream_av_thrust() * self.thrust_correction/self.vessel.mass), 1)

        burn_height = 0.5 * (vel_mag**2 + self.const_g_2*height) / (self.stream_av_thrust() * thrust_multiplier / self.vessel.mass)

        if burn_height < 100: thrust_multiplier = min(0.6, thrust_multiplier)

        print("Burn height:", burn_height)
        print("Thrust Multiplier", thrust_multiplier)

        self.land_calculator.thrust_multiplier = thrust_multiplier
        self.land_calculator.calculate(round(min(height, burn_height + 500), 1), self.vessel.mass)

    def landing_burn(self):
        vel = Vector3(self.stream_vel())
        alt = self.get_altitude()

        hor_speed = sqrt(vel.y**2 + vel.z**2)
        mag_speed = vel.magnitude()

        v_x_2 = vel.x*abs(vel.x)

        # Trajectory 
        if USE_TRAJECTORY and self.trajectory.RUNNING:
            if hor_speed > self.trajectory_min_vel_hor:
                dist = Vector3(self.stream_position_body()).distance(self.trajectory.land_pos)
            else:
                self.trajectory.end()

                t_free_fall = (vel.x + sqrt_(self.const_g_2*alt - v_x_2)) / self.a_g
                dist = Vector3(alt, vel.y*t_free_fall, vel.z*t_free_fall).magnitude()
        else:
            t_free_fall = (vel.x + sqrt_(self.const_g_2*alt - v_x_2)) / self.a_g
            dist = Vector3(alt, vel.y*t_free_fall, vel.z*t_free_fall).magnitude()

        thrust = self.stream_av_thrust() * self.thrust_correction
        mass = self.stream_mass()
        a_eng = thrust / mass
        
        target_speed = self.land_calculator.get_v_target(dist - FINAL_ALTITUDE)
        delta_speed = target_speed + mag_speed
                
        self._throttle = self.throttle_controller.pid_control(delta_speed, a_eng)
        self._aim_direction = -vel

        burn_dist = 0.5 * (mag_speed**2 + self.const_g_2*dist) / a_eng
        t_to_burn = self.const_g_inv * (vel.x + sqrt_(self.const_g_2 * (dist - burn_dist) - (vel.x*abs(vel.x))))
        t_burning = sqrt(2*burn_dist / (a_eng - self.a_g))

        t_fall = t_to_burn + t_burning

        if t_fall <= GEAR_DELAY: self.control.gear = True

        system(CLEAR_CMD)
        print(f"_LANDING BURN_\nH: {dist:.2f}\nΔv: {delta_speed:.2f}\nT to burn: {t_to_burn:.2f}")

        # END CHECK
        if alt <= FINAL_ALTITUDE + 2:
            self.phase_controller.next_phase()

    def landing_burn_transition(self):
        if USE_TRAJECTORY:
            self.trajectory.end()

        self.control.gear = True

        # Close streams
        self.stream_ut.remove()
        self.stream_position_body.remove()
    
    def final_burn(self):
        vel = Vector3(self.stream_vel())
        alt = self.get_altitude()

        # AIM CONTROL
        target_dir = -vel
        target_dir.x = abs(target_dir.x) * 10
        self._aim_direction = target_dir

        # THROTTLE CONTROL
        hor_speed = sqrt(vel.y**2 + vel.z**2)
        if hor_speed > MAX_HOR_SPEED:
            delta_h = FINAL_ALTITUDE - alt
            v_target = sqrt(2*self.a_g*abs(delta_h)) * sign(delta_h)
        else:
            v_target = FINAL_SPEED

        delta_speed = v_target - vel.x

        a_eng = self.stream_av_thrust() * self.thrust_correction / self.stream_mass()

        self._throttle = self.throttle_controller.linear_control(delta_speed, a_eng, self.a_g, 2)

        system(CLEAR_CMD)
        print(f"FINAL BURN\nH: {alt:.2f}\nΔv: {delta_speed:.2f}")

        # END CHECK
        situation = self.stream_situation()
        if situation == self.landed_situation or situation == self.splashed_situation:
            self.phase_controller.next_phase()

    def touch_down(self):
        system(CLEAR_CMD)
        print("Touchdown!")

        self.progressive_engine_cut()

        self.control.brakes = False

        print(f"{self.vessel.name} has landed!")

        self.sas_aim_up()

        self.conn.close()

        exit()

    def progressive_engine_cut(self):
        # Check if rcs thrust is greather than
        self.control.rcs = True
        if self.vessel.available_rcs_force[0][1] / (self.a_g * self.stream_mass()) > 0.9:
            self.control.rcs = False

        # Engine cut-off
        smooth_func(self.control.throttle, 0, lambda v: setattr(self.control, "throttle", v), 1.0, 20)

    def sas_aim_up(self):
        self.auto_pilot.disengage()
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

    def end_modules(self):
        if USE_TRAJECTORY:
            self.trajectory.end()

    def get_altitude(self):
        return max(0, self.stream_surface_altitude() + self.stream_bbox()[0][0])
    
if __name__ == "__main__":
    LandingBurn()