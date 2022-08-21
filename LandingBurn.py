import krpc
import numpy as np
from time import perf_counter, sleep

class LandingBurn:
    def __init__(self, vessel=None, end_function=None):
        self.conn = krpc.connect('LandingBurn')
        self.space_center = self.conn.space_center
        self.vessel = self.space_center.active_vessel if vessel == None else vessel
        self.body = self.vessel.orbit.body
        self.orbit_ref = self.body.reference_frame
        self.surface_ref = self.vessel.surface_reference_frame
        self.flight = self.vessel.flight(self.orbit_ref)

        # Streams
        self.velocity = self.conn.add_stream(getattr, self.flight, "velocity")
        self.vertical_speed = self.conn.add_stream(getattr, self.flight, "vertical_speed")
        self.mag_speed = self.conn.add_stream(getattr, self.flight, "speed")
        self.surface_altitude = self.conn.add_stream(getattr, self.flight, "surface_altitude")
        self.pitch = self.conn.add_stream(getattr, self.vessel.flight(self.surface_ref), "pitch")

        # Propriedade Computacional
        self.final_speed = -2
        self.hover_altitude = 5
        self.final_burn = False
        self.dt = 0.1
        self.S = np.array([0])
        self.V = np.array([0])

        # Initializing
        self.vessel.control.throttle = 0
        self.vessel.control.brakes = True
        self.vessel.control.rcs = True
        self.vessel.auto_pilot.engage()
        self.vessel.auto_pilot.target_roll = -90
        self.vessel.auto_pilot.reference_frame = self.orbit_ref

        while self.vertical_speed() > 0 or self.altitude() > 8000: # WAIT
            self.aim_vessel(self.vertical_speed())

        # Propriedades do Corpo
        self.rb = self.body.equatorial_radius
        self.gm = self.body.gravitational_parameter
        self.atm = self.body.has_atmosphere
        self.surface_gravity = self.body.surface_gravity

        # Propriedade do Foguete
        self.Cd = 0.47
        self.r = self.rocket_radius()
        self.A = np.pi * self.r**2
        self.gears_delay = 4 /2
        self.m0 = self.vessel.mass

        # Thrust de acordo com ângulo de montagem do motor
        thrust = 0
        for engine in self.vessel.parts.engines:
            if engine.active:
                thrust += engine.available_thrust * engine.part.direction(self.vessel.reference_frame)[1]

        self.aeng0 = (thrust/self.m0)
        if self.surface_gravity+2 > self.aeng0:
            self.aeng0 = self.surface_gravity+2
            print('Motor muito fraco! -> Mínimo TEP aplicado')
        elif self.aeng0 > self.surface_gravity*4:
            self.aeng0 = self.surface_gravity*4
            print('Motor muito forte! -> Limite de TEP aplicado')


        print(f'Raio do foguete: {self.r:.2f} metros')
        print('Iniciando Cálculos...')
        alt_final = self.altitude()
        t0 = perf_counter()
        while self.S[-1] < alt_final: # Talvez usar Runge Kutta
            self.V = np.append(self.V, self.V[-1] - self.dVdt())
            self.S = np.append(self.S, self.S[-1] - self.V[-1] * self.dt)
            #print(f'{min((S[-1]/alt_final)*100, 100):.0f}%')
        print(f'Cálculos concluídos em: {perf_counter() - t0:.1f} segundos')

        while True:
            sleep(0.01)
            if self.vessel.situation == self.vessel.situation.landed or self.vessel.situation == self.vessel.situation.splashed:
                self.vessel.control.throttle = 0
                self.vessel.control.brakes = False
                print(f'{self.vessel.name} Pousou!')
                self.vessel.auto_pilot.disengage()
                self.vessel.control.sas = True
                sleep(0.1)
                try:
                    self.vessel.control.sas_mode = self.vessel.control.sas_mode.radial
                except:
                    self.vessel.auto_pilot.engage()
                    self.vessel.auto_pilot.target_direction = self.space_center.transform_direction((1, 0, 0), self.surface_ref, self.orbit_ref)
                    self.vessel.control.rcs = True
                    sleep(4)
                    self.vessel.control.rcs = False
                    self.vessel.auto_pilot.disengage()
                break
            
            vert_speed = self.vertical_speed()
            aeng = self.vessel.available_thrust/self.vessel.mass
            pitch = self.pitch()

            self.aim_vessel(vert_speed)

            if vert_speed < 0 and pitch > 0:
                if self.final_burn:
                    self.vessel.gear = True
                    self.vessel.control.throttle = (self.surface_gravity + (self.final_speed - vert_speed)*2) / (aeng * np.sin(np.radians(pitch)))
                else:
                    alt = self.altitude()

                    if alt <= self.hover_altitude+2:
                        self.final_burn = True
                    elif self.time_fall(-.5 * self.surface_gravity, vert_speed, alt) <= self.gears_delay:
                        self.vessel.control.gear = True

                    target_speed = np.interp(alt-self.hover_altitude, self.S, self.V)
                    delta_speed = target_speed + self.mag_speed()

                    self.vessel.control.throttle = (self.surface_gravity + delta_speed*10) / (aeng * np.sin(np.radians(pitch)))
                    #print(f'{delta_speed:.2f}')
            else:
                self.vessel.control.throttle = 0
        
        if end_function is not None:
            end_function()


    def rocket_radius(self):
        size = 0.5
        for fuel in self.vessel.resources.with_resource('LiquidFuel'):
            size = max(0.5, fuel.part.bounding_box(self.vessel.reference_frame)[1][0] - fuel.part.bounding_box(self.vessel.reference_frame)[0][0])
        return abs(size)/2

    def time_fall(self, a, v, h):
        d = np.sqrt((v * v) - 4 * a * h)
        result_1 = (-v + d) / (2 * a)
        result_2 = (-v - d) / (2 * a)
        return max(result_1, result_2)

    def aim_vessel(self, v_speed):
        if v_speed < 0:
            vel = self.space_center.transform_direction(self.velocity(), self.orbit_ref, self.surface_ref)
            target_pitch = ((10 if self.final_burn else 1) * -vel[0], -vel[1], -vel[2])
        else:
            target_pitch = (1, 0, 0)

        self.vessel.auto_pilot.target_direction = self.space_center.transform_direction(target_pitch, self.surface_ref, self.orbit_ref)

    def altitude(self):
        return max(0, self.surface_altitude() + self.vessel.bounding_box(self.surface_ref)[0][0])

    def ag(self, alt):
        return self.gm / (self.rb+alt)**2

    def dVdt(self):
        accel_net = self.aeng0 - self.ag(self.S[-1])
        if self.atm: 
            accel_net += (self.body.density_at(float(self.S[-1]))*self.Cd*self.A*self.V[-1]**2) / (2*self.m0)
        return accel_net * self.dt
    

if __name__ == '__main__':
    LandingBurn()