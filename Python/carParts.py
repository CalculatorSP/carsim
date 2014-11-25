#!usr/bin/env python3
__author__ = 'CalculatorSP'

import math


class Environment(object):
    def __init__(self, air_density, gravity):
        self.airDensity = air_density
        self.gravity = gravity


class Frame(object):
    def __init__(self, wheelbase, mass, frontal_area, drag_coefficient, rear_weight_distr, height):
        self.wheelbase = wheelbase
        self.mass = mass
        self.frontalArea = frontal_area
        self.dragCoefficient = drag_coefficient
        self.staticRearWeightDistr = rear_weight_distr
        self.height = height
        self.rearNormal = 0.0
        self.dragForce = 0.0

    def update(self, vehicle):
        self.rearNormal = self.mass * self.staticRearWeightDistr * vehicle.environment.gravity
        self.rearNormal += self.mass * vehicle.acceleration * self.height / self.wheelbase
        self.dragForce = 1 / 2 * vehicle.environment.airDensity * self.dragCoefficient * self.frontalArea
        self.dragForce *= vehicle.velocity * vehicle.velocity


class Engine(object):
    def __init__(self):
        self.rpm = 0.0
        self.torque = 0.0

    def update(self, vehicle):
        if vehicle.clutch.depressed == 0.0 and vehicle.accelerator.depressed == 1.0:
            self.rpm = vehicle.wheel.rpm * vehicle.gearbox.activeGearRatio
            # TODO set torque based on interpolated curve


class Gearbox(object):
    def __init__(self, gear_ratios, primary_drive, final_drive):
        self.gearRatios = gear_ratios
        self.primaryDrive = primary_drive
        self.finalDrive = final_drive
        self.activeGear = 1
        self.activeGearRatio = 0.0

    def update(self, vehicle):
        if self.gearRatios.containskey(vehicle.shifter.selected_gear):
            self.activeGear = vehicle.shifter.selected_gear
            self.activeGearRatio = self.gearRatios(self.activeGear)
            self.activeGearRatio *= self.primaryDrive * self.finalDrive


class Shifter(object):
    def __init__(self, num_gears):
        self.num_gears = num_gears
        self.selected_gear = 1

    def shift_up(self):
        self.selected_gear += 1
        if self.selected_gear > self.num_gears:
            self.selected_gear = self.num_gears

    def shift_down(self):
        self.selected_gear -= 1
        if self.selected_gear < 1:
            self.selected_gear = 1

    def set_gear(self, gear):
        if 1 <= gear <= self.num_gears:
            self.selected_gear = gear


class Clutch(object):
    def __init__(self):
        self.depressed = 0.0


class Accelerator(object):
    def __init__(self):
        self.depressed = 0.0


class Wheel(object):
    def __init__(self, radius, static_friction, kinetic_friction):
        self.radius = radius
        self.staticFriction = static_friction
        self.kineticFriction = kinetic_friction
        self.rpm = 0.0
        self.torque = 0.0
        self.force = 0.0
        self.maxStaticFriction = 0.0

    def update_rpm(self, vehicle):
        self.rpm = vehicle.velocity / self.radius * 30.0 / math.pi

    def update_torque(self, vehicle):
        self.torque = vehicle.engine.torque * vehicle.gearbox.activeGearRatio
        self.force = self.torque / self.radius
        self.maxStaticFriction = self.staticFriction * vehicle.frame.rearNormal
        if self.force > self.maxStaticFriction:
            self.force = self.kineticFriction * vehicle.frame.rearNormal


class Vehicle(object):
    def __init__(self, environment, frame, engine, gearbox, shifter, clutch, accelerator, wheel):
        self.environment = environment
        self.frame = frame
        self.engine = engine
        self.gearbox = gearbox
        self.shifter = shifter
        self.clutch = clutch
        self.accelerator = accelerator
        self.wheel = wheel
        self.distance = 0.0
        self.velocity = 0.0
        self.acceleration = 0.0

    def update(self, time_step):
        self.gearbox.update(self)
        self.frame.update(self)
        self.wheel.update_rpm(self)
        self.engine.update(self)
        self.wheel.update_torque(self)
        self.acceleration = (self.wheel.force - self.frame.dragForce) / self.frame.mass
        newVelocity = self.velocity + self.acceleration
        self.distance += (self.velocity + newVelocity) / 2.0 * time_step
        self.velocity = newVelocity


class Driver(object):
    def __init__(self):
        pass

    def drive_to_finish(self, vehicle, finish_line, time_step):
        totalTime = 0.0
        while vehicle.distance < finish_line:
            totalTime += time_step
            # TODO Control clutch, accelerator, and shifter
            vehicle.update(time_step)

        return totalTime
