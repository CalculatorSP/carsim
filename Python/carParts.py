#!usr/bin/env python3
__author__ = 'CalculatorSP'

import math


class Environment(object):
    def __init__(self, air_density, gravity):
        self.airDensity = air_density
        self.gravity = gravity


class Frame(object):
    def __init__(self, wheelbase, weight, frontal_area, drag_coefficient, rear_weight_distr, ground_to_cg):
        self.wheelbase = wheelbase
        self.weight = weight
        self.frontalArea = frontal_area
        self.dragCoefficient = drag_coefficient
        self.rearWeightDistribution = rear_weight_distr
        self.frontToCG = rear_weight_distr * self.wheelbase
        self.groundToCG = ground_to_cg


class Engine(object):
    def __init__(self):
        self.rpm = 0.0
        self.torque = 0.0

    def update(self, vehicle):
        if vehicle.clutch.depressed == 0.0:
            self.rpm = vehicle.wheel.rpm * vehicle.gearbox.get_active_gear_ratio()
            # TODO set torque based on interpolated curve


class Gearbox(object):
    def __init__(self, gear_ratios, primary_drive, final_drive):
        self.gearRatios = gear_ratios
        self.primaryDrive = primary_drive
        self.finalDrive = final_drive
        self.activeGear = 1

    def update(self, vehicle):
        if self.gearRatios.containskey(vehicle.shifter.selected_gear):
            self.activeGear = vehicle.shifter.selected_gear

    def get_active_gear_ratio(self):
        return self.gearRatios(self.activeGear)


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
    def __init__(self, radius, static_friction):
        self.radius = radius
        self.staticFriction = static_friction
        self.rpm = 0.0

    def update(self, vehicle):
        self.rpm = vehicle.velocity / self.radius * 30.0 / math.pi


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
        self.wheel.update(self)
        self.gearbox.update(self)
        self.engine.update(self)


class Driver(object):
    def __init__(self):
        pass

    def drive_to_finish(self, vehicle, finish_line, time_step):
        while vehicle.distance < finish_line:
            # TODO Control clutch, accelerator, and shifter
            vehicle.update(time_step)
