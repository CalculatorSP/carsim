#!usr/bin/env python3
__author__ = 'CalculatorSP'

from carParts import *


# Wheel Parameters
RADIUS = 5.0 / 12
STATIC_FRICTION = 1.0

# Frame Parameters
WHEELBASE = 71.26 / 12
WEIGHT = 600.0
FRONTAL_AREA = 13.8
DRAG_COEFFICIENT = 0.5
REAR_WEIGHT_DISTR = 0.6
GROUND_TO_CG = 13.0 / 12

# Gearbox Parameters
GEAR_RATIOS = {
    1: 2.75,
    2: 2.0,
    3: 1.667,
    4: 1.444,
    5: 1.304,
    6: 1.208
}
PRIMARY_DRIVE = 2.111
FINAL_DRIVE = 2.0

# Environment Parameters
AIR_DENSITY = 0.00236
GRAVITY = 32.2

# Other Parameters
TIME_STEP = 0.001
SIM_TIME = 8.0
FINISH_LINE = 256.063


def main():
    environment = Environment(AIR_DENSITY, GRAVITY)
    frame = Frame(WHEELBASE, WEIGHT, FRONTAL_AREA, DRAG_COEFFICIENT, REAR_WEIGHT_DISTR, GROUND_TO_CG)
    engine = Engine()
    gearbox = Gearbox(GEAR_RATIOS, PRIMARY_DRIVE, FINAL_DRIVE)
    shifter = Shifter(len(GEAR_RATIOS))
    clutch = Clutch()
    accelerator = Accelerator()
    wheel = Wheel(RADIUS, STATIC_FRICTION)

    car = Vehicle(environment, frame, engine, gearbox, shifter, clutch, accelerator, wheel)
    driver = Driver()
    driver.drive_to_finish(car, FINISH_LINE, TIME_STEP)


if __name__ == "__main__":
    main()