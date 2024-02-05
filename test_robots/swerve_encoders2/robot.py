#!/usr/bin/env python3
import math

import rev
import wpilib
from wpimath.filter import MedianFilter, LinearFilter
import random


class Robot(wpilib.TimedRobot):
    def robotInit(self):
        self.joystick = wpilib.Joystick(0)
        self.printing = False
        self.print_button = False

        self.ids = {'lf':0, 'rf':1, 'lb':2, 'rb':3}


        self.turning_motors = {'lf': {'id': 20, 'max': 1, 'offset': 5.252, 'analog_port': 0},
                               'rf': {'id': 24, 'max': 1, 'offset': 4.682, 'analog_port': 1},
                               'lb': {'id': 22, 'max': 1, 'offset': 4.531, 'analog_port': 2},
                               'rb': {'id': 26, 'max': 1, 'offset': 5.478, 'analog_port': 3 }}

        for (key, motor_info) in self.turning_motors.items():
            motor = rev.CANSparkMax(motor_info['id'], rev.CANSparkMax.MotorType.kBrushless)
            analog = wpilib.AnalogPotentiometer(motor_info['analog_port'], math.tau, -motor_info['offset'])
            # analog = wpilib.AnalogPotentiometer(motor_info['analog_port'], 1, 0)

            temp_dict = {
                'id': motor_info['id'],
                'max': motor_info['max'],
                'offset': motor_info['offset'],
                'motor': motor,
                'encoder': motor.getEncoder(),
                'analog_encoder': analog,
                'filter': MedianFilter(50)
            }
            self.turning_motors[key].update(temp_dict)

        wpilib.SmartDashboard.putStringArray('_ids', [key for key in self.ids.keys()])
        wpilib.SmartDashboard.putNumberArray('_offsets', [value['offset'] for value in self.turning_motors.values()])
        wpilib.SmartDashboard.putNumberArray('_raw_offsets', [value['offset']/math.tau for value in self.turning_motors.values()])
        # print(self.turning_motors)

    def teleopPeriodic(self):

        b1 = self.joystick.getRawButton(1)
        if b1 and not self.print_button:  # don't print twice in a row
            self.printing = True
            print(f'\nFiltered abs encoders taken at {wpilib.Timer.getFPGATimestamp():.1f}s')  # print a blank line
        else:
            self.printing = False

        analog_positions = [0]*4
        filtered_angles = [0]*4
        sparkmax_encoders = [0]*4

        for idx, (key, motor_info) in enumerate(self.turning_motors.items()):
            if wpilib.RobotBase.isReal():
                analog_position = -motor_info['analog_encoder'].get()  # negative because it is reversed wrt turning direction
            else:  # simulate some noise
                analog_position = random.gauss(float(motor_info['id']), .25)
            filtered_analog_position = motor_info['filter'].calculate(analog_position)  # smooth out the noise
            #wpilib.SmartDashboard.putNumber(f"{key}_abs_encoder_{motor_info['id']}", analog_position)
            #wpilib.SmartDashboard.putNumber(f"{key}_filtered_abs_{motor_info['id']}", filtered_analog_position)
            #wpilib.SmartDashboard.putNumber(f"{key}_reg_encoder_{motor_info['id']}", motor_info['encoder'].getPosition())
            #radians = analog_position - motor_info['offset']
            #wpilib.SmartDashboard.putNumber(f"{key}_abs_encoder_rad_{motor_info['id']}", radians)

            analog_positions[idx] = analog_position
            filtered_angles[idx] = filtered_analog_position
            sparkmax_encoders[idx] = motor_info['encoder'].getPosition()

            if self.printing:
                print(f"{key}_filtered_abs_{motor_info['id']} : {filtered_analog_position: .3f}")

        self.print_button = b1  # remember what it was


        wpilib.SmartDashboard.putNumberArray('analog_positions', analog_positions)
        wpilib.SmartDashboard.putNumberArray('filtered_apositions', filtered_angles)
        wpilib.SmartDashboard.putNumberArray('sparkmax_encoders', sparkmax_encoders)

if __name__ == "__main__":
    wpilib.run(Robot)