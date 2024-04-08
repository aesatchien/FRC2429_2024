import random

import commands2
import wpilib
from wpilib import SmartDashboard
import rev
import pickle as pkl
import random


class CANStatus(commands2.Command):  # change the name for your command

    def __init__(self, container, ) -> None:
        super().__init__()
        self.setName('CANStatusCheck')
        self.container = container
        #self.addRequirements()  # commandsv2 version of requirements

        self.can_ids = {3: {'name': 'climber', 'motor': self.container.climber.right_winch},
                        4: {'name': 'climber', 'motor': self.container.climber.left_winch},
                        5: {'name': 'intake', 'motor': self.container.intake.motor},
                        7: {'name': 'crank_arm', 'motor': self.container.crank_arm.motor},
                        8: {'name': 'shooter_arm', 'motor': self.container.shooter_arm.motor},
                        9: {'name': 'shooter_arm', 'motor': self.container.shooter_arm.follower},
                        10: {'name': 'shooter', 'motor': self.container.shooter.flywheel_lower_left},
                        11: {'name': 'shooter', 'motor': self.container.shooter.flywheel_upper_left},
                        12: {'name': 'indexer', 'motor': self.container.indexer.motor},
                        20: {'name': 'turn',  'motor': self.container.drive.swerve_modules[0].turningSparkMax},
                        22: {'name': 'turn', 'motor': self.container.drive.swerve_modules[1].turningSparkMax},
                        24: {'name': 'turn', 'motor': self.container.drive.swerve_modules[2].turningSparkMax},
                        26: {'name': 'turn', 'motor': self.container.drive.swerve_modules[3].turningSparkMax},
                        21: {'name': 'drive', 'motor': self.container.drive.swerve_modules[0].drivingSparkMax},
                        23: {'name': 'drive', 'motor': self.container.drive.swerve_modules[1].drivingSparkMax},
                        25: {'name': 'drive', 'motor': self.container.drive.swerve_modules[2].drivingSparkMax},
                        27: {'name': 'drive', 'motor': self.container.drive.swerve_modules[3].drivingSparkMax}
                        }
        self.fault_ids = {0:'kBrownout', 1:'kOvercurrent', 2:'kIWDTReset', 3:'kMotorFault', 4:'kSensorFault',
                          5:'kStall', 6: 'kEEPROMCRC', 7: 'kCANTX', 8: 'kCANRX', 9: 'kHasReset',
                          10: 'kDRVFault', 11: 'kOtherFault', 12: 'kSoftLimitFwd', 13: 'kSoftLimitRev',
                            14:'kHardLimitFwd', 15:'kHardLimitRev'}

        self.write_log = False

    def runsWhenDisabled(self) -> bool:
        return True

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time:.1f} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.1f} s **")

        for key in self.can_ids.keys():
            motor: rev.CANSparkBase =  self.can_ids[key]['motor']

            sticky_faults = motor.getStickyFaults()  # will be an integer representing the bit mask of the faults
            if wpilib.RobotBase.isSimulation():
                sticky_faults = random.randint(0,2048)
            active_faults = motor.getFaults()  # TODO - parse these too
            motor.clearFaults()  # apparently does not clear sticky faults, active only

            set_bits = []
            binary_string = bin(sticky_faults)[2:]  # convert to binary and ignore the initial two 0b characters
            for i, bit in enumerate(reversed(binary_string), start=0):
                # Check if the bit is set (i.e., equals '1')
                if bit == '1':
                    # Add the position (0-based indexing) of the set bit to the list
                    set_bits.append(i)

            fault_codes = [self.fault_ids[id] for id in set_bits]
            self.can_ids[key].update({'sticky_faults': sticky_faults})
            self.can_ids[key].update({'set_bits': set_bits})
            self.can_ids[key].update({'fault_codes': fault_codes})
            print(f"CANID {key:02d}: {self.can_ids[key]['name']:13} sticky_faults: {sticky_faults} {set_bits} {fault_codes}")

        if self.write_log:
            try:
                with open('can.pkl', 'rb') as file:
                    records = pkl.load(file)
            except FileNotFoundError:  # start with a header row
                header = [f"{key} {self.can_ids[key]['name']}" for key in self.can_ids.keys()]
                records = [header]

            output = {}
            for key in self.can_ids: # can't pickle a motor
                output.update({key:{'NAME': self.can_ids[key]['name'], 'STICKY_FAULTS': self.can_ids[key]['sticky_faults'],
                                    'SET_BITS': self.can_ids[key]['set_bits'], 'FAULT_CODES': self.can_ids[key]['fault_codes']}})

            just_faults = [self.can_ids[key]['fault_codes'] for key in self.can_ids.keys()]

            records.append(just_faults)

            with open('can.pkl', 'wb') as file:
                pkl.dump(records, file)
            print('Wrote CAN faults to can.pkl')

            # to read, just do this script
            # import pickle as pkl
            # import pandas as pd
            # with open('can.pkl', 'rb') as file:
            #     data = pkl.load(file)
            # column_names = data[0]
            # df = pd.DataFrame(data[1:], columns=column_names)
            # df

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print_message = True
        if print_message:
            print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
            SmartDashboard.putString(f"alert",
                                     f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")