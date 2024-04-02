import commands2
from wpilib import SmartDashboard
import rev




class CANStatus(commands2.Command):  # change the name for your command

    def __init__(self, container, ) -> None:
        super().__init__()
        self.setName('CANStatusCheck')  # change this to something appropriate for this command
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

    def runsWhenDisabled(self) -> bool:
        return True

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

        print('Need to parse the sticky fault bits ...')
        for key in self.can_ids.keys():
            motor: rev.CANSparkBase =  self.can_ids[key]['motor']
            sticky_faults = motor.getStickyFaults()
            self.can_ids[key].update({'sticky_faults': sticky_faults})
            print(f"CANID {key:02d}: {self.can_ids[key]['name']:13} sticky_faults: {sticky_faults} ")

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