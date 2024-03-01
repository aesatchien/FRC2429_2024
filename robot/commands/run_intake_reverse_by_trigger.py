import commands2
from wpilib import SmartDashboard

from commands2.button import CommandXboxController

from subsystems.intake import Intake
from subsystems.indexer import Indexer
from subsystems.shooter import Shooter


class RunIntakeReverseByTrigger(commands2.CommandBase):  # change the name for your command
    # When testing, make sure that an incoming shooter command will override it and it will let go of the shooter.

    def __init__(self, container, intake: Intake, indexer: Indexer, shooter: Shooter, controller: CommandXboxController) -> None:
        super().__init__()
        self.setName('Run intake reverse by trigger')  # change this to something appropriate for this command
        self.container = container
        self.controller = controller
        self.intake = intake
        self.indexer = indexer
        self.shooter = shooter
        self.addRequirements(self.intake, self.indexer, self.shooter)  # commandsv2 version of requirements

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

    def execute(self) -> None:
        desired_voltage_intake = -self.controller.getLeftTriggerAxis() * 4
        desired_indexer = -self.controller.getLeftTriggerAxis() / 3 # set_indexer seems to take a value from 0 to 1 instead of volts
        desired_voltage_shooter = -self.controller.getLeftTriggerAxis() * 2
        # 4v intake 4v indexer 2 shooter
        self.intake.set_intake_motor_volts(desired_voltage_intake)
        self.indexer.set_indexer(desired_indexer)
        self.shooter.set_flywheel(rpm=0, volts=desired_voltage_shooter, use_voltage=True)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.intake.stop_intake()
        self.indexer.stop_indexer()
        self.shooter.stop_shooter()
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")