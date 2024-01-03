import commands2
from wpilib import SmartDashboard
import rev
from subsystems.arm import Arm


class ArmCalibration(commands2.CommandBase):

    def __init__(self, container, arm:Arm, velocity=-2500) -> None:
        super().__init__()
        self.setName('ArmCalibration')
        self.container = container
        self.arm = arm
        self.velocity = velocity
        self.addRequirements(self.arm)  # commandsv2 version of requirements

    def initialize(self) -> None:
        self.print_start_message()
        # turn off the reverse limit
        self.arm.arm_controller.clearFaults()
        self.arm.arm_controller.enableSoftLimit(rev.CANSparkMax.SoftLimitDirection.kReverse, False)
        self.arm.pid_controller.setReference(-2, rev.CANSparkMax.ControlType.kVoltage)

    def execute(self) -> None:  # nothing to do, the sparkmax is doing all the work
        pass
        # self.arm.pid_controller.setReference(-1000, rev.CANSparkMax.ControlType.kVelocity)

        SmartDashboard.putNumber("ArmCurrent", self.arm.arm_controller.getOutputCurrent())
        SmartDashboard.putNumber("ArmVoltage", self.arm.arm_controller.getAppliedOutput())

    def isFinished(self) -> bool:
        return self.arm.arm_controller.getOutputCurrent() > 20  # figure out the stall current on the arm

    def end(self, interrupted: bool) -> None:
        self.arm.set_encoder_position(distance=0)  # now we have a new zero
        self.arm.arm_controller.enableSoftLimit(rev.CANSparkMax.SoftLimitDirection.kReverse, True)
        self.arm.pid_controller.setReference(0, rev.CANSparkMax.ControlType.kVoltage)
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s at {self.arm.sparkmax_encoder.getPosition():.1f} after {end_time - self.start_time:.1f} s **")
        # SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")

    def print_start_message(self):
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")