import math
import commands2
import wpilib
from commands2.button import JoystickButton
from wpilib import SmartDashboard
import rev
import constants
from subsystems.drivetrain import Drivetrain

# TODO: Add a version of this with a lower scale as a fine-control version
class DriveByJoystickVelocity(commands2.CommandBase):

    def __init__(self, container, drive: Drivetrain, control_type='velocity', scaling=1.0) -> None:
        super().__init__()
        self.setName('drive_by_joystick_velocity')  # change this to something appropriate for this command
        self.container = container
        self.drive = drive
        self.control_type = control_type
        self.scaling = scaling
        self.addRequirements(self.drive)  # commandsv2 version of requirements

        self.max_thrust_velocity = constants.k_max_thrust_velocity  # 2.75 m/s
        self.max_twist_velocity = constants.k_max_twist_velocity  # 1.25 m/s
        self.deadband = 0.03
        self.multipliers = [1.0, 1.0]

        self.max_arcade_thrust = constants.k_arcade_thrust_scale
        self.max_arcade_twist = constants.k_arcade_twist_scale
        self.previous_thrust = 0

        # Last year we limited the thrust differential to 0.04 - 0.05 was too tippy
        self.max_thrust_differential = 0.2  # how high can we go on this?  0.1 means 10 cycles to go from 0 to full

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 1)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.1f} s **")

    def execute(self) -> None:
        # get thrust and twist from the controller
        thrust = -(self.container.driver_controller.getRawAxis(constants.k_controller_thrust_axis))
        thrust = 0 if abs(thrust) < self.deadband else math.copysign(1, thrust) * (abs(thrust) ** self.scaling)

        twist = self.container.driver_controller.getRawAxis(constants.k_controller_twist_axis)
        twist = 0 if abs(twist) < self.deadband else math.copysign(1, twist) * (abs(twist) ** self.scaling)

        slowmode_multiplier = constants.k_slowmode_multiplier if self.container.driver_controller.getRawButton(5) else 1.0

        # try to limit the change in thrust  BE VERY CAREFUL WITH THIS!  IT CAUSES RUNAWAY ROBOTS!
        d_thrust = self.previous_thrust - thrust
        #limit_decel = 'simple'
        limit_decel = 'none'
        #limit_decel = SmartDashboard.getString('drive_limit', 'thrust')
        if limit_decel == 'simple':  # global limit in both directions, limits fwd acceleration
            max_thrust_differential = self.max_thrust_differential
            if abs(d_thrust) > max_thrust_differential:
                thrust = self.previous_thrust - max_thrust_differential * math.copysign(1, d_thrust)
                thrust_sign = '+' if math.copysign(1, thrust) > 0 else '-'
        else:  # no acceleration limit
            pass

        self.previous_thrust = thrust


        if self.control_type == 'velocity':
            self.drive.feed()
            # left front left back  right front  right back
            thrust_vel = thrust * self.max_thrust_velocity
            twist_vel = twist * self.max_twist_velocity
            velocities = [thrust_vel + twist_vel, thrust_vel - twist_vel]
            if (thrust**2 + twist**2)**0.5 > 0.05:
                for controller, velocity, multiplier in zip(self.drive.pid_controllers, velocities, self.multipliers):
                    controller.setReference(velocity * multiplier * slowmode_multiplier, rev.CANSparkMaxLowLevel.ControlType.kSmartVelocity, 1)
            else:
                for controller in self.drive.pid_controllers:
                    controller.setReference(0, rev.CANSparkMaxLowLevel.ControlType.kSmartVelocity, 1)
        else:  # arcade drive
            # fix this - get the thrusts again
            self.drive.arcade_drive(thrust * self.max_arcade_thrust, twist * self.max_arcade_twist)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        for controller in self.drive.pid_controllers:
            controller.setReference(0, rev.CANSparkMaxLowLevel.ControlType.kSmartVelocity, 0)

        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **", flush=True)
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")


