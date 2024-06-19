import commands2
from wpilib import SmartDashboard, Timer
import ntcore
from wpimath.geometry import Translation2d
from subsystems.swerve import Swerve


class LapTimer(commands2.Command):  # change the name for your command

    def __init__(self, container, swerve: Swerve, finish_zone_radius: float) -> None:
        super().__init__()
        self.setName('Lap Timer')  # change this to something appropriate for this command
        self.container = container
        self.swerve = swerve
        self.finish_zone_radius = finish_zone_radius
        self.laps = []
        self.timer = Timer()
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.lap_publisher = self.inst.getDoubleTopic('laps').publish()
        # self.addRequirements(self.container.)  # commandsv2 version of requirements

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

        self.laps = []
        self.finish_zone_center = Translation2d(self.swerve.get_pose().X(), self.swerve.get_pose().Y())
        self.prev_in_finish_zone = Translation2d(self.swerve.get_pose().X(), self.swerve.get_pose().Y()).distance(self.finish_zone_center) < self.finish_zone_radius

    def execute(self) -> None:
        in_finish_zone = Translation2d(self.swerve.get_pose().X(), self.swerve.get_pose().Y()).distance(self.finish_zone_center) < self.finish_zone_radius
        if not in_finish_zone and self.prev_in_finish_zone: # just exited the finish zone, start a lap
            print('STARTING A LAP!!')
            self.timer.reset()
            self.timer.start()
        elif in_finish_zone and not self.prev_in_finish_zone: # Just entered the finish zone, lap ended
            # self.laps.append(self.timer.get())
            print(f'LAP FINISHED AFTER {self.timer.get()} SECONDS!!')
            SmartDashboard.putNumber('latest_lap_time', self.timer.get())
            self.lap_publisher.set(self.timer.get())
            # send data here

        self.prev_in_finish_zone = in_finish_zone


    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:

        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print_end_message = True
        if print_end_message:
            print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
            SmartDashboard.putString(f"alert",
                                     f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")