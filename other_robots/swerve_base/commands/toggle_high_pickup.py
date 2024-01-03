import commands2
import commands2.cmd as cmd
import constants

from commands.elevator_move import ElevatorMove
from commands.wrist_move import WristMove
from commands.turret_move import TurretMove
from commands.manipulator_toggle import ManipulatorToggle
from autonomous.turret_move_by_vision import TurretMoveByVision

from subsystems.elevator import Elevator
from subsystems.turret import Turret
from subsystems.pneumatics import Pneumatics
from subsystems.wrist import Wrist
from subsystems.vision import Vision

class ToggleHighPickup(commands2.SequentialCommandGroup):

    def __init__(self, container, turret: Turret, elevator: Elevator, wrist: Wrist, pneumatics: Pneumatics, vision: Vision) -> None:
        super().__init__()
        self.setName('ToggleHighPickup')

        # ToDo: all timeouts need tweaking

        #move elevator
        self.addCommands(WristMove(container=container, wrist=wrist, setpoint=wrist.positions['stow'], wait_to_finish=False))
        self.addCommands(ElevatorMove(container=container, elevator=elevator, setpoint=elevator.positions['upper_pickup'], wait_to_finish=True))

        #open wrist
        self.addCommands(ManipulatorToggle(container=container, pneumatics=pneumatics, force='open'))
        # self.addCommands(commands2.WaitCommand(0.25))

        # rotate turret
        # turret prioritizes green targets first, so turn off the ring light
        self.addCommands(cmd.runOnce(action=lambda: vision.set_relay(state=False)))
        # self.addCommands(TurretMoveByVision(container=container, turret=turret, vision=vision, wait_to_finish=True)
        #                  .withTimeout(2))
        self.addCommands(cmd.runOnce(action=lambda: vision.set_relay(state=True)))

        # deploy wrist
        self.addCommands(WristMove(container=container, wrist=wrist, setpoint=wrist.positions['flat'], wait_to_finish=True)
                                   .withTimeout(1))

        # ToDo: extend arm until within range using ToF?

        """
        # clamp
        self.addCommands(commands2.WaitCommand(0.25))
        self.addCommands(ManipulatorToggle(container=container, pneumatics=pneumatics, force='close'))
        self.addCommands(commands2.WaitCommand(1))

        # stow
        self.addCommands(WristMove(container=container, wrist=wrist, setpoint=wrist.positions['stow'], wait_to_finish=True)
                         .withTimeout(2))
        self.addCommands(TurretMove(container=container, turret=turret, setpoint=0, wait_to_finish=False))
        self.addCommands(ElevatorMove(container=container, elevator=elevator, setpoint=elevator.positions['bottom'], wait_to_finish=True))
        """