from commands2.button import Button
from wpilib import XboxController


class AxisButton(Button):
    """
    A custom button that is used when pretending an axis button is digital.
    Note - don't forget the init for the parent!  Will crash in 2020 w/o it.
    In 2022 it seems we have to override the isPressed in init
    """

    def __init__(self, joystick:XboxController, axis):
        # In 2022 it seems we have to override the isPressed in init
        super().__init__(isPressed=lambda: joystick.getRawAxis(axis) > 0.2)
        #super().__init__()
        self.joystick = joystick
        self.axis = axis
        self.threshold = 0.2
        print(f'Initialized AxisButton with axis {self.axis}')

    # this does nothing!  can't override the get() function anymore, it is now the isPressed
    # def isPressed(self):
    #     raw_axis = self.joystick.getRawAxis(self.axis)
    #     return abs(raw_axis) > self.threshold
