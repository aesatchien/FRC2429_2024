import enum
import commands2
from wpilib import AddressableLED
from wpilib import Color
import constants


class Led(commands2.SubsystemBase):
    class Mode(enum.Enum):
        CONE = Color(255, 255, 0)
        CUBE = Color(255, 0, 255)
        READY = Color(0, 255, 0)
        OFF = Color(0, 0, 0)

    def __init__(self):
        super().__init__()
        self.setName('Led')
        self.counter = 0

        self.led_count = constants.k_led_count
        self.led_strip = AddressableLED(constants.k_led_pwm_port)
        self.led_data = [AddressableLED.LEDData() for _ in range(self.led_count)]

        [led.setRGB(0, 0, 0) for led in self.led_data]

        self.led_strip.setLength(self.led_count)
        self.led_strip.setData(self.led_data)
        self.led_strip.start()
        self.mode = Led.Mode.OFF

    def set_mode(self, mode: Mode) -> None:
        self.mode = mode

    def periodic(self) -> None:
        # update LEDs
        if self.counter % 10 == 0:
            [led.setLED(self.mode.value) for led in self.led_data]
            self.led_strip.setData(self.led_data)

        self.counter += 1








