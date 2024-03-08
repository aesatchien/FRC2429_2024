import enum
import math
import commands2
import wpilib
from wpilib import AddressableLED
from wpilib import Color, SmartDashboard
from subsystems import shooter

import constants


class Led(commands2.Subsystem):
    class Mode(enum.Enum):
        RING = 'RING'  # orange
        CUBE = 'CUBE'  # purple

    # temporary indicators (flashing for pickup, strafing, etc)
    class Indicator(enum.Enum):
        READY_SHOOT = 'READY_SHOOT'  # flashing white
        AMP = 'AMP'  # red
        INTAKE = 'INTAKE'  # flashing blue
        PICKUP_COMPLETE = 'AUTO_STRAFE_COMPLETE'  # solid blue
        RAINBOW = 'RAINBOW'
        INTAKE_ON = 'INTAKE_ON'
        CALIBRATION_START = 'CALIBRATION_START'
        CALIBRATION_SUCCESS = 'CALIBRATION_SUCCESS'
        CALIBRATION_FAIL = 'CALIBRATION_FAIL'
        KILL = 'KILL'
        SHOOTER_ON = 'SHOOTER_ON'
        NONE = 'NONE'

    def __init__(self):
        super().__init__()
        self.setName('Led')
        self.counter = 0
        self.animation_counter = 0

        self.led_count = constants.k_led_count
        self.led_strip = AddressableLED(constants.k_led_pwm_port)
        self.led_data = [AddressableLED.LEDData() for _ in range(self.led_count)]

        [led.setRGB(0, 0, 0) for led in self.led_data]

        self.led_strip.setLength(self.led_count)
        self.led_strip.setData(self.led_data)
        self.led_strip.start()

        self.mode = Led.Mode.RING
        self.indicator = Led.Indicator.NONE

    def set_mode(self, mode: Mode) -> None:
        self.prev_mode = self.mode
        self.mode = mode

    def get_mode(self) -> Mode:
        return self.mode

    def set_indicator(self, indicator) -> None:
        self.indicator = indicator

    def set_indicator_with_timeout(self, indicator: Indicator, timeout: float) -> commands2.StartEndCommand:
        return commands2.StartEndCommand(
            lambda: self.set_indicator(indicator),
            lambda: self.set_indicator(Led.Indicator.NONE),
        ).withTimeout(timeout)

    def periodic(self) -> None:

        # update LEDs
        if self.counter % 5 == 0:
            SmartDashboard.putString('led_mode', self.mode.value)
            SmartDashboard.putString('led_indicator', self.indicator.value)

            # advertise our state to the dash
            SmartDashboard.putBoolean('cone_selected', self.mode == self.Mode.RING)

            self.animation_counter += 1

            for i in range(constants.k_led_count):
                led = self.led_data[i]

                # check if there is an indicator, and override
                if self.indicator != Led.Indicator.NONE:
                    if self.indicator == Led.Indicator.READY_SHOOT:
                        # flashing white
                        freq = 1  # 10 /s > 2x /s
                        cycle = math.floor(self.animation_counter / freq)

                        if cycle % 2 == 0:
                            led.setRGB(0, 0, 0)
                        else:
                            led.setRGB(255, 255, 255)

                    elif self.indicator == Led.Indicator.AMP:
                        # solid red
                        led.setRGB(255, 0, 0)

                    elif self.indicator == Led.Indicator.INTAKE:
                        # solid blue
                        # freq = 1  # 10 /s > 2x /s
                        # cycle = math.floor(self.animation_counter / freq)
                        #
                        # if cycle % 2 == 0:
                        #     led.setRGB(0, 0, 0)
                        # else:
                        led.setRGB(0, 0, 255)

                    elif self.indicator == Led.Indicator.PICKUP_COMPLETE:
                        # solid orange
                        led.setRGB(255, 40, 0)

                    elif self.indicator == Led.Indicator.RAINBOW: # Haochen emote
                        # rainbow
                        hue = (i + self.animation_counter) % constants.k_led_count
                        hue /= constants.k_led_count
                        hue *= 180

                        led.setHSV(math.floor(hue), 255, 255)

                    elif self.indicator == Led.Indicator.INTAKE_ON: # Haochen emote
                        # solid orange
                        # freq = 2  # 10 /s > 2x /s
                        # cycle = math.floor(self.animation_counter / freq)

                        # if cycle % 2 == 0:
                        #     led.setRGB(0, 0, 0)
                        # else:
                        led.setRGB(255, 40, 0)

                    elif self.indicator == Led.Indicator.SHOOTER_ON:
                        led.setRGB(255, 255, 0)

                    elif self.indicator == Led.Indicator.KILL:
                        led.setRGB(200, 0, 255)

                    elif self.indicator == Led.Indicator.CALIBRATION_START:
                        led.setRGB(0, 255, 0)

                    elif self.indicator == Led.Indicator.CALIBRATION_SUCCESS:
                        # flashing green
                        freq = 1  # 10 /s > 2x /s
                        cycle = math.floor(self.animation_counter / freq)

                        if cycle % 2 == 0:
                            led.setRGB(0, 0, 0)
                        else:
                            led.setRGB(0, 255, 0)

                    elif self.indicator == Led.Indicator.CALIBRATION_FAIL:
                        # flashing red
                        freq = 1  # 10 /s > 2x /s
                        cycle = math.floor(self.animation_counter / freq)

                        if cycle % 2 == 0:
                            led.setRGB(0, 0, 0)
                        else:
                            led.setRGB(255, 0, 0)


                else:
                    if self.mode == Led.Mode.RING:
                        # solid orange
                        led.setRGB(255, 40, 0)

                    elif self.mode == Led.Mode.CUBE:
                        # solid purple
                        led.setRGB(255, 0, 255)

            self.led_strip.setData(self.led_data)

        self.counter += 1








