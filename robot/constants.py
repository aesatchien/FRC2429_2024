"""
A place for the constant values in the code that may be used in more than one place.
This offers a convenient resources to teams who need to make both quick and universal
changes.

2024 robot for team 2429 - the blockheads
"""

# top level items
k_field_centric = True  # True sets the robot driving to the driver's point of view, not the robot's
k_burn_flash = False  # if we want to burn the settings to the sparkmaxes - usually false unless setting up
k_volt_compensation = 12.0  # allow sparkmaxes to scale the requests when the battery is low/hi
k_rate_limited = True  # on swerve, use slew limiters to keep acceleration from being too abrupt
k_debugging_messages = False  # turn these off for competition
k_slowmode_multiplier = 0.2  # cut max velocity of the robot for fine movement control

# --------------  OI  ---------------
# ID for the driver's joystick (template)
k_driver_controller_port = 0

# --------------  SIMULATION AND FIELD INITIALIZATION  ---------------
k_start_x = 2.1  # eventually this will have to be set by the autonomous choices
k_start_y = 4.7