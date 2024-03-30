"""
A place for the constant values in the code that may be used in more than one place. 
This offers a convenient resources to teams who need to make both quick and universal
changes.

2023 robot for team 2429 - the blockheads
"""

k_competition_mode = False  # use for compressor and some joystick settings
k_burn_flash = False  # if we want to burn the settings to the sparkmaxes
k_enable_soft_limts = True

# --------------  OI  ---------------
# ID for the driver's joystick (template)
k_driver_controller_port = 0
k_co_driver_controller_port = 1
k_controller_thrust_axis = 1
k_controller_twist_axis = 4
k_arcade_thrust_scale = 0.7  # used in drive by joystick arcade mode
k_arcade_twist_scale = 0.45  # used in drive by joystick arcade mode
k_max_thrust_velocity = 210  # meters per MINUTE  for smartmotion was 150 at start of Hueneme
k_max_twist_velocity = 150 # meters per MINUTE - for smartmotion
k_slowmode_multiplier  = 0.3

#  co-driver++++

k_controller_elevator_axis = 1
k_controller_turret_axis = 4


# --------------  DRIVETRAIN  ---------------
# The CAN IDs for the drivetrain SparkMAX motor controllers
# For when turret is front
k_left_motor1_port = 1
k_left_motor2_port = 2
k_right_motor1_port = 3
k_right_motor2_port = 4

# For when battery is front
# k_left_motor1_port = 3
# k_left_motor2_port = 4
# k_right_motor1_port = 1
# k_right_motor2_port = 2

# --------------  BUCKET  ---------------
k_bucket = 5
k_bucket_conversion_factor = 360/21.0
k_PID_dict_vel_bucket = {'kP': 0.00001, 'kI': 0, 'kD': 0.00001, 'kIz': 2e-4, 'kFF': 8.1e-7, 'kArbFF':0,
                         'kMaxOutput': 0.9, 'kMinOutput': -0.9, 'SM_MaxVel':35000,
                         'SM_MaxAccel':35000}

# drivetrain constants
k_wheel_diameter_in = 6  # wheel diameter in inches
k_wheel_diameter_m =  k_wheel_diameter_in * 0.0254  # wheel diameter in meters
k_robot_length = 33 * 0.0254
k_track_width_meters = 27 * 0.0254
k_robot_wheelbase = 18 * 0.5 * 0.0254
k_gear_ratio = 10.72  # REV 6in slow (10T) is 11.79:  medium (11T) is 10.72.  Both use the 30/68, so 2.26 * 52/PinionT
k_sparkmax_conversion_factor_meters = k_wheel_diameter_m * 3.14159 / k_gear_ratio  # used in drivetrain - 0.044 m/rev

# testing ON BLOCKS shows that flat out at 90% power we top out at 4m/s - still pretty fast
# kff = 0.24 using k and k/60  but this screws smart motion - it ain't that smart so 0.0040 is the kFF and m/min is vel

k_PID_dict_pos = {'kP': 0.002, 'kI': 0, 'kD': 0.002, 'kIz': 0, 'kFF': 0.008, 'kArbFF':0, 'kMaxOutput': 0.99, 'kMinOutput': -0.99,
                'SM_MaxVel': 5000 / k_sparkmax_conversion_factor_meters, 'SM_MaxAccel': 5000 / k_sparkmax_conversion_factor_meters}
k_PID_dict_vel = {'kP': 0.0, 'kI': 0.000, 'kD': 0.00, 'kIz': 0.001, 'kFF': 0.0040, 'kArbFF':0, 'kMaxOutput': 0.95,
                'kMinOutput': -0.95, 'SM_MaxVel':180, 'SM_MaxAccel':120}  # 180 is 3 m/s and 3m/s/s
k_PID_dict_vel_slow = {'kP': 1e-5, 'kI': 4e-6, 'kD': 0.00, 'kIz': 0, 'kFF': 0.0040, 'kArbFF':0, 'kMaxOutput': 0.5,
                'kMinOutput': -0.5, 'SM_MaxVel':120, 'SM_MaxAccel':120}  # 120 is 2 m/s and 2m/s/s
k_drive_accumulator_max = 0.5  # limit on forward I - negative has no limit :(  Units in volts?

# ------------------- LED -------------------
k_led_pwm_port = 7
k_led_count = 40

# --------------  SIMULATION  ---------------
k_start_x = 7.647
k_start_y = 1.935
k_start_heading = -90  # looking at the drawing originally tried -109
k_drivetrain_motor_count = 4
k_wheel_diameter_m = 6 * 0.0254  # wheel diameter in meters
k_gear_ratio = 5.39  # 4.17 # high gear 2022
k_track_width_meters = 24 * 0.0254
robot_characterization = {'ks':0.291, 'kv':1.63, 'ka':0.293, 'track_width':0.89}  # 2022 climberbot
ks_volts = robot_characterization['ks']  # so far this is only used in the Ramsete command, but in 2021 we used it in tank model as well
kv_volt_seconds_per_meter = robot_characterization['kv']  # used in physics.py LinearSystemId and Ramsete
ka_volt_seconds_squared_per_meter = robot_characterization['ka']  # used in physics.py LinearSystemId and Ramsete

# --------------  HELPER FUNCTIONS  ---------------
def clamp(value: float, bottom: float, top: float) -> float:
    return max(bottom, min(value, top))
