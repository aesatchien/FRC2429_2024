"""
A place for the constant values in the code that may be used in more than one place. 
This offers a convenient resources to teams who need to make both quick and universal
changes.

2023 robot for team 2429 - the blockheads
"""
import math

k_competition_mode = False  # use for compressor and some joystick settings
k_burn_flash = True  # if we want to burn the settings to the sparkmaxes
k_enable_soft_limits = False

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

# drivetrain constants
k_wheel_diameter_in = 6  # wheel diameter in inches
k_wheel_diameter_m =  k_wheel_diameter_in * 0.0254  # wheel diameter in meters
k_robot_length = 33 * 0.0254
k_track_width_meters = 27 * 0.0254
k_robot_wheelbase = 18 * 0.5 * 0.0254
k_gear_ratio = 10.72  # REV 6in slow (10T) is 11.79:  medium (11T) is 10.72.  Both use the 30/68, so 2.26 * 52/PinionT
k_sparkmax_conversion_factor_meters = k_wheel_diameter_m * 3.14159 / k_gear_ratio  # used in drivetrain - 0.044 m/rev
k_neo_freespeed = 5700  # published is 5676

# testing ON BLOCKS shows that flat out at 90% power we top out at 4m/s - still pretty fast
# kff = 0.24 using k and k/60  but this screws smart motion - it ain't that smart so 0.0040 is the kFF and m/min is vel

k_PID_dict_pos = {'kP': 0.002, 'kI': 0, 'kD': 0.002, 'kIz': 0, 'kFF': 0.008, 'kArbFF':0, 'kMaxOutput': 0.99, 'kMinOutput': -0.99,
                'SM_MaxVel': 5000 / k_sparkmax_conversion_factor_meters, 'SM_MaxAccel': 5000 / k_sparkmax_conversion_factor_meters}
k_PID_dict_vel = {'kP': 0.0, 'kI': 0.000, 'kD': 0.00, 'kIz': 0.001, 'kFF': 0.0040, 'kArbFF':0, 'kMaxOutput': 0.95,
                'kMinOutput': -0.95, 'SM_MaxVel':180, 'SM_MaxAccel':120}  # 180 is 3 m/s and 3m/s/s
k_PID_dict_vel_slow = {'kP': 1e-5, 'kI': 4e-6, 'kD': 0.00, 'kIz': 0, 'kFF': 0.0040, 'kArbFF':0, 'kMaxOutput': 0.5,
                'kMinOutput': -0.5, 'SM_MaxVel':120, 'SM_MaxAccel':120}  # 120 is 2 m/s and 2m/s/s
k_drive_accumulator_max = 0.5  # limit on forward I - negative has no limit :(  Units in volts?

# ------------------- SHOOTER -------------------
k_flywheel_lower_left_neo_port = 10 #CAN ID
k_flywheel_upper_left_neo_port = 11 #CAN ID

# ------------------- Top CRANK -------------------
k_top_crank_gear_ratio = 5 * 5 * 4 * 1  # 554 (maxplanetary) * 1 (pulley) = 100
k_top_crank_abs_encoder_position_conversion_factor = 2 * math.pi  # shooter crank is 1:1 with thru-bore encoder
# k_top_crank_encoder_conversion_factor = 360. / k_top_crank_gear_ratio  # motor revs to degrees
kFF_top_crank = 1 / (k_neo_freespeed * k_top_crank_abs_encoder_position_conversion_factor)

# trapezoidal system constants - estimated from reca.lc/arm
# using 100:1 reduction and two motors, 12in and 15lbs, 95% efficiency
k_shooter_arm_dict = {
    'name': 'shooter_arm',
    'max_angle': 109, 'min_angle': -85,
    'motor_can_id': 8, 'follower_can_id': 9,
    'abs_encoder_zero_offset':  0.45, # 0.420,  # makes horizontal 0
    'encoder_position_conversion_factor': 2 * math.pi,  # shooter crank is 1:1 with thru-bore encoder,
    'k_motor_count': 2,  #
    'k_kArmOffsetRads': -1.5,  # # The offset of the arm from the horizontal in its neutral position, measured from the horizontal
    'k_MaxVelocityRadPerSecond': 1,
    'k_MaxAccelerationRadPerSecSquared': 0.5,
    'k_kSVolts': 0.1,  # not estimated by recalc, so we have to make something up
    'k_kGVolts': 0.71 / 2,  # cuts in half with two motors, goes up with mass and distance, down with efficiency
    'k_kVVoltSecondPerRad': 1.95,  # stays the same with one or two motors, based on the NEO itself and gear ratio
    'k_kAVoltSecondSquaredPerRad': 0.02 / 2, # cuts in half with two motors
    'k_kP': 0.8  # if we use radians, then it's this much power per radian of error (1 would be 100% power per 180 degrees)
}
# velocity and acceleration targets will be in radians per second, and remember SmartMotion no good for position slot
k_PID_dict_pos_shooter_arm = {'kP': k_shooter_arm_dict['k_kP'], 'kI': 0, 'kD': 0, 'kIz': 1e-5, 'kFF': kFF_top_crank, 'kArbFF':0,
                         'kMaxOutput': 0.35, 'kMinOutput': -0.35, 'SM_MaxVel':1, 'SM_MaxAccel':1}
k_PID_dict_vel_shooter_arm = {'kP': 0, 'kI': 0, 'kD': 0, 'kIz': 1e-5, 'kFF': kFF_top_crank, 'kArbFF':0,
                         'kMaxOutput': 0.35, 'kMinOutput': -0.35, 'SM_MaxVel':100, 'SM_MaxAccel':100}


# ------------------- Lower CRANK -------------------
k_lower_crank_gear_ratio = 5 * 5 * 3 * 4  # 553 (maxplanetary) * 4 (pulley) = 300
# trapezoidal system constants - estimated from reca.lc/arm
# using 300:1 reduction and one motor, 20in and 20lbs, 95% efficiency
k_crank_arm_dict = {
    'name': 'crank_arm',
    'max_angle': 120, 'min_angle': 60,
    'motor_can_id': 7, 'follower_can_id': 6,
    'gearing': 300, 'arm_length': 20 * 0.0254, 'arm_mass': 8, # meters and kg
    'abs_encoder_zero_offset': 0.515,  # measered at arm=90 degrees - set the sparkmax's encoder and can still use abs p/m 45 deg
    'encoder_position_conversion_factor': 2 * math.pi / k_lower_crank_gear_ratio,  # using sparkmax internal encoder
    'k_motor_count': 1,  #
    'k_kArmOffsetRads': 1.57,  # # The offset of the arm from the horizontal in its neutral position, measured from the horizontal
    'k_MaxVelocityRadPerSecond': 0.2,
    'k_MaxAccelerationRadPerSecSquared': 0.2,
    'k_kSVolts': 0.01,  # not estimated by recalc, so we have to make something up
    'k_kGVolts': 0.51 / 1,  # cuts in half with two motors, goes up with mass and distance, down with efficiency
    'k_kVVoltSecondPerRad': 5.85,  # stays the same with one or two motors, based on the NEO itself and gear ratio
    'k_kAVoltSecondSquaredPerRad': 0.02 / 1, # cuts in half with two motors
    'k_kP': 0.00  # if we use radians, then it's this much power per radian of error (1 would be 100% power per 180 degrees)
}
# velocity and acceleration targets will be in degrees per second, SmartMotion no good for position slot
k_PID_dict_pos_lower_crank_arm = {'kP': k_crank_arm_dict['k_kP'], 'kI': 0, 'kD': 0, 'kIz': 1e-5, 'kFF': 0, 'kArbFF':0,
                         'kMaxOutput': 0.051, 'kMinOutput': -0.01, 'SM_MaxVel':1, 'SM_MaxAccel':1}
k_PID_dict_vel_lower_crank_arm = {'kP': 0, 'kI': 0, 'kD': 0, 'kIz': 1e-5, 'kFF': 0, 'kArbFF':0,
                         'kMaxOutput': 0.05, 'kMinOutput': -0.05, 'SM_MaxVel':1, 'SM_MaxAccel':1}

# ------------------- Indexer -------------------
k_indexer_neo_port = 5

# ------------------- Intake -------------------
k_intake_neo_port = 12  # CAN ID

# ------------------- LED -------------------
k_led_pwm_port = 3
k_led_count = 36

# --------------  SIMULATION  ---------------
k_start_x = 7.647
k_start_y = 1.935
k_start_heading = -90  # looking at the drawing originally tried -109

# --------------  HELPER FUNCTIONS  ---------------
def clamp(value: float, bottom: float, top: float) -> float:
    return max(bottom, min(value, top))
