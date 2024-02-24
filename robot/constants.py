"""
A place for the constant values in the code that may be used in more than one place.
This offers a convenient resources to teams who need to make both quick and universal
changes.

2024 robot for team 2429 - the blockheads
"""

import math

# top level items
k_field_centric = True  # True sets the robot driving to the driver's point of view, not the robot's
k_burn_flash = True  # if we want to burn the settings to the sparkmaxes - usually false unless setting up
k_volt_compensation = 12.0  # allow sparkmaxes to scale the requests when the battery is low/hi
k_rate_limited = True  # on swerve, use slew limiters to keep acceleration from being too abrupt
k_debugging_messages = True  # turn these off for competition
k_slowmode_multiplier = 0.2  # cut max velocity of the robot for fine movement control

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


# ------------------- Intake -------------------
k_intake_neo_port = 5  # CAN ID


# --------------  SIMULATION AND FIELD INITIALIZATION  ---------------
k_start_x = 2.2  # eventually this will have to be set by the autonomous choices
k_start_y = 5.85

# ------------------- SHOOTER -------------------
k_flywheel_lower_left_neo_port = 10 #CAN ID
k_flywheel_upper_left_neo_port = 11 #CAN ID

# ------------------- Top CRANK -------------------
k_top_crank_gear_ratio = 5 * 5 * 4 * 1  # 554 (maxplanetary) * 1 (pulley) = 100
k_top_crank_abs_encoder_position_conversion_factor = 2 * math.pi  # shooter crank is 1:1 with thru-bore encoder
# k_top_crank_encoder_conversion_factor = 360. / k_top_crank_gear_ratio  # motor revs to degrees
#kFF_top_crank = 1 / (k_neo_freespeed * k_top_crank_abs_encoder_position_conversion_factor)

# trapezoidal system constants - estimated from reca.lc/arm
# using 100:1 reduction and two motors, 12in and 15lbs, 95% efficiency
k_shooter_arm_dict = {
    'name': 'upper_arm',
    'max_angle': 109, 'min_angle': -85,
    'motor_can_id': 8, 'follower_can_id': 9,
    'abs_encoder_zero_offset':  0.372,  # 0.45, # 0.420,  # makes horizontal 0
    'encoder_position_conversion_factor': 2 * math.pi,  # shooter crank is 1:1 with thru-bore encoder,
    'k_motor_count': 2,  #
    'k_kArmOffsetRads': -1.5,  # # The offset of the arm from the horizontal in its neutral position, measured from the horizontal
    'k_MaxVelocityRadPerSecond': 1.25,
    'k_MaxAccelerationRadPerSecSquared': 1.5,
    'k_kSVolts': 0.3,  # not estimated by recalc, so we have to make something up
    'k_kGVolts': 0.71 / 2,  # cuts in half with two motors, goes up with mass and distance, down with efficiency
    'k_kVVoltSecondPerRad': 1.95,  # stays the same with one or two motors, based on the NEO itself and gear ratio
    'k_kAVoltSecondSquaredPerRad': 0.02 / 2, # cuts in half with two motors
    'k_kP': 1.2  # if we use radians, then it's this much power per radian of error (1 would be 100% power per 180 degrees)
}
# velocity and acceleration targets will be in radians per second, and remember SmartMotion no good for position slot
k_PID_dict_pos_shooter_arm = {'kP': k_shooter_arm_dict['k_kP'], 'kI': 0, 'kD': 0, 'kIz': 1e-5, 'kArbFF':0,
                         'kMaxOutput': 0.5, 'kMinOutput': -0.5, 'SM_MaxVel':1, 'SM_MaxAccel':1}
# k_PID_dict_pos_shooter_arm = {'kP': k_shooter_arm_dict['k_kP'], 'kI': 0, 'kD': 0, 'kIz': 1e-5, 'kFF': kFF_top_crank, 'kArbFF':0,'kMaxOutput': 0.5, 'kMinOutput': -0.5, 'SM_MaxVel':1, 'SM_MaxAccel':1}
k_PID_dict_vel_shooter_arm = {'kP': 0, 'kI': 0, 'kD': 0, 'kIz': 1e-5, 'kArbFF':0,
                         'kMaxOutput': 0.35, 'kMinOutput': -0.35, 'SM_MaxVel':100, 'SM_MaxAccel':100}
#k_PID_dict_vel_shooter_arm = {'kP': 0, 'kI': 0, 'kD': 0, 'kIz': 1e-5, 'kFF': kFF_top_crank, 'kArbFF':0,'kMaxOutput': 0.35, 'kMinOutput': -0.35, 'SM_MaxVel':100, 'SM_MaxAccel':100}


# ------------------- Lower CRANK -------------------
k_lower_crank_gear_ratio = 5 * 5 * 3 * 4  # 553 (maxplanetary) * 4 (pulley) = 300
# trapezoidal system constants - estimated from reca.lc/arm
# using 300:1 reduction and one motor, 20in and 20lbs, 95% efficiency
k_crank_arm_dict = {
    'name': 'crank_arm',
    'max_angle': 115, 'min_angle': 65,
    'motor_can_id': 7, 'follower_can_id': 6,
    'gearing': 300, 'arm_length': 20 * 0.0254, 'arm_mass': 8, # meters and kg
    'abs_encoder_zero_offset': 0.536,  # measered at arm=90 degrees - set the sparkmax's encoder and can still use abs p/m 45 deg
    'encoder_position_conversion_factor': 2 * math.pi / k_lower_crank_gear_ratio,  # using sparkmax internal encoder
    'k_motor_count': 1,  #
    'k_kArmOffsetRads': 1.57,  # # The offset of the arm from the horizontal in its neutral position, measured from the horizontal
    'k_MaxVelocityRadPerSecond': 0.5,
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

# ------------------- Indexer -------------------
k_follower_winch_neo_port =4
k_left_winch_neo_port = 3

# ------------------- Intake -------------------
k_intake_neo_port = 12  # CAN ID

# ------------------- LED -------------------
k_led_pwm_port = 3
k_led_count = 36

# --------------  HELPER FUNCTIONS  ---------------
def clamp(value: float, bottom: float, top: float) -> float:
    return max(bottom, min(value, top))
