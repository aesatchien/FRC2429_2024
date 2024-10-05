"""
A place for the constant values in the code that may be used in more than one place.
This offers a convenient resources to teams who need to make both quick and universal
changes.

2024 robot for team 2429 - the la canada engineering club
"""

import math
import wpilib

# top level items

k_burn_flash = True  # if we want to burn the settings to the sparkmaxes - usually false unless setting up
k_volt_compensation = 12.0  # allow sparkmaxes to scale the requests when the battery is low/hi
k_enable_soft_limits = False  # arm will have soft limits as one way to prevent mishaps
k_swerve_only = False  # mode for debugging the drivetrain
k_path_from_robot_to_pathplanner_files = "deploy/pathplanner/paths"
k_path_from_robot_to_pathplanner_autos = 'deploy/pathplanner/autos'
k_path_to_abs_encoder_data = '/home/lvuser/abs_encoder_data.json'
k_enable_copilot = True
k_reset_sparks_to_default = False

k_swerve_state_messages = True  # these currently send the pose data to the sim - keep them on
k_swerve_debugging_messages = False  # these show the target angles and actual.  Only for debugging.

# --------------  DRIVER OI  ---------------
# ID for the driver's joystick (template)
k_driver_controller_port = 0
k_co_pilot_controller_port = 1

k_slowmode_multiplier = 0.3  # cut max velocity of the robot for fine movement control
k_field_centric = True  # True sets the robot driving to the driver's point of view, not the robot's
k_rate_limited = True  # on swerve, use slew limiters to keep acceleration from being too abrupt

#  co-driver++++


# ------------------- HARDWARE, FROM BOTTOM TO TOP -------------------

# --------- BUMPER MEASUREMENTS ----------
k_bumper_height = 0.22


# ------------------- CLIMBER -------------------
# k_follower_winch_neo_port =4
# k_left_winch_neo_port = 3
k_left_servo_port = 0
k_right_servo_port = 1
k_trap_servo_port = 3  # we broke 2 at Houston

k_follower_winch_neo_port =3
k_left_winch_neo_port = 4


# ------------------- Intake -------------------
k_intake_neo_port = 5  # CAN ID

#cranks
k_crank_presets = {
    'intake': {'upper':-58, 'lower': 46},  # was 63
    'shoot': {'upper': 1, 'lower':90}, # used to be -40 but changed it for climb
    'low_shoot': {'upper':-84, 'lower': 45},
    'amp': {'upper': 52, 'lower': 100},
    'low_amp': {'upper': -70, 'lower': 108},
    'bottom': {'upper': -84, 'lower': 40},
    'climb_second': {'upper':12, 'lower':90}
}
# ------------------- Lower CRANK -------------------
k_lower_crank_gear_ratio = 5 * 5 * 3 * 4.44  # 553 (maxplanetary) * 4.44 (pulley) = 333
# trapezoidal system constants - estimated from reca.lc/arm
# using 300:1 reduction and one motor, 20in and 20lbs, 95% efficiency
k_crank_arm_dict = {
    'name': 'crank_arm',
    'max_angle': 116, 'min_angle': 40,
    'motor_can_id': 7, 'follower_can_id': 6,
    'gearing': 300, 'arm_length': 20 * 0.0254, 'arm_mass': 8, # meters and kg
    'crank_gearbox_ratio': 80/18,  # This is the new gear ratio on the bottom crank
    # WANT 90 DEGREES TO BE ABOUT 0.7 ON THE ENCODER - GIVES HIGHEST RANGE FOR US
    'abs_encoder_zero_offset': 0.750,  # measered at arm=90 degrees - set the sparkmax's encoder and can still use abs p/m 45 deg.  in revolutions.
    'encoder_position_conversion_factor': 2 * math.pi / k_lower_crank_gear_ratio,  # using sparkmax internal encoder
    'k_motor_count': 1,  #
    'k_kArmOffsetRads': 1.57,  # # The offset of the arm from the horizontal in its neutral position, measured from the horizontal
    'k_MaxVelocityRadPerSecond': 3.0,  # was 1.5 starting Tempe
    'k_MaxAccelerationRadPerSecSquared': 3.0,  # was 1.0 starting Tempe
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
# The most "folded" the lower crank can be while still allowing the upper crank to do whatever it likes
k_min_lower_crank_angle_where_deploying_upper_crank_safe_rad = math.radians(80)
k_lower_crank_limit_switch_channel = 9
k_lower_crank_position_when_limit_switch_true_rad = math.radians(42)
k_lower_crank_current_where_jammed = 20

# ------------------- Top CRANK -------------------
k_top_crank_gear_ratio = 5 * 5 * 4 * 1  # 554 (maxplanetary) * 1 (pulley) = 100
k_top_crank_abs_encoder_position_conversion_factor = 2 * math.pi  # shooter crank is 1:1 with thru-bore encoder
# k_top_crank_encoder_conversion_factor = 360. / k_top_crank_gear_ratio  # motor revs to degrees
#kFF_top_crank = 1 / (k_neo_freespeed * k_top_crank_abs_encoder_position_conversion_factor)

# trapezoidal system constants - estimated from reca.lc/arm
# using 100:1 reduction and two motors, 12in and 15lbs, 95% efficiency
k_shooter_arm_dict = {
    'name': 'upper_arm',
    'max_angle': 109, 'min_angle': -95,  # was -79 before Q7
    'motor_can_id': 8, 'follower_can_id': 9,
    'abs_encoder_zero_offset':  0.595,  # 0.45, # 0.420,  # makes horizontal 0 with no position factor
    'encoder_position_conversion_factor': 2 * math.pi,  # shooter crank is 1:1 with thru-bore encoder,
    'k_motor_count': 2,  #
    'k_kArmOffsetRads': -1.5,  # # The offset of the arm from the horizontal in its neutral position, measured from the horizontal
    'k_MaxVelocityRadPerSecond': 5.5,  # was 3 starting Tempe (but we added a spring)
    'k_MaxAccelerationRadPerSecSquared': 5.5,  # was 3 starting Tempe
    'k_kSVolts': 1.0,  # was 0.3 before chain # not estimated by recalc, so we have to make something up
    'k_kGVolts': 0.71 / 2,  # cuts in half with two motors, goes up with mass and distance, down with efficiency
    'k_kVVoltSecondPerRad': 1.95,  # stays the same with one or two motors, based on the NEO itself and gear ratio
    'k_kAVoltSecondSquaredPerRad': 0.02 / 2, # cuts in half with two motors
    'k_kP': 1.2  # was 1.2 before chain  # if we use radians, then it's this much power per radian of error (1 would be 100% power per 180 degrees)
}
# velocity and acceleration targets will be in radians per second, and remember SmartMotion no good for position slot
k_PID_dict_pos_shooter_arm = {'kP': k_shooter_arm_dict['k_kP'], 'kI': 0, 'kD': 0, 'kIz': 1e-5, 'kFF':0, 'kArbFF':0,
                         'kMaxOutput': 0.5, 'kMinOutput': -0.5, 'SM_MaxVel':1, 'SM_MaxAccel':1}
# k_PID_dict_pos_shooter_arm = {'kP': k_shooter_arm_dict['k_kP'], 'kI': 0, 'kD': 0, 'kIz': 1e-5, 'kFF': kFF_top_crank, 'kArbFF':0,'kMaxOutput': 0.5, 'kMinOutput': -0.5, 'SM_MaxVel':1, 'SM_MaxAccel':1}
k_PID_dict_vel_shooter_arm = {'kP': 0, 'kI': 0, 'kD': 0, 'kIz': 1e-5, 'kFF':0, 'kArbFF':0,
                         'kMaxOutput': 0.35, 'kMinOutput': -0.35, 'SM_MaxVel':100, 'SM_MaxAccel':100}
#k_PID_dict_vel_shooter_arm = {'kP': 0, 'kI': 0, 'kD': 0, 'kIz': 1e-5, 'kFF': kFF_top_crank, 'kArbFF':0,'kMaxOutput': 0.35, 'kMinOutput': -0.35, 'SM_MaxVel':100, 'SM_MaxAccel':100}

# The least "folded" the upper crank can be while still allowing the lower crank to retract as much as it likes
k_max_upper_crank_where_retracting_lower_crank_safe_rad = math.radians(-65)

# ------------------- SHOOTER -------------------
k_flywheel_lower_left_neo_port = 10 #CAN ID
k_flywheel_upper_left_neo_port = 11 #CAN ID
k_shooter_timeoflight = 14  # time of flight CAN ID


# ------------------- Indexer -------------------
k_indexer_neo_port = 12

# ------------------- LED -------------------
k_led_pwm_port = 9
k_led_count = 44  # 26 old

# ------------------- Standard Values -------------------
k_speed_multiplier = 1.5
k_indexer_output = 0.33 * k_speed_multiplier  # was 1 (12V) when we had a 45:1, should be .33 (4V) for when we have a 15:1
k_intake_output = 0.5 * k_speed_multiplier

# ------------------- CAMERA -------------------
k_camera_name = "camera name" # todo: this is just a placeholder 2/28/24 LHACK

# --------------  SIMULATION AND FIELD INITIALIZATION  ---------------
k_start_x = 1.26  # eventually this will have to be set by the autonomous choices
k_start_y = 7.08

# --------------  HELPER FUNCTIONS  ---------------
def clamp(value: float, bottom: float, top: float) -> float:
    return max(bottom, min(value, top))

# --------------  AMP POSITIONS  ---------------
k_blue_amp = [1.9, 7.4, 90]  # (x, y, rotation)
k_red_amp = [14.75, 7.74, 90]  # (x, y, rotation)

# --------------  SPEAKER POSITIONS  ---------------
k_blue_speaker = [0, 5.55, 180]  # (x, y, rotation)
k_red_speaker = [16.5, 5.555, 0]  # (x, y, rotation)

k_speaker_opening_height = 2  # height of speaker opening

#---------------- APRILTAG POSITIONS ---------------
k_speaker_tags_poses = {"blue": [-0.038100, 5.547868], "red": [16.579342, 5.547868]}
