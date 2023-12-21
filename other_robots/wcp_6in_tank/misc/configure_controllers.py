import time
import rev

def configure_sparkmax(sparkmax: rev.CANSparkMax, pid_controller : rev.SparkMaxPIDController, pid_dict : dict,
                       can_id=0, slot=0, pid_only=False, burn_flash=False):
    """Set the PIDs, etc for the controllers, slot 0 is often position and slot 1 is often velocity
    Expects a sparkmax and a pid controller, as well as a dictionary with values - looks like this:
    pid_dict = {'kP': 0.002, 'kI': 0.004, 'kD': 0, 'kIz': 0.002, 'kFF': 0.0075, 'kArbFF': 0,
                           'kMaxOutput': 0.99, 'kMinOutput': -0.99, 'SM_MaxVel':5000, 'SM_MaxAccel':5000}
    """
    error_dict = {}
    error_dict.update({f'kP_ID{can_id}_S{slot}': pid_controller.setP(pid_dict['kP'], slot)})
    error_dict.update({f'kI_ID{can_id}_S{slot}': pid_controller.setI(pid_dict['kI'], slot)})
    error_dict.update({f'kD_ID{can_id}_S{slot}': pid_controller.setD(pid_dict['kD'], slot)})
    error_dict.update({f'kFF_ID{can_id}_S{slot}': pid_controller.setFF(pid_dict['kFF'], slot)})
    error_dict.update({f'kIZ_ID{can_id}_S{slot}': pid_controller.setIZone(pid_dict['kIz'], slot)})
    error_dict.update({f'MinMax_ID{can_id}_S{slot}': pid_controller.setOutputRange(pid_dict['kMinOutput'], pid_dict['kMaxOutput'], slot)})
    error_dict.update({f'Accel_ID{can_id}_S{slot}': pid_controller.setSmartMotionMaxVelocity(pid_dict['SM_MaxVel'], slot)})
    error_dict.update({f'Vel_ID{can_id}_S{slot}': pid_controller.setSmartMotionMaxAccel(pid_dict['SM_MaxAccel'], slot)})

    if not pid_only:
        error_dict.update({'VoltComp_' + str(can_id): sparkmax.enableVoltageCompensation(12)})

    if len(set(error_dict)) > 1:
        print('', f'*Sparkmax ID {can_id} setting*     *Response*')
        for key in sorted(error_dict.keys()):
            print(f'     {key:19} \t {error_dict[key]}', flush=True)
    else:
        print(f'\n *All SparkMax on id {can_id} report {list(set(error_dict))[0]}')

    if burn_flash:
        start_time = time.time()
        can_error = sparkmax.burnFlash()
        time.sleep(0.05)
        print(f'Burn flash on controller {can_id}: {can_error} {int(1000 * (time.time() - start_time)):2d}ms after starting')

    return error_dict
