from dronekit import connect
from pymavlink import mavutil
import argparse


vehicle = connect('tcp:127.0.0.1:14550', wait_ready=True)

# Enable Battery Readings
vehicle.parameters['BATT_MONITOR'] = 4 
vehicle.parameters['BATT_VOLT_PIN'] = 2
vehicle.parameters['BATT_CURR_PIN'] = 3
vehicle.parameters['BATT_CAPACITY'] = 5000

# Frame Settings
vehicle.parameters['FRAME_CLASS'] = 1
vehicle.parameters['FRAME_TYPE'] = 1

# Failsafe
vehicle.parameters['FS_CRASH_CHECK'] = 1 # Crash
vehicle.parameters['BATT_LOW_TIMER'] = 1 # Time battery needs to be below failsafe to trigger in seconds
vehicle.parameters['BATT_LOW_VOLT'] = 14.3 #Battery low voltage
vehicle.parameters['BATT_LOW_MAH'] = 1000 #Battery low current
vehicle.parameters['BATT_FS_LOW_ACT'] = 3 # Lands on battery failsafe
vehicle.parameters['BATT_ARM_VOLT'] = 14.0 #Arms at minimum voltage
vehicle.parameters['FS_THR_ENABLE'] = 3 # Throttle Failsafe
vehicle.parameters['FS_THR_VALUE'] = 988 #Minumum throttle value
vehicle.parameters['FS_EKF_THRESH'] = 0.6 # EKF failsafe threshold

#Tuning
vehicle.parameters['MOT_BAT_VOLT_MAX'] = 16.8 #Sets the maximum/minumum voltage for scaling of prop speeds.
vehicle.parameters['MOT_BAT_VOLT_MIN'] = 13.2

vehicle.parameters['MOT_THST_EXPO'] = 0.65 
vehicle.parameters['MOT_PWM_MAX'] = 2000
vehicle.parameters['MOT_PWM_MIN'] = 1000
vehicle.parameters['MOT_SPIN_MAX'] = 0.95
vehicle.parameters['MOT_THST_HOVER'] = .25

vehicle.parameters['INS_ACCEL_FILTER'] = 10 #Sets how resistant the filters are to vibrations. The lower the number the more resistant.
vehicle.parameters['INS_GYRO_FILTER'] = 40

vehicle.parameters['ATC_ACCEL_P_MAX'] = 110000 #Sets pitch/roll/yaw max rotation speeds
vehicle.parameters[ 'ATC_ACCEL_R_MAX'] = 110000
vehicle.parameters['ATC_ACCEL_Y_MAX'] = 27000

vehicle.parameters['ACRO_YAW_P'] = 3

vehicle.parameters['ATC_RAT_PIT_FLTD'] = 20
vehicle.parameters['ATC_RAT_PIT_FLTT'] = 20
vehicle.parameters['ATC_RAT_RLL_FLTD'] = 20
vehicle.parameters['ATC_RAT_RLL_FLTT'] = 20
vehicle.parameters['ATC_RAT_YAW_FLTE'] = 2
vehicle.parameters['ATC_RAT_YAW_FLTT'] = 20

# Flight Mode Control Channel
vehicle.parameters['FLTMODE_CH'] = 0 # Disabled

# Throttle Deadzone used by AltHold, Loiter, PosHold
vehicle.parameters['THR_DZ'] = 100

# Landing
vehicle.parameters['LAND_SPEED'] = 50 # (cm/s) Increments of 10

# Arg Parser
parser = argparse.ArgumentParser(description='Choose calibration.')
parser.add_argument('--esc', help='Calibrate escs', action='store_true')
args = parser.parse_args()

# Calibrations


vehicle.parameters['INS_GYR_CAL'] = 1 # Gyro calibration on startup



if args.esc:
    if vehicle.battery.voltage > 13.2:
        print 'Calibrating ESCs'
        vehicle.parameters['ESC_CALIBRATION'] = 3
        print 'Reboot RPI'
        print 'Allow ESCs to go through beeping sequence'
        print 'Reboot one more time'
    else:
        print 'Battery Voltage: %s. Please charge' % vehicle.battery.voltage

vehicle.close()
