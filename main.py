# main.py
# v1.0
# d18m05y2025
# Project: Towards AAMPDR System implementation code
# Current Objective: To read Rx-Tx PPM signals for:
# Roll (Channel 1), Pitch (Channel 2), Throttle
# (Channel 3) and Yaw (Channel 4):
# Sources: Navio2 RCInput.py code
#

# Import modules and libraries:
import sys, time

# Adding path
#sys.path.insert(0, 'Navio2/Python/navio')
sys.path.append('Navio2/Python/navio')

import pwm

sys.path.append('Navio2/Python')

import navio.rcinput    # For Rx, Tx
import navio.mpu9250    # For IMU
import navio.util

navio.util.check_apm()    # To check status of apm (Is apm alias for ArduPilot?)

# Setup RCIO:
# Initialize vector, rcin, with object from navio.rcinput parent
# class:
rcin = navio.rcinput.RCInput()

# Setup IMU:
imu = navio.mpu9250.MPU9250()    # Initialize IMU object
imu.initialize()                 # Activate/initialize newly created
                                 # IMU object

# Next, first channel, 'ch0' initialized by calling accessor
# fxn and passing index, accordingly:
while(True):
    # Step 1: Read RCIO data
    ch0 = rcin.read(0)    # Corresponds to roll on Tx
    ch1 = rcin.read(1)    # pitch
    ch2 = rcin.read(2)    # throtthle
    ch3 = rcin.read(3)    # yaw
    print(ch0, ch1, ch2, ch3)
    time.sleep(1)
    # Step 2: Read IMU data:
    imu_data = imu.getMotion9()
    acc_data = imu_data[0]
    print('')    # newline
    #print(m9a[0], m9a[1], m9a[2])
    print(imu_data[0])
    # print(imu_data[0])
    time.sleep(1)
