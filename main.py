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

# Adding Navio2/Python path
sys.path.insert(0, 'Navio2/Python')

import navio.rcinput
import navio.util

navio.util.check_apm()

# Initialize vector, rcin, with object from navio.rcinput parent
# class:
rcin = navio.rcinput.RCInput()

while (1):
    ch0 = rcin.read(0)    # Call accessor fxn, pass address for first element and...
    ch1 = rcin.read(1)    # initialize variables, accordingly
    ch2 = rcin.read(2)
    ch3 = rcin.read(3)
    print(ch0, ch1, ch2, ch3)
    time.sleep(1)
