import sys, time

import navio.rcinput
import navio.util

navio.util.check_apm()

rcin = navio.rcinput.RCInput()

while (True):
    Pitch = rcin.read(0)    #Channel 1
    Roll = rcin.read(1)     #Channel 2
    Throttle = rcin.read(2) #Channel 3
    Yaw = rcin.read(3)      #Channel 4

    SWA = rcin.read(5)      #Channel 6
    SWB = rcin.read(6)      #Channel 7
    print (Throttle," ", Yaw," ", Pitch," ", Roll," ", SWA," ", SWB)
    #time.sleep(0.5)
