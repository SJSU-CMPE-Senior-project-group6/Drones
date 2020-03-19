import sys
import time

import navio.pwm
import navio.rcinput
import navio.util

navio.util.check_apm()
rcin = navio.rcinput.RCInput() 


# PWM_x = [0,1,2,3]

SERVO_MIN = 1.025 #ms
SERVO_MAX = 1.995 #ms

PWM1 = navio.pwm.PWM(0)
PWM2 = navio.pwm.PWM(1)
PWM3 = navio.pwm.PWM(2)
PWM4 = navio.pwm.PWM(3)

# PWM_s = [PWM1, PWM2, PWM3, PWM4]
PWM1.initialize()
PWM1.set_period(50)
PWM1.enable()

PWM2.initialize()
PWM2.set_period(50)
PWM2.enable()

PWM3.initialize()
PWM3.set_period(50)
PWM3.enable()

PWM4.initialize()
PWM4.set_period(50)
PWM4.enable()

while (True):
    Throttle = rcin.read(2)
    duty = int(Throttle)
    duty = ((float(duty)-1000)/1000)*(0.97) + float(SERVO_MIN)
    PWM1.set_duty_cycle(duty)
    PWM2.set_duty_cycle(duty)
    PWM3.set_duty_cycle(duty)
    PWM4.set_duty_cycle(duty)


