import sys
import time

import navio.pwm
import navio.rcinput
import navio.leds
import navio.util
import spidev
import argparse 
import navio.mpu9250

navio.util.check_apm()
rcin = navio.rcinput.RCInput() 

class Motor:

    def __init__(self, PWM_MIN_Time, PWM_MAX_Time, Frequency_in_Hz, imu_type):
    # PWM_x = [0,1,2,3]
        self.PWM_MIN = PWM_MIN_Time #ms
        self.PWM_MAX = PWM_MAX_Time #ms
        self.range_of_PWM = self.PWM_MAX - self.PWM_MIN
        self.Frequency = Frequency_in_Hz
        self.is_initialized = False
        self.is_enable = False
        self.PWM1 = navio.pwm.PWM(0)
        self.PWM2 = navio.pwm.PWM(1)
        self.PWM3 = navio.pwm.PWM(2)
        self.PWM4 = navio.pwm.PWM(3)
        self.led = navio.leds.Led()
        self.imu_status = True
        self.imu_name = imu_type
        self.imu = navio.mpu9250.MPU9250()

    def initialize(self):
        #for PWM
        # PWM_s = [PWM1, PWM2, PWM3, PWM4]
        self.PWM1.initialize()
        self.PWM1.set_period(self.Frequency)
        self.PWM1.enable()

        self.PWM2.initialize()
        self.PWM2.set_period(self.Frequency)
        self.PWM2.enable()

        self.PWM3.initialize()
        self.PWM3.set_period(self.Frequency)
        self.PWM3.enable()

        self.PWM4.initialize()
        self.PWM4.set_period(self.Frequency)
        self.PWM4.enable()

        #for IMU
        if self.imu_name == 'mpu':
            # print "Selected: MPU9250"
            self.imu = navio.mpu9250.MPU9250()
        elif self.imu_name == 'lsm':
            # print "Selected: LSM9DS1"
            self.imu = navio.lsm9ds1.LSM9DS1()
        else:
            print "Wrong sensor name. Select: mpu or lsm"
            self.imu_status = False
            sys.exit(1)
        if self.imu.testConnection() and self.imu_status == True:
            print "IMU Connection established: True"
        else:
            sys.exit("IMU Connection established: False")

        if self.imu_status == True:
            self.imu.initialize()
            time.sleep(1)
            self.is_initialized = True
        else: 
            self.is_initialized = False
        
        # imu.read_all()
        # imu.read_gyro()
        # imu.read_acc()
        # imu.read_temp()
        # imu.read_mag()

        # print "Accelerometer: ", imu.accelerometer_data
        # print "Gyroscope:     ", imu.gyroscope_data
        # print "Temperature:   ", imu.temperature
        # print "Magnetometer:  ", imu.magnetometer_data

        # time.sleep(0.1)
        
    def enable(self):
        self.is_enable = True

    def disable(self):
        self.is_enable = False

    def led_status(self):
        #Black, Red, Green, Blue, Cyan, Magenta, Yellow, White        
        if self.is_initialized == False:
            self.led.setColor('Red')

            time.sleep(1)
        elif self.is_initialized == True:
            if self.is_enable == False:
                self.led.setColor('Yellow')
            else:
                self.led.setColor('Green')


    def run_pwm(self):
        if self.is_initialized == False :
            print("The motor driver is not initialized")
        else:
            if(self.is_enable == False):
                print("The motor is disable, please enable the driver first")
            else:
                while (True):
                    Channel1 = rcin.read(0)
                    Channel2 = rcin.read(1)
                    Channel3 = rcin.read(2)
                    Channel4 = rcin.read(3)
                    Channel5 = rcin.read(4)
                    Channel6 = rcin.read(5)

                    Roll = int(Channel1)
                    Pitch = int(Channel2)
                    Throttle = int(Channel3)
                    Yaw = int(Channel4)
                    SWA = int(Channel5)
                    SWB = int(Channel6)
                    
                    duty_in_ms = Throttle
                    duty_in_ms = ((float(duty_in_ms)-1000)/1000)*(0.97) + float(self.PWM_MIN)
                    self.PWM1.set_duty_cycle(duty_in_ms)
                    self.PWM2.set_duty_cycle(duty_in_ms)
                    self.PWM3.set_duty_cycle(duty_in_ms)
                    self.PWM4.set_duty_cycle(duty_in_ms)
                    self.led_status()
                    
                    #print RC_input
                    print(Throttle," ", Yaw, " ", Pitch, " ", Roll, " ", SWA, " ", SWB)
                   
                    #print IMU data
                    m9a, m9g, m9m = self.imu.getMotion9()
                    print ("Acc:", "{:+7.3f}".format(m9a[0]), "{:+7.3f}".format(m9a[1]), "{:+7.3f}".format(m9a[2]), 
                            " Gyr:", "{:+8.3f}".format(m9g[0]), "{:+8.3f}".format(m9g[1]), "{:+8.3f}".format(m9g[2]), 
                            " Mag:", "{:+7.3f}".format(m9m[0]), "{:+7.3f}".format(m9m[1]), "{:+7.3f}".format(m9m[2]) )
                    time.sleep(0.01)



if __name__ == "__main__":
    motor = Motor(1.025,1.995,50,'mpu') #min, max in ms
    motor.initialize()
    motor.enable()
    motor.run_pwm()