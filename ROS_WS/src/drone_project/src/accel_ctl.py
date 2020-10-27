import rospy
import time
from mavros_msgs.msg import OverrideRCIn
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import threading

"""
channel:    Min    Max    Default
#1:Roll     1000 - 2001   1500
#2:Pitch    1000 - 2001   1500
#3:Throttle 1019 - 2001   1030
#4:Yawl     1000 - 2001   1500

#5:for mode 1200 - 1401   
#6:for mode 1000 - 2001
        #5    #6
mode#1: 1200  1000
mode#2: 1300  1500
mode#3: 1400  2000
          Roll     Pitch   Throttle Yawl
Takeoff:  1023     1999    1990     1000
Land:     1021     1000    1990     1995
hold for more than 3s to send above command            
"""
print_msgs = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Throttle & Yawl:
        w
    a  s  d

Pitch & Roll:
Arrow key: 
    up
left  down  right

Takeoff: q
Land:    e
CTRL-C to quit
"""
class Accel_Publisher(object):
    def __init__(self):
        self.msg_info = print_msgs
        self.throttle_change_rate = 0
        self.pitch_change_rate = 0
        self.yaw_change_rate = 0
        self.roll_change_rate = 0
        
        self.target_yaw = 0
        self.target_pitch = 0
                         #Min, Max, Default
        self.Roll =      [1000,2001,1500]
        self.Pitch =     [1400,1600,1500] #[1000,2001,1500]
        self.Throttle =  [1019,2001,1030]
        self.Yaw =       [1300,1700,1500] #[1000,2001,1500]

        #Mode channel[4:5]
        self.Mode1 = [1200,1000] #Stable
        self.Mode2 = [1300,1500] #AltHold
        self.Mode3 = [1400,2000] #Land 30cm/s descends

        #Channel defalut channel[0:3]
        self.channel = [1500,1500,1030,1500,1200,1000,1500,1500] #defalut mode is mode#1: Stabilize

        #essential command channel[0:3]
        self.Take_off = [1500,1500,1030,1997]
        self.Land = [1500,1500,1030,1000]

        self.altitude_data = 0
        self.init_altitude = 0.0
        self.set_init_altitude = False
        self.target_hight = 0 
        self.target_hight_offset = 0 # wanted 1.5
        self.launch_status = False
        self.key = 'q'

        #PID 
        self.curr_error = 0.0
        self.prev_error = 0.0
        self.sum_error = 0.0
        self.curr_error_deriv = 0.0
        self.control = 0.0
        self.dt = 0.02

        # locker for thread safe
        # self.lock = threading.Lock()

        #for command 
        self.command = [0.0, 0.0, 0.0]

        print(self.msg_info)
        rospy.init_node('Override_RCIn_by_keyboard')
        self.pub = rospy.Publisher("/mavros/rc/override",OverrideRCIn, queue_size = 10)
        self.RC_data = OverrideRCIn()
        self.RC_data.channels = self.channel #set default
        self.pub.publish(self.RC_data)
        
        self.listener()
        rospy.Rate(10000)
        rospy.spin()

    def pid_control(self, current_error, Kp, Ki, Kd):
	    # insert your code from here
        self.prev_error = self.curr_error
        self.curr_error = current_error

        self.sum_error = self.sum_error + self.curr_error * self.dt
        self.curr_error_deriv = (self.curr_error - self.prev_error) / self.dt

        # Calculate PID control
        self.control = Kp * self.curr_error + Ki * self.sum_error + Kd * self.curr_error_deriv
        return self.control

    def set_default_channel(self):
        # default_channel = [1500,1500,1030,1500,1200,1000,1500,1500]
        print("Reset channel:")
        self.channel[0] = self.Roll[2]
        self.channel[1] = self.Pitch[2]
        self.channel[2] = self.Throttle[2]
        self.channel[3] = self.Yaw[2]

        self.channel[4] = self.Mode1[0]
        self.channel[5] = self.Mode1[1]
        self.channel[6] = 1500
        self.channel[7] = 1500

    def check_channel_boundary(self):
        #channel 0 Roll [1000,2001,1500]
        if self.channel[0] <= self.Roll[0]:
            self.channel[0] = self.Roll[0]
        elif self.channel[0] >= self.Roll[1]:
            self.channel[0] = self.Roll[1]
        
        #channel 1 Pitch [1000,2001,1500]
        if self.channel[1] <= self.Pitch[0]:
            self.channel[1] = self.Pitch[0]
        elif self.channel[1] >= self.Pitch[1]:
            self.channel[1] = self.Pitch[1]

        #channel 2 Throttle [1019,2001,1030]
        if self.channel[2] <= self.Throttle[0]:
            self.channel[2] = self.Throttle[0]
        elif self.channel[2] >= self.Throttle[1]:
            self.channel[2] = self.Throttle[1]

        #channel 3 Yawl [1000,2001,1500]
        if self.channel[3] <= self.Yaw[0]:
            self.channel[3] = self.Yaw[0]
        elif self.channel[3] >= self.Yaw[1]:
            self.channel[3] = self.Yaw[1]


    def land_command(self):
        self.channel[:4] = self.Land

    def takeoff_command(self):
        self.channel[:4] = self.Take_off

    def listener(self):
        rospy.Subscriber("/cv_command",Twist,self.command_callback)
        rospy.Subscriber("/mavros/global_position/rel_alt",Float64,self.altitude_callback)

    def accel_callback(self):
        if self.set_init_altitude == True:
            try:
                throttle_error = self.target_hight - self.altitude_data
                yaw_error = self.target_yaw - self.channel[3]
                pitch_error = self.target_pitch - self.channel[1]
                print("pitch:",self.target_pitch, "yaw:",self.target_yaw)
                self.throttle_change_rate = self.pid_control(throttle_error, 3, 1.5, 0.1)
                self.yaw_change_rate = self.pid_control(yaw_error, 0.75, 0.5, 0.05)
                self.pitch_change_rate = self.pid_control(pitch_error, 4, 0.001, 5)
                # self.roll_change_rate = self.pid_control(current_error, 4, 0.001, 5)

                #print("throttle rate: ",round(self.throttle_change_rate,2), "error: ",round(throttle_error,2))
                print("yaw rate: ",round(self.yaw_change_rate,2), "error: ",round(yaw_error,2))
                #print("pitch rate: ",round(self.throttle_change_rate,2), "error: ",round(pitch_error,2))

                if self.launch_status == True:
                    if self.throttle_change_rate > 0:
                        self.key = 'w'
                    else:
                        self.key = 's'

                self.channel[0] += int(self.roll_change_rate)
                self.channel[1] += int(self.pitch_change_rate)
                self.channel[2] += int(self.throttle_change_rate)
                self.channel[3] += int(self.yaw_change_rate)
                self.check_channel_boundary() #check range of the channel not be exceed
                #print("get: ",self.key)
        
                if self.key is 'q' or self.key is 'e': #take off or land
                    if self.key is 'q': #take off
                        self.takeoff_command()
                        print("Takeoff: ",self.channel)
                        self.RC_data.channels = self.channel
                        self.pub.publish(self.RC_data)
                        time.sleep(3) #need at least 3 second
                        self.pub.publish(self.RC_data)
                        self.set_default_channel() #restore back default state
                        self.launch_status = True

                    else: #land
                        self.land_command()
                        print("Land: ",self.channel)
                        self.RC_data.channels = self.channel
                        self.pub.publish(self.RC_data)
                        time.sleep(3) #need at least 3 second
                        self.set_default_channel() #restore back default state
                        self.launch_status = False
                
                elif self.key is 'z': #reset channel
                    print("get: ",self.key)
                    self.set_default_channel()

                print(self.channel)
                self.RC_data.channels = self.channel
                self.pub.publish(self.RC_data)
            except Exception as e:
                print(e)

            finally:
                pass
                # self.set_default_channel()
                # self.RC_data.channels = self.channel
                # self.pub.publish(self.RC_data)
        else:
            print("Initial altitude data is not set")

    def altitude_callback(self, msgs):      
        self.altitude_data = msgs.data
        if self.set_init_altitude == False:
            self.init_altitude = self.altitude_data
            self.set_init_altitude = True
            self.target_hight = self.target_hight_offset + self.init_altitude
            print("Altitude: ",self.altitude_data, "Target: ",self.target_hight,rospy.Time.now())

        self.accel_callback()

    def command_callback(self, msgs):
        self.command[0] = msgs.linear.x
        self.command[1] = msgs.linear.y
        self.command[2] = msgs.linear.z
        '''
        x < 0: yaw need to turn left --
        x = 0: yaw no change
        x > 0: yaw need to turn right ++

        y < 0: throttle need to go down --
        y = 0: throttle no change
        y > 0: throttle need to go up ++

        z < 0: pitch up ++
        z = 0: pitch no change
        z > 0: pitch down --
        '''
        # print(self.command)
        #for yaw
        if self.command[0] < 0:
            self.target_yaw = self.Yaw[0] #1300
        elif self.command[0] == 0:
            self.target_yaw = self.Yaw[2] #1500
        else:
            self.target_yaw = self.Yaw[1] #1700
        
        #for pitch
        if self.command[2] < 0:
            self.target_pitch = self.Pitch[1] #1600
        elif self.command[2] == 0:
            self.target_pitch = self.Pitch[2] #1500
        else:
            self.target_pitch = self.Pitch[0] #1400

        #                  #Min, Max, Default
        # self.Roll =      [1000,2001,1500]
        # self.Pitch =     [1400,1600,1500] #[1000,2001,1500]
        # self.Throttle =  [1019,2001,1030]
        # self.Yaw =       [1300,1700,1500] #[1000,2001,1500]
if __name__=="__main__":  
    flight_rc_ctl = Accel_Publisher()

