import rospy
import time
from mavros_msgs.msg import OverrideRCIn
from std_msgs.msg import Float64
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
        self.inc_rate = 100
        self.dec_rate = -1*self.inc_rate
        self.moveBindings = {
                'w':(0,0,self.inc_rate,0), #"Throttle up"
                's':(0,0,self.dec_rate,0), #"Throttle down"
                'a':(0,0,0,self.dec_rate), #"Yawl left"
                'd':(0,0,0,self.inc_rate), #"Yawl right"
                'up':(0,self.dec_rate,0,0), #"Pitch up"
                'down':(0,self.inc_rate,0,0), #"Pitch down"
                'left':(self.dec_rate,0,0,0), #"Roll Left"
                'right':(self.inc_rate,0,0,0), #"Roll right"
        }
                         #Min, Max, Default
        self.Roll =      [1000,2001,1500]
        self.Pitch =     [1000,2001,1500]
        self.Throttle =  [1019,2001,1030]
        self.Yawl =      [1000,2001,1500]

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
        self.target_hight_offset = 1.1 # wanted 1.5
        self.launch_status = False
        self.key = 'q'

        # locker for thread safe
        # self.lock = threading.Lock()
        print(self.msg_info)
        rospy.init_node('Override_RCIn_by_keyboard')
        self.pub = rospy.Publisher("/mavros/rc/override",OverrideRCIn, queue_size = 10)
        self.RC_data = OverrideRCIn()
        self.listener()
        # rospy.Rate(1000)
        rospy.spin()

    def set_default_channel(self):
        # default_channel = [1500,1500,1030,1500,1200,1000,1500,1500]
        print("Reset channel:")
        self.channel[0] = self.Roll[2]
        self.channel[1] = self.Pitch[2]
        self.channel[2] = self.Throttle[2]
        self.channel[3] = self.Yawl[2]

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
        if self.channel[3] <= self.Yawl[0]:
            self.channel[3] = self.Yawl[0]
        elif self.channel[3] >= self.Yawl[1]:
            self.channel[3] = self.Yawl[1]
            
    def land_command(self):
        self.channel[:4] = self.Land

    def takeoff_command(self):
        self.channel[:4] = self.Take_off

    def listener(self):
        rospy.Subscriber("/mavros/global_position/rel_alt",Float64,self.callback)

    def callback(self, msgs):        
        self.altitude_data = msgs.data
        self.RC_data.channels = self.channel #set default
        self.pub.publish(self.RC_data)

        if self.set_init_altitude == False:
            self.init_altitude = self.altitude_data
            self.set_init_altitude = True
            self.target_hight = self.target_hight_offset + self.init_altitude
            print("Altitude: ",self.altitude_data, "Target: ",self.target_hight,rospy.Time.now())
        
        if self.set_init_altitude == True:
            try:
                # self.key = raw_input("Enter your command\n")       
                if self.altitude_data < self.target_hight:
                    self.key = 'w'
                else:
                    self.key = 's'
                if self.key in self.moveBindings.keys():
                    self.channel[0] = self.channel[0] + self.moveBindings[self.key][0]
                    self.channel[1] = self.channel[1] + self.moveBindings[self.key][1]
                    self.channel[2] = self.channel[2] + self.moveBindings[self.key][2]
                    self.channel[3] = self.channel[3] + self.moveBindings[self.key][3]
                    self.check_channel_boundary() #check range of the channel not be exceed
                    print("get: ",self.key)
        
                elif self.key is 'q' or self.key is 'e': #take off or land
                    if self.key is 'q': #take off
                        self.takeoff_command()
                        print("Takeoff: ",self.channel)
                        self.RC_data.channels = self.channel
                        self.pub.publish(self.RC_data)
                        time.sleep(3) #need at least 3 second
                        # self.pub.publish(self.RC_data)
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

                else:
                    print("Not a command: ",self.key,"\n")

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
             print("Initial altitude hight not set")

if __name__=="__main__":  
    flight_rc_ctl = Accel_Publisher()

