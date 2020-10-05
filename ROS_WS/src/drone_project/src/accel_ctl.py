import rospy
import time
from mavros_msgs.msg import OverrideRCIn
msg = """
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

inc_rate = 1.03
dec_rate = 2 - inc_rate
moveBindings = {
        'w':(1,1,inc_rate,1), #"Throttle up"
        's':(1,1,dec_rate,1), #"Throttle down"
        'a':(1,1,1,dec_rate), #"Yawl left"
        'd':(1,1,1,inc_rate), #"Yawl right"
        '\x1b[A':(1,dec_rate,1,1), #"Pitch up"
        '\x1b[B':(1,inc_rate,1,1), #"Pitch down"
        '\x1b[D':(dec_rate,1,1,1), #"Roll Left"
        '\x1b[C':(inc_rate,1,1,1), #"Roll right"
}

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

#Min, Max, Default
Roll =      [1000,2001,1500]
Pitch =     [1000,2001,1500]
Throttle =  [1019,2001,1030]
Yawl =      [1000,2001,1500]

#Mode channel[4:5]
Mode1 = [1200,1000] #Stable
Mode2 = [1300,1500] #AltHold
Mode3 = [1400,2000] #Land

#Channel defalut channel[0:3]
channel = [1500,1500,1030,1500,1200,1000,1500,1500] #defalut mode is mode#1: Stabilize

#essential command channel[0:3]
Take_off = [1500,1500,1030,1997]
Land = [1500,1500,1030,1000]

def set_default_channel():
    # default_channel = [1500,1500,1030,1500,1200,1000,1500,1500]
    print("Reset channel:")
    channel[0] = Roll[2]
    channel[1] = Pitch[2]
    channel[2] = Throttle[2]
    channel[3] = Yawl[2]

    channel[4] = Mode1[0]
    channel[5] = Mode1[1]
    channel[6] = 1500
    channel[7] = 1500

def check_channel_boundary():
    #channel 0 Roll
    if channel[0] <= Roll[0]:
        channel[0] = Roll[0]
    elif channel[0] >= Roll[1]:
        channel[0] = Roll[1]
    
    #channel 1 Pitch
    elif channel[1] <= Pitch[0]:
        channel[1] = Pitch[0]
    elif channel[1] >= Pitch[1]:
        channel[1] = Pitch[1]

    #channel 2 Throttle
    elif channel[2] <= Throttle[0]:
        channel[2] = Throttle[0]
    elif channel[2] >= Throttle[1]:
        channel[2] = Throttle[1]

    #channel 3 Yawl
    elif channel[3] <= Yawl[0]:
        channel[3] = Yawl[0]
    elif channel[3] >= Yawl[1]:
        channel[3] = Yawl[1]
    
    else:
        land_command()
        
def land_command():
    channel[:4] = Land

def takeoff_command():
    channel[:4] = Take_off

if __name__=="__main__":  
    pub = rospy.Publisher("/mavros/rc/override",OverrideRCIn, queue_size = 1)
    print("Set Publisher")
    rospy.init_node('Override_RCIn_by_keyboard')
    RC_data = OverrideRCIn()
    RC_data.channels = channel #set default
    pub.publish(RC_data)
    try:
        print(msg)
        while(1):
            key = raw_input("Enter your command\n")
            if key in moveBindings.keys():
		        print(moveBindings[key])
                channel[0] = int(channel[0]*moveBindings[key][0])
                channel[1] = int(channel[1]*moveBindings[key][1])
                channel[2] = int(channel[2]*moveBindings[key][2])
                channel[3] = int(channel[3]*moveBindings[key][3])
                # check_channel_boundary() #check range of the channel not be exceed
                    #channel 0 Roll
                if channel[0] <= Roll[0]:
                    channel[0] = Roll[0]
                elif channel[0] >= Roll[1]:
                    channel[0] = Roll[1]
                
                #channel 1 Pitch
                elif channel[1] <= Pitch[0]:
                    channel[1] = Pitch[0]
                elif channel[1] >= Pitch[1]:
                    channel[1] = Pitch[1]

                #channel 2 Throttle
                elif channel[2] <= Throttle[0]:
                    channel[2] = Throttle[0]
                elif channel[2] >= Throttle[1]:
                    channel[2] = Throttle[1]

                #channel 3 Yawl
                elif channel[3] <= Yawl[0]:
                    channel[3] = Yawl[0]
                elif channel[3] >= Yawl[1]:
                    channel[3] = Yawl[1]
                
                else:
                    land_command()

            elif key is 'q' or key is 'e': #take off or land
                if key is 'q': #take off
                    takeoff_command()
                    print("Takeoff: ",channel)
                    RC_data.channels = channel
                    pub.publish(RC_data)
                    time.sleep(3) #need at least 3 second
                    set_default_channel() #restore back default state

                else: #land
                    land_command()
                    print("Land: ",channel)
                    RC_data.channels = channel
                    pub.publish(RC_data)
                    time.sleep(3) #need at least 3 second
                    set_default_channel() #restore back default state
            
            elif key is 'z': #reset channel
                set_default_channel()

            else:
                print("Not a command\n")
                if (key == '\x03'):
                    break
            print(channel)
            RC_data.channels = channel
            pub.publish(RC_data)

    except Exception as e:
        print(e)

    finally:
        set_default_channel()
        RC_data.channels = channel
        pub.publish(RC_data)
        print("End Command\n")
