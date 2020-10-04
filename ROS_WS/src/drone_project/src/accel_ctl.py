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

moveBindings = {
        'w':"Throttle up",
        's':"Throttle down",
        'a':"Yawl left",
        'd':"Yawl right",
        'q':"Takeoff",
        'e':"Land",
        '\x1b[A':"Pitch up",
        '\x1b[B':"Pitch down",
        '\x1b[D':"Roll Left",
        '\x1b[C':"Roll right",
    }

if __name__=="__main__":   
    try:
        print(msg)
        while(1):
            key = raw_input("Enter your command\n")
            if key in moveBindings.keys():
                print(moveBindings[key])
            else:
                print("Not a command\n")
                if (key == '\x03'):
                    break

    except Exception as e:
        print(e)

    finally:
        print("End Command\n")