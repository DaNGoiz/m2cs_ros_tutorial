#!/usr/bin/env python
import rospy
from m2_ps4.msg import Ps4Data # read the ps4 input
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, EmptyRequest
from turtlesim.srv import SetPen, SetPenRequest

old_data = Ps4Data()
level = 3 #level

def callback(data):
    global old_data #means the previous input state
    global pub

    global level

    #get input (move and rotate) from ds5
    moveX = data.hat_lx
    moveY = data.hat_ly

    rotate = data.hat_rx

    #give input to twist
    twist = Twist()

    twist.linear.x = data.hat_ly * level

    twist.angular.z = data.hat_rx
    pub.publish(twist)
    

    # hint: to detect a button being pressed, you can use the following pseudocode:
    # 
    # if ((data.button is pressed) and (old_data.button not pressed)),
    # then do something...

    #Velocity scale modification "up"/"down"
    if ((data.dpad_y == 1) and (old_data.dpad_y != 1) and (level <= 5)):
        level += 1

    if ((data.dpad_y == -1) and (old_data.dpad_y != -1) and (level >= 1)):
        level -= 1

    #Clear background "ps"
    if ((data.ps == True) and (old_data.ps == False)):
        #call the service
        msg = EmptyRequest() # have to send request when calling (rather than Empty)
        clear(msg)
    

    #Change pen color △/○/×/□ to green/red/blue/purple
    if ((data.triangle == True) and (old_data.triangle == False)):
        msg = SetPenRequest()
        msg.r = 0
        msg.g = 255
        msg.b = 80
        set_color(msg)
    elif((data.circle == True) and (old_data.circle == False)):
        msg = SetPenRequest()
        msg.r = 255
        msg.g = 0
        msg.b = 80
        set_color(msg)
    elif((data.cross == True) and (old_data.cross == False)):
        msg = SetPenRequest()
        msg.r = 0
        msg.g = 0
        msg.b = 255
        set_color(msg)
    elif((data.square == True) and (old_data.square == False)):
        msg = SetPenRequest()
        msg.r = 155
        msg.g = 100
        msg.b = 0
        set_color(msg)

    old_data = data

if __name__ == '__main__':
    rospy.init_node('ps4_controller')
    
    pub = rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=1)# publisher object goes here... hint: the topic type is Twist
    sub = rospy.Subscriber('input/ps4_data',Ps4Data,callback)# subscriber object goes here
    
    # one service object is needed for each service called!
    #srv_col = # service client object goes here... hint: the srv type is SetPen
    # fill in the othcc

    clear = rospy.ServiceProxy('clear', Empty)
    set_color = rospy.ServiceProxy('turtle1/set_pen',SetPen)

    rospy.loginfo("Ready to move the turtle.")
    rospy.spin()