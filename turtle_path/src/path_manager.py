#!/usr/bin/env python
import rospy
from math import pi, fmod, sin, cos, sqrt
from geometry_msgs.msg import Twist
# hint: some imports are missing
from turtlesim.msg import Pose
from turtle_path.srv import SetOrientation #syntax?
from turtle_path.srv import WalkDistance

cur_pos = Pose()

def cb_pose(data): # get the current position from subscribing the turtle position
    global cur_pos
    cur_pos = data

def cb_walk(req):
    if (req.distance < 0):
        return False
    
    #===== Boundary Test =====
    # hint: calculate the projected (x, y) after walking the distance,
    # and return false if it is outside the boundary

    #req = requested, cur_pos = current position

    moveX = req.distance * cos(cur_pos.theta) + cur_pos.x
    moveY = req.distance * sin(cur_pos.theta) + cur_pos.y

    if(moveX < 0 or moveX > 11 or moveY < 0 or moveY > 11):
        return False

    rate = rospy.Rate(100) # 100Hz control loop

    #===== Move Towards Target =====
    twist = Twist()
    dis = distance(cur_pos.x, cur_pos.y, moveX, moveY)
    print(dis," ",cur_pos.x," ",cur_pos.y," ",moveX," ",moveY)

    while (dis > 0.05): # control loop...
        
        # in each iteration of the control loop, publish a velocity
        twist.linear.x = 2 
        pub.publish(twist)
        # hint: you need to use the formula for distance between two points
        dis = distance(cur_pos.x, cur_pos.y, moveX, moveY)
        # print(dis)
        print(dis," ",cur_pos.x," ",cur_pos.y," ",moveX," ",moveY)
        
        rate.sleep()
    
    vel = Twist() # publish a velocity 0 at the end, to ensure the turtle really stops
    pub.publish(vel)

    return True

#===== Calculate Distance Between Two Points =====
def distance(cur_x,cur_y,goal_x,goal_y):
    distance = sqrt(pow((goal_x - cur_x), 2) + pow((goal_y - cur_y), 2))
    return distance

def cb_orientation(req):

    rate = rospy.Rate(100) # 100Hz control loop
    
    #===== change direction =====
    twist = Twist()
    dis = angle(cur_pos.theta,req.orientation)
    while (dis > 0.05): # control loop
        
        # in each iteration of the control loop, publish a velocity
        #twist.angular = 
        twist.angular = 2
        pub.publish(twist)
        # hint: signed smallest distance between two angles: 
        # see https://stackoverflow.com/questions/1878907/the-smallest-difference-between-2-angles
        #     dist = fmod(req.orientation - cur_pos.theta + pi + 2 * pi, 2 * pi) - pi
        dis = angle(cur_pos.theta,req.orientation)
        
        rate.sleep()
    
    vel = Twist() # publish a velocity 0 at the end, to ensure the turtle really stops
    pub.publish(vel)

    return True

#===== Calculate Angle Between Two Points =====
def angle(cur_ang, goal_ang):
    distance = fmod(goal_ang - cur_ang + pi + 2 * pi, 2 * pi) - pi
    return distance

if __name__ == '__main__':
    rospy.init_node('path_manager')
    
    pub = rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size = 1) # publisher of the turtle velocity
    sub = rospy.Subscriber('turtle1/Pose',Pose,cb_pose) # subscriber of the turtle position, callback to cb_pose
    
    ## init each service server here:
    rospy.Service('/set_orientation',SetOrientation,cb_orientation)		# callback to cb_orientation
    rospy.Service('/walk_distance',WalkDistance,cb_walk)		# callback to cb_walk
    
    rospy.spin()
