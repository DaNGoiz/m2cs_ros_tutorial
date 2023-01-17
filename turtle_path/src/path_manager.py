#!/usr/bin/env python
import rospy
from math import pi, fmod, sin, cos, sqrt
from geometry_msgs.msg import Twist
# hint: some imports are missing
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
    pose = Pose()

    moveX = req.distance * cos(pose.theta) + pose.x # may have prob
    moveY = req.distance * sin(pose.theta) + pose.y

    if(moveX < 0 or moveX > 11 or moveY < 0 or moveY > 11):
        return False

    rate = rospy.Rate(100) # 100Hz control loop

    #===== Move Towards Target =====
    while (moveX != pose.x or moveY != pose.y): # control loop... i guess it stucks
        
        # in each iteration of the control loop, publish a velocity
        twist.linear.x = 2
        pub.Publish(twist)
        # hint: you need to use the formula for distance between two points
        pose = Pose()
        
        rate.sleep()
    
    vel = Twist() # publish a velocity 0 at the end, to ensure the turtle really stops
    pub.publish(vel)

    return True

def cb_orientation(req):

    rate = rospy.Rate(100) # 100Hz control loop
    
    #===== change direction =====
    pose = Pose()
    twist = Twist()

    while (req.orientation != pose.theta): # control loop
        
        # in each iteration of the control loop, publish a velocity
        #twist.angular = 
        pub.Publish(twist)
        # hint: signed smallest distance between two angles: 
        # see https://stackoverflow.com/questions/1878907/the-smallest-difference-between-2-angles
        #     dist = fmod(req.orientation - cur_pos.theta + pi + 2 * pi, 2 * pi) - pi
        
        rate.sleep()
    
    vel = Twist() # publish a velocity 0 at the end, to ensure the turtle really stops
    pub.publish(vel)

    return True

if __name__ == '__main__':
    rospy.init_node('path_manager')
    
    pub = rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=1) # publisher of the turtle velocity
    sub = rospy.Subscriber('turtlesim/Pose',Pose,cb_pose) # subscriber of the turtle position, callback to cb_pose
    
    ## init each service server here:
    rospy.Service('/set_orientation',SetOrientation,cb_orientation)		# callback to cb_orientation
    rospy.Service('/walk_distance',WalkDistance,cb_walk)		# callback to cb_walk
    
    rospy.spin()
