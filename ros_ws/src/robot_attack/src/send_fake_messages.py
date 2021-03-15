#!/usr/bin/env python

import rospy
import math
import array
import random

import numpy as np
import matplotlib.pyplot as plt

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan

def SendFakeMoveMessage(linear_vel, angular_vel):
    # Create the message
    twist = Twist()
    twist.linear.x          = linear_vel
    twist.linear.y          = 0.0
    twist.linear.z          = 0.0
    twist.angular.x         = 0.0
    twist.angular.y         = 0.0
    twist.angular.z         = angular_vel
    # Publish the message
    pub_vel.publish(twist)


def SendFakeScanMessage(length):
    range_readings = array.array('f',(random.uniform(0,4) for i in range(0,length)))
    intensity_readings = array.array('f',(random.uniform(0,4) for i in range(0,length)))

    # Create a fake object at 2 degrees
    range_readings[2] = 0.3 

    # Create the message
    laser = LaserScan()
    laser.angle_min             = 0.0
    laser.angle_max             = 6.26573181152
    laser.angle_increment       = 0.0174532923847
    laser.time_increment        = 2.99000002997e-05
    laser.scan_time             = 0.0
    laser.range_min             = 0.119999997318
    laser.range_max             = 3.5
    laser.ranges                = range_readings
    laser.intensities           = intensity_readings
    # Publish the message
    pub_scn.publish(laser)

def SendFakePosMessage(x,y):
    # Create the message
    tfstamped = TransformStamped()
    tfstamped.transform.translation.x       = x
    tfstamped.transform.translation.y       = y
    tfstamped.transform.translation.z       = 0.0
    tfstamped.transform.rotation.x          = 0.0
    tfstamped.transform.rotation.y          = 0.0
    tfstamped.transform.rotation.z          = 0.0
    tfstamped.transform.rotation.w          = 0.0
    # Publish the message
    pub_pos.publish(tfstamped)



if __name__=="__main__":
    global pos
    global scanner_readings

    # Declares if we are learning or not
    learning = True

    # Start the ros node
    rospy.init_node('turtle_attack')

    # Subscribers and publishers
    pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    pub_scn = rospy.Publisher('/scan', LaserScan, queue_size=10)
    pub_pos = rospy.Publisher('vicon/BUGS/BUGS', TransformStamped, queue_size=10)

    # Setting the rate
    set_rate = 200
    rate = rospy.Rate(set_rate)

    slow_down_counter = 0

    while not rospy.is_shutdown():

        # Sending fake messages
        SendFakeMoveMessage(0,0.3)

        slow_down_counter += 1
        if slow_down_counter > 2:
            slow_down_counter = 0
            #SendFakeScanMessage(360)
            
        #SendFakePosMessage(random.uniform(-2,2),random.uniform(-2,2))

        # Sleep
        rate.sleep()